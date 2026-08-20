#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <algorithm>

// ============================================================================
// pro_apriltag_detection_v2.cpp —— 视觉瞄准改进版（v2.1）
// 与原版 3.pro_apriltag_detection.cpp 接口完全一致（_tag:=X、/shoot、退出码），
// 可无缝替换。改进点（针对"瞄不准"）：
//
//  1. 停稳确认：进停稳区 → 发零速 → 持续 0.5s 确认仍对准 → 才调 /shoot；
//     出容差立刻取消确认重新瞄。
//  2. 角度误差：atan2(x, z) 换成弧度，与距离无关；容差 0.02 rad ≈ 1.15°
//     （0.5m 处约 1cm），且该比值对 apriltag 尺寸参数标错不敏感（x/z 同比例缩放）。
//  3. 最小速度地板 + 限幅积分：弥补轮子/电机死区，保证最后那点误差能被修正。
//  4. 固定 20Hz 控制环：回调只缓存最新位姿，主循环统一发速度；
//     检测数据 1.0s 未更新 → 停车（防"检测流断掉后车身空转到超时"）。
//  5. 射击前打印最终误差角与距离，方便现场调参。
//
//  v2.1 修复/增强：
//   A. stale_timeout 0.3s → 1.0s：实测相机+apriltag 检测出帧率仅 3~4Hz 且
//      间隔不均匀（0.25~0.35s），0.3s 的过期阈值恰好和检测间隔撞车，
//      导致"进容差后确认状态被反复清零 → 容差内干等 5 秒最后超时"。
//   B. 两级停稳：进入 1.15° 容差后不停车，先以最小速度慢速爬到 0.46°
//      停稳区才停车确认；若爬行 3s 仍进不了停稳区（机械死区/回差），
//      按当前误差（仍在射击容差内）兜底开火，绝不白白超时。
//   C. 最小速度地板 0.03 → 0.05 rad/s：保证"差半度"时的修正指令真能推动车。
//
// 退出码：0=射击成功，1=瞄准超时，2=射击服务失败。
// ============================================================================

class AprilTagController
{
public:
    AprilTagController() : private_nh_("~")
    {
        tag_sub_ = nh_.subscribe("tag_detections", 1, &AprilTagController::tagCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        shoot_client = nh_.serviceClient<std_srvs::Empty>("/shoot");
        shoot_client.waitForExistence(ros::Duration(5.0));
        if (!shoot_client.exists())
            ROS_ERROR("Shoot service NOT found! (is shoot_service_node running?)");

        if (!private_nh_.getParam("tag", tag_id_))
        {
            ROS_ERROR("Param 'tag' not set! Launch with _tag:=X");
            tag_id_ = -1;
        }
        ROS_INFO("Target tag = %d", tag_id_);
        start_time_ = ros::Time::now();
        last_ctrl_time_ = start_time_;
        near_start_ = start_time_;
    }

    int exitCode() const { return exit_code_; }

    // 回调只负责"记录最新位姿"，不做控制 → 控制节奏与检测节奏解耦
    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
        if (done_) return;
        for (const auto &detection : msg->detections)
        {
            if (detection.id.empty()) continue;
            if (detection.id[0] != tag_id_) continue;   // 其他靶子一概忽略

            last_x_ = detection.pose.pose.pose.position.x;
            last_z_ = detection.pose.pose.pose.position.z;

            // 时间戳取检测自身，无效则退回消息头时间戳
            ros::Time t = detection.pose.header.stamp;
            if (!t.isValid()) t = msg->header.stamp;
            if (!t.isValid()) t = ros::Time::now();
            last_stamp_ = t;
            tag_seen_ = true;
            return;
        }
        // 这一帧没看到目标：不清除旧值，由 update() 里的 staleness 判断停车
        // （避免单帧漏检导致控制闪断）
    }

    // 主循环每 50ms 调用一次：状态机 + 控制输出
    void update()
    {
        if (done_) return;

        ros::Time now = ros::Time::now();

        // 总超时：放弃（防卡死）
        if ((now - start_time_).toSec() > aim_timeout)
        {
            ROS_WARN("Aiming timeout for tag %d, give up", tag_id_);
            finish(1);
            return;
        }

        // 没看到过目标 / 数据过期（1.0s）：静止等待（一直发 0 速度，不会残留转向）
        if (!tag_seen_ || !last_stamp_.isValid() || (now - last_stamp_).toSec() > stale_timeout)
        {
            publishVel(0.0, 0.0);
            confirming_ = false;
            in_near_zone_ = false;
            return;
        }

        // z 异常（目标在身后等）：停车等待
        if (last_z_ <= 0.05)
        {
            publishVel(0.0, 0.0);
            confirming_ = false;
            in_near_zone_ = false;
            return;
        }

        // 偏航角误差：正 = 目标在视野右侧（相机光轴系 x 向右）
        double yaw_err = std::atan2(last_x_, last_z_);
        double abs_err = std::fabs(yaw_err);

        // ============ 射击容差区（|err| < 1.15°）============
        if (abs_err < yaw_tolerance)
        {
            if (!in_near_zone_)
            {
                in_near_zone_ = true;
                near_start_ = now;
            }

            // ---- 停稳区（|err| < 0.46°）：停车 + 0.5s 确认 → 射击 ----
            if (abs_err < settle_tolerance)
            {
                publishVel(0.0, 0.0);
                if (!confirming_)
                {
                    confirming_ = true;
                    confirm_start_ = now;
                    integral_ = 0.0;   // 停车前清积分，防止再启动时猛冲
                    ROS_INFO("Tag %d within settle tolerance (err=%.2f deg), settling...",
                             tag_id_, yaw_err * 180.0 / M_PI);
                }
                else if ((now - confirm_start_).toSec() >= confirm_duration)
                {
                    ROS_INFO("Tag %d settled, final err=%.2f deg (z=%.2fm), shooting!",
                             tag_id_, yaw_err * 180.0 / M_PI, last_z_);
                    shootNow();
                }
                return;
            }

            // ---- 爬行区（0.46° ~ 1.15°）：慢速逼近中心 ----
            confirming_ = false;
            double dt = (now - last_ctrl_time_).toSec();
            last_ctrl_time_ = now;
            if (dt > 0.0 && dt < 0.5)
            {
                integral_ += yaw_err * dt;
                integral_ = std::max(-integral_limit_, std::min(integral_limit_, integral_));
            }
            double cmd = Kp * yaw_err + Ki * integral_;

            // 兜底：在容差区爬了 creep_max 秒还进不了停稳区（机械死区/回差）
            // → 按当前误差直接开火，避免白白超时
            if ((now - near_start_).toSec() > creep_max)
            {
                ROS_INFO("Tag %d creep grace exceeded, fire at err=%.2f deg (z=%.2fm)",
                         tag_id_, yaw_err * 180.0 / M_PI, last_z_);
                shootNow();
                return;
            }

            if (std::fabs(cmd) < min_omega) cmd = (cmd >= 0.0 ? min_omega : -min_omega);
            publishVel(0.0, -cmd);   // 负号：目标在右(x>0) → 顺时针(ω<0) 转回中心
            return;
        }

        // ============ 正常修正区（|err| >= 1.15°）============
        in_near_zone_ = false;
        confirming_ = false;

        double dt = (now - last_ctrl_time_).toSec();
        last_ctrl_time_ = now;
        if (dt > 0.0 && dt < 0.5)
        {
            integral_ += yaw_err * dt;
            integral_ = std::max(-integral_limit_, std::min(integral_limit_, integral_));
        }
        double cmd = Kp * yaw_err + Ki * integral_;
        if (std::fabs(cmd) < min_omega) cmd = (cmd >= 0.0 ? min_omega : -min_omega);
        publishVel(0.0, -cmd);
    }

private:
    // ==== 可调参数 ====
    const double Kp = 2.0;                 // 角度比例增益（对"弧度"）
    const double Ki = 0.3;                 // 积分增益
    const double integral_limit_ = 0.15;   // 积分限幅（v2.1: 0.05→0.15，够补偿死区）
    const double yaw_tolerance = 0.02;     // 射击容差：0.02 rad ≈ 1.15°（0.5m 处约 1cm）
    const double settle_tolerance = 0.008; // 停稳容差：0.008 rad ≈ 0.46°（进入才停车确认）
    const double min_omega = 0.05;         // 最小修正角速度（v2.1: 0.03→0.05，够推动车子）
    const double confirm_duration = 0.5;   // 停稳确认时长（秒）
    const double creep_max = 3.0;          // 容差区爬行最长秒数，超时兜底开火
    const double stale_timeout = 1.0;      // 检测数据超过该秒数未更新 → 停车
                                           // （v2.1: 0.3→1.0。必须大于实测最大检测间隔，
                                           //   否则确认状态被反复清零导致"容差内干等超时"）
    const double aim_timeout = 8.0;        // 总瞄准超时（秒）

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber tag_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::ServiceClient shoot_client;
    std_srvs::Empty empty_srv;

    int tag_id_ = -1;
    ros::Time start_time_;
    int exit_code_ = 0;
    bool done_ = false;

    // 最新目标位姿（回调写入，update 读取）
    double last_x_ = 0.0;
    double last_z_ = 0.0;
    ros::Time last_stamp_;
    bool tag_seen_ = false;

    // 停稳确认状态
    bool confirming_ = false;
    ros::Time confirm_start_;

    // 容差区计时（爬行兜底用）
    bool in_near_zone_ = false;
    ros::Time near_start_;

    // 积分状态
    double integral_ = 0.0;
    ros::Time last_ctrl_time_;

    void shootNow()
    {
        if (done_) return;
        bool ok = shoot_client.exists() && shoot_client.call(empty_srv);
        if (ok) { ROS_INFO("Shoot service called OK"); finish(0); }
        else    { ROS_ERROR("Shoot service call FAILED"); finish(2); }
    }

    void publishVel(double linear_x, double angular_z)
    {
        // 限幅，防止过冲/震荡
        angular_z = std::max(-0.35, std::min(0.35, angular_z));
        linear_x  = std::max(-0.3,  std::min(0.3,  linear_x));
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_x;
        cmd_vel.angular.z = angular_z;
        cmd_vel_pub_.publish(cmd_vel);
    }

    void finish(int code)
    {
        publishVel(0.0, 0.0);   // 先停车再退出
        exit_code_ = code;
        done_ = true;
        ros::shutdown();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pro_apriltag_detect_node");
    AprilTagController controller;
    ros::Rate loop_rate(20);   // 固定 20Hz 控制环（原版 10Hz + 回调里发速度）
    while (ros::ok())
    {
        ros::spinOnce();
        controller.update();
        loop_rate.sleep();
    }
    ROS_INFO("Node exiting with code %d", controller.exitCode());
    return controller.exitCode();
}
