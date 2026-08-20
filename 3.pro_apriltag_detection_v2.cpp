#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <algorithm>

// ============================================================================
// pro_apriltag_detection_v2.cpp —— 视觉瞄准改进版
// 与原版 3.pro_apriltag_detection.cpp 接口完全一致（_tag:=X、/shoot、退出码），
// 可无缝替换。改进点（针对"瞄不准"）：
//
//  1. 角度误差代替米误差：atan2(x, z) 把横向偏移换算成相对光轴的偏航角，
//     与距离无关；容差 0.02 rad ≈ 1.15°（0.5m 处约 1cm），比原 0.04m(≈4.6°)
//     收紧约 4 倍。且该比值对 apriltag 尺寸参数标错不敏感。
//  2. 停稳确认：进容差 → 发零速 → 持续 0.5s 确认仍对准 → 才射击。
//     彻底消除"转着/晃着就开火"导致的脱靶（命中率最大杀手）。
//  3. 固定 20Hz 控制环 + 最新位姿缓存：控制平滑，不受检测消息抖动影响；
//     检测数据 0.3s 没更新 → 停车，杜绝"检测流断掉后车身空转到超时"。
//  4. 最小速度地板 + 限幅积分项：弥补轮子/电机死区，保证最后一点误差
//     能被修正；积分限幅防累积震荡。
//  5. 射击前打印最终角度误差，方便调参/标定。
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

        // 没看到过目标 / 数据过期：静止等待（保证一直发 0 速度，不会残留转向）
        if (!tag_seen_ || !last_stamp_.isValid() || (now - last_stamp_).toSec() > stale_timeout)
        {
            publishVel(0.0, 0.0);
            confirming_ = false;
            return;
        }

        // z 异常（目标在身后等）：停车等待
        if (last_z_ <= 0.05)
        {
            publishVel(0.0, 0.0);
            confirming_ = false;
            return;
        }

        // 偏航角误差：正 = 目标在视野右侧（相机光轴系 x 向右）
        double yaw_err = std::atan2(last_x_, last_z_);

        if (std::fabs(yaw_err) < yaw_tolerance)
        {
            // ==== 停稳确认：先停车，持续 confirm_duration 秒仍对准才射击 ====
            publishVel(0.0, 0.0);
            if (!confirming_)
            {
                confirming_ = true;
                confirm_start_ = now;
                ROS_INFO("Tag %d within tolerance (err=%.2f deg), settling...",
                         tag_id_, yaw_err * 180.0 / M_PI);
            }
            else if ((now - confirm_start_).toSec() >= confirm_duration)
            {
                ROS_INFO("Tag %d settled, final err=%.2f deg, shooting!",
                         tag_id_, yaw_err * 180.0 / M_PI);
                bool ok = shoot_client.exists() && shoot_client.call(empty_srv);
                if (ok) { ROS_INFO("Shoot service called OK"); finish(0); }
                else    { ROS_ERROR("Shoot service call FAILED"); finish(2); }
            }
            return;
        }

        // 出容差 → 取消确认状态，重新瞄准
        confirming_ = false;

        // 积分项（限幅），消除轮子/摩擦造成的稳态残差
        double dt = (now - last_ctrl_time_).toSec();
        last_ctrl_time_ = now;
        if (dt > 0.0 && dt < 0.5)
        {
            integral_ += yaw_err * dt;
            integral_ = std::max(-integral_limit_, std::min(integral_limit_, integral_));
        }

        double cmd = Kp * yaw_err + Ki * integral_;

        // 最小速度地板：P 输出小于机器人最小可控转速时，强制给最小修正速度，
        // 防止"差一点点却永远修不动"→ 超时
        if (std::fabs(cmd) < min_omega)
            cmd = (cmd >= 0.0 ? min_omega : -min_omega);

        publishVel(0.0, -cmd);   // 负号：目标在右(x>0) → 顺时针(ω<0) 转回中心
    }

private:
    // ==== 可调参数（改这里调准度/稳定性） ====
    const double Kp = 2.0;                 // 角度比例增益（原版对"米"用 1.0，现对"弧度"）
    const double Ki = 0.3;                 // 积分增益
    const double integral_limit_ = 0.05;   // 积分限幅
    const double yaw_tolerance = 0.02;     // 对准容差：0.02 rad ≈ 1.15°（0.5m 处约 1cm）
    const double min_omega = 0.03;         // 最小修正角速度（rad/s），弥补电机死区
    const double confirm_duration = 0.5;   // 停稳确认时长（秒）
    const double stale_timeout = 0.3;      // 检测数据超过该时长未更新 → 停车
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

    // 积分状态
    double integral_ = 0.0;
    ros::Time last_ctrl_time_;

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
