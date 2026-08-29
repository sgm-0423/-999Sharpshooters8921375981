// ============================================================================
// shoot_robot3.cpp —— 2026 中国机器人大赛 · 武术擂台赛 · 视觉挑战 · 总决赛(国赛)
//
// 【代码来源】
//   基于 shoot_robot2.cpp 修改,只改「第 2~4 个普通靶」的到达与打靶逻辑:
//
//   ▸ 靶1(与 shoot_robot2.cpp 完全一致):不导航,原点向右转 20° 后直接开打
//   ▸ 靶2~4(新逻辑):
//       1) 打完靶1 后,从当前位置直线直冲固定点
//          map 系下 (x=0.517381, y=1.926876, yaw=1.512686)(tf_echo 实测点);
//          途中完全不避障——不看激光、不看代价地图,遇到障碍物直接撞过去;
//       2) 到点后先转到绝对朝向 yaw=1.512686;
//       3) 依次:向右转头 45° → 开启打靶(靶2)
//                再向左偏转 100° → 开启打靶(靶3)
//                再向左偏转 100° → 开启打靶(靶4)
//   ▸ 最后一个靶(基地标靶,与 shoot_robot2.cpp 完全一致):
//       导航到基地前置点 → 前进 BASE_FORWARD_DIST → 打 BASE_TAG_ID
//
// 【重要】打靶微调 AprilTagController(对中 P 控制 + ±0.45/±0.40/±0.30 扫描、
//         丢失 1.8s 判击倒、15s 超时等)与 shoot_robot2.cpp 一字不差,未做任何修改。
//         激光全程常开、参数区、Move2goal/moveForDuration/rotate 等外壳全部一致。
// ============================================================================

#include <ros/ros.h>
#include <ros/package.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>      // 【新增】直冲/闭环转向用
#include <tf/transform_datatypes.h>     // 【新增】tf::getYaw 用
#include <std_srvs/Empty.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// ============================================================================
// ★★★ ============ 可调参数区（改这里，不用改下面代码） ============ ★★★
// ============================================================================

// ---------------------------------------------------------------------------
// 【A. 团队身份与赛制】
// ---------------------------------------------------------------------------
// ★★★ 【改动】基地标靶 ID：赛制变时只改这一个数即可（不再靠 team_color 颜色推导）：
//         排位赛 ：黄方出发打蓝方基地 → 3
//         淘汰赛 ：黄方打蓝方基地 → 3 ；蓝方打黄方基地 → 2
#define BASE_TAG_ID  3   // ★ 改这里：排位赛=3，淘汰赛若打黄方基地则改 2

// ---------------------------------------------------------------------------
// 【B. 打靶数量与是否打基地】
//   NORMAL_TARGET_COUNT : 普通靶总数(仅作说明保留；靶2~4 已改为直冲逻辑,不再循环)
//   SHOOT_BASE          : 是否在普通靶之后去打基地标靶。排位赛/淘汰赛都应 = true。
// ---------------------------------------------------------------------------
#define NORMAL_TARGET_COUNT 4          // ★ 普通靶总数=4（靶1原点打 + 靶2~4 直冲打）
#define SHOOT_BASE          true       // ★ 打完普通靶后是否打基地标靶
#define FIRST_TARGET_TURN_DEG  20.0    // ★ 第一个靶：从原点向右(顺时针)转的角度（度）
#define BASE_FORWARD_DIST    0.30      // ★ 基地靶：到达前置点后继续前进的距离(米)，0.30=30cm
#define BASE_FORWARD_SPEED   0.20      // ★ 基地靶前进速度(米/秒)

// ---------------------------------------------------------------------------
// 【C. 靶位坐标表】（★ 靶2~4 已改直冲逻辑,此表不再使用;仅保留 base_target 供基地靶用 ★）
// ---------------------------------------------------------------------------
struct Waypoint { double x; double y; double yaw; };

// 【改动 2026-08-29】normal_targets 整表注释：靶2~4 已改直冲逻辑(RUSH_TARGET + 原地旋转打靶),
// 不再走 move_base 逐个导航,本表成为死代码(全文件仅 base_target 仍被阶段二使用),故整体注释保留备查。
// Waypoint normal_targets[8] = {
//     //          x         y         yaw
//     { 0.933763,  1.380080,  1.380214 },   // 普通靶2（第二点）【改动 2026-08-29】tf_echo 实测替换
//     { 0.866270,  2.409344,  2.868367 },   // 普通靶3（第三点）【改动 2026-08-29】tf_echo 实测替换
//     { 0.219239,  2.568503, -1.857999 },   // 普通靶4（第四点）【改动 2026-08-29】tf_echo 实测替换
//     // ---- 普通靶5~8 暂时注释 ----
//     // { 0.0,       0.0,       0.0 },   // 普通靶5  ★待实测
//     // { 0.0,       0.0,       0.0 },   // 普通靶6  ★待实测
//     // { 0.0,       0.0,       0.0 },   // 普通靶7  ★待实测
//     // { 0.0,       0.0,       0.0 },   // 普通靶8  ★待实测
// };

// 对方基地标靶的前置点（打基地标靶时走到这里）。
Waypoint base_target = { 1.562416, 1.530104, 0.811659 };   // 对方基地主靶点

// ---------------------------------------------------------------------------
// 【D. 靶2~4 直冲逻辑参数】（★ shoot_robot3 新增,实测/微调改这里 ★）
//   打完靶1 后,从当前位置直线直冲 RUSH_TARGET 点(map 系,tf_echo 实测),
//   途中不避障、直接撞过去;到点后转到 RUSH_ARRIVE_YAW,再依次旋转打靶。
//   旋转是"相对到达朝向"累加:右转45°(负=顺时针) → 左转100° → 左转100°。
// ---------------------------------------------------------------------------
#define RUSH_TARGET_X   0.517381     // ★ 直冲目标点 x（map 系）
#define RUSH_TARGET_Y   1.926876     // ★ 直冲目标点 y（map 系）
#define RUSH_ARRIVE_YAW 1.512686     // ★ 到点后车头绝对朝向 yaw（map 系,弧度）
#define TURN2_DEG       -45.0        // ★ 靶2:向右转头 45°（负 = 顺时针）
#define TURN3_DEG        100.0       // ★ 靶3:再向左偏转 100°（正 = 逆时针）
#define TURN4_DEG        100.0       // ★ 靶4:再向左偏转 100°（正 = 逆时针）【改动 2026-08-29】120°→100°
#define RUSH_SPEED       0.25        // ★ 直冲线速度(米/秒)
#define RUSH_TIMEOUT     30.0        // ★ 直冲超时(秒)：撞不动/到不了就放弃进入下一步
#define RUSH_ARRIVE_DIST 0.15        // ★ 直冲到达判定距离(米)

#define DEG2RAD (M_PI / 180.0)

// ============================================================================
// ★★★ ===================== 可调参数区结束 ===================== ★★★
// ============================================================================


// ---------------------------------------------------------------------------
// move_base 闭环导航到目标点 (x, y, yaw) —— 与 shoot_robot2.cpp 完全一致
// ---------------------------------------------------------------------------
void Move2goal(MoveBaseClient& ac, double x, double y, double yaw)
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The Goal Reached Successfully!!!");
    else
        ROS_WARN("The Goal Planning Failed for some reason");
}

// ---------------------------------------------------------------------------
// 开环直线运动/旋转：障碍区强制平移或原地转，不走 move_base —— 与 shoot_robot2.cpp 一致
// ---------------------------------------------------------------------------
void moveForDuration(ros::Publisher& cmd_vel_pub, double linear_x, double angular_z, double duration)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_x;
    cmd_vel.angular.z = angular_z;

    ros::Rate rate(10);                 // 10Hz
    int total = (int)(duration * 10);
    for (int i = 0; i < total && ros::ok(); i++)
    {
        cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }

    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    cmd_vel_pub.publish(cmd_vel);
}

// ---------------------------------------------------------------------------
// 原地旋转指定角度（angle_deg 单位度；angular_z 正=逆时针，角速度约 1.0 rad/s）
// —— 与 shoot_robot2.cpp 完全一致
// ---------------------------------------------------------------------------
void rotate(ros::Publisher& cmd_vel_pub, double angular_z, double angle_deg)
{
    double angle_rad = angle_deg * 0.0174533;
    moveForDuration(cmd_vel_pub, 0.0, angular_z, angle_rad / 1.0);
}

// ---------------------------------------------------------------------------
// 【新增】直冲函数：完全不避障，直接撞过去
//   从当前位置朝目标点 (tx, ty)（map 系）直线硬冲：
//     - 每 50ms 取一次 map→base_link 位姿,实时把车头对准目标方向（闭环对准,不开环漂）
//     - 不订阅激光、不看代价地图、不做任何避障 —— 途中遇到障碍物直接撞/推过去
//     - 距离 < RUSH_ARRIVE_DIST 或超过 max_duration 秒即停车
// ---------------------------------------------------------------------------
void goStraightNoAvoid(ros::Publisher& cmd_vel_pub, tf::TransformListener& listener,
                       double tx, double ty, double speed, double max_duration)
{
    const double Kp_turn = 0.8;     // 转向增益
    const double max_omega = 0.6;   // 最大转向角速度(rad/s)
    ros::Time start = ros::Time::now();
    ros::Rate rate(20);             // 20Hz

    while (ros::ok())
    {
        tf::StampedTransform tf;
        try
        {
            listener.lookupTransform("map", "base_link", ros::Time(0), tf);
        }
        catch (tf::TransformException& ex)
        {
            ros::Duration(0.1).sleep();
            continue;
        }

        const double cx = tf.getOrigin().x();
        const double cy = tf.getOrigin().y();
        const double cyaw = tf::getYaw(tf.getRotation());

        const double dx = tx - cx;
        const double dy = ty - cy;
        const double dist = std::sqrt(dx*dx + dy*dy);
        if (dist < RUSH_ARRIVE_DIST)
        {
            ROS_INFO("Rush reached (%.3f, %.3f), dist=%.3f", tx, ty, dist);
            break;
        }

        // 车头方向误差（归一化到 [-π, π]）
        double yaw_err = std::atan2(dy, dx) - cyaw;
        while (yaw_err > M_PI)  yaw_err -= 2.0 * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

        double omega = Kp_turn * yaw_err;
        if (omega > max_omega)  omega = max_omega;
        if (omega < -max_omega) omega = -max_omega;

        geometry_msgs::Twist cmd;
        cmd.linear.x = speed;
        cmd.angular.z = omega;
        cmd_vel_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();

        if ((ros::Time::now() - start).toSec() > max_duration)
        {
            ROS_WARN("Rush timeout, still %.2f m away from target, give up", dist);
            break;
        }
    }

    geometry_msgs::Twist stop;
    cmd_vel_pub.publish(stop);
}

// ---------------------------------------------------------------------------
// 【新增】原地转到绝对朝向 target_yaw（map 系,弧度）
//   闭环修正：每 50ms 取一次当前朝向,按误差比例转到目标,误差 < ±1.15° 停止。
//   每次打靶前用它精确摆正车头,避免打靶微调前的朝向误差累积。
// ---------------------------------------------------------------------------
void rotateToYawAbs(ros::Publisher& cmd_vel_pub, tf::TransformListener& listener, double target_yaw)
{
    const double tol = 0.02;        // ±0.02 rad ≈ 1.15° 停
    const double Kp = 1.0;          // 转向增益
    const double max_omega = 0.8;   // 最大转向角速度(rad/s)
    ros::Rate rate(20);

    for (int k = 0; k < 400 && ros::ok(); k++)   // 最多 20s
    {
        tf::StampedTransform tf;
        try
        {
            listener.lookupTransform("map", "base_link", ros::Time(0), tf);
        }
        catch (tf::TransformException& ex)
        {
            ros::Duration(0.1).sleep();
            continue;
        }

        const double cyaw = tf::getYaw(tf.getRotation());
        double err = target_yaw - cyaw;
        while (err > M_PI)  err -= 2.0 * M_PI;
        while (err < -M_PI) err += 2.0 * M_PI;

        if (std::fabs(err) < tol)
            break;

        double omega = Kp * err;
        if (omega > max_omega)  omega = max_omega;
        if (omega < -max_omega) omega = -max_omega;

        geometry_msgs::Twist cmd;
        cmd.angular.z = omega;
        cmd_vel_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::Twist stop;
    cmd_vel_pub.publish(stop);
    ROS_INFO("Rotated to absolute yaw %.3f", target_yaw);
}


// ============================================================================
// AprilTagController —— 内嵌瞄准控制器
//
// ★★ 本类与 shoot_robot2.cpp 一字不差（含所有常量、算法、话题、判命中逻辑）★★
// 瞄准原理（照抄 shoot_robot2.cpp / ultimate.cpp）：
//   1) 订阅 tag_detections，找到 tag_id 对应的码，取相机系 x(横向) z(纵深)
//   2) 对中阶段：yaw_err = atan2(x,z) - yaw_offset，P 控制转到竖直中缝，停住 0.3s
//   3) 微调阶段：按 0 → ±0.45 → 0 → ±0.40 → 0 → ±0.30 → 0 序列扫描，激光常开
//   4) 击倒判定：对中/微调期间连续 1.8s 看不到该码 → 视为靶已倒
//   5) 超时：码仍在画面里但对中+微调合计 15s 还没倒 → 放弃该靶(返回 1)
//
//   【可调参数都在类末尾 private 区，以 ★ 标注，一般无需改动】
// ============================================================================
class AprilTagController
{
public:
    AprilTagController() : private_nh_("~")
    {
        tag_sub_ = nh_.subscribe("tag_detections", 1, &AprilTagController::tagCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        private_nh_.param("calib_file", calib_file_, std::string(""));
        if (calib_file_.empty())
        {
            const std::string share = ros::package::getPath("shoot_robot");
            calib_file_ = share.empty()
                ? std::string("optical_axis_calib.yaml")
                : share + "/config/optical_axis_calib.yaml";
        }
        loadCalibFile();
        private_nh_.param("yaw_offset", yaw_offset_, yaw_offset_);
        ROS_INFO("Aim yaw_offset = %.3f deg  (%s)",
                 yaw_offset_ * 180.0 / M_PI, calib_file_.c_str());
    }

    int exitCode() const { return exit_code_; }

    void beginTarget(int tag_id)
    {
        tag_id_ = tag_id;
        done_ = false;
        exit_code_ = 0;
        tag_seen_ = false;
        last_x_ = 0.0;
        last_z_ = 0.0;
        confirming_ = false;
        fine_tune_ = false;
        fine_step_ = 0;
        align_holding_ = false;
        tracking_lost_ = true;
        lost_warn_tenths_ = 0;
        start_time_ = ros::Time::now();
        lost_since_ = start_time_;
        ROS_INFO("Begin aiming tag = %d (center, then 0.45 / 0.40 / 0.30 sweep)", tag_id_);
    }

    int aimUntilDone()
    {
        ros::Duration(1.0).sleep();
        start_time_ = ros::Time::now();
        if (tracking_lost_)
            lost_since_ = start_time_;
        ros::Rate loop_rate(20);
        while (ros::ok() && !done_)
        {
            ros::spinOnce();
            update();
            loop_rate.sleep();
        }
        publishVel(0.0, 0.0);
        ROS_INFO("Shooting finished, exit code = %d", exit_code_);
        return exit_code_;
    }

    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
        if (done_) return;

        bool found = false;
        for (const auto &detection : msg->detections)
        {
            if (detection.id.empty()) continue;
            if (detection.id[0] != tag_id_) continue;

            last_x_ = detection.pose.pose.pose.position.x;
            last_z_ = detection.pose.pose.pose.position.z;

            ros::Time t = detection.pose.header.stamp;
            if (!t.isValid()) t = msg->header.stamp;
            if (!t.isValid()) t = ros::Time::now();
            last_stamp_ = t;
            tag_seen_ = true;
            tracking_lost_ = false;
            lost_warn_tenths_ = 0;
            found = true;
            break;
        }

        if (!found)
        {
            ros::Time t = msg->header.stamp;
            if (!t.isValid()) t = ros::Time::now();
            if (!tracking_lost_)
            {
                tracking_lost_ = true;
                lost_since_ = t;
            }
        }
    }

    void update()
    {
        if (done_) return;

        ros::Time now = ros::Time::now();

        const double lost_dur = tracking_lost_ ? (now - lost_since_).toSec() : 0.0;
        if (reportLostAndMaybeKnockdown(lost_dur))
            return;

        if ((now - start_time_).toSec() > aim_timeout)
        {
            ROS_INFO("Tag %d still standing after %.0fs aim (align+fine), next target",
                     tag_id_, aim_timeout);
            finish(1);
            return;
        }

        if (lost_dur > stale_timeout)
        {
            publishVel(0.0, 0.0);
            confirming_ = false;
            align_holding_ = false;
            return;
        }

        if (last_z_ <= 0.05)
        {
            publishVel(0.0, 0.0);
            confirming_ = false;
            align_holding_ = false;
            return;
        }

        const double yaw_err = std::atan2(last_x_, last_z_) - yaw_offset_;
        const double abs_err = std::fabs(yaw_err);

        if (!fine_tune_)
        {
            updateAlign(now, yaw_err, abs_err);
            return;
        }
        updateFine(now, yaw_err, abs_err);
    }

    void updateAlign(const ros::Time& now, double yaw_err, double abs_err)
    {
        if (abs_err < align_tolerance)
        {
            publishVel(0.0, 0.0);
            if (!align_holding_)
            {
                align_holding_ = true;
                align_hold_start_ = now;
                ROS_INFO("Tag %d on FOV centerline (err=%.2f deg), holding...",
                         tag_id_, yaw_err * 180.0 / M_PI);
            }
            else if ((now - align_hold_start_).toSec() >= align_hold)
            {
                fine_tune_ = true;
                fine_step_ = 0;
                confirming_ = false;
                ROS_INFO("Tag %d at FOV center, sweep ±0.45 then ±0.40 then ±0.30", tag_id_);
            }
            return;
        }

        align_holding_ = false;
        rotateToYaw(yaw_err);
    }

    void updateFine(const ros::Time& now, double yaw_err, double abs_err)
    {
        (void)abs_err;
        const double setpoint = sweepSetpoint();
        const double track_err = yaw_err - setpoint;

        if (fine_step_ >= sweep_count_ - 1)
        {
            if (std::fabs(track_err) < sweep_reach_)
            {
                publishVel(0.0, 0.0);
                ROS_INFO_THROTTLE(2.0, "Tag %d sweep done, hold center (laser on)", tag_id_);
            }
            else
            {
                rotateFineFixed(track_err);
            }
            return;
        }

        if (std::fabs(track_err) < sweep_reach_)
        {
            publishVel(0.0, 0.0);
            if (!confirming_)
            {
                confirming_ = true;
                confirm_start_ = now;
                ROS_INFO("Tag %d reached sweep %.2f deg (err=%.2f)",
                         tag_id_, setpoint * 180.0 / M_PI, yaw_err * 180.0 / M_PI);
            }
            else if ((now - confirm_start_).toSec() >= band_hold)
            {
                ++fine_step_;
                confirming_ = false;
                ROS_INFO("Tag %d next sweep target %.2f deg",
                         tag_id_, sweepSetpoint() * 180.0 / M_PI);
            }
            return;
        }

        confirming_ = false;
        rotateFineFixed(track_err);
    }

    double sweepSetpoint() const
    {
        static const double kDeg[13] = {
            0.0, 0.45, 0.0, -0.45,
            0.0, 0.40, 0.0, -0.40,
            0.0, 0.30, 0.0, -0.30,
            0.0
        };
        const int i = std::max(0, std::min(fine_step_, sweep_count_ - 1));
        return kDeg[i] * M_PI / 180.0;
    }

    void rotateToYaw(double yaw_err)
    {
        if (yaw_err == 0.0)
        {
            publishVel(0.0, 0.0);
            return;
        }

        double cmd = yaw_kp_ * yaw_err;
        if (cmd > yaw_omega_max_align_) cmd = yaw_omega_max_align_;
        if (cmd < -yaw_omega_max_align_) cmd = -yaw_omega_max_align_;
        if (std::fabs(cmd) < yaw_omega_min_)
            cmd = (cmd >= 0.0) ? yaw_omega_min_ : -yaw_omega_min_;
        publishVel(0.0, -cmd);
    }

    void rotateFineFixed(double track_err)
    {
        if (track_err == 0.0)
        {
            publishVel(0.0, 0.0);
            return;
        }
        const double cmd = (track_err > 0.0) ? yaw_omega_fine_ : -yaw_omega_fine_;
        publishVel(0.0, -cmd);
    }

private:
    // -----------------------------------------------------------------------
    // ★ 瞄准可调参数（一般不用改，微调瞄准手感时改这里）
    // -----------------------------------------------------------------------
    const double align_tolerance = 0.10;  // 对中容差（约 5.73°）★
    const double align_hold = 0.3;        // 对中后停留时长 s ★
    const double band_hold = 0.5;         // 每个扫描点停留时长 s ★
    const double sweep_reach_ = 0.5 * M_PI / 180.0;  // 扫描点容差 0.5° ★
    static const int sweep_count_ = 13;   // 扫描序列长度（0/±0.45/±0.40/±0.30/0）★
    const double yaw_kp_ = 1.5;           // 对中 P 比例系数 ★
    const double yaw_omega_max_align_ = 0.08;  // 对中最大角速度（约 4.6°/s）★
    const double yaw_omega_min_ = 0.02;        // 对中最小角速度（越过底盘死区）★
    const double yaw_omega_fine_ = 0.03;       // 微调固定角速度 ★
    const double stale_timeout = 1.0;     // 短暂丢失容忍（未到判击倒）★
    const double lost_timeout = 1.8;      // 丢失满此时长 → 视为靶倒 ★
    const double aim_timeout = 15.0;      // 对中+微调合计上限；码仍在才等到此时长 ★

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber tag_sub_;
    ros::Publisher cmd_vel_pub_;

    int tag_id_ = -1;
    ros::Time start_time_;
    int exit_code_ = 0;
    bool done_ = true;

    double last_x_ = 0.0;
    double last_z_ = 0.0;
    ros::Time last_stamp_;
    bool tag_seen_ = false;
    bool tracking_lost_ = true;
    ros::Time lost_since_;

    double yaw_offset_ = 0.0;
    std::string calib_file_;

    bool confirming_ = false;
    ros::Time confirm_start_;
    bool fine_tune_ = false;
    int fine_step_ = 0;
    bool align_holding_ = false;
    ros::Time align_hold_start_;
    int lost_warn_tenths_ = 0;

    void loadCalibFile()
    {
        std::ifstream in(calib_file_.c_str());
        if (!in)
        {
            ROS_WARN("No calib file (%s), yaw_offset=0 (aim optical axis)", calib_file_.c_str());
            return;
        }
        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);
            std::string key;
            ss >> key;
            if (!key.empty() && key.back() == ':') key.pop_back();
            if (key == "yaw_offset") ss >> yaw_offset_;
        }
        ROS_INFO("Loaded yaw_offset=%.4f rad (%.3f deg)",
                 yaw_offset_, yaw_offset_ * 180.0 / M_PI);
    }

    void publishVel(double linear_x, double angular_z)
    {
        angular_z = std::max(-0.25, std::min(0.25, angular_z));
        linear_x  = std::max(-0.3,  std::min(0.3,  linear_x));
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_x;
        cmd_vel.angular.z = angular_z;
        cmd_vel_pub_.publish(cmd_vel);
    }

    bool reportLostAndMaybeKnockdown(double lost_dur)
    {
        if (lost_dur < 0.1)
            return false;

        const int max_tenths = static_cast<int>(lost_timeout * 10.0 + 0.5);
        const int tenths = std::min(max_tenths, static_cast<int>(std::floor(lost_dur * 10.0 + 1e-6)));
        const char* phase = fine_tune_ ? "fine tune" : "center align";
        while (lost_warn_tenths_ < tenths)
        {
            ++lost_warn_tenths_;
            ROS_WARN("Tag %d 丢失 %.1f/%.1f s（%s）",
                     tag_id_, lost_warn_tenths_ * 0.1, lost_timeout, phase);
        }

        if (lost_dur >= lost_timeout)
        {
            ROS_INFO("Tag %d lost %.1f s during %s, treat as knocked down",
                     tag_id_, lost_dur, phase);
            finish(0);
            return true;
        }
        return false;
    }

    void finish(int code)
    {
        publishVel(0.0, 0.0);
        exit_code_ = code;
        done_ = true;
    }
};


// ---------------------------------------------------------------------------
// 激光开关（/shoot = 开，/close = 关）。串口 0xA3 / 0xA0，见 shoot_service.cpp
// 与 shoot_robot2.cpp 一致：开场 setLaser(true) 全程常开，结束才 setLaser(false)
// ---------------------------------------------------------------------------
void setLaser(bool on)
{
    static ros::ServiceClient on_cli;
    static ros::ServiceClient off_cli;
    static bool inited = false;
    if (!inited)
    {
        ros::NodeHandle nh;
        on_cli = nh.serviceClient<std_srvs::Empty>("/shoot");
        off_cli = nh.serviceClient<std_srvs::Empty>("/close");
        inited = true;
    }
    std_srvs::Empty empty;
    if (on)
    {
        on_cli.waitForExistence(ros::Duration(5.0));
        if (on_cli.call(empty)) ROS_INFO("Laser ON");
        else ROS_ERROR("Laser ON failed (/shoot)");
    }
    else
    {
        off_cli.waitForExistence(ros::Duration(2.0));
        if (off_cli.call(empty)) ROS_INFO("Laser OFF");
        else ROS_ERROR("Laser OFF failed (/close)");
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "shoot_robot3_node");
    ros::NodeHandle nh;

    // 基地标靶 ID：直接取顶部 BASE_TAG_ID（排位赛=3；淘汰赛打黄方基地时改成 2）
    int enemy_base_tag_id = BASE_TAG_ID;
    ROS_INFO("enemy_base_tag_id=%d", enemy_base_tag_id);

    // 与 shoot_robot2.cpp 一致：激光全程常开
    setLaser(true);

    AprilTagController aimer;
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    // 【新增】tf 监听器：直冲/闭环转向取当前位姿用
    tf::TransformListener listener;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // ======================================================================
    // 阶段一：依次击倒普通靶（ID=1），拆对方基地护甲①②③④
    // ======================================================================

    // ----- 靶1：不导航，直接从原点向右转 FIRST_TARGET_TURN_DEG 度后开打 -----
    // (与 shoot_robot2.cpp 完全一致)
    ROS_INFO("===== Normal target 1 (tag=1, shoot from origin) =====");
    rotate(cmd_vel_pub, -1.0, FIRST_TARGET_TURN_DEG);   // 向右(顺时针)转
    aimer.beginTarget(1);
    aimer.aimUntilDone();

    // ----- 靶2~4：直冲固定点(不避障) → 依次 右45°/左100°/左100° 打靶 -----
    // 【shoot_robot3 新逻辑】打完靶1 后从当前位置直线直冲该点,途中不避障、直接撞过去
    ROS_INFO("===== Rush straight to (%.3f, %.3f) ignoring obstacles =====",
             RUSH_TARGET_X, RUSH_TARGET_Y);
    goStraightNoAvoid(cmd_vel_pub, listener, RUSH_TARGET_X, RUSH_TARGET_Y,
                      RUSH_SPEED, RUSH_TIMEOUT);

    // 到点后先转到到达朝向（绝对 yaw = RUSH_ARRIVE_YAW）
    rotateToYawAbs(cmd_vel_pub, listener, RUSH_ARRIVE_YAW);

    // 靶2：向右转头 45°（相对到达朝向 -45°）
    ROS_INFO("===== Normal target 2: turn right 45 deg, shoot =====");
    rotateToYawAbs(cmd_vel_pub, listener,
                   RUSH_ARRIVE_YAW + TURN2_DEG * DEG2RAD);
    aimer.beginTarget(1);
    aimer.aimUntilDone();

    // 靶3：再向左偏转 100°（相对到达朝向 -45°+100° = +55°）
    ROS_INFO("===== Normal target 3: turn left 100 deg, shoot =====");
    rotateToYawAbs(cmd_vel_pub, listener,
                   RUSH_ARRIVE_YAW + TURN2_DEG * DEG2RAD + TURN3_DEG * DEG2RAD);
    aimer.beginTarget(1);
    aimer.aimUntilDone();

    // 靶4：再向左偏转 100°（相对到达朝向 -45°+100°+100° = +155°）
    ROS_INFO("===== Normal target 4: turn left 100 deg, shoot =====");
    rotateToYawAbs(cmd_vel_pub, listener,
                   RUSH_ARRIVE_YAW + TURN2_DEG * DEG2RAD + TURN3_DEG * DEG2RAD + TURN4_DEG * DEG2RAD);
    aimer.beginTarget(1);
    aimer.aimUntilDone();

    // ======================================================================
    // 阶段二：击倒对方基地标靶（ID 由身份决定）—— 与 shoot_robot2.cpp 完全一致
    // ======================================================================
    if (SHOOT_BASE && ros::ok())
    {
        ROS_INFO("===== Enemy base target (tag=%d) =====", enemy_base_tag_id);
        Move2goal(ac, base_target.x, base_target.y, base_target.yaw);
        // 到达基地前置点后，再前进 BASE_FORWARD_DIST 米靠近基地标靶
        moveForDuration(cmd_vel_pub, BASE_FORWARD_SPEED, 0.0, BASE_FORWARD_DIST / BASE_FORWARD_SPEED);
        aimer.beginTarget(enemy_base_tag_id);
        aimer.aimUntilDone();
    }

    setLaser(false);
    ROS_INFO("All done");
    return 0;
}
