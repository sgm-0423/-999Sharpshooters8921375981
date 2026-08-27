#include <ros/ros.h>
#include <ros/package.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Empty.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// shoot_robot_base.cpp —— 主任务
//
// 导航逻辑 / 9 靶点位 / 靶间强制开环（障碍区）/ 回程：
//   与 promax2_shoot_sequence.cpp 保持一致，不要改成纯闭环。
//   靶3/4/6/9 打完后的原地旋转、靶3后前进 3s、回程前点再开环前进 4s，
//   都是为了在障碍区走得流畅（move_base 在窄通道容易绕/卡住）。
//
// 打靶与 promax2 的差别：本进程内瞄准画面竖直中缝
//   yaw_err = atan2(x,z) - yaw_offset（optical_axis_calib.yaml），
//   不 fork pro_apriltag_detect_node。
// ============================================================================

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Waypoint
{
    double x;
    double y;
    double yaw;
};

// 靶位前置点：走到该点后相机正对靶面（距离靶约 0.5m），
// yaw 为 map 坐标系下"正对靶面"的绝对朝向（弧度）。
// 与 promax2_shoot_sequence.cpp 点位表完全一致。
Waypoint targets[9] = {
    // ==== 改动 2026-08-19：9 个靶位全部用 tf_echo 重新实测 ====
    { 0.737119, 1.558215, -2.864421 },   // 靶1  实测 2026-08   0.737119, 1.688215, -2.794421
    { 0.808333, 1.559432, 1.266753 },   // 靶2  实测 2026-08-19
    { 0.844113, 2.444746, 3.041902 },   // 靶3  实测 2026-08-19（更新：0.751088→0.844113, 2.306890→2.444746, 2.769863→2.941902）
    { 1.623315, 0.944502, 0.210267 },   // 靶4  [修改] 2026-08-20 tf_echo 重测（原 2.143109, 0.992947, 0.339523）
    { 1.710640, 0.874454, -1.740096 },   // 靶5  [修改] 2026-08-20 与靶6顺序互换，此处用原靶6坐标
    { 1.621460, 0.114269, -0.329898 },   // 靶6  [修改] 2026-08-20 tf_echo 重测（原 2.412625, 1.073127, -1.371876；与靶5顺序互换）
    { 1.734985, 1.603428, 1.771142 },   // 靶7  [修改] 2026-08-20 tf_echo 重测（原 1.685180, 1.497509, 1.954459）
    { 1.685180, 1.497509, -0.229734 },   //  靶8  实测 2026-08-19
    { 2.242642, 1.557939, 1.195612 },   // 靶9  [修改] 2026-08-20 tf_echo 重测（原 2.385051, 1.571942, 1.307579）
};

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

void moveForDuration(ros::Publisher& cmd_vel_pub, double linear_x, double angular_z, double duration)
{
    // 开环速度：障碍区强制平移/旋转，不走 move_base
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

// 原地旋转指定角度（正=逆时针，负=顺时针；角速度 ±1.0 rad/s）
// angle_deg 单位是度，内部换算成弧度：时长 = 弧度 / 1.0
void rotate(ros::Publisher& cmd_vel_pub, double angular_z, double angle_deg)
{
    double angle_rad = angle_deg * 0.0174533;
    moveForDuration(cmd_vel_pub, 0.0, angular_z, angle_rad / 1.0);
}

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

        // 激光常开：对中/扫描时都可能打到。检测帧连续无该码满 1.8s 即视为靶倒。
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
        // 慢速转到竖直中缝附近，不开火
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
            // 扫完停在中缝，激光常开；丢失 1.8s 即下一靶，仍看见才等到对中+微调共 15s
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

    // 对中用 P。微调用固定角速度 ±0.03 rad/s。
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
    const double align_tolerance = 0.10;  // 约 5.73°
    const double align_hold = 0.3;
    const double band_hold = 0.5;
    const double sweep_reach_ = 0.5 * M_PI / 180.0;  // 扫描点容差：0.5°
    static const int sweep_count_ = 13;
    const double yaw_kp_ = 1.5;
    const double yaw_omega_max_align_ = 0.08;  // 对中：约 4.6°/s
    const double yaw_omega_min_ = 0.02;        // 对中最小转速，越过底盘死区
    const double yaw_omega_fine_ = 0.03;       // 微调固定角速度
    const double stale_timeout = 1.0;
    const double lost_timeout = 1.8;     // 对中/微调丢失满此时长 → 视为靶倒
    const double aim_timeout = 15.0;     // 对中+微调合计；码仍在画面里才等到此时长

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
    ros::init(argc, argv, "shoot_robot_base_node");
    ros::NodeHandle nh;

    setLaser(true);

    AprilTagController aimer;
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 与 promax2_shoot_sequence 相同：闭环到靶位前置点 → 打靶 → 障碍区开环过渡
    for (int i = 0; i < 9; i++)
    {
        ROS_INFO("===== Target %d =====", i + 1);
        Move2goal(ac, targets[i].x, targets[i].y, targets[i].yaw);

        aimer.beginTarget(1);
        aimer.aimUntilDone();

        // 靶3 打完：开环逆时针旋转 + 前进 3s（障碍区，保证去靶4 路线流畅）
        if (i == 2)
        {
            ROS_INFO("Target 3 finished, rotate counter-clockwise 120 deg");
            rotate(cmd_vel_pub, 1.0, 120);
            ros::Duration(0.5).sleep();
            moveForDuration(cmd_vel_pub, 0.2, 0.0, 3.0);
        }

        if (i == 3)
        {
            ROS_INFO("Target 4 finished, rotate clockwise 100 deg");
            rotate(cmd_vel_pub, -1.0, 100);
            ros::Duration(0.5).sleep();
        }

        if (i == 5)
        {
            ROS_INFO("Target 6 finished, rotate counter-clockwise 100 deg");
            rotate(cmd_vel_pub, 1.0, 120);
            ros::Duration(0.5).sleep();
        }

        if (i == 8)
        {
            ROS_INFO("Target 9 finished, rotate counter-clockwise 90 deg");
            rotate(cmd_vel_pub, 1.0, 120);
            ros::Duration(0.5).sleep();
        }

        ROS_INFO("finish");
    }

    // 回程：闭环到终点前点，再开环前进 4s 到终点（避免终点附近规划绕路）
    ROS_INFO("===== Back to home node =====");
    Move2goal(ac, 0.286222, 0.766131, -2.137307);
    moveForDuration(cmd_vel_pub, 0.2, 0.0, 4.0);

    setLaser(false);
    ROS_INFO("All done");
    return 0;
}
