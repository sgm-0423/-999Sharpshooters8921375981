// ============================================================================
// promax_shoot_sequence.cpp —— 自研打靶流程节点（promax 版）
//
// ============================================================================
// 【2026 新方案：9 靶闭环 + 回程点】
//   不再用"转90°+前进2秒"的开环推算（误差累积 → 靶子出画面）：
//   1. 9 个靶位前置点全部用 tf_echo 实测，填进 targets[] 表；
//   2. 每个靶：Move2goal 闭环导航到位（位置精确、yaw 正对靶面、自动避障）→ 打靶；
//   3. 打靶产生的车身漂移，到达下一个靶时被 move_base 重新对准，误差不累积；
//   4. 全部打完，Move2goal 回到回程辅助点（起始位置附近）。
//
// 坐标测量方法（每个点一次）：
//   遥控机器人到靶位正前方约 0.5m、相机正对靶面，然后：
//     rosrun shoot_robot tf_echo_node
//   记录日志里的 x, y, yaw，填入下方 targets[] / home。
//
// 打靶节点（startShootingProgram）：
//   直接 fork 启动 pro_apriltag_detect_node 可执行文件（绕过 roslaunch 冷启动），
//   传 _tag:=X，打完自己退出；识别不到则 8 秒超时放弃，继续下一个靶。
// ============================================================================

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <cstdio>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// --- 到达指定航点（move_base 闭环导航）---
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

// ============================================================================
// 【新方案点位表】所有坐标用 tf_echo 实测后填入
// ============================================================================

// 靶位前置点：走到该点后相机正对靶面（距离靶约 0.5m），
// yaw 为 map 坐标系下"正对靶面"的绝对朝向（弧度）。
struct Waypoint
{
    double x;
    double y;
    double yaw;
};

// 9 个靶位（顺序 = 比赛路线顺序）
Waypoint targets[9] = {
    // ==== 改动 2026-08-19：9 个靶位全部用 tf_echo 重新实测 ====
    { 0.737119, 1.688215, -2.794421 },   // 靶1  实测 2026-08-19（xy 更新：0.836166→0.737119, 1.425910→1.688215）
    { 0.808333, 1.559432, 1.266753 },   // 靶2  实测 2026-08-19
    { 0.844113, 2.444746, 2.941902 },   // 靶3  实测 2026-08-19（更新：0.751088→0.844113, 2.306890→2.444746, 2.769863→2.941902）
    { 1.623315, 0.944502, 0.108267 },   // 靶4  [修改] 2026-08-20 tf_echo 重测（原 2.143109, 0.992947, 0.339523）
    { 1.710640, 0.874454, -1.850096 },   // 靶5  [修改] 2026-08-20 与靶6顺序互换，此处用原靶6坐标
    { 1.621460, 0.114269, -0.329898 },   // 靶6  [修改] 2026-08-20 tf_echo 重测（原 2.412625, 1.073127, -1.371876；与靶5顺序互换）
    { 1.734985, 1.603428, 1.771142 },   // 靶7  [修改] 2026-08-20 tf_echo 重测（原 1.685180, 1.497509, 1.954459）
    { 1.685180, 1.497509, -0.259734 },   // 靶8  实测 2026-08-19
    { 2.242642, 1.557939, 1.270612 },   // 靶9  [修改] 2026-08-20 tf_echo 重测（原 2.385051, 1.571942, 1.307579）
};

// ============================================================================

// --- 打靶：直接 fork 启动打靶节点可执行文件（绕过 roslaunch，省掉每次 ~10 秒冷启动）---
// tag_id 要打的靶；timeout 等节点退出的最大秒数。打靶节点打完自己退出，
// 进程消失，不常驻后台，导航期间不会干扰 move_base，也不会被其他靶子干扰。
bool startShootingProgram(int tag_id, double timeout)
{
    // 打靶节点可执行文件绝对路径。
    // 如果路径不对，在机器人上执行：find /home/bcsh/shoot_robot -name "pro_apriltag_detect_node" -type f
    static const std::string detect_node =
        "/home/bcsh/shoot_robot/devel/lib/shoot_robot/pro_apriltag_detect_node";

    char tag_arg[32];
    snprintf(tag_arg, sizeof(tag_arg), "_tag:=%d", tag_id);

    // move_base 刚"到达"（容差内）可能还有残余运动，等它完全停稳再打靶，避免震荡
    ros::Duration(2.0).sleep();

    pid_t pid = fork();
    if (pid == 0)
    {
        setpgid(0, 0);
        execlp(detect_node.c_str(), detect_node.c_str(), tag_arg, (char*)NULL);
        ROS_ERROR("Failed to exec %s: %s", detect_node.c_str(), strerror(errno));
        _exit(127);
    }
    else if (pid < 0)
    {
        ROS_ERROR("fork() failed: %s", strerror(errno));
        return false;
    }

    ROS_INFO("Trigger shooting tag = %d (timeout %.1f s)", tag_id, timeout);

    // 等打靶节点退出，最多 timeout 秒；超时强杀整个进程组
    int status = 0;
    bool reaped = false;
    ros::Time t0 = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - t0).toSec() < timeout)
    {
        pid_t r = waitpid(pid, &status, WNOHANG);
        if (r == pid) { reaped = true; break; }
        if (r == -1)  { ROS_WARN("waitpid failed: %s", strerror(errno)); reaped = true; break; }
        ros::Duration(0.1).sleep();
    }

    if (reaped)
    {
        if (WIFEXITED(status))
            ROS_INFO("Shooting program finished, exit code = %d", WEXITSTATUS(status));
        else if (WIFSIGNALED(status))
            ROS_WARN("Shooting program terminated by signal %d", WTERMSIG(status));
        else
            ROS_WARN("Shooting program exited abnormally");
    }
    else
    {
        ROS_WARN("Shooting timeout after %.1f s, killing node...", timeout);
        kill(-pid, SIGINT);
        ros::Duration(0.5).sleep();
        kill(-pid, SIGKILL);
        waitpid(pid, &status, 0);
        ROS_WARN("Shooting node killed (timeout), continue to next waypoint");
    }

    // 安全网：补发零速度，防止残留 cmd_vel 让小车继续跑
    static ros::NodeHandle nh;
    static ros::Publisher stop_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    geometry_msgs::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    stop_pub.publish(stop);

    return true;
}

// --- 开环速度控制（用于回程前进 8 秒 & 各靶打完后的原地旋转）---
void moveForDuration(ros::Publisher& cmd_vel_pub, double linear_x, double angular_z, double duration)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_x;
    cmd_vel.angular.z = angular_z;

    ros::Rate rate(10);                 // 10Hz
    int total = (int)(duration * 10);   // 总发布次数 = 时长 × 频率
    for (int i = 0; i < total && ros::ok(); i++)
    {
        cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }

    // 停车
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    cmd_vel_pub.publish(cmd_vel);
}

// --- 原地旋转指定角度（正=逆时针，负=顺时针；角速度 ±1.0 rad/s）---
// angle_deg 单位是度，内部换算成弧度：时长 = 弧度 / 1.0
// 改动 2026-08-19：各靶打完后按需原地旋转指定角度对准下一靶路线
void rotate(ros::Publisher& cmd_vel_pub, double angular_z, double angle_deg)
{
    double angle_rad = angle_deg * 0.0174533;   // 度 → 弧度
    moveForDuration(cmd_vel_pub, 0.0, angular_z, angle_rad / 1.0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "promax_shoot_sequence_node");
    ros::NodeHandle nh;

    // 等打靶完成的最大秒数（视觉节点内部瞄准超时 8 秒，这里留余量）
    double shoot_timeout = 10.0;

    // move_base 客户端
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    // cmd_vel 发布器（靶5 打完旋转 + 回程前进共用）
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // ============================================================================
    // 【新方案】依次走到每个靶位前置点打靶：
    //   move_base 闭环导航：位置精确、yaw 正对靶面、自动避障；
    //   打靶后车身漂移由下一个靶的导航重新对准，误差不累积。
    // ============================================================================
    for (int i = 0; i < 9; i++)
    {
        // [修改] 2026-08-20：取消跳过靶5；靶4~6 顺序调整（靶5↔靶6 互换，见 targets[] 表）
        ROS_INFO("===== Target %d =====", i + 1);
        Move2goal(ac, targets[i].x, targets[i].y, targets[i].yaw);

        // 打靶（tag id 按实际靶改：startShootingProgram 第一个参数）
        startShootingProgram(1, shoot_timeout);

        // 改动 2026-08-19：靶3 打完 → 逆时针原地旋转 → 暂停 1s → 再前往下一节点
        // [修改] 2026-08-20：旋转角 90° → 120°
        if (i == 2)
        {
            ROS_INFO("Target 3 finished, rotate counter-clockwise 120 deg");
            ros::Duration(2.0).sleep();   // 改动 2026-08-20：旋转动作前停顿 2s
            rotate(cmd_vel_pub, 1.0, 120);
            ros::Duration(1.0).sleep();
        }

        // 改动 2026-08-19：靶4 打完 → 顺时针原地旋转 → 暂停 1s → 再前往下一节点
        // [修改] 2026-08-20：旋转角 90° → 100°；并删除原「靶4 打靶前旋转 90°」块
        if (i == 3)
        {
            ROS_INFO("Target 4 finished, rotate clockwise 100 deg");
            ros::Duration(2.0).sleep();   // 改动 2026-08-20：旋转动作前停顿 2s
            rotate(cmd_vel_pub, -1.0, 100);
            ros::Duration(1.0).sleep();
        }

        // 改动 2026-08-19：靶6 打完 → 逆时针原地旋转 → 暂停 1s → 再前往下一节点
        // [修改] 2026-08-20：旋转角 220° → 100°（靶5↔靶6 互换后，i==5 现为原靶5 的点）
        if (i == 5)
        {
            ROS_INFO("Target 6 finished, rotate counter-clockwise 100 deg");
            ros::Duration(2.0).sleep();   // 改动 2026-08-20：旋转动作前停顿 2s
            rotate(cmd_vel_pub, 1.0, 100);
            ros::Duration(1.0).sleep();
        }

        // 改动 2026-08-19：靶9 打完 → 逆时针原地旋转 90° → 暂停 1s → 再前往下一节点
        if (i == 8)
        {
            ROS_INFO("Target 9 finished, rotate counter-clockwise 90 deg");
            ros::Duration(2.0).sleep();   // 改动 2026-08-20：旋转动作前停顿 2s
            rotate(cmd_vel_pub, 1.0, 90);
            ros::Duration(1.0).sleep();
        }

        ROS_INFO("finish");
        ros::Duration(1.0).sleep();
    }

    // ==== 改动 2026-08-19：回程改为"先闭环导航到回程节点，再前进 8 秒" ====
    // 回程节点坐标直接写死（先全零，坐标待调）；到达后前进 8 秒（0.2 m/s）
    ROS_INFO("===== Back to home node =====");
    Move2goal(ac, 1.236615, 1.210887, -2.278829);

    moveForDuration(cmd_vel_pub, 0.2, 0.0, 8.0);

    ROS_INFO("All done");

    return 0;
}



