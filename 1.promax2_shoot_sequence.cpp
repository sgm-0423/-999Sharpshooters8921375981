// ============================================================================
// promax_shoot_sequence.cpp —— 自研打靶流程节点（promax 版）
//
// 流程（先完整执行 pro_shoot_sequence.cpp 的流程，再接着执行新增的 promax 流程）：
//
// —— 第一部分：原 shoot_sequence 流程 ——
//   1. move_base 到达第一个节点
//   2. 停止 1 秒
//   3. 开启打靶程序（第一个靶），打靶结束打印 "finish"，停顿 1 秒
//   4. 逆时针旋转 90 度
//   5. 前进 2 秒
//   6. 开启打靶程序（第二个靶），打靶结束打印 "finish"，停顿 1 秒
//   7. 逆时针旋转 90 度
//   8. 前进 2 秒
//   9. 开启打靶程序（第三个靶），打靶结束打印 "finish"，停顿 1 秒
//  10. 逆时针旋转 90 度
//  11. 前进 2 秒
//  12. 回到指定点 (1.236615, 1.210887, yaw=-2.278829)
//  13. 前进 7 秒后停车
//
// —— 第二部分：pro 流程 ——
//  14. move_base 到达指定点 (1.236615, 1.210887, yaw=-2.278829)
//  15. 顺时针旋转 60 度
//  16. 停止 1 秒
//  17. 前进 2 秒
//  18. 开启打靶程序（第一个靶），打靶结束打印 "finish"
//  19. 逆时针旋转 90 度
//  20. 前进 2 秒
//  21. 开启打靶程序（第二个靶），打靶结束打印 "finish"
//  22. 逆时针旋转 90 度
//  23. 前进 2 秒
//  24. 开启打靶程序（第三个靶），打靶结束打印 "finish"
//  25. 逆时针旋转 90 度
//  26. 回到指定点 (1.236615, 1.210887, yaw=-2.278829)
//  27. 前进 7.5 秒后停车
//
// —— 第三部分：新增 promax 流程 ——
//  28. move_base 到达另一个点（坐标待确认，见代码内 TODO）
//  29. 开启打靶程序（第一个靶），打靶结束打印 "finish"
//  30. 顺时针旋转 90 度
//  31. 前进 1.5 秒
//  32. 开启打靶程序（第二个靶），打靶结束打印 "finish"
//  33. 顺时针旋转 90 度
//  34. 前进 2 秒
//  35. 开启打靶程序（第三个靶），打靶结束打印 "finish"
//  36. 顺时针旋转 90 度
//  37. 停止 1 秒
//  38. 回到指定点 (1.236615, 1.210887, yaw=-2.278829)
//  39. 前进 8 秒后停车
//
// 依赖包：move_base_msgs / actionlib / geometry_msgs / tf2
// 编译：需加入某个 catkin 包的 CMakeLists.txt（本文件暂放在 self_research 目录）
// ============================================================================

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cstdlib>   // system() 拉起 roslaunch
#include <cstring>
#include <cerrno>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <ros/master.h>
#include <dirent.h>   // opendir/readdir 遍历 /proc
#include <fstream>    // 读 /proc/<pid>/stat、cmdline
#include <sstream>    // 解析 stat 字段
#include <cctype>     // isdigit

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

// --- 开启打靶程序：拉起 apriltag 视觉打靶节点（阻塞直到该节点打靶完成退出）---
//void startShootingProgram()
//{
//    system("roslaunch shoot_robot pro_shoot_shoot_tag_1.launch");  //song corrected before
//}
// a/////////////////////////////////////////////////////////////////////////////////////////////////////
// --- 检查：我们这次 fork 出来的 roslaunch 进程组里，是否真的出现了打靶节点进程 ---
// 不用 ros::master 全局列表判断就绪的原因：机器上常驻的 apriltag_ros 一直在发
// tag_detections 话题，全局判断会立刻"就绪"，但实际这次 roslaunch 还在冷启动，
// 导致打靶计时过早开始、还没瞄准就被杀。这里直接扫 /proc 查进程，最可靠。
bool shootingNodeProcessUp(pid_t pgid)
{
    DIR* dir = opendir("/proc");
    if (!dir) return false;
    struct dirent* ent;
    bool up = false;
    while ((ent = readdir(dir)) != NULL)
    {
        if (!isdigit((unsigned char)ent->d_name[0])) continue;
        std::string proc_path = std::string("/proc/") + ent->d_name;

        // /proc/<pid>/stat 第5个字段是进程组号 pgrp；comm 可能含空格/括号，从最后一个 ')' 后解析
        std::ifstream stat_file((proc_path + "/stat").c_str());
        if (!stat_file.is_open()) continue;
        std::string line;
        std::getline(stat_file, line);
        stat_file.close();
        size_t close_paren = line.rfind(')');
        if (close_paren == std::string::npos) continue;
        std::istringstream iss(line.substr(close_paren + 1));
        char state; int ppid, pgrp;
        if (!(iss >> state >> ppid >> pgrp)) continue;
        if (pgrp != pgid) continue;   // 只关心我们这个 roslaunch 进程组

        // 读 cmdline，确认是打靶节点（冷启动期的 rosversion/rospack 子进程不算）
        std::ifstream cmd_file((proc_path + "/cmdline").c_str());
        std::string cmdline((std::istreambuf_iterator<char>(cmd_file)), std::istreambuf_iterator<char>());
        cmd_file.close();
        if (cmdline.find("apriltag_detect") != std::string::npos)
        {
            up = true;
            break;
        }
    }
    closedir(dir);
    return up;
}
// --- 开启打靶程序：拉起 apriltag 视觉打靶节点 ---
// 阶段1：等「真正的打靶节点进程」起来（本机 roslaunch 冷启动慢，给足时间，期间不杀）；
// 阶段2：节点起来后，给打靶/瞄准时间，超时（打倒靶后卡住）才强杀走人。
void startShootingProgram()
{
    const double startup_grace = 20.0;   // 阶段1：等节点进程出现的最大秒数
    const double shoot_timeout = 8.0;    // 阶段2：节点就绪后打靶的最大秒数

    // 零速度安全网发布器（只创建一次）
    static ros::NodeHandle nh;
    static ros::Publisher stop_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    pid_t pid = fork();
    if (pid == 0)
    {
        // 子进程：独立进程组运行 roslaunch，便于整体杀掉
        setpgid(0, 0);
        execlp("roslaunch", "roslaunch", "shoot_robot", "pro_shoot_shoot_tag_1.launch", (char*)NULL);
        ROS_ERROR("Failed to exec roslaunch: %s", strerror(errno));
        _exit(127);
    }
    else if (pid < 0)
    {
        ROS_ERROR("fork() failed: %s", strerror(errno));
        return;
    }

    int status = 0;
    bool reaped = false;
    bool pipeline_up = false;

    // ---------- 阶段1：等打靶节点进程出现（期间不杀） ----------
    ros::Time t0 = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - t0).toSec() < startup_grace)
    {
        pid_t r = waitpid(pid, &status, WNOHANG);
        if (r == pid) { reaped = true; break; }   // 启动期就退出（launch 文件错误等）
        if (r == -1)  { ROS_WARN("waitpid failed: %s", strerror(errno)); reaped = true; break; }

        if (shootingNodeProcessUp(pid)) { ROS_INFO("Shooting node process is up"); pipeline_up = true; break; }
        ros::Duration(0.2).sleep();
    }

    if (!reaped && !pipeline_up)
    {
        // 节点一直没起来（launch 失败或太慢），杀掉，不干等
        ROS_WARN("Shooting node did not start within %.1f s, killing launch...", startup_grace);
        kill(-pid, SIGINT);
        ros::Duration(1.0).sleep();
        kill(-pid, SIGKILL);
        waitpid(pid, &status, 0);
        reaped = true;
    }

    // ---------- 阶段2：节点已起来，给打靶时间 ----------
    if (!reaped)
    {
        t0 = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - t0).toSec() < shoot_timeout)
        {
            pid_t r = waitpid(pid, &status, WNOHANG);
            if (r == pid) { reaped = true; break; }
            if (r == -1)  { ROS_WARN("waitpid failed: %s", strerror(errno)); reaped = true; break; }
            ros::Duration(0.1).sleep();
        }
    }

    if (reaped)
    {
        // 打靶程序自己退出了（打完靶）
        if (WIFEXITED(status))
            ROS_INFO("Shooting program finished, exit code = %d", WEXITSTATUS(status));
        else if (WIFSIGNALED(status))
            ROS_WARN("Shooting program terminated by signal %d", WTERMSIG(status));
        else
            ROS_WARN("Shooting program exited abnormally");
    }
    else
    {
        // 阶段2超时：说明打倒靶后卡住，强杀继续下一航点
        ROS_WARN("Shooting timeout after %.1f s (node up), killing it...", shoot_timeout);
        kill(-pid, SIGINT);
        ros::Duration(1.0).sleep();
        kill(-pid, SIGKILL);
        waitpid(pid, &status, 0);
        ROS_WARN("Shooting program killed (timeout), continue to next waypoint");
    }

    // 安全网：补发零速度，防止残留 cmd_vel 让小车继续跑
    geometry_msgs::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    stop_pub.publish(stop);
}

// a///////////////////////////////////////////////////////////////////

// --- 开环速度控制：按给定线速度/角速度持续运动指定时长（秒）后停车 ---
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "promax_shoot_sequence_node");
    ros::NodeHandle nh;

    // move_base 客户端
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    // /cmd_vel 发布器（开环旋转/前进用）
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // ============================================================================
    // 第一部分：原 shoot_sequence 流程
    // ============================================================================

    // 1. 到达第一个节点
    Move2goal(ac, 0.874185, 2.262532, 1.328710);   // TODO: 改成实际的第一个航点坐标 (x, y, yaw)

    // 2. 停止 1 秒
    ros::Duration(1.0).sleep();

    // 3. 开启打靶程序（第一个靶）
    startShootingProgram();

    // 打靶结束打印 "finish"
    ROS_INFO("finish");
    // 停顿 1 秒
    ros::Duration(1.0).sleep();

    // 4. 逆时针旋转 90 度（z 轴向上，右手定则：正角速度 = 逆时针）
    //    角速度 0.785 rad/s(≈45°/s) × 2s ≈ 90°
    moveForDuration(cmd_vel_pub, 0.0, 0.785, 2.0);

    // 5. 前进 2 秒（线速度 0.2 m/s）
    moveForDuration(cmd_vel_pub, 0.2, 0.0, 2.0);

    // 6. 开启打靶程序（第二个靶）
    startShootingProgram();

    // 打靶结束打印 "finish"
    ROS_INFO("finish");
    // 停顿 1 秒
    ros::Duration(1.0).sleep();

    // 7. 逆时针旋转 90 度
    moveForDuration(cmd_vel_pub, 0.0, 0.785, 2.0);

    // 8. 前进 2 秒
    moveForDuration(cmd_vel_pub, 0.2, 0.0, 2.0);

    // 9. 开启打靶程序（第三个靶）
    startShootingProgram();

    // 打靶结束打印 "finish"
    ROS_INFO("finish");
    // 停顿 1 秒
    ros::Duration(1.0).sleep();

    // 10. 逆时针旋转 90 度
    moveForDuration(cmd_vel_pub, 0.0, 0.785, 2.0);

    // 停顿 1 秒
    ros::Duration(1.0).sleep();

    // ============================================================================
    // 第二部分：pro 流程
    // ============================================================================

    // 14. 到达指定点
    Move2goal(ac, 1.660014, 1.214490, -0.143725);

    // 15. 顺时针旋转 60 度（z 轴向上，右手定则：负角速度 = 顺时针）
    //     角速度 -0.785 × 2s = 90°
    moveForDuration(cmd_vel_pub, 0.0, -0.785, 2.0);

    // 16. 停止 1 秒
    ros::Duration(1.0).sleep();

    // 17. 前进 2 秒（线速度 0.2 m/s）
    moveForDuration(cmd_vel_pub, 0.2, 0.0, 2.0);

    // 18. 开启打靶程序（第一个靶）
    startShootingProgram();

    // 打靶结束打印 "finish"
    ROS_INFO("finish");

    // 19. 逆时针旋转 90 度
    //     角速度 0.785 rad/s(≈45°/s) × 2s ≈ 90°
    moveForDuration(cmd_vel_pub, 0.0, 0.785, 2.0);

    // 20. 前进 2 秒
    moveForDuration(cmd_vel_pub, 0.2, 0.0, 2.0);

    // 21. 开启打靶程序（第二个靶）
    startShootingProgram();

    // 打靶结束打印 "finish"
    ROS_INFO("finish");

    // 22. 逆时针旋转 90 度
    moveForDuration(cmd_vel_pub, 0.0, 0.785, 2.0);

    // 23. 前进 2 秒
    moveForDuration(cmd_vel_pub, 0.2, 0.0, 2.0);

    // 24. 开启打靶程序（第三个靶）
    startShootingProgram();

    // 打靶结束打印 "finish"
    ROS_INFO("finish");

    // 25. 结束后逆时针旋转 90 度
    moveForDuration(cmd_vel_pub, 0.0, 0.785, 2.0);

    //停止1s
    ros::Duration(1.0).sleep();

    // ============================================================================
    // 第三部分：新增 promax 流程
    // ============================================================================

    // 28. 到达「另一个点」
    // 【新增】promax 第三部分：先导航到另一个点（坐标待确认）
    // TODO: 坐标待用户确认，先用占位坐标 (x, y, yaw)
    Move2goal(ac, 1.463914, 1.999223, 1.938471);

    // 29. 开启打靶程序（第一个靶）
    startShootingProgram();

    // 打靶结束打印 "finish"
    ROS_INFO("finish");
    // 停顿 1 秒
    ros::Duration(1.0).sleep();

    // 30. 顺时针旋转 90 度（z 轴向上，右手定则：负角速度 = 顺时针）
    //     角速度 -0.785 rad/s(≈45°/s) × 2s ≈ -90°
    moveForDuration(cmd_vel_pub, 0.0, -0.785, 2.0);

    // 31. 前进 1.5 秒（线速度 0.2 m/s）
    moveForDuration(cmd_vel_pub, 0.2, 0.0, 1.5);

    // 32. 开启打靶程序（第二个靶）
    startShootingProgram();

    // 打靶结束打印 "finish"
    ROS_INFO("finish");
    // 停顿 1 秒
    ros::Duration(1.0).sleep();

    // 33. 顺时针旋转 90 度
    moveForDuration(cmd_vel_pub, 0.0, -0.785, 2.0);

    // 34. 前进 2 秒（线速度 0.2 m/s）
    moveForDuration(cmd_vel_pub, 0.2, 0.0, 2.0);

    // 35. 开启打靶程序（第三个靶）
    startShootingProgram();

    // 打靶结束打印 "finish"
    ROS_INFO("finish");
    // 停顿 1 秒
    ros::Duration(1.0).sleep();

    // 36. 顺时针旋转 90 度
    moveForDuration(cmd_vel_pub, 0.0, -0.785, 2.0);

    // 37. 停止 1 秒
    ros::Duration(1.0).sleep();

    // 38. 回到指定点 (1.236615, 1.210887, yaw=-2.278829)（move_base 闭环导航）
    Move2goal(ac, 1.236615, 1.210887, -2.278829);

    // 39. 前进 8 秒后停车
    moveForDuration(cmd_vel_pub, 0.2, 0.0, 8.0);

    return 0;
}
