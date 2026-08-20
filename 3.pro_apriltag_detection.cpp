#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <cmath>

// 【打完即退版】由序列节点 fork 直接启动（不经 roslaunch），启动即打靶：
//   1. 读取 ~tag（序列节点用 _tag:=X 传入）指定要打的靶；
//   2. 只匹配该 tag，其他靶子一概忽略；找不到目标时静止等待（不搜索旋转，
//      避免被其他靶干扰、也不会把朝向转丢）；
//   3. 对准后调用 /shoot 服务射击，然后退出（进程消失，绝无后台干扰）；
//   4. 总超时 aim_timeout 秒没对准 → 放弃退出（防卡死）。
// 退出码：0=射击成功，1=瞄准超时，2=射击服务失败。

class AprilTagController
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber tag_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::ServiceClient shoot_client;

    // ==== 改动 2026-08-19：参考注释掉的旧代码，只调朝向幅度（减小摆动）、不控距离 ====
    const double Kp = 1.0;                  // 比例系数（3 -> 1.0，减小朝向摆动/超调幅度）
    const double target_x_tolerance = 0.04;
    // const double z_target_distance = 0.52;   // 不控距离，注释掉
    // const double target_z_tolerance = 0.04;

    const double aim_timeout = 8.0;    // 单次运行最多瞄准秒数

    int tag_id_ = -1;
    ros::Time start_time_;
    int exit_code_ = 0;
    bool done_ = false;

    std_srvs::Empty empty_srv;

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
    }

    int exitCode() const { return exit_code_; }

    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
        if (done_) return;

        // 总超时：放弃（防卡死）
        if ((ros::Time::now() - start_time_).toSec() > aim_timeout)
        {
            ROS_WARN("Aiming timeout for tag %d, give up", tag_id_);
            finish(1);
            return;
        }

        for (const auto &detection : msg->detections)
        {
            if (detection.id.empty()) continue;
            if (detection.id[0] != tag_id_) continue;   // 其他靶子一概忽略

            double current_x = detection.pose.pose.pose.position.x;
            // double current_z = detection.pose.pose.pose.position.z;   // 不控距离，不再使用

            // ==== 改动 2026-08-19：射击条件只判断朝向(x)，去掉距离判断 ====
            if (fabs(current_x) < target_x_tolerance)
            {
                // ==== 对准了：射击！ ====
                ROS_INFO("Tag %d aligned, shooting!", tag_id_);
                bool ok = shoot_client.exists() && shoot_client.call(empty_srv);
                if (ok) { ROS_INFO("Shoot service called OK"); finish(0); }
                else    { ROS_ERROR("Shoot service call FAILED"); finish(2); }
                return;
            }
            else if (fabs(current_x) > target_x_tolerance)
            {
                // 左右偏：转向修正
                publishVel(0.0, Kp * (-current_x));
            }
            // ==== 改动：注释掉距离前进/后退修正，不控距离 ====
            // else if (fabs(current_z - z_target_distance) > target_z_tolerance)
            // {
            //     // 距离偏：前进/后退修正
            //     publishVel(Kp * 0.3 * (current_z - z_target_distance), 0.0);
            // }
            return;
        }

        // 没找到目标：静止等待（发 0 速度，不搜索旋转，避免被其他靶干扰）
        publishVel(0.0, 0.0);
    }

private:
//    void publishVel(double linear_x, double angular_z)
//    {
//        geometry_msgs::Twist cmd_vel;
//        cmd_vel.linear.x = linear_x;
//        cmd_vel.angular.z = angular_z;
//        cmd_vel_pub_.publish(cmd_vel);
//    }
// a//////////////////////////////////////////////////////////////////////////////////////////////
    void publishVel(double linear_x, double angular_z)
    {
        // 限幅，防止过冲/震荡
        if (angular_z > 0.3) angular_z = 0.3;   // ==== 改动：0.5 -> 0.3，限制旋转速度，进一步减小摆动 ====
        if (angular_z < -0.3) angular_z = -0.3;
        if (linear_x > 0.3) linear_x = 0.3;
        if (linear_x < -0.3) linear_x = -0.3;
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_x;
        cmd_vel.angular.z = angular_z;
        cmd_vel_pub_.publish(cmd_vel);
    }
// a//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Node exiting with code %d", controller.exitCode());
    return controller.exitCode();
}
