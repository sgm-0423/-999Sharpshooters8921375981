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
#include <deque>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// shoot3.cpp —— 主任务：导航到 5 个定航点 + 打靶对齐
//
// 本版把打靶部分整体换成 tag3.cpp 的实现（去掉距离对齐，只原地摆头）。
//
// ---------------------------------------------------------------------------
// 三个模块共用 /cmd_vel，必须避免互相抢话题（否则车原地抖、摆不到位）
//   导航：move_base  → /cmd_vel
//   瞄准：本进程      → /cmd_vel
//   apriltag_detect_node 也会发 /cmd_vel（未识别到靶时发 0,0）——别同时跑它
//
//   三条保证不冲突的措施：
//   1. 串行 + 交接：Move2goal 返回后先 cancelGoal 并连发 0 速度 0.3s
//      （handoverToAim），把 move_base 最后一条速度冲掉，再开始瞄准。
//      取消那条路径（30s 超时）尤其重要：cancelGoal 之后 move_base 还会
//      再发一段速度，不交接就会和瞄准的角速度交替下发、互相抵消。
//   2. 瞄准期间不阻塞回调：aimUntilDone 的预热和主循环都走 spinOnce，
//      回调里只有赋值、没有 sleep/循环，不会把识别堵死。
//   3. 运行期监视：订阅 /cmd_vel 回显，发现收到的速度不是本节点刚发的，
//      即判定有外部发布者在抢话题并告警（只看不动，不影响控制）。
//
// ---------------------------------------------------------------------------
// 打靶对齐（同 tag3.cpp）：
//   先转到竖直中缝，再左右摆动扫描（幅度见 ~sweep_deg），全程只发角速度。
//
//   底盘约束（实测）：zoo_driver 只是把 angular.z 转发下位机，软件层无死区，
//   死区在电机静摩擦上。W2C 两轮差速轮距 0.216m：0.25 rad/s(轮速 27mm/s)
//   转不动，0.30 rad/s 实测最低可动，1.00 rad/s 键盘手感。
//   → 最小一档就是 0.3 rad/s = 50Hz 下每拍 0.34°，车无法"慢到停准"。
//
//   两条应对：
//   a. 到位判定用"落在容差内 或 已越过目标"（reachedTarget）。只认容差窗口
//      的话，车必然冲过窗口再反向追，保持计时被反复清零，扫描永远推进不下去。
//   b. 盲走预算限速（blindCap）：按"距上次识别过了多久"自动限速，使
//      角速度 × 盲走时长 ≤ ~max_blind_deg。识别 30Hz 时等于不限制，
//      识别 1Hz 时自动降成小步走，避免长时间盲走甩过头。
//
// 用法：
//   rosrun shoot_robot shoot3_node                 # 跑完整任务（导航 + 打靶）
//   rosrun shoot_robot shoot3_node _sweep_deg:=2.0,1.5,1.0
//
// 调参（全部可用 _参数:=值 覆盖）：
//   _yaw_omega_min/_yaw_omega_max/_yaw_omega_fine/_omega_limit/_max_blind_deg
//   _align_tolerance/_sweep_reach_deg/_sweep_deg/_aim_timeout/_loop_hz
//
// 单点调试（不导航、不起 move_base）用 tag3_node，它带 _spin_test 底盘自检。
// ============================================================================

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Waypoint
{
    double x;
    double y;
    double yaw;
    int tag;   // 该点要打的 tag 编号（与 test.cpp 一致）
};

// 9 个定航点在 map 坐标系下的坐标与朝向（tf_echo map base_link 实测）
// 注意：shoot2.cpp 和仓库根目录的 test.cpp 里也各有一份同样的表，
// 改这里之后那两处不会自动跟着变，需要同步（或改用同一份参数文件）。
// 站点位置以本表为准，其余两份是旧版本。
Waypoint targets[] = {
    { 0.917713, -0.616153, -1.245308, 1 },   // 靶1（tag=1）  0.795064, -0.340489, -1.423348
    { 0.981674,  1.024670,  1.253492, 1 },   // 靶2（tag=1）  0.619589,  0.929203,  1.291642
    { 0.637313,  1.519877,  2.968472, 1 },   // 靶3（tag=1）  0.558637,  1.639529,  2.911886
    { 0.062483,  1.190061, -1.816754, 1 },   // 靶4（tag=1）（tf_echo map base_link 实测） -0.227088, 1.601632, -1.742827
    { 1.670428,  0.006250, -1.860557, 1 },   // 靶5（tag=1）（新增）
    { 1.607453, -0.764071, -0.390011, 1 },   // 靶6（tag=1）（新增）
    { 2.354074, -0.908431,  1.258401, 1 },   // 靶7（tag=1）（新增） 备选：2.423173, -0.737032,  1.338811
    { 1.653173,  1.058574,  2.040054, 1 },   // 靶8（tag=1）（新增）
    { 1.656717,  1.067206,  0.523966, 2 },   // 靶9 = 原靶5（黄方基地）  备选：3号护甲处（2.279096,  0.886176, 1.139608） 1号护甲处（2.498792, 0.899157, 1.370662）
};

// 返回是否导航成功。加 30s 超时，超时取消目标并返回 false，
// 避免 move_base 振荡/恢复失败时 waitForResult() 永久阻塞。
bool Move2goal(MoveBaseClient& ac, double x, double y, double yaw)
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

    const ros::Duration nav_timeout(30.0);
    const bool finished = ac.waitForResult(nav_timeout);
    if (!finished)
    {
        ac.cancelGoal();
        ROS_WARN("MoveBase timeout (%.0fs), goal cancelled", nav_timeout.toSec());
        return false;
    }

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal Reached Successfully!!!");
        return true;
    }

    ROS_WARN("The Goal Planning Failed for some reason");
    return false;
}

// ============================================================================
// AprilTagController —— 打靶对齐（同 tag3.cpp，只发角速度原地摆头）
// ============================================================================
class AprilTagController
{
public:
    AprilTagController() : private_nh_("~")
    {
        tag_sub_ = nh_.subscribe("tag_detections", 1, &AprilTagController::tagCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        // 抢话题监视用：订阅自己发的 /cmd_vel，比对内容判断有没有外部发布者
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 20, &AprilTagController::cmdVelEcho, this);

        private_nh_.param("calib_file", calib_file_, std::string(""));
        if (calib_file_.empty())
        {
            const std::string share = ros::package::getPath("shoot_robot");
            calib_file_ = share.empty()
                ? std::string("optical_axis_calib.yaml")
                : share + "/config/optical_axis_calib.yaml";
        }
        loadCalibFile();

        // ---- 可调参数 ----
        private_nh_.param("yaw_offset", yaw_offset_, yaw_offset_);
        private_nh_.param("yaw_kp", yaw_kp_, yaw_kp_);
        private_nh_.param("yaw_omega_max", yaw_omega_max_align_, yaw_omega_max_align_);
        private_nh_.param("yaw_omega_min", yaw_omega_min_, yaw_omega_min_);
        private_nh_.param("yaw_omega_fine", yaw_omega_fine_, yaw_omega_fine_);
        private_nh_.param("omega_limit", omega_limit_, omega_limit_);
        private_nh_.param("align_tolerance", align_tolerance_, align_tolerance_);
        private_nh_.param("align_hold", align_hold_, align_hold_);
        private_nh_.param("band_hold", band_hold_, band_hold_);
        private_nh_.param("sweep_reach_deg", sweep_reach_deg_, sweep_reach_deg_);
        private_nh_.param("creep_linear", creep_linear_, creep_linear_);
        private_nh_.param("stale_timeout", stale_timeout_, stale_timeout_);
        private_nh_.param("lost_timeout", lost_timeout_, lost_timeout_);
        private_nh_.param("aim_timeout", aim_timeout_, aim_timeout_);
        private_nh_.param("loop_hz", loop_hz_, loop_hz_);
        private_nh_.param("max_blind_deg", max_blind_deg_, max_blind_deg_);
        sweep_reach_ = sweep_reach_deg_ * M_PI / 180.0;

        // 扫描幅度（度），逗号分隔；每档展开成 0, +a, 0, -a，最后回到 0。
        // 原值 0.45,0.40,0.30 小于底盘一拍转角，物理上扫不出来；
        // 想恢复原值传 _sweep_deg:=0.45,0.40,0.30 即可。
        std::string sweep_deg_str;
        private_nh_.param("sweep_deg", sweep_deg_str, std::string("1.0,0.8,0.6"));
        buildSweep(sweep_deg_str);

        reportSelfCheck(sweep_deg_str);
    }

    int exitCode() const { return exit_code_; }

    // /tag_detections 上的发布者数量。没有识别节点时必须提前退出：
    // 否则每个靶都会在 1.8s 后"判定为靶倒"，任务一路报成功却什么都没做。
    int detectionPublishers() const { return tag_sub_.getNumPublishers(); }

    // 停住（交接用）：瞄准结束/开始前后发零速度，确保底盘不残留旧指令
    void holdStill()
    {
        geometry_msgs::Twist stop;
        cmd_vel_pub_.publish(stop);
        rememberTx(0.0, 0.0);
    }

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
        foreign_cmd_count_ = 0;
        resetReachTracking();
        start_time_ = ros::Time::now();
        lost_since_ = start_time_;
        last_det_time_ = start_time_;
        ROS_INFO("Begin aiming tag = %d (center, then sweep %s deg)",
                 tag_id_, sweepDegStr().c_str());
    }

    int aimUntilDone()
    {
        // 先空转 1s 让识别出第一帧，期间照常收回调（不能直接 sleep，
        // 否则这一秒收不到任何识别帧）
        ros::Rate warm_rate(loop_hz_);
        const ros::Time warm_end = ros::Time::now() + ros::Duration(1.0);
        while (ros::ok() && ros::Time::now() < warm_end)
        {
            ros::spinOnce();
            warm_rate.sleep();
        }

        start_time_ = ros::Time::now();
        if (tracking_lost_)
            lost_since_ = start_time_;
        if (last_det_time_ == ros::Time(0) || !last_det_time_.isValid())
            last_det_time_ = start_time_;

        ros::Rate loop_rate(loop_hz_);
        while (ros::ok() && !done_)
        {
            ros::spinOnce();
            update();
            loop_rate.sleep();
        }
        holdStill();
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

            // 识别刷新率是控制质量的上限：两次识别之间车只能"盲走"，
            // 把间隔直接打出来，便于判断瓶颈在检测还是在本程序。
            const ros::Time cb_now = ros::Time::now();
            if (last_det_time_.isValid() && last_det_time_ != ros::Time(0))
            {
                const double dt = (cb_now - last_det_time_).toSec();
                ROS_INFO_THROTTLE(1.0, "[DET] tag %d refresh %.3f s (%.1f Hz)",
                                  tag_id_, dt, dt > 1e-3 ? 1.0 / dt : 0.0);
            }
            last_det_time_ = cb_now;

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

    // 抢话题监视：收到的速度如果不是本节点最近发过的值，说明有别的节点
    // （move_base / apriltag_detect_node）在往 /cmd_vel 写。只告警，不干预。
    void cmdVelEcho(const geometry_msgs::Twist::ConstPtr &msg)
    {
        for (size_t i = 0; i < recent_tx_.size(); ++i)
        {
            if (msg->angular.z == recent_tx_[i].angular.z &&
                msg->linear.x  == recent_tx_[i].linear.x)
                return;   // 是自己发的
        }
        ++foreign_cmd_count_;
    }

    void update()
    {
        if (done_) return;

        ros::Time now = ros::Time::now();

        // 检测帧连续无该码满 lost_timeout_ 即视为靶倒（对齐模式下：码丢失，提前结束）。
        const double lost_dur = tracking_lost_ ? (now - lost_since_).toSec() : 0.0;
        if (reportLostAndMaybeKnockdown(lost_dur))
            return;

        if ((now - start_time_).toSec() > aim_timeout_)
        {
            ROS_INFO("Tag %d still standing after %.0fs aim (align+fine), next target",
                     tag_id_, aim_timeout_);
            finish(1);
            return;
        }

        if (foreign_cmd_count_ > 20)
            ROS_WARN_THROTTLE(2.0,
                "/cmd_vel 有外部发布者抢话题（收到 %ld 条非本节点指令）："
                "瞄准会被打断、摆不到位。检查 move_base 是否还在发速度、"
                "apriltag_detect_node 是否在跑。",
                foreign_cmd_count_);

        // 数据过期或深度无效：停住并清掉保持计时（保留 fine_step_/扫描进度，
        // 让识别恢复后能从当前档继续，而不是从头扫）
        if (lost_dur > stale_timeout_ || last_z_ <= 0.05)
        {
            standDown();
            return;
        }

        const double yaw_err = std::atan2(last_x_, last_z_) - yaw_offset_;
        if (!fine_tune_)
            updateAlign(now, yaw_err);
        else
            updateFine(now, yaw_err);
    }

    // ---- 阶段一：对中到竖直中缝 ----
    void updateAlign(const ros::Time& now, double yaw_err)
    {
        if (!reachedTarget(yaw_err, align_tolerance_))
        {
            align_holding_ = false;
            rotateToYaw(yaw_err);
            return;
        }

        holdStill();
        if (!align_holding_)
        {
            align_holding_ = true;
            align_hold_start_ = now;
            ROS_INFO("Tag %d on FOV centerline (err=%.2f deg), holding...",
                     tag_id_, deg(yaw_err));
        }
        else if ((now - align_hold_start_).toSec() >= align_hold_)
        {
            beginSweep();
        }
    }

    // ---- 阶段二：左右摆动扫描 ----
    void updateFine(const ros::Time& now, double yaw_err)
    {
        const double track_err = yaw_err - sweepSetpoint();

        if (!reachedTarget(track_err, sweep_reach_))
        {
            confirming_ = false;
            rotateFineFixed(track_err);
            return;
        }

        holdStill();
        if (!confirming_)
        {
            confirming_ = true;
            confirm_start_ = now;
            ROS_INFO("Tag %d reached sweep %.2f deg (err=%.2f)",
                     tag_id_, deg(sweepSetpoint()), deg(yaw_err));
        }
        else if ((now - confirm_start_).toSec() >= band_hold_)
        {
            advanceSweep();
        }
    }

    void beginSweep()
    {
        fine_tune_ = true;
        fine_step_ = 0;
        confirming_ = false;
        align_holding_ = false;
        resetReachTracking();
        ROS_INFO("Tag %d at FOV center, begin sweep %s deg (%d steps)",
                 tag_id_, sweepDegStr().c_str(), sweepCount());
    }

    void advanceSweep()
    {
        confirming_ = false;
        resetReachTracking();

        if (fine_step_ >= sweepCount() - 1)
        {
            // 最后一档是回到中缝：原地保持，等靶倒或超时
            ROS_INFO_THROTTLE(2.0, "Tag %d sweep done, hold center (laser on)", tag_id_);
            return;
        }

        ++fine_step_;
        ROS_INFO("Tag %d sweep step %d/%d -> %.2f deg",
                 tag_id_, fine_step_ + 1, sweepCount(), deg(sweepSetpoint()));
    }

    double sweepSetpoint() const
    {
        const int i = std::max(0, std::min(fine_step_, sweepCount() - 1));
        return sweep_rad_[static_cast<size_t>(i)];
    }

    // 对中：角速度比例控制（apriltag_detect 驱动原理 angular = Kp*(-err)），
    // 比例项低于底盘最小转速时抬到最小转速，否则底盘根本不动。
    void rotateToYaw(double yaw_err)
    {
        if (yaw_err == 0.0)
        {
            holdStill();
            return;
        }

        double cmd = yaw_kp_ * yaw_err;
        if (cmd > yaw_omega_max_align_) cmd = yaw_omega_max_align_;
        if (cmd < -yaw_omega_max_align_) cmd = -yaw_omega_max_align_;
        if (std::fabs(cmd) < yaw_omega_min_)
            cmd = (cmd >= 0.0) ? yaw_omega_min_ : -yaw_omega_min_;
        publishVel(creep_linear_, -cmd);
    }

    // 扫描：固定角速度（小角度下比比例控制更可预测）
    void rotateFineFixed(double track_err)
    {
        if (track_err == 0.0)
        {
            holdStill();
            return;
        }
        const double cmd = (track_err > 0.0) ? yaw_omega_fine_ : -yaw_omega_fine_;
        publishVel(creep_linear_, -cmd);
    }

private:
    // 扫描序列（rad）：由 ~sweep_deg 展开，形如 0, +a, 0, -a, 0, +b, 0, -b, ..., 0
    std::vector<double> sweep_rad_;
    double loop_hz_ = 50.0;          // 控制频率；每拍转角 = omega/loop_hz

    // 可调参数（构造时用 ~参数 覆盖）
    double align_tolerance_ = 0.015; // rad，0.015 rad ≈ 0.86°（≥ 2 拍）
    double align_hold_ = 0.3;
    double band_hold_ = 0.4;
    double sweep_reach_deg_ = 0.5;   // 扫描到位判定，度（≥ 2 拍）
    double sweep_reach_ = 0.8 * M_PI / 180.0;
    // 角速度已压到贴着底盘死区(实测 0.3)：再小就完全不动，这即是精度上限。
    double yaw_kp_ = 1.5;               // 角速度比例系数；越小越晚触及 max，逼近越柔  原值：3.0
    double yaw_omega_max_align_ = 0.25;  // 对中最大角速度，约 29°/s   原值： 0.5
    double yaw_omega_min_ = 0.3;        // 对中最小转速 = 底盘死区实测值，不能再低
    double yaw_omega_fine_ = 0.3;       // 扫描固定角速度 = 底盘死区实测值
    double omega_limit_ = 1.5;          // 角速度硬限幅
    // 盲走预算：两次识别之间允许车转过多少度，见 blindCap()。
    double max_blind_deg_ = 3.0;
    double creep_linear_ = 0.0;         // 非 0 时转的同时前进（底盘需带速才能转时用）
    double stale_timeout_ = 1.0;
    double lost_timeout_ = 1.8;     // 对中/扫描丢失满此时长 → 视为靶倒
    double aim_timeout_ = 25.0;     // 对中+扫描合计；码仍在画面里才等到此时长

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber tag_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher cmd_vel_pub_;

    int tag_id_ = -1;
    ros::Time start_time_;
    int exit_code_ = 0;
    bool done_ = true;

    double last_x_ = 0.0;
    double last_z_ = 0.0;
    ros::Time last_stamp_;
    ros::Time last_det_time_;   // 上次成功识别到该码的时刻（本地时钟，避开相机时间戳偏移）
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

    // "越过目标"判定用的上一次误差（每个阶段/每一档开始时清零）
    bool step_has_prev_ = false;
    double step_prev_err_ = 0.0;
    double cross_slack_ = 2.5;  // 越过判定的最大幅度（× 容差）

    // 抢话题监视：本节点最近发过的速度（存几拍，容忍回显延迟）
    std::deque<geometry_msgs::Twist> recent_tx_;
    long foreign_cmd_count_ = 0;

    int sweepCount() const { return static_cast<int>(sweep_rad_.size()); }

    double deg(double rad) const { return rad * 180.0 / M_PI; }
    double stepDeg() const { return yaw_omega_min_ / loop_hz_ * 180.0 / M_PI; }

    // 盲走限速：距上次成功识别越久，允许的角速度越小，使
    // "角速度 × 盲走时长" 不超过 max_blind_deg_，即盲走转角有上界。
    //   识别 30Hz(0.033s) → 上限 90°/s，远用不到，等于不限制；
    //   识别 0.7Hz(1.5s)  → 上限 2°/s，车基本停在原地等新识别。
    double blindCap() const
    {
        const double blind_t = (ros::Time::now() - last_det_time_).toSec();
        const double t = std::max(blind_t, 1.0 / loop_hz_);   // 防 0 除
        return (max_blind_deg_ * M_PI / 180.0) / t;
    }

    std::string sweepDegStr() const
    {
        std::ostringstream os;
        bool first = true;
        for (size_t i = 0; i < sweep_rad_.size(); ++i)
        {
            if (sweep_rad_[i] == 0.0) continue;
            if (!first) os << ",";
            first = false;
            os << std::fabs(deg(sweep_rad_[i]));
        }
        return os.str();
    }

    // "0.45,0.40,0.30" -> {0, +0.45, 0, -0.45, 0, +0.40, 0, -0.40, ..., 0}
    void buildSweep(const std::string& deg_csv)
    {
        sweep_rad_.clear();
        sweep_rad_.push_back(0.0);
        std::istringstream ss(deg_csv);
        std::string tok;
        while (std::getline(ss, tok, ','))
        {
            if (tok.empty()) continue;
            double d = 0.0;
            std::istringstream ts(tok);
            if (!(ts >> d) || d == 0.0) continue;
            const double rad = d * M_PI / 180.0;
            sweep_rad_.push_back(rad);
            sweep_rad_.push_back(0.0);
            sweep_rad_.push_back(-rad);
        }
        sweep_rad_.push_back(0.0);
        if (sweep_rad_.size() < 2)
            sweep_rad_.assign(2, 0.0);
    }

    // 到位判定：落在容差内，或已经越过目标（符号翻转）。
    // 车最小转速 0.3rad/s，一拍就走 stepDeg()° ，反向去追必然来回过冲、
    // 保持计时被反复清零；对扫描而言"扫过即到位"，所以越过也算到位。
    // 但越过的幅度要限制（cross_slack × reach）：惯性冲过头很多时不算到位，
    // 否则会把"冲过 5°"误判成"已对中"。
    bool reachedTarget(double err, double reach)
    {
        const bool within = std::fabs(err) <= reach;
        const bool crossed = step_has_prev_ && (err * step_prev_err_ < 0.0) &&
                             (std::fabs(err) <= cross_slack_ * reach);
        step_prev_err_ = err;
        step_has_prev_ = true;
        return within || crossed;
    }

    void resetReachTracking()
    {
        step_has_prev_ = false;
        step_prev_err_ = 0.0;
    }

    // 停住并清掉保持计时（数据过期时用；保留 fine_step_ 与扫描进度）。
    // 同时清掉"上一次误差"：丢帧后重新识别时误差可能直接跳到另一侧，
    // 不清就会误判为"越过目标"而提前推进。
    void standDown()
    {
        holdStill();
        confirming_ = false;
        align_holding_ = false;
        resetReachTracking();
    }

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
                 yaw_offset_, deg(yaw_offset_));
    }

    void rememberTx(double linear_x, double angular_z)
    {
        geometry_msgs::Twist p;
        p.linear.x = linear_x;
        p.angular.z = angular_z;
        recent_tx_.push_back(p);
        while (recent_tx_.size() > 40)   // 40 拍 ≈ 0.8s，足够容忍回显延迟
            recent_tx_.pop_front();
    }

    void publishVel(double linear_x, double angular_z)
    {
        const double cap = std::min(omega_limit_, blindCap());
        angular_z = std::max(-cap, std::min(cap, angular_z));
        linear_x  = std::max(-0.3, std::min(0.3, linear_x));
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_x;
        cmd_vel.angular.z = angular_z;
        cmd_vel_pub_.publish(cmd_vel);
        rememberTx(linear_x, angular_z);

        // 现场排查用：每 0.5s 打印一次真正下发的速度与当前观测量。
        // cap 明显小于 limit 时，说明识别跟不上、正在被盲走预算压速。
        ROS_INFO_THROTTLE(0.5,
            "[CMD] vx=%.3f wz=%.3f (cap=%.3f) | tag=%d x=%.3f z=%.3f err=%.2fdeg %s%s",
            linear_x, angular_z, cap, tag_id_, last_x_, last_z_,
            deg(std::atan2(last_x_, last_z_) - yaw_offset_),
            fine_tune_ ? "fine" : "align",
            tracking_lost_ ? " [LOST]" : "");
    }

    // 启动自检：容差/扫描幅度小于底盘一拍转角就物理上无法达成，直接报警，
    // 免得现场看到的是"扫了半天不推进"却找不到原因。
    void reportSelfCheck(const std::string& sweep_deg_str)
    {
        const double step = stepDeg();
        ROS_INFO("Aim yaw_offset = %.3f deg  (%s)", deg(yaw_offset_), calib_file_.c_str());
        ROS_INFO("Aim speed: kp=%.1f  align[%.3f,%.3f]  fine=%.3f  limit=%.3f  creep=%.3f",
                 yaw_kp_, yaw_omega_min_, yaw_omega_max_align_,
                 yaw_omega_fine_, omega_limit_, creep_linear_);
        ROS_INFO("Aim loop %.0f Hz, sweep %s (deg), tolerance %.2f deg, reach %.2f deg",
                 loop_hz_, sweep_deg_str.c_str(), deg(align_tolerance_), sweep_reach_deg_);
        ROS_INFO("Aim chassis: min %.2f rad/s -> %.2f deg per tick, timeout %.0fs",
                 yaw_omega_min_, step, aim_timeout_);
        ROS_INFO("Aim blind budget %.1f deg: at 0.2s since last detection cap=%.1f deg/s,"
                 " at 1.0s cap=%.1f deg/s",
                 max_blind_deg_, max_blind_deg_ / 0.2, max_blind_deg_ / 1.0);

        if (deg(align_tolerance_) < step)
            ROS_WARN("align_tolerance %.2f deg < one tick %.2f deg: 车无法停在中缝，"
                     "会来回过冲。请调大 _align_tolerance 或降低 _yaw_omega_min。",
                     deg(align_tolerance_), step);
        if (sweep_reach_deg_ < step)
            ROS_WARN("sweep_reach %.2f deg < one tick %.2f deg: 扫描到位判定无法满足。"
                     "请调大 _sweep_reach_deg 或降低 _yaw_omega_min。",
                     sweep_reach_deg_, step);
        for (size_t i = 0; i < sweep_rad_.size(); ++i)
        {
            const double amp = std::fabs(deg(sweep_rad_[i]));
            if (amp > 0.0 && amp < sweep_reach_deg_)
                ROS_WARN("sweep amplitude %.2f deg < sweep_reach %.2f deg: 该档会被判为已到位，"
                         "实际没扫到。请调大 _sweep_deg 或调小 _sweep_reach_deg。",
                         amp, sweep_reach_deg_);
        }
    }

    bool reportLostAndMaybeKnockdown(double lost_dur)
    {
        if (lost_dur < 0.1)
            return false;

        const int max_tenths = static_cast<int>(lost_timeout_ * 10.0 + 0.5);
        const int tenths = std::min(max_tenths, static_cast<int>(std::floor(lost_dur * 10.0 + 1e-6)));
        const char* phase = fine_tune_ ? "fine tune" : "center align";
        while (lost_warn_tenths_ < tenths)
        {
            ++lost_warn_tenths_;
            ROS_WARN("Tag %d LOST %.1f/%.1f s (%s)",
                     tag_id_, lost_warn_tenths_ * 0.1, lost_timeout_, phase);
        }

        if (lost_dur >= lost_timeout_)
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
        holdStill();
        exit_code_ = code;
        done_ = true;
    }
};

// ============================================================================
// 导航 → 瞄准 的交接
//   move_base 与瞄准程序共用 /cmd_vel：如果 move_base 还在发速度就开始瞄准，
//   两边指令会交替下发到 /zoo_driver、互相抵消，表现是车原地抖/摆不动。
//   所以每次导航结束后：取消残留目标 + 连发 0 速度 0.3s 冲掉它的最后一条指令。
// ============================================================================
void handoverToAim(MoveBaseClient& ac, AprilTagController& aimer)
{
    ac.cancelGoal();   // 幂等：目标已成功/已取消时无害
    const ros::Time t0 = ros::Time::now();
    ros::Rate r(20);
    while (ros::ok() && (ros::Time::now() - t0).toSec() < 0.3)
    {
        aimer.holdStill();
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("Handover to aim done (move_base state: %s)", ac.getState().toString().c_str());
}

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
    ros::init(argc, argv, "shoot3_node");
    ros::NodeHandle nh;

    AprilTagController aimer;
    MoveBaseClient ac("move_base", true);

    // move_base 没起来时 waitForServer() 会永久阻塞，加超时并把原因打出来
    ROS_INFO("Waiting for move_base action server...");
    if (!ac.waitForServer(ros::Duration(10.0)))
    {
        ROS_FATAL("move_base action server not available in 10s, exit");
        return 1;
    }

    // 识别节点没起来时也要提前退出，否则每个靶都会在 1.8s 后被误判为"靶倒"
    {
        const ros::Time t0 = ros::Time::now();
        ros::Rate wait_rate(20);
        while (ros::ok() && aimer.detectionPublishers() == 0 &&
               (ros::Time::now() - t0).toSec() < 10.0)
        {
            // 必须 spinOnce：发布者数量是靠 publisherUpdate 回调刷新的，
            // 而那个回调走全局队列，不 spin 就永远读到 0。
            ros::spinOnce();
            ROS_WARN_THROTTLE(2.0, "No publisher on /tag_detections yet, "
                                   "start the apriltag detection node");
            wait_rate.sleep();
        }
        if (aimer.detectionPublishers() == 0)
        {
            ROS_FATAL("/tag_detections has no publisher, exit "
                      "(otherwise every target would be misjudged as knocked down)");
            return 1;
        }
        ROS_INFO("/tag_detections publishers: %d", aimer.detectionPublishers());
    }

    // 闭环到定航点 → 交接 → 进程内瞄准打靶。
    // 激光只在"导航到位后、瞄准期间"打开，导航途中不发射；
    // 导航失败则跳过该点（不开激光、不瞄准）。
    // tag=0 表示仅导航经过、不启动打靶流程（当前表中无此类点）。
    // 航点数按数组实际大小取，避免手写长度和表项数不一致。
    const int n_waypoints = static_cast<int>(sizeof(targets) / sizeof(targets[0]));
    int shot_seq = 0;
    for (int i = 0; i < n_waypoints; i++)
    {
        if (targets[i].tag <= 0)
        {
            ROS_INFO("===== Transit point %d (no shooting) =====", i + 1);
            bool reached = Move2goal(ac, targets[i].x, targets[i].y, targets[i].yaw);
            if (!reached)
            {
                ROS_ERROR("Transit point %d navigation failed", i + 1);
            }
            continue;
        }

        ++shot_seq;
        ROS_INFO("===== Target %d (tag=%d) =====", shot_seq, targets[i].tag);

        bool reached = Move2goal(ac, targets[i].x, targets[i].y, targets[i].yaw);
        if (!reached)
        {
            ROS_ERROR("Target %d navigation failed, skip shooting", shot_seq);
            continue;
        }

        // 关键：先把 /cmd_vel 从 move_base 手里交接过来，再开始瞄准
        handoverToAim(ac, aimer);

        setLaser(true);
        aimer.beginTarget(targets[i].tag);
        aimer.aimUntilDone();
        setLaser(false);
    }

    ROS_INFO("All done");
    return 0;
}
