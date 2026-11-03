#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <br_robot_interfaces/action/build_stack.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

using BuildStack = br_robot_interfaces::action::BuildStack;
using GoalHandleBuildStack = rclcpp_action::ServerGoalHandle<BuildStack>;

class BRPlanner : public rclcpp::Node {
public:
    BRPlanner() : Node("br_planner"), current_x_(0.0), current_y_(0.0), current_yaw_(0.0) {
        pub_target_ = create_publisher<geometry_msgs::msg::Point>("/br/target_position", 10);
        pub_rotate_ = create_publisher<std_msgs::msg::Float64>("/br/rotate_by", 10);
        pub_gripper_ = create_publisher<std_msgs::msg::Float64>("/br/gripper_cmd", 10);
        pub_lifter_ = create_publisher<std_msgs::msg::Float64>("/br/z_axis_cmd", 10);

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/br/odom", 10, std::bind(&BRPlanner::odom_callback, this, std::placeholders::_1));

        action_server_ = rclcpp_action::create_server<BuildStack>(
            this, "/br/build_stack",
            std::bind(&BRPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BRPlanner::handle_cancel, this, std::placeholders::_1),
            std::bind(&BRPlanner::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "=== BR PLANNER (4-PHASE) AKTIF ===");
    }

private:
    rclcpp_action::Server<BuildStack>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_target_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rotate_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_gripper_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_lifter_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    double current_x_, current_y_, current_yaw_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }

    bool wait_until_arrived(double tx, double ty, double tol=0.15, int timeout_sec=30) {
        rclcpp::Rate rate(10);
        int cnt = 0;
        while (rclcpp::ok()) {
            double dist = std::hypot(tx - current_x_, ty - current_y_);
            if (dist < tol) return true;
            if (cnt++ > timeout_sec * 10) return false;
            rate.sleep();
        }
        return false;
    }

    bool wait_until_rotated(double target_yaw, double tol=0.05, int timeout_sec=15) {
        rclcpp::Rate rate(10);
        int cnt = 0;
        while (rclcpp::ok()) {
            double err = std::atan2(std::sin(target_yaw - current_yaw_), std::cos(target_yaw - current_yaw_));
            if (std::abs(err) < tol) return true;
            if (cnt++ > timeout_sec * 10) return false;
            rate.sleep();
        }
        return false;
    }

    bool rotate_robot(double delta) {
        double target_yaw = std::atan2(std::sin(current_yaw_ + delta),
                                       std::cos(current_yaw_ + delta));
        std_msgs::msg::Float64 rot;
        rot.data = delta;
        pub_rotate_->publish(rot);
        return wait_until_rotated(target_yaw);
    }

    void gripper(double value) {
        std_msgs::msg::Float64 g;
        g.data = value;
        pub_gripper_->publish(g);
    }

    void lifter(double value) {
        std_msgs::msg::Float64 l;
        l.data = value;
        pub_lifter_->publish(l);
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &, std::shared_ptr<const BuildStack::Goal> goal) {
        RCLCPP_INFO(get_logger(), ">>> Menerima Goal mode: %s | phase: %s",
                    goal->mode.c_str(), goal->phase.c_str());
        if (goal->mode != "MOVE" && goal->mode != "PICK") {
            RCLCPP_WARN(get_logger(), "Mode tidak dikenal: %s", goal->mode.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->mode == "MOVE" &&
            goal->phase != "1" && goal->phase != "2" &&
            goal->phase != "3" && goal->phase != "4") {
            RCLCPP_WARN(get_logger(), "Phase tidak dikenal: %s (harus '1' sampai '4')", goal->phase.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleBuildStack>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleBuildStack> goal_handle) {
        std::thread{std::bind(&BRPlanner::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleBuildStack> goal_handle) {
        auto feedback = std::make_shared<BuildStack::Feedback>();
        auto result = std::make_shared<BuildStack::Result>();
        const auto goal = goal_handle->get_goal();

        if (goal->mode == "MOVE") {
            if (goal->phase == "1") execute_move_phase1(goal_handle, feedback, result);
            else if (goal->phase == "2") execute_move_phase2(goal_handle, feedback, result);
            else if (goal->phase == "3") execute_move_phase3(goal_handle, feedback, result);
            else if (goal->phase == "4") execute_move_phase4(goal_handle, feedback, result);
        } else if (goal->mode == "PICK") {
            execute_pick(goal_handle, feedback, result, goal->label, goal->zone_x, goal->zone_y);
        }
    }

    struct Step {
        enum Type { GOTO, ROTATE, GRIPPER_OPEN, GRIPPER_CLOSE, LIFTER } type;
        double a;
        double b;
        const char* desc;
    };

    void run_step(const Step& s,
                  std::shared_ptr<GoalHandleBuildStack> gh,
                  std::shared_ptr<BuildStack::Feedback> fb,
                  std::shared_ptr<BuildStack::Result> rs,
                  int step, int total_steps) {
        if (gh->is_canceling()) {
            rs->success = false;
            rs->message = "Dibatalkan";
            gh->canceled(rs);
            throw std::runtime_error("canceled");
        }

        std::string prefix;
        switch (s.type) {
            case Step::GOTO:           prefix = "Menuju: "; break;
            case Step::ROTATE:         prefix = "Rotasi: "; break;
            case Step::GRIPPER_OPEN:   prefix = "Buka capit: "; break;
            case Step::GRIPPER_CLOSE:  prefix = "Tutup capit: "; break;
            case Step::LIFTER:         prefix = "Lifter: "; break;
        }
        fb->current_state = prefix + s.desc;
        fb->progress = static_cast<double>(step) / total_steps;
        gh->publish_feedback(fb);

        bool ok = true;
        switch (s.type) {
            case Step::GOTO: {
                geometry_msgs::msg::Point msg;
                msg.x = s.a; msg.y = s.b;
                pub_target_->publish(msg);
                ok = wait_until_arrived(s.a, s.b);
                break;
            }
            case Step::ROTATE:
                ok = rotate_robot(s.a);
                break;
            case Step::GRIPPER_OPEN:
                gripper(0.0);
                break;
            case Step::GRIPPER_CLOSE:
                gripper(0.5);
                break;
            case Step::LIFTER:
                lifter(s.a);
                break;
        }

        if (!ok) {
            rs->success = false;
            rs->message = std::string("Gagal: ") + s.desc;
            gh->abort(rs);
            throw std::runtime_error("failed");
        }

        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    void execute_move_phase1(const std::shared_ptr<GoalHandleBuildStack> gh,
                             std::shared_ptr<BuildStack::Feedback> fb,
                             std::shared_ptr<BuildStack::Result> rs) {
        const std::vector<Step> steps = {
            {Step::GOTO,         0.5,     -8.5, "Posisi awal"},
            {Step::GOTO,        -0.8,     -8.5, "Geser kiri"},
            {Step::GOTO,        -0.8,     -3.65, "Maju ke transfer"},
            {Step::GOTO,        -2.0,     -3.65, "Maju sedikit ke area ambil"},
            {Step::ROTATE,      -1.5708,   0.0, "Hadap kanan (90 CW)"}
        };

        try {
            for (size_t i = 0; i < steps.size(); i++) {
                run_step(steps[i], gh, fb, rs, i, (int)steps.size());
            }
        } catch (const std::exception&) { return; }

        fb->current_state = "STANDBY: Komando 1 selesai, menunggu Komando 2";
        fb->progress = 1.0;
        gh->publish_feedback(fb);
        rs->success = true;
        rs->message = "Komando 1 selesai. Kirim Goal Phase 2 untuk lanjut.";
        gh->succeed(rs);
        RCLCPP_INFO(get_logger(), ">>> Phase 1 selesai. Kirim Goal Phase 2.");
    }

    void execute_move_phase2(const std::shared_ptr<GoalHandleBuildStack> gh,
                             std::shared_ptr<BuildStack::Feedback> fb,
                             std::shared_ptr<BuildStack::Result> rs) {
        const std::vector<Step> steps = {
            {Step::GOTO,        -2.5,     -3.65, "Mendekat ke balok"},
            {Step::GRIPPER_OPEN, 0.0,      0.0, "Buka capit"},
            {Step::GRIPPER_CLOSE,0.0,      0.0, "Capit balok (tutup)"},
            {Step::LIFTER,       0.4,      0.0, "Angkat lifter level 1"},
            {Step::GOTO,        -2.0,     -3.65, "Mundur sedikit"},
            {Step::ROTATE,       1.5708,   0.0, "Hadap kiri (90 CCW)"},
            {Step::GOTO,        -2.0,     -6.8, "Menuju titik bawah"},
            {Step::GOTO,        -2.65,    -6.8, "Geser kiri ke titik akhir"},
            {Step::GOTO,        -2.65,    -7.0, "Maju ke area susun"},
            {Step::LIFTER,       0.0,      0.0, "Turunkan lifter"},
            {Step::GRIPPER_OPEN, 0.0,      0.0, "Lepaskan balok"},
            {Step::GOTO,        -2.65,    -6.4, "Kembali ke titik akhir"}
        };

        try {
            for (size_t i = 0; i < steps.size(); i++) {
                run_step(steps[i], gh, fb, rs, i, (int)steps.size());
            }
        } catch (const std::exception&) { return; }

        fb->current_state = "Selesai - Semua waypoint tercapai";
        fb->progress = 1.0;
        gh->publish_feedback(fb);
        rs->success = true;
        rs->message = "MOVE sequence selesai.";
        gh->succeed(rs);
        RCLCPP_INFO(get_logger(), ">>> Phase 2 selesai. Susunan komplit.");
    }

    void execute_move_phase3(const std::shared_ptr<GoalHandleBuildStack> gh,
                             std::shared_ptr<BuildStack::Feedback> fb,
                             std::shared_ptr<BuildStack::Result> rs) {
        const std::vector<Step> steps = {
            {Step::GOTO,        -2.2,     -6.4, "Menuju titik awal Phase 3"},
            {Step::GOTO,        -2.2,     -3.67, "Maju ke area ambil"},
            {Step::ROTATE,      -1.5708,   0.0, "Hadap kanan (90 CW)"}
        };

        try {
            for (size_t i = 0; i < steps.size(); i++) {
                run_step(steps[i], gh, fb, rs, i, (int)steps.size());
            }
        } catch (const std::exception&) { return; }

        fb->current_state = "STANDBY: Komando 3 selesai, menunggu Komando 4";
        fb->progress = 1.0;
        gh->publish_feedback(fb);
        rs->success = true;
        rs->message = "Phase 3 selesai. Kirim Goal Phase 4 untuk lanjut.";
        gh->succeed(rs);
        RCLCPP_INFO(get_logger(), ">>> Phase 3 selesai. Kirim Goal Phase 4.");
    }

    void execute_move_phase4(const std::shared_ptr<GoalHandleBuildStack> gh,
                             std::shared_ptr<BuildStack::Feedback> fb,
                             std::shared_ptr<BuildStack::Result> rs) {
        const std::vector<Step> steps = {
            {Step::GOTO,        -2.78,     -3.7, "Mendekat ke balok"},
            {Step::GRIPPER_OPEN, 0.0,      0.0, "Buka capit"},
            {Step::GRIPPER_CLOSE,0.0,      0.0, "Tutup capit"},
            {Step::LIFTER,       0.4,      0.0, "Angkat lifter level 1"},
            {Step::GOTO,        -2.2,     -3.7, "Mundur sedikit"},
            {Step::ROTATE,       1.5708,   0.0, "Hadap kiri (90 CCW)"},
            {Step::GOTO,        -2.2,     -6.8, "Menuju titik bawah"},
            {Step::GOTO,        -2.75,     -6.8, "Geser kiri ke titik akhir"},
            {Step::GOTO,        -2.75,     -7.0, "Maju ke area susun"},
            {Step::LIFTER,       0.4,      0.0, "Turunkan lifter"},
            {Step::GRIPPER_OPEN, 0.0,      0.0, "Lepaskan balok"},
            {Step::GOTO,        -2.75,     -6.4, "Kembali ke titik akhir"},
            {Step::LIFTER,       0.0,      0.0, "Turunkan lifter (akhir)"} // <-- TAMBAHAN
        };

        try {
            for (size_t i = 0; i < steps.size(); i++) {
                run_step(steps[i], gh, fb, rs, i, (int)steps.size());
            }
        } catch (const std::exception&) { return; }

        fb->current_state = "Selesai - Phase 4 tercapai";
        fb->progress = 1.0;
        gh->publish_feedback(fb);
        rs->success = true;
        rs->message = "Phase 4 selesai. Susunan komplit.";
        gh->succeed(rs);
        RCLCPP_INFO(get_logger(), ">>> Phase 4 selesai.");
    }

    void execute_pick(const std::shared_ptr<GoalHandleBuildStack> gh,
                      std::shared_ptr<BuildStack::Feedback> fb,
                      std::shared_ptr<BuildStack::Result> rs,
                      const std::string& label, double zone_x, double zone_y) {
        (void)zone_x; (void)zone_y;
        fb->current_state = "PICK mode tidak diimplementasikan";
        fb->progress = 1.0;
        gh->publish_feedback(fb);
        rs->success = false;
        rs->message = "PICK tidak diimplementasikan. Gunakan mode MOVE.";
        gh->abort(rs);
        RCLCPP_WARN(get_logger(), "PICK mode tidak dipakai. Semua sequence ada di MOVE.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BRPlanner>());
    rclcpp::shutdown();
    return 0;
}