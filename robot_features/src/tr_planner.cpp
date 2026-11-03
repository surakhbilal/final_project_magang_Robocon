#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tr_robot_interfaces/action/place_block.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

using PlaceBlock = tr_robot_interfaces::action::PlaceBlock;
using GoalHandlePlaceBlock = rclcpp_action::ServerGoalHandle<PlaceBlock>;

class TRPlanner : public rclcpp::Node {
public:
    TRPlanner() : Node("tr_planner"), current_x_(0.0), current_y_(0.0), current_yaw_(0.0) {
        pub_target_ = create_publisher<geometry_msgs::msg::Point>("/target_position", 10);
        pub_rotate_ = create_publisher<std_msgs::msg::Float64>("/tr/rotate_by", 10);
        pub_gripper_ = create_publisher<std_msgs::msg::Float64>("/tr/gripper_cmd", 10);
        pub_tip_ = create_publisher<std_msgs::msg::Float64>("/tr/gripper_tip_cmd", 10);

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/tr/odom", 10, std::bind(&TRPlanner::odom_callback, this, std::placeholders::_1));

        action_server_ = rclcpp_action::create_server<PlaceBlock>(
            this, "/tr/place_block",
            std::bind(&TRPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TRPlanner::handle_cancel, this, std::placeholders::_1),
            std::bind(&TRPlanner::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "=== TR PLANNER (2-PHASE) AKTIF ===");
    }

private:
    rclcpp_action::Server<PlaceBlock>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_target_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rotate_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_gripper_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_tip_;
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
        pub_tip_->publish(l);
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &, std::shared_ptr<const PlaceBlock::Goal> goal) {
        RCLCPP_INFO(get_logger(), ">>> Menerima Goal phase: %s", goal->phase.c_str());
        if (goal->phase != "1" && goal->phase != "2") {
            RCLCPP_WARN(get_logger(), "Phase tidak dikenal: %s", goal->phase.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePlaceBlock>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePlaceBlock> goal_handle) {
        std::thread{std::bind(&TRPlanner::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandlePlaceBlock> goal_handle) {
        auto feedback = std::make_shared<PlaceBlock::Feedback>();
        auto result = std::make_shared<PlaceBlock::Result>();
        const auto goal = goal_handle->get_goal();

        if (goal->phase == "1")      execute_move_phase1(goal_handle, feedback, result);
        else if (goal->phase == "2") execute_move_phase2(goal_handle, feedback, result);
    }

    struct Step {
        enum Type { GOTO, ROTATE, GRIPPER_OPEN, GRIPPER_CLOSE, LIFTER } type;
        double a;
        double b;
        const char* desc;
    };

    void run_step(const Step& s,
                  std::shared_ptr<GoalHandlePlaceBlock> gh,
                  std::shared_ptr<PlaceBlock::Feedback> fb,
                  std::shared_ptr<PlaceBlock::Result> rs,
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
            case Step::LIFTER:         prefix = "Tip: "; break;
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

    // ============================================================
    // PHASE 1: (Full Sequence)
    // ============================================================
    void execute_move_phase1(const std::shared_ptr<GoalHandlePlaceBlock> gh,
                             std::shared_ptr<PlaceBlock::Feedback> fb,
                             std::shared_ptr<PlaceBlock::Result> rs) {
        const std::vector<Step> steps = {
            {Step::GOTO,          -0.18,  -9.5, "Titik keberangkatan"},
            {Step::GRIPPER_CLOSE,  0.0,    0.0, "Capit balok"},
            {Step::LIFTER,         7.3,    0.0, "Angkat capit ke atas"},
            {Step::ROTATE,         3.14159, 0.0, "Putar 180 derajat (CCW)"},
            {Step::GOTO,          -0.18,  -8.3, "Mundur sedikit"},
            {Step::GOTO,          -1.55,   -8.3, "Geser kanan"},
            {Step::GOTO,          -1.55,   -4.225, "Titik akhir"},
            {Step::LIFTER,         0.0,    0.0, "Turunkan capit"},
            {Step::GRIPPER_OPEN,   0.0,    0.0, "Buka capit"},
            {Step::GOTO,          -1.55,  -8.3, "Geser kanan"},
            {Step::ROTATE,        3.14159, 0.0, "Putar 180 derajat (CW)"}
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

    // ============================================================
    // PHASE 2: (Full Sequence)
    // ============================================================
    void execute_move_phase2(const std::shared_ptr<GoalHandlePlaceBlock> gh,
                             std::shared_ptr<PlaceBlock::Feedback> fb,
                             std::shared_ptr<PlaceBlock::Result> rs) {
        const std::vector<Step> steps = {
            {Step::GOTO,        -1.048,     -8.3, "Menuju titik Komando 2"},
            {Step::GOTO,        -1.048,     -9.47, "Menuju titik Komando 2"},
            {Step::GRIPPER_CLOSE, 0.0,      0.0, "Capit balok"},
            {Step::LIFTER,       7.25,      0.0, "Angkat capit ke atas"},
            {Step::ROTATE,       3.14159,  0.0, "Putar 180 derajat (CCW)"},
            {Step::GOTO,        -1.485,     -9.48, "Geser kanan"},
            {Step::GOTO,        -1.485,     -4.25, "Titik akhir"},
            {Step::LIFTER,       0.0,      0.0, "Turunkan objek"},
            {Step::GRIPPER_OPEN, 0.0,      0.0, "Buka capit"},
            {Step::GOTO,        -1.48,     -8.3, "Geser kanan"},
            {Step::ROTATE,      1.5708,   0.0, "Rotasi 90 derajat (CW)"}
        };

        try {
            for (size_t i = 0; i < steps.size(); i++) {
                run_step(steps[i], gh, fb, rs, i, (int)steps.size());
            }
        } catch (const std::exception&) { return; }

        fb->current_state = "STANDBY: Komando 2 selesai, menunggu Komando 3";
        fb->progress = 1.0;
        gh->publish_feedback(fb);
        rs->success = true;
        rs->message = "Komando 2 selesai. Kirim Goal Phase 3 untuk lanjut.";
        gh->succeed(rs);
        RCLCPP_INFO(get_logger(), ">>> Phase 2 selesai. Kirim Goal Phase 3.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TRPlanner>());
    rclcpp::shutdown();
    return 0;
}