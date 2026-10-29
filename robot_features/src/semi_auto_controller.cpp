#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

using namespace std::chrono_literals;

class SemiAutoController : public rclcpp::Node {
public:
    SemiAutoController() : Node("semi_auto_controller") {
        // ===== Parameter PID (Tuned for Speed 1.0 m/s) =====
        kp_             = declare_parameter<double>("kp", 1.5);          // Agresif
        ki_             = declare_parameter<double>("ki", 0.0);          // Matiin integral
        kd_             = declare_parameter<double>("kd", 0.5);          // Rem kuat
        tolerance_      = declare_parameter<double>("tolerance", 0.05);  // 5 cm (jangan 1 cm)
        max_speed_      = declare_parameter<double>("max_speed", 1.0);   // 1 m/s
        min_speed_      = declare_parameter<double>("min_speed", 0.0);
        max_accel_      = declare_parameter<double>("max_accel", 2.0);   // Cepat nyampe top speed
        integral_limit_ = declare_parameter<double>("integral_limit", 0.3);
        integral_zone_  = declare_parameter<double>("integral_zone", 0.3);
        deriv_filter_   = declare_parameter<double>("deriv_filter", 0.3);
        odom_timeout_   = declare_parameter<double>("odom_timeout", 0.5);
        hold_heading_   = declare_parameter<bool>("hold_heading", false);
        kp_yaw_         = declare_parameter<double>("kp_yaw", 1.0);
        max_yaw_rate_   = declare_parameter<double>("max_yaw_rate", 1.0);

        param_cb_ = this->add_on_set_parameters_callback(
            std::bind(&SemiAutoController::on_parameters, this, std::placeholders::_1));

        // ===== Pub / Sub =====
        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&SemiAutoController::odom_callback, this, std::placeholders::_1));

        sub_target_ = create_subscription<geometry_msgs::msg::Point>(
            "/target_position", 10,
            std::bind(&SemiAutoController::target_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::duration<double>(kControlPeriod),
            std::bind(&SemiAutoController::control_loop, this));

        RCLCPP_INFO(get_logger(), "=== SEMI-AUTO SPEED DEMON (1 m/s) ===");
        RCLCPP_INFO(get_logger(), "Kirim target ke: /target_position");
        RCLCPP_INFO(get_logger(), "Contoh: ros2 topic pub --once /target_position geometry_msgs/msg/Point \"{x: 3.0, y: 0.0}\"");
        RCLCPP_INFO(get_logger(), "Tuning live: ros2 param set /semi_auto_controller kp 1.0");
    }

private:
    static constexpr double kControlPeriod = 0.05; // 20 Hz

    // ===== Parameter =====
    double kp_, ki_, kd_;
    double tolerance_, max_speed_, min_speed_, max_accel_;
    double integral_limit_, integral_zone_, deriv_filter_;
    double odom_timeout_;
    bool hold_heading_;
    double kp_yaw_, max_yaw_rate_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_;

    // ===== State robot =====
    double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
    bool has_odom_ = false;
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

    // ===== State target =====
    double target_x_ = 0.0, target_y_ = 0.0, target_yaw_ = 0.0;
    bool has_target_ = false;

    // ===== State PID =====
    double integral_x_ = 0.0, integral_y_ = 0.0;
    double prev_x_ = 0.0, prev_y_ = 0.0;
    double vel_x_f_ = 0.0, vel_y_f_ = 0.0;
    double cmd_x_ = 0.0, cmd_y_ = 0.0;
    bool first_run_ = true;
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

    static double normalize_angle(double a) {
        return std::atan2(std::sin(a), std::cos(a));
    }

    void publish_stop() {
        pub_cmd_->publish(geometry_msgs::msg::Twist());
        cmd_x_ = cmd_y_ = 0.0;
    }

    void reset_pid() {
        integral_x_ = integral_y_ = 0.0;
        vel_x_f_ = vel_y_f_ = 0.0;
        first_run_ = true;
    }

    rcl_interfaces::msg::SetParametersResult
    on_parameters(const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &p : params) {
            if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                const double v = p.as_double();
                if (!std::isfinite(v) || v < 0.0) {
                    result.successful = false;
                    result.reason = p.get_name() + " harus >= 0";
                    return result;
                }
                if (p.get_name() == "deriv_filter" && (v <= 0.0 || v > 1.0)) {
                    result.successful = false;
                    result.reason = "deriv_filter harus di (0, 1]";
                    return result;
                }
            }
        }

        for (const auto &p : params) {
            const auto &n = p.get_name();
            if      (n == "kp")             kp_ = p.as_double();
            else if (n == "ki")             { ki_ = p.as_double(); integral_x_ = integral_y_ = 0.0; }
            else if (n == "kd")             kd_ = p.as_double();
            else if (n == "tolerance")      tolerance_ = p.as_double();
            else if (n == "max_speed")      max_speed_ = p.as_double();
            else if (n == "min_speed")      min_speed_ = p.as_double();
            else if (n == "max_accel")      max_accel_ = p.as_double();
            else if (n == "integral_limit") integral_limit_ = p.as_double();
            else if (n == "integral_zone")  integral_zone_ = p.as_double();
            else if (n == "deriv_filter")   deriv_filter_ = p.as_double();
            else if (n == "odom_timeout")   odom_timeout_ = p.as_double();
            else if (n == "kp_yaw")         kp_yaw_ = p.as_double();
            else if (n == "max_yaw_rate")   max_yaw_rate_ = p.as_double();
            else if (n == "hold_heading")   hold_heading_ = p.as_bool();
        }
        return result;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        const auto &q = msg->pose.pose.orientation;
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        robot_yaw_ = std::atan2(siny_cosp, cosy_cosp);

        last_odom_time_ = this->now();
        has_odom_ = true;
    }

    void target_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        if (!std::isfinite(msg->x) || !std::isfinite(msg->y)) {
            RCLCPP_WARN(get_logger(), "Target ditolak: NaN/Inf");
            return;
        }
        target_x_ = msg->x;
        target_y_ = msg->y;
        target_yaw_ = robot_yaw_;
        has_target_ = true;
        reset_pid();
        RCLCPP_INFO(get_logger(), "🎯 Target baru: (%.2f, %.2f)", target_x_, target_y_);
    }

    void control_loop() {
        if (!has_target_) return;

        if (!has_odom_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Menunggu /odom...");
            return;
        }
        const rclcpp::Time now = this->now();
        if ((now - last_odom_time_).seconds() > odom_timeout_) {
            publish_stop();
            reset_pid();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odom basi, robot stop");
            return;
        }

        double dt = kControlPeriod;
        if (!first_run_) {
            dt = (now - last_time_).seconds();
            if (dt < 1e-3 || dt > 0.5) dt = kControlPeriod;
        }
        last_time_ = now;

        const double ex = target_x_ - robot_x_;
        const double ey = target_y_ - robot_y_;
        const double distance = std::hypot(ex, ey);

        if (distance < tolerance_) {
            has_target_ = false;
            publish_stop();
            reset_pid();
            RCLCPP_INFO(get_logger(), "✅ Target tercapai! Posisi: (%.2f, %.2f)", robot_x_, robot_y_);
            return;
        }

        if (first_run_) {
            prev_x_ = robot_x_; prev_y_ = robot_y_;
            vel_x_f_ = vel_y_f_ = 0.0;
            first_run_ = false;
        } else {
            const double vx_raw = (robot_x_ - prev_x_) / dt;
            const double vy_raw = (robot_y_ - prev_y_) / dt;
            vel_x_f_ += deriv_filter_ * (vx_raw - vel_x_f_);
            vel_y_f_ += deriv_filter_ * (vy_raw - vel_y_f_);
            prev_x_ = robot_x_; prev_y_ = robot_y_;
        }

        if (ki_ > 0.0 && distance < integral_zone_) {
            integral_x_ = std::clamp(integral_x_ + ex * dt, -integral_limit_, integral_limit_);
            integral_y_ = std::clamp(integral_y_ + ey * dt, -integral_limit_, integral_limit_);
        } else {
            integral_x_ = integral_y_ = 0.0;
        }

        double vx = kp_ * ex + ki_ * integral_x_ - kd_ * vel_x_f_;
        double vy = kp_ * ey + ki_ * integral_y_ - kd_ * vel_y_f_;

        double speed = std::hypot(vx, vy);
        if (speed > max_speed_ && speed > 1e-9) {
            const double s = max_speed_ / speed;
            vx *= s; vy *= s; speed = max_speed_;
        }
        if (min_speed_ > 0.0 && speed < min_speed_ && speed > 1e-9) {
            const double s = min_speed_ / speed;
            vx *= s; vy *= s;
        }

        if (max_accel_ > 0.0) {
            const double dvx = vx - cmd_x_;
            const double dvy = vy - cmd_y_;
            const double dv = std::hypot(dvx, dvy);
            const double max_dv = max_accel_ * dt;
            if (dv > max_dv) {
                const double s = max_dv / dv;
                vx = cmd_x_ + dvx * s;
                vy = cmd_y_ + dvy * s;
            }
        }
        cmd_x_ = vx; cmd_y_ = vy;

        const double c = std::cos(robot_yaw_);
        const double s = std::sin(robot_yaw_);

        geometry_msgs::msg::Twist twist;
        twist.linear.x =  c * vx + s * vy;
        twist.linear.y = -s * vx + c * vy;

        if (hold_heading_) {
            const double yaw_err = normalize_angle(target_yaw_ - robot_yaw_);
            twist.angular.z = std::clamp(kp_yaw_ * yaw_err, -max_yaw_rate_, max_yaw_rate_);
        } else {
            twist.angular.z = 0.0;
        }

        pub_cmd_->publish(twist);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "Dist: %.2fm | Cmd: vx=%.2f, vy=%.2f, wz=%.2f",
            distance, twist.linear.x, twist.linear.y, twist.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_target_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemiAutoController>());
    rclcpp::shutdown();
    return 0;
}