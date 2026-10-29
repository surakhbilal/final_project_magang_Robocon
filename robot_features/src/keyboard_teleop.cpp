#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp> // TAMBAHAN BUAT GRIPPER
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <sys/select.h> 
#include <chrono>

using namespace std::chrono_literals;

class KeyboardTeleop : public rclcpp::Node {
public:
    KeyboardTeleop() : Node("keyboard_teleop_cpp"), robot_yaw_(0.0), last_key_time_(this->now()) {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&KeyboardTeleop::odom_callback, this, std::placeholders::_1));
        
        // --- TAMBAHAN PUBLISHER GRIPPER ---
        gripper_pub_ = this->create_publisher<std_msgs::msg::Float64>("/gripper_cmd", 10);
        tip_pub_ = this->create_publisher<std_msgs::msg::Float64>("/gripper_tip_cmd", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&KeyboardTeleop::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "=== Controller connected ===");
        RCLCPP_INFO(this->get_logger(), "WASD = Gerak, Q/E = Muter");
        RCLCPP_INFO(this->get_logger(), "O = Nyapit, P = Lepas");
    }

    void run() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        char c;
        while (rclcpp::ok()) {
            fd_set set;
            struct timeval timeout;
            FD_ZERO(&set);
            FD_SET(STDIN_FILENO, &set);
            timeout.tv_sec = 0;
            timeout.tv_usec = 0;

            int rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
            if (rv > 0) {
                c = std::cin.get();
                if (c == 3) { // Ctrl+C
                    rclcpp::shutdown();
                    break;
                }
                
                // --- LOGIKA TOMBOL GRIPPER ---
                if (c == 'o' || c == 'O') {
                    RCLCPP_INFO(this->get_logger(), ">>> MENGUNCI CAPIT...");
                    auto grip_msg = std_msgs::msg::Float64();
                    grip_msg.data = 0.7;
                    gripper_pub_->publish(grip_msg);
                    
                    auto tip_msg = std_msgs::msg::Float64();
                    tip_msg.data = 7.5;
                    tip_pub_->publish(tip_msg);
                } 
                else if (c == 'p' || c == 'P') {
                    RCLCPP_INFO(this->get_logger(), ">>> MELEPAS CAPIT...");
                    auto tip_msg = std_msgs::msg::Float64();
                    tip_msg.data = 0.0;
                    tip_pub_->publish(tip_msg);
                    
                    auto grip_msg = std_msgs::msg::Float64();
                    grip_msg.data = 0.0;
                    gripper_pub_->publish(grip_msg);
                }
                // -----------------------------

                // Simpan tombol & waktu buat gerak (WASD)
                current_key_ = c;
                last_key_time_ = this->now(); 
            }

            rclcpp::spin_some(this->shared_from_this());
            std::this_thread::sleep_for(10ms); 
        }
        
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

private:
    void control_loop() {
        auto twist = geometry_msgs::msg::Twist();
        
        auto elapsed = (this->now() - last_key_time_).seconds();
        if (elapsed > 0.15) {
            current_key_ = 0; 
        }

        double vx = 0.0, vy = 0.0, wz = 0.0;
        double speed = 1.0;

        char c = current_key_;
        if (c == 'w') vx = speed;
        else if (c == 's') vx = -speed;
        else if (c == 'a') vy = speed;
        else if (c == 'd') vy = -speed;
        else if (c == 'q') wz = 0.5;
        else if (c == 'e') wz = -0.5;

        // Rotasi Global ke Lokal
        if (vx != 0.0 || vy != 0.0) {
            twist.linear.x = vx * cos(robot_yaw_) + vy * sin(robot_yaw_);
            twist.linear.y = -vx * sin(robot_yaw_) + vy * cos(robot_yaw_);
        }
        twist.angular.z = wz;

        pub_->publish(twist);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        robot_yaw_ = -std::atan2(siny_cosp, cosy_cosp); 
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // --- VARIABEL TAMBAHAN GRIPPER ---
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tip_pub_;

    double robot_yaw_;
    char current_key_ = 0;
    rclcpp::Time last_key_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardTeleop>();
    node->run();
    rclcpp::shutdown();
    return 0;
}