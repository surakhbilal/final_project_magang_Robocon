#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

class FoxgloveGlobalNode : public rclcpp::Node {
public:
    FoxgloveGlobalNode() : Node("foxglove_global_node"), robot_yaw_(0.0) {
        // Subscriber dari UI HTML (TETAP, karena ini input dari Foxglove UI)
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/foxglove_cmd", 10, std::bind(&FoxgloveGlobalNode::cmd_callback, this, std::placeholders::_1));
        
        // UBAH KE TOPIK TR
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/tr/odom", 10, std::bind(&FoxgloveGlobalNode::odom_callback, this, std::placeholders::_1));

        // Subscriber capit dari UI HTML (TETAP)
        sub_grip_ = this->create_subscription<std_msgs::msg::Float64>(
            "/ui/gripper_cmd", 10, std::bind(&FoxgloveGlobalNode::grip_callback, this, std::placeholders::_1));
        
        // Publisher ke robot TR
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/tr/cmd_vel", 10);
        pub_grip_ = this->create_publisher<std_msgs::msg::Float64>("/tr/gripper_cmd", 10);
        pub_tip_ = this->create_publisher<std_msgs::msg::Float64>("/tr/gripper_tip_cmd", 10);
        
        RCLCPP_INFO(this->get_logger(), "FOXGLOVE GLOBAL NODE (TR) + GRIPPER STARTED");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        robot_yaw_ = -std::atan2(siny_cosp, cosy_cosp); 
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto out_msg = geometry_msgs::msg::Twist();
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        double wz = msg->angular.z;

        // Rotasi Global ke Lokal
        if (vx != 0.0 || vy != 0.0) {
            out_msg.linear.x = vx * std::cos(robot_yaw_) + vy * std::sin(robot_yaw_);
            out_msg.linear.y = -vx * std::sin(robot_yaw_) + vy * std::cos(robot_yaw_);
        }
        out_msg.angular.z = wz;
        pub_->publish(out_msg);
    }

    void grip_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        // Terima perintah capit dari UI
        // Nilai: 0.0 = Buka, 1.0 = Tutup + Angkat
        if (msg->data == 1.0) {
            // Tutup capit utama
            auto grip = std_msgs::msg::Float64();
            grip.data = 0.7;
            pub_grip_->publish(grip);
            
            // Angkat ujung capit
            auto tip = std_msgs::msg::Float64();
            tip.data = 7.5;
            pub_tip_->publish(tip);
            
            RCLCPP_INFO(this->get_logger(), "🦾 CAPIT NUTUP & ANGKAT");
        } else {
            // Buka capit
            auto tip = std_msgs::msg::Float64();
            tip.data = 0.0;
            pub_tip_->publish(tip);
            
            auto grip = std_msgs::msg::Float64();
            grip.data = 0.0;
            pub_grip_->publish(grip);
            
            RCLCPP_INFO(this->get_logger(), "🖐️ CAPIT BUKA");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_grip_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_grip_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_tip_;
    double robot_yaw_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FoxgloveGlobalNode>());
    rclcpp::shutdown();
    return 0;
}