#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cmath>
#include <cctype>
#include <iostream>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class KeyboardTeleop : public rclcpp::Node {
public:
    KeyboardTeleop()
        : Node("keyboard_teleop_br"),
          robot_yaw_(0.0),
          last_key_time_(this->now()),
          last_z_char_time_(this->now())
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/br/cmd_vel", 10);

        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/br/odom", 10, std::bind(&KeyboardTeleop::odom_callback, this, std::placeholders::_1));

        gripper_pub_ = this->create_publisher<std_msgs::msg::Float64>("/br/gripper_cmd", 10);
        z_axis_pub_  = this->create_publisher<std_msgs::msg::Float64>("/br/z_axis_cmd", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&KeyboardTeleop::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "=== BR Controller Connected ===");
        RCLCPP_INFO(this->get_logger(), "WASD = Gerak, Q/E = Muter");
        RCLCPP_INFO(this->get_logger(), "O = Mencapit, P = Membuka Capit");
        RCLCPP_INFO(this->get_logger(), "L = Naik (tap: 0 -> tengah -> atas, hold: langsung ke atas)");
        RCLCPP_INFO(this->get_logger(), "I = Turun (tap: atas -> tengah -> 0, hold: langsung ke posisi awal)");
    }

    void run() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (rclcpp::ok()) {
            fd_set set;
            struct timeval timeout;
            FD_ZERO(&set);
            FD_SET(STDIN_FILENO, &set);
            timeout.tv_sec = 0;
            timeout.tv_usec = 0;

            int rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
            if (rv > 0) {
                char c = 0;
                ssize_t n = read(STDIN_FILENO, &c, 1);

                if (n > 0) {
                    if (c == 3) { // Ctrl+C
                        rclcpp::shutdown();
                        break;
                    }

                    // --- LOGIKA TOMBOL CAPIT ---
                    if (c == 'o' || c == 'O') {
                        RCLCPP_INFO(this->get_logger(), ">>> MENCAPIT...");
                        auto grip_msg = std_msgs::msg::Float64();
                        grip_msg.data = 0.5;
                        gripper_pub_->publish(grip_msg);
                    }
                    else if (c == 'p' || c == 'P') {
                        RCLCPP_INFO(this->get_logger(), ">>> MEMBUKA CAPIT...");
                        auto grip_msg = std_msgs::msg::Float64();
                        grip_msg.data = 0.0;
                        gripper_pub_->publish(grip_msg);
                    }

                    auto now = this->now();
                    char key = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

                    // --- LOGIKA TOMBOL LENGAN (L / I) ---
                    if (key == 'l' || key == 'i') {
                        handle_z_key(key, now);
                    }

                    // Simpan tombol & waktu buat gerak roda (WASD, Q, E)
                    current_key_ = key;
                    last_key_time_ = now;
                }
            }

            rclcpp::spin_some(this->shared_from_this());
            std::this_thread::sleep_for(10ms);
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

private:
    // ===== Parameter yang bisa kamu atur =====
    static constexpr double HOLD_TIMEOUT_SEC = 0.15; // batas hold untuk roda
    static constexpr double HOLD_CONFIRM_SEC = 0.12; // 2 karakter sama lebih cepat dari ini = hold
    static constexpr double HOLD_RELEASE_SEC = 0.25; // tidak ada karakter selama ini = hold selesai
    static constexpr double Z_MIN            = 0.0;  // posisi awal / paling bawah
    static constexpr double Z_MID            = 0.4;  // ketinggian "5m" (SESUAIKAN, harus < Z_MAX)
    static constexpr double Z_MAX            = 0.8;  // paling atas (sesuaikan dengan maxStop proto)
    static constexpr double Z_EPS            = 1e-6;
    // =========================================

    // Dipanggil setiap ada karakter L / I yang masuk dari terminal
    void handle_z_key(char key, const rclcpp::Time &now) {
        double gap = (now - last_z_char_time_).seconds();
        bool same_key = (key == last_z_key_);

        // Ganti tombol saat ada tap yang masih menunggu -> selesaikan tap sebelumnya dulu
        if (z_pending_ && !same_key) {
            z_pending_ = false;
            apply_tap(last_z_key_);
        }

        last_z_char_time_ = now;
        last_z_key_ = key;

        // Hold sedang berlangsung, karakter repeat berikutnya diabaikan
        if (same_key && hold_active_ && gap < HOLD_RELEASE_SEC) {
            return;
        }

        // Dua karakter sama datang sangat cepat -> ini key-repeat, berarti HOLD
        if (same_key && z_pending_ && gap < HOLD_CONFIRM_SEC) {
            z_pending_ = false;
            hold_active_ = true;
            apply_hold(key);
            return;
        }

        // Selain itu: kandidat tap baru (dieksekusi di control_loop kalau tidak ada lanjutan)
        hold_active_ = false;
        z_pending_ = true;
    }

    // Tap: pindah satu level (0 <-> tengah <-> atas)
    void apply_tap(char key) {
        if (key == 'l') {
            if (z_target_ < Z_MID - Z_EPS) z_target_ = Z_MID;
            else                           z_target_ = Z_MAX;
            RCLCPP_INFO(this->get_logger(), ">>> LENGAN NAIK ke %.2f", z_target_);
        } else {
            if (z_target_ > Z_MID + Z_EPS) z_target_ = Z_MID;
            else                           z_target_ = Z_MIN;
            RCLCPP_INFO(this->get_logger(), ">>> LENGAN TURUN ke %.2f", z_target_);
        }
        publish_z(z_target_);
    }

    // Hold: L langsung ke paling atas, I langsung balik ke posisi awal
    void apply_hold(char key) {
        if (key == 'l') {
            z_target_ = Z_MAX;
            RCLCPP_INFO(this->get_logger(), ">>> HOLD L: LENGAN KE ATAS (%.2f)", z_target_);
        } else {
            z_target_ = Z_MIN;
            RCLCPP_INFO(this->get_logger(), ">>> HOLD I: LENGAN KE POSISI AWAL (%.2f)", z_target_);
        }
        publish_z(z_target_);
    }

    void control_loop() {
        auto twist = geometry_msgs::msg::Twist();
        auto now = this->now();

        // Tap yang sudah tidak ada lanjutan karakter -> benar-benar tap, eksekusi sekarang
        if (z_pending_ && (now - last_z_char_time_).seconds() >= HOLD_CONFIRM_SEC) {
            z_pending_ = false;
            apply_tap(last_z_key_);
        }

        // ===== Kontrol Roda (hold-to-move) =====
        double elapsed = (now - last_key_time_).seconds();
        if (elapsed > HOLD_TIMEOUT_SEC) {
            current_key_ = 0;
        }

        char c = current_key_;
        double vx = 0.0, vy = 0.0, wz = 0.0;
        double speed = 1.0;

        if (c == 'w') vx = speed;
        else if (c == 's') vx = -speed;
        else if (c == 'a') vy = speed;
        else if (c == 'd') vy = -speed;
        else if (c == 'q') wz = 0.5;
        else if (c == 'e') wz = -0.5;

        if (vx != 0.0 || vy != 0.0) {
            twist.linear.x = vx * cos(robot_yaw_) + vy * sin(robot_yaw_);
            twist.linear.y = -vx * sin(robot_yaw_) + vy * cos(robot_yaw_);
        }
        twist.angular.z = wz;

        pub_->publish(twist);
    }

    void publish_z(double value) {
        auto z_msg = std_msgs::msg::Float64();
        z_msg.data = value;
        z_axis_pub_->publish(z_msg);
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

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_axis_pub_;

    double robot_yaw_;
    char current_key_ = 0;
    rclcpp::Time last_key_time_;

    // State tombol lengan
    rclcpp::Time last_z_char_time_;
    char last_z_key_ = 0;
    bool z_pending_ = false;   // ada tap yang menunggu konfirmasi (bukan hold)
    bool hold_active_ = false; // sedang di-hold
    double z_target_ = 0.0;    // posisi awal lengan (asumsi di bawah / 0)
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardTeleop>();
    node->run();
    rclcpp::shutdown();
    return 0;
}