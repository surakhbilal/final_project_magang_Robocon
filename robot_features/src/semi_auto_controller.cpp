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
#include <std_msgs/msg/float64.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

using namespace std::chrono_literals;

class SemiAutoController : public rclcpp::Node {
public:
    SemiAutoController() : Node("semi_auto_controller") {
        // ===== PARAMETER POSISI (X,Y) =====
        kp_             = declare_parameter<double>("kp", 1.5);
        ki_             = declare_parameter<double>("ki", 0.0);
        kd_             = declare_parameter<double>("kd", 0.5);
        tolerance_      = declare_parameter<double>("tolerance", 0.05);
        max_speed_      = declare_parameter<double>("max_speed", 1.0);
        min_speed_      = declare_parameter<double>("min_speed", 0.0);
        max_accel_      = declare_parameter<double>("max_accel", 2.0);
        integral_limit_ = declare_parameter<double>("integral_limit", 0.3);
        integral_zone_  = declare_parameter<double>("integral_zone", 0.3);
        deriv_filter_   = declare_parameter<double>("deriv_filter", 0.3);
        odom_timeout_   = declare_parameter<double>("odom_timeout", 0.5);
        hold_heading_   = declare_parameter<bool>("hold_heading", false);

        // ===== PARAMETER YAW (PID PUTAR) =====
        kp_yaw_            = declare_parameter<double>("kp_yaw", 2.0);
        ki_yaw_            = declare_parameter<double>("ki_yaw", 0.0);
        kd_yaw_            = declare_parameter<double>("kd_yaw", 0.2);
        max_yaw_rate_      = declare_parameter<double>("max_yaw_rate", 1.0);    // rad/s
        min_yaw_rate_      = declare_parameter<double>("min_yaw_rate", 0.10);   // rad/s (deadband motor)
        yaw_accel_         = declare_parameter<double>("yaw_accel", 2.0);       // rad/s^2
        yaw_tolerance_     = declare_parameter<double>("yaw_tolerance", 0.0175);// rad (~1 derajat)
        yaw_settle_time_   = declare_parameter<double>("yaw_settle_time", 0.3); // detik
        yaw_timeout_       = declare_parameter<double>("yaw_timeout", 10.0);    // detik
        yaw_integral_zone_ = declare_parameter<double>("yaw_integral_zone", 0.2);
        yaw_deriv_filter_  = declare_parameter<double>("yaw_deriv_filter", 0.3);
        // Estimasi keterlambatan total (perintah -> robot bergerak -> odom terbaca), detik.
        // Makin besar = mulai mengerem lebih awal (overshoot turun, pendekatan sedikit lebih lambat).
        yaw_lag_           = declare_parameter<double>("yaw_lag", 0.35);
        yaw_in_degrees_    = declare_parameter<bool>("yaw_in_degrees", false);

        // Arah perintah putar terhadap arah yaw di odom.
        // yaw_invert_cmd=true  -> angular.z dikirim dengan tanda dibalik.
        // yaw_auto_sign=true   -> jika terdeteksi umpan balik terbalik, otomatis dibalik.
        const bool invert = declare_parameter<bool>("yaw_invert_cmd", false);
        yaw_auto_sign_    = declare_parameter<bool>("yaw_auto_sign", true);
        yaw_cmd_sign_     = invert ? -1.0 : 1.0;
        yaw_sign_verified_ = invert; // jika user set manual, jangan dibalik otomatis

        param_cb_ = this->add_on_set_parameters_callback(
            std::bind(&SemiAutoController::on_parameters, this, std::placeholders::_1));

        // ===== PUBLISHER & SUBSCRIBER =====
        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/tr/cmd_vel", 10);

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/tr/odom", 10,
            std::bind(&SemiAutoController::odom_callback, this, std::placeholders::_1));

        // Target posisi X,Y (topik tetap seperti aslinya)
        sub_target_ = create_subscription<geometry_msgs::msg::Point>(
            "/target_position", 10,
            std::bind(&SemiAutoController::target_callback, this, std::placeholders::_1));

        // Yaw ABSOLUT (jalur terpendek)
        sub_yaw_target_ = create_subscription<std_msgs::msg::Float64>(
            "/tr/target_yaw", 10,
            std::bind(&SemiAutoController::yaw_callback, this, std::placeholders::_1));

        // Yaw RELATIF: positif = berlawanan jarum jam (CCW), negatif = searah jarum jam (CW)
        sub_rotate_by_ = create_subscription<std_msgs::msg::Float64>(
            "/tr/rotate_by", 10,
            std::bind(&SemiAutoController::rotate_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::duration<double>(kControlPeriod),
            std::bind(&SemiAutoController::control_loop, this));

        RCLCPP_INFO(get_logger(), "=== TR SEMI-AUTO CONTROLLER ===");
        RCLCPP_INFO(get_logger(), "Target X,Y     : /target_position");
        RCLCPP_INFO(get_logger(), "Target Yaw abs : /tr/target_yaw (%s)", yaw_in_degrees_ ? "derajat" : "radian");
        RCLCPP_INFO(get_logger(), "Putar relatif  : /tr/rotate_by (+CCW / -CW, %s)", yaw_in_degrees_ ? "derajat" : "radian");
    }

private:
    static constexpr double kControlPeriod = 0.05;
    static constexpr double kDecelMargin   = 0.7;   // profil pengereman memakai 70% yaw_accel
    static constexpr double kYawRateStill  = 0.05;  // rad/s: dianggap diam
    static constexpr double kSettleMax     = 1.0;   // detik: batas tunggu jika rate odom berisik
    static constexpr double kStallRate     = 0.02;  // rad/s: dianggap tidak bergerak
    static constexpr double kStallDelay    = 0.3;   // detik sebelum dorongan (boost) dimulai
    static constexpr double kStallRamp     = 0.4;   // rad/s per detik pertambahan dorongan
    static constexpr double kStallAbort    = 3.0;   // detik tidak bergerak -> batalkan
    static constexpr double kVerifyRad     = 0.10;  // kemajuan minimal untuk verifikasi arah (~6 deg)
    static constexpr double kDivergeRad    = 0.20;  // error membesar sejauh ini -> menyimpang (~11 deg)
    static constexpr int    kStopBurst     = 5;     // jumlah pesan stop berulang
    static constexpr double kRelaxedFactor = 3.0;   // zona "cukup dekat" = 3 x yaw_tolerance
    static constexpr double kRelaxedStall  = 0.6;   // detik diam di zona itu -> dianggap selesai

    // ---- parameter posisi ----
    double kp_, ki_, kd_;
    double tolerance_, max_speed_, min_speed_, max_accel_;
    double integral_limit_, integral_zone_, deriv_filter_;
    double odom_timeout_;
    bool hold_heading_;

    // ---- parameter yaw ----
    double kp_yaw_, ki_yaw_, kd_yaw_;
    double max_yaw_rate_, min_yaw_rate_, yaw_accel_;
    double yaw_tolerance_, yaw_settle_time_, yaw_timeout_;
    double yaw_integral_zone_, yaw_deriv_filter_, yaw_lag_;
    bool yaw_in_degrees_;
    bool yaw_auto_sign_;

    OnSetParametersCallbackHandle::SharedPtr param_cb_;

    // ---- state odom ----
    double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
    double yaw_unwrapped_ = 0.0;   // yaw kumulatif (tidak lompat di +-180 derajat)
    double yaw_rate_f_ = 0.0;      // kecepatan sudut terfilter (rad/s), dalam ruang odom
    bool has_odom_ = false;
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

    // ---- state target posisi ----
    double target_x_ = 0.0, target_y_ = 0.0, target_yaw_ = 0.0;
    bool has_target_ = false;

    // ---- state target yaw ----
    bool has_yaw_target_ = false;
    double target_yaw_unwrapped_ = 0.0;
    double yaw_start_unwrapped_ = 0.0;
    double yaw_start_abs_err_ = 0.0;
    double yaw_dir_ = 1.0;             // +1 CCW, -1 CW
    double yaw_max_overshoot_ = 0.0;   // rad, lewat dari target searah putaran
    double yaw_integral_ = 0.0;
    double yaw_stall_time_ = 0.0;
    double cmd_wz_ = 0.0;          // perintah dalam ruang odom (sebelum dikali yaw_cmd_sign_)
    double yaw_cmd_sign_ = 1.0;
    bool yaw_sign_verified_ = false;
    bool yaw_sign_flipped_ = false;
    bool yaw_first_run_ = true;
    bool yaw_in_tol_ = false;
    int stop_burst_ = 0;
    rclcpp::Time yaw_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time yaw_last_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time yaw_in_tol_since_{0, 0, RCL_ROS_TIME};

    // ---- state PID posisi ----
    double integral_x_ = 0.0, integral_y_ = 0.0;
    double prev_x_ = 0.0, prev_y_ = 0.0;
    double vel_x_f_ = 0.0, vel_y_f_ = 0.0;
    double cmd_x_ = 0.0, cmd_y_ = 0.0;
    bool first_run_ = true;
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

    static double normalize_angle(double a) {
        return std::atan2(std::sin(a), std::cos(a));
    }

    double to_rad(double v) const {
        return yaw_in_degrees_ ? v * M_PI / 180.0 : v;
    }

    void publish_stop() {
        pub_cmd_->publish(geometry_msgs::msg::Twist());
        cmd_x_ = cmd_y_ = 0.0;
        cmd_wz_ = 0.0;
        stop_burst_ = kStopBurst;
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
            }
        }
        for (const auto &p : params) {
            const auto &n = p.get_name();
            if      (n == "kp")                 kp_ = p.as_double();
            else if (n == "ki")                 { ki_ = p.as_double(); integral_x_ = integral_y_ = 0.0; }
            else if (n == "kd")                 kd_ = p.as_double();
            else if (n == "tolerance")          tolerance_ = p.as_double();
            else if (n == "max_speed")          max_speed_ = p.as_double();
            else if (n == "min_speed")          min_speed_ = p.as_double();
            else if (n == "max_accel")          max_accel_ = p.as_double();
            else if (n == "integral_limit")     integral_limit_ = p.as_double();
            else if (n == "integral_zone")      integral_zone_ = p.as_double();
            else if (n == "deriv_filter")       deriv_filter_ = p.as_double();
            else if (n == "odom_timeout")       odom_timeout_ = p.as_double();
            else if (n == "hold_heading")       hold_heading_ = p.as_bool();
            // ---- yaw ----
            else if (n == "kp_yaw")             kp_yaw_ = p.as_double();
            else if (n == "ki_yaw")             { ki_yaw_ = p.as_double(); yaw_integral_ = 0.0; }
            else if (n == "kd_yaw")             kd_yaw_ = p.as_double();
            else if (n == "max_yaw_rate")       max_yaw_rate_ = p.as_double();
            else if (n == "min_yaw_rate")       min_yaw_rate_ = p.as_double();
            else if (n == "yaw_accel")          yaw_accel_ = p.as_double();
            else if (n == "yaw_tolerance")      yaw_tolerance_ = p.as_double();
            else if (n == "yaw_settle_time")    yaw_settle_time_ = p.as_double();
            else if (n == "yaw_timeout")        yaw_timeout_ = p.as_double();
            else if (n == "yaw_integral_zone")  yaw_integral_zone_ = p.as_double();
            else if (n == "yaw_deriv_filter")   yaw_deriv_filter_ = p.as_double();
            else if (n == "yaw_lag")            yaw_lag_ = p.as_double();
            else if (n == "yaw_in_degrees")     yaw_in_degrees_ = p.as_bool();
            else if (n == "yaw_auto_sign")      yaw_auto_sign_ = p.as_bool();
            else if (n == "yaw_invert_cmd") {
                const bool inv = p.as_bool();
                yaw_cmd_sign_ = inv ? -1.0 : 1.0;
                yaw_sign_verified_ = inv;
            }
        }
        return result;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        const auto &q = msg->pose.pose.orientation;
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        const double new_yaw = std::atan2(siny_cosp, cosy_cosp);
        const rclcpp::Time t = this->now();

        if (!has_odom_) {
            yaw_unwrapped_ = new_yaw;
            yaw_rate_f_ = 0.0;
        } else {
            // Akumulasi selisih sudut -> yaw kumulatif tanpa lompatan di +-pi
            const double delta = normalize_angle(new_yaw - robot_yaw_);
            yaw_unwrapped_ += delta;

            // Estimasi kecepatan sudut (terfilter) untuk suku D
            const double dt_o = (t - last_odom_time_).seconds();
            if (dt_o > 1e-3 && dt_o < 0.5) {
                const double raw = delta / dt_o;
                yaw_rate_f_ += yaw_deriv_filter_ * (raw - yaw_rate_f_);
            }
        }

        robot_yaw_ = new_yaw;
        last_odom_time_ = t;
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
        has_yaw_target_ = false; // Matikan mode putar jika ada target X,Y baru
        stop_burst_ = 0;
        reset_pid();
        RCLCPP_INFO(get_logger(), "🎯 Target Posisi: (%.2f, %.2f)", target_x_, target_y_);
    }

    // Mulai rotasi sebesar 'delta' radian dari sudut sekarang.
    //   delta > 0 : berlawanan jarum jam (CCW)
    //   delta < 0 : searah jarum jam (CW)
    // Arah DIKUNCI di sini, tidak berubah sampai selesai.
    void start_yaw_rotation(double delta) {
        // Sudah di target (selisih < toleransi): tidak perlu memutar
        if (!has_yaw_target_ && std::abs(delta) < yaw_tolerance_) {
            RCLCPP_INFO(get_logger(), "Sudah di sudut target (selisih %.2f derajat), tidak berputar.",
                        std::abs(delta) * 180.0 / M_PI);
            return;
        }
        target_yaw_unwrapped_ = yaw_unwrapped_ + delta;
        target_yaw_ = normalize_angle(robot_yaw_ + delta);
        yaw_start_unwrapped_ = yaw_unwrapped_;
        yaw_start_abs_err_ = std::abs(delta);
        yaw_dir_ = (delta >= 0.0) ? 1.0 : -1.0;
        yaw_max_overshoot_ = 0.0;

        if (!has_yaw_target_) cmd_wz_ = 0.0;
        yaw_integral_ = 0.0;
        yaw_stall_time_ = 0.0;
        yaw_sign_flipped_ = false;
        yaw_first_run_ = true;
        yaw_in_tol_ = false;
        stop_burst_ = 0;

        has_yaw_target_ = true;
        has_target_ = false; // Matikan mode X,Y
        reset_pid();

        RCLCPP_INFO(get_logger(), "🔄 Putar %s %.1f derajat (%.3f rad) -> yaw target %.1f derajat",
                    delta >= 0.0 ? "CCW (berlawanan jarum jam)" : "CW (searah jarum jam)",
                    std::abs(delta) * 180.0 / M_PI, std::abs(delta),
                    target_yaw_ * 180.0 / M_PI);
    }

    // Target yaw ABSOLUT: jalur terpendek, arah dikunci di awal.
    // (Untuk 180 derajat dengan arah tertentu, pakai /tr/rotate_by +-pi.)
    void yaw_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        if (!std::isfinite(msg->data)) {
            RCLCPP_WARN(get_logger(), "Yaw target ditolak: NaN/Inf");
            return;
        }
        if (!has_odom_) {
            RCLCPP_WARN(get_logger(), "Yaw target ditolak: belum ada odom");
            return;
        }
        const double target = to_rad(msg->data);
        // Pesan berulang dengan target yang sama tidak me-restart apa pun
        if (has_yaw_target_ && std::abs(normalize_angle(target - target_yaw_)) < 1e-3) return;
        start_yaw_rotation(normalize_angle(target - robot_yaw_));
    }

    // Putar RELATIF. Selama robot masih berputar, perintah baru DIABAIKAN
    // (supaya publish berulang tidak menambah putaran).
    void rotate_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        if (!std::isfinite(msg->data)) {
            RCLCPP_WARN(get_logger(), "Rotate ditolak: NaN/Inf");
            return;
        }
        if (!has_odom_) {
            RCLCPP_WARN(get_logger(), "Rotate ditolak: belum ada odom");
            return;
        }
        if (has_yaw_target_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "rotate_by diabaikan: robot masih berputar");
            return;
        }
        start_yaw_rotation(to_rad(msg->data));
    }

    void finish_yaw(const char *reason) {
        const double rotated  = yaw_unwrapped_ - yaw_start_unwrapped_;
        const double residual = target_yaw_unwrapped_ - yaw_unwrapped_;
        has_yaw_target_ = false;
        yaw_integral_ = 0.0;
        yaw_in_tol_ = false;
        publish_stop();
        RCLCPP_INFO(get_logger(),
            "%s Terukur odom: putar %.2f derajat | sisa error %.2f derajat | overshoot maks %.1f derajat | yaw akhir %.1f derajat",
            reason, rotated * 180.0 / M_PI, residual * 180.0 / M_PI,
            yaw_max_overshoot_ * 180.0 / M_PI, robot_yaw_ * 180.0 / M_PI);
        if (yaw_max_overshoot_ > 3.0 * yaw_tolerance_) {
            RCLCPP_WARN(get_logger(),
                "Overshoot %.1f derajat. Naikkan param yaw_lag (sekarang %.2f s) agar pengereman lebih awal.",
                yaw_max_overshoot_ * 180.0 / M_PI, yaw_lag_);
        }
    }

    void abort_yaw(const char *reason) {
        has_yaw_target_ = false;
        yaw_integral_ = 0.0;
        yaw_in_tol_ = false;
        publish_stop();
        RCLCPP_ERROR(get_logger(), "⛔ Putar DIBATALKAN, robot stop: %s", reason);
    }

    // ==========================================
    // PID YAW (putar di tempat)
    // ==========================================
    void yaw_control_step(const rclcpp::Time &now) {
        if (yaw_first_run_) {
            yaw_start_time_ = now;
            yaw_last_time_ = now;
            yaw_in_tol_ = false;
            yaw_integral_ = 0.0;
            yaw_stall_time_ = 0.0;
            yaw_first_run_ = false;
        }

        double dt = (now - yaw_last_time_).seconds();
        if (dt < 1e-3 || dt > 0.5) dt = kControlPeriod;
        yaw_last_time_ = now;

        // Error memakai yaw kumulatif -> tidak ada lompatan di +-180 derajat
        const double err = target_yaw_unwrapped_ - yaw_unwrapped_;
        const double abs_err = std::abs(err);
        const bool in_tol = abs_err < yaw_tolerance_;
        yaw_max_overshoot_ = std::max(yaw_max_overshoot_, -err * yaw_dir_);

        if ((now - yaw_start_time_).seconds() > yaw_timeout_) {
            if (abs_err < kRelaxedFactor * yaw_tolerance_) {
                finish_yaw("⚠️ Timeout, berhenti dekat target (sisa error di atas toleransi).");
            } else {
                abort_yaw("timeout");
            }
            return;
        }

        // ---- Verifikasi arah umpan balik: robot harus MENDEKAT ke target ----
        if (!yaw_sign_verified_ && abs_err <= yaw_start_abs_err_ - kVerifyRad) {
            yaw_sign_verified_ = true;
            RCLCPP_INFO(get_logger(), "Arah putar terverifikasi (yaw_invert_cmd=%s)",
                        yaw_cmd_sign_ < 0.0 ? "true" : "false");
        }

        // ---- Deteksi menjauh dari target (umpan balik terbalik / runaway) ----
        if (abs_err > yaw_start_abs_err_ + kDivergeRad) {
            if (yaw_auto_sign_ && !yaw_sign_verified_ && !yaw_sign_flipped_) {
                yaw_cmd_sign_ = -yaw_cmd_sign_;
                yaw_sign_flipped_ = true;
                yaw_start_abs_err_ = abs_err; // referensi baru setelah dibalik
                yaw_integral_ = 0.0;
                yaw_stall_time_ = 0.0;
                publish_stop();
                RCLCPP_WARN(get_logger(),
                    "Robot MENJAUH dari target -> arah angular.z terbalik terhadap odom. "
                    "Otomatis dibalik. Agar permanen set param yaw_invert_cmd:=%s",
                    yaw_cmd_sign_ < 0.0 ? "true" : "false");
                return;
            }
            abort_yaw("yaw menjauh dari target. Periksa arah odom/angular.z (yaw_invert_cmd) dan kualitas odom");
            return;
        }

        // ---- Deteksi selesai ----
        if (in_tol) {
            if (!yaw_in_tol_) {
                yaw_in_tol_ = true;
                yaw_in_tol_since_ = now;
            }
            const double held = (now - yaw_in_tol_since_).seconds();
            const double settle_max = kSettleMax;
            if (held >= yaw_settle_time_ &&
                (std::abs(yaw_rate_f_) < kYawRateStill || held >= std::max(yaw_settle_time_, settle_max))) {
                finish_yaw("✅ Putar Selesai!");
                return;
            }
        } else {
            yaw_in_tol_ = false;
        }

        // ---- Deteksi macet: robot tidak bergerak padahal belum sampai ----
        if (!in_tol && std::abs(yaw_rate_f_) < kStallRate) {
            yaw_stall_time_ += dt;
        } else {
            yaw_stall_time_ = 0.0;
        }
        // Robot berhenti dekat target tapi tidak masuk toleransi (deadband motor) -> jangan hunting terus
        if (!in_tol && yaw_stall_time_ >= kRelaxedStall && abs_err < kRelaxedFactor * yaw_tolerance_) {
            finish_yaw("⚠️ Berhenti dekat target, sisa error di atas toleransi (deadband motor? naikkan min_yaw_rate).");
            return;
        }
        if (yaw_stall_time_ > kStallAbort) {
            abort_yaw("robot tidak berputar / yaw odom tidak berubah. Cek driver cmd_vel dan odom");
            return;
        }
        // Dorongan bertahap untuk melawan gesekan statis
        const double boost_cap = std::max(0.0, max_yaw_rate_ - min_yaw_rate_);
        const double boost = std::clamp(kStallRamp * (yaw_stall_time_ - kStallDelay), 0.0, boost_cap);

        // ---- Integral (hanya dekat target, anti-windup) ----
        if (ki_yaw_ > 0.0 && abs_err < yaw_integral_zone_) {
            yaw_integral_ = std::clamp(yaw_integral_ + err * dt, -integral_limit_, integral_limit_);
        } else {
            yaw_integral_ = 0.0;
        }

        // ---- Error prediksi: memperhitungkan lag (perintah -> gerak -> odom) ----
        // Robot masih akan berputar sejauh (rate * lag) walau perintah sudah dikurangi.
        const double err_pred = err - yaw_rate_f_ * yaw_lag_;

        // ---- PID: P (pada error prediksi) + I, D memakai kecepatan sudut terukur ----
        double wz = kp_yaw_ * err_pred + ki_yaw_ * yaw_integral_ - kd_yaw_ * yaw_rate_f_;

        // ---- Profil pengereman yang memperhitungkan lag ----
        // Kecepatan maks agar bisa berhenti tepat di target:
        //   v = sqrt((a*lag)^2 + 2*a*err) - a*lag
        if (wz * err > 0.0) {
            double v_lim = max_yaw_rate_;
            const double a = kDecelMargin * yaw_accel_;
            if (a > 0.0) {
                const double a_lag = a * yaw_lag_;
                v_lim = std::min(v_lim, std::sqrt(a_lag * a_lag + 2.0 * a * abs_err) - a_lag);
            }
            wz = std::copysign(std::min(std::abs(wz), v_lim), err);
        } else {
            // Diprediksi akan melewati target: lepas gas (0), JANGAN dibalik arah (tanpa reverse),
            // sebab reverse + lag membuat robot undershoot lalu bergoyang.
            wz = 0.0;
        }

        // ---- Kecepatan minimum (deadband) di luar toleransi ----
        // Tidak dipaksa jika prediksi sudah masuk toleransi (biarkan meluncur berhenti, tidak overshoot)
        const bool pred_in_tol = std::abs(err_pred) < yaw_tolerance_;
        const double floor_rate = min_yaw_rate_ + boost;
        if (!in_tol && !pred_in_tol && err_pred * err > 0.0 &&
            floor_rate > 0.0 && std::abs(wz) < floor_rate && wz * err >= 0.0) {
            wz = std::copysign(floor_rate, err);
        }

        // ---- Batas percepatan sudut ----
        if (yaw_accel_ > 0.0) {
            const double max_d = yaw_accel_ * dt;
            wz = std::clamp(wz, cmd_wz_ - max_d, cmd_wz_ + max_d);
        }
        cmd_wz_ = wz;

        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.angular.z = yaw_cmd_sign_ * wz;
        pub_cmd_->publish(twist);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "Putar: err %.1f deg | rate %.2f rad/s | cmd wz=%.2f",
            err * 180.0 / M_PI, yaw_rate_f_, twist.angular.z);
    }

    void control_loop() {
        if (!has_odom_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Menunggu /tr/odom...");
            return;
        }

        const rclcpp::Time now = this->now();
        if ((now - last_odom_time_).seconds() > odom_timeout_) {
            publish_stop();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odom basi, robot stop");
            return;
        }

        // MODE 1: PUTAR DI TEMPAT
        if (has_yaw_target_) {
            yaw_control_step(now);
            return;
        }

        // Setelah selesai/batal, kirim stop beberapa kali agar pasti berhenti
        if (!has_target_) {
            if (stop_burst_ > 0) {
                --stop_burst_;
                pub_cmd_->publish(geometry_msgs::msg::Twist());
            }
            return;
        }

        // MODE 2: NAIK KE TARGET POSISI (X,Y)
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
            RCLCPP_INFO(get_logger(), "✅ Target Posisi Tercapai! (%.2f, %.2f)", robot_x_, robot_y_);
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
            twist.angular.z = yaw_cmd_sign_ *
                std::clamp(kp_yaw_ * yaw_err, -max_yaw_rate_, max_yaw_rate_);
        } else {
            twist.angular.z = 0.0;
        }

        pub_cmd_->publish(twist);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "Dist: %.2fm | Cmd: vx=%.2f, vy=%.2f",
            distance, twist.linear.x, twist.linear.y);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_target_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_yaw_target_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_rotate_by_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemiAutoController>());
    rclcpp::shutdown();
    return 0;
}