// object_detector_br.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

using Image = sensor_msgs::msg::Image;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using namespace cv;

struct Target {
    std::string label;
    std::vector<std::pair<Scalar, Scalar>> ranges;
    Scalar color;
    double real_w, real_h, min_aspect, max_aspect, min_fill, min_ratio, max_ratio;
    // Label target lain yang punya prioritas lebih tinggi.
    // Piksel & kotak milik target tsb tidak boleh dipakai target ini.
    std::vector<std::string> exclude;
};

struct Detection {
    bool valid = false;
    Rect rect;
    double range = 1e9;
    bool from_lidar = false;
};

class ObjectDetectorBR : public rclcpp::Node {
public:
    ObjectDetectorBR() : Node("object_detector_br") {
        cam_fov_ = declare_parameter("camera_fov", 1.5);
        lidar_y_ = declare_parameter("lidar_to_camera_y", 0.08);
        max_dist_ = declare_parameter("max_distance", 1.5);
        max_ang_ = declare_parameter("max_angle_deg", 25.0);
        grasp_dist_ = declare_parameter("grasp_distance", 0.3);
        min_area_ = declare_parameter("min_area", 300.0);
        fallback_ = declare_parameter("use_camera_fallback", true);
        flip_y_ = declare_parameter("flip_lidar_y", false);
        debug_ = declare_parameter("publish_debug_mask", false);
        overlap_thr_ = declare_parameter("overlap_suppress_ratio", 0.3);

        build_targets();
        k_close_ = getStructuringElement(MORPH_RECT, Size(5, 5));
        k_open_ = getStructuringElement(MORPH_RECT, Size(3, 3));
        k_excl_ = getStructuringElement(MORPH_RECT, Size(9, 9));

        // TOPIK BR
        // Kamera dipublish otomatis oleh webots_ros2_driver (frame_id "camera"),
        // sama seperti TR (/tr_robot/camera/image_color). Header image dipakai
        // ulang untuk /br/camera/detected, jadi cocok dengan /br_robot/camera/camera_info.
        sub_img_ = create_subscription<Image>(
            "/br_robot/camera/image_color", 10,
            std::bind(&ObjectDetectorBR::img_cb, this, std::placeholders::_1));
        sub_lidar_ = create_subscription<PointCloud2>(
            "/br/lidar/point_cloud", 10,
            std::bind(&ObjectDetectorBR::lidar_cb, this, std::placeholders::_1));

        pub_ = create_publisher<Image>("/br/camera/detected", 10);
        if (debug_) pub_mask_ = create_publisher<Image>("/br/camera/debug_mask", 10);

        RCLCPP_INFO(get_logger(),
            "OBJECT DETECTOR BR V8 (ANTI-SKY, SKYBLOCK/MUSTIKA SEPARATED) STARTED");
    }

private:
    void build_targets() {
        targets_ = {
            {"EarthBlue",
                {{Scalar(102, 170, 15), Scalar(130, 255, 200)}},
                Scalar(255, 0, 0),
                0.35, 0.35, 0.6, 1.2, 0.6, 0.6, 1.8, {}},
            {"EarthRed",
                {{Scalar(0, 80, 10), Scalar(12, 255, 255)},
                 {Scalar(168, 80, 10), Scalar(180, 255, 255)}},
                Scalar(0, 0, 255),
                0.35, 0.35, 0.55, 1.45, 0.45, 0.6, 1.8, {}},
            {"SkyBlock",
                {{Scalar(22, 200, 150), Scalar(35, 255, 255)}},
                Scalar(0, 255, 255),
                0.20, 0.20, 0.7, 1.3, 0.7, 0.6, 1.8, {}},
            // H atas diturunkan 28 -> 20 (tidak menyentuh SkyBlock 22-35),
            // min_aspect 0.8 -> 1.1 (Mustika tinggi, SkyBlock persegi),
            // exclude SkyBlock (mask & kotak SkyBlock menang).
            {"Mustika",
                {{Scalar(3, 80, 10), Scalar(20, 255, 220)}},
                Scalar(0, 165, 255),
                0.27, 0.50, 1.1, 4.5, 0.40, 0.4, 2.5, {"SkyBlock"}}
        };
    }

    int find_idx(const std::string &label) const {
        for (size_t i = 0; i < targets_.size(); ++i)
            if (targets_[i].label == label) return static_cast<int>(i);
        return -1;
    }

    void lidar_cb(const PointCloud2::SharedPtr msg) { last_cloud_ = msg; }

    bool get_lidar_dist(double lo, double hi, double &range, double &depth) const {
        if (!last_cloud_) return false;

        sensor_msgs::PointCloud2ConstIterator<float> ix(*last_cloud_, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iy(*last_cloud_, "y");

        double best = 1e9;
        double y_sign = flip_y_ ? -1.0 : 1.0;

        for (size_t i = 0; i < last_cloud_->width * last_cloud_->height; ++i, ++ix, ++iy) {
            if (!std::isfinite(*ix) || !std::isfinite(*iy)) continue;
            double x = *ix, y = y_sign * (*iy) + lidar_y_;
            if (x <= 0.05) continue;
            double ang = std::atan2(y, x) * 180.0 / M_PI;
            if (ang >= lo && ang <= hi && std::hypot(x, y) < best) {
                best = std::hypot(x, y); depth = x;
            }
        }
        if (best > 1e8) return false;
        range = best;
        return true;
    }

    double px2ang(double px) const {
        return -std::atan2((px - cx_), f_px_) * 180.0 / M_PI;
    }

    Rect tall_core(const Mat &blob) const {
        std::vector<int> col(blob.cols, 0);
        int best_c = 0;
        for (int x = 0; x < blob.cols; ++x) {
            col[x] = countNonZero(blob.col(x));
            if (col[x] > col[best_c]) best_c = x;
        }
        if (col[best_c] == 0) return Rect();

        int thr = std::max(1, static_cast<int>(0.6 * col[best_c]));
        int x0 = best_c, x1 = best_c;
        while (x0 > 0 && col[x0 - 1] >= thr) --x0;
        while (x1 < blob.cols - 1 && col[x1 + 1] >= thr) ++x1;

        Mat sub = blob(Rect(x0, 0, x1 - x0 + 1, blob.rows));
        int row_thr = std::max(1, static_cast<int>(0.35 * sub.cols));
        int y0 = -1, y1 = -1;
        for (int y = 0; y < blob.rows; ++y) {
            if (countNonZero(sub.row(y)) >= row_thr) {
                if (y0 < 0) y0 = y;
                y1 = y;
            }
        }
        return (y0 < 0) ? Rect() : Rect(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
    }

    bool check_measure(Rect &r, const Target &t, double &range, bool &from_lidar) const {
        double a_c = px2ang(r.x + r.width / 2.0);
        if (std::abs(a_c) > max_ang_) return false;

        double a_l = px2ang(r.x), a_r = px2ang(r.x + r.width);
        double mid = 0.5 * ((a_l - 0.15 * (a_l - a_r)) + (a_r + 0.15 * (a_l - a_r)));
        double half = std::max(
            0.5 * ((a_l - 0.15 * (a_l - a_r)) - (a_r + 0.15 * (a_l - a_r))), 1.5);

        double lr, ld;
        if (get_lidar_dist(mid - half, mid + half, lr, ld) && ld > 0.05) {
            double ratio = r.width / (f_px_ * t.real_w / ld);
            if (ratio >= t.min_ratio && ratio <= t.max_ratio) {
                int max_h = std::lround(1.2 * (f_px_ * t.real_h / ld));
                if (r.height > max_h) r.height = max_h;
                range = lr; from_lidar = true; return true;
            }
        }
        if (fallback_ && r.width > 0) {
            range = (f_px_ * t.real_w / r.width) / std::cos(a_c * M_PI / 180.0);
            from_lidar = false;
            return range > 0.05;
        }
        return false;
    }

    void draw_and_log(Mat &img, Rect r, const Target &t, double range, bool from_lidar) {
        rectangle(img, r, t.color, 2);
        std::string dist = (from_lidar ? "" : "~") + std::to_string(range).substr(0, 4);
        Point org(r.x, std::max(r.y - 8, 15));

        if (range <= grasp_dist_) {
            putText(img, t.label + " | SIAP CAPIT! (" + dist + "m)",
                    org, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "[SIAP CAPIT BR] %s: %.2fm", t.label.c_str(), range);
        } else {
            putText(img, t.label + " | " + dist + "m",
                    org, FONT_HERSHEY_SIMPLEX, 0.6, t.color, 2);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "%s: %.2fm (%s)", t.label.c_str(), range,
                from_lidar ? "lidar" : "kamera");
        }
    }

    // Bikin mask warna mentah (belum kena aturan exclude)
    Mat build_mask(const Mat &hsv, const Target &t) const {
        Mat mask = Mat::zeros(hsv.size(), CV_8U), tmp;
        for (const auto &rg : t.ranges) {
            inRange(hsv, rg.first, rg.second, tmp);
            bitwise_or(mask, tmp, mask);
        }
        GaussianBlur(mask, mask, Size(5, 5), 0);
        morphologyEx(mask, mask, MORPH_CLOSE, k_close_);
        morphologyEx(mask, mask, MORPH_OPEN, k_open_);
        return mask;
    }

    // Cari kandidat terbaik dari satu mask (tanpa menggambar)
    Detection detect_target(const Mat &mask, const Target &t) const {
        Detection best;

        std::vector<std::vector<Point>> contours;
        findContours(mask.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); ++i) {
            if (contourArea(contours[i]) < min_area_) continue;
            Rect box = boundingRect(contours[i]);
            Mat blob = Mat::zeros(box.size(), CV_8U);
            drawContours(blob, contours, static_cast<int>(i), Scalar(255),
                         FILLED, LINE_8, noArray(), INT_MAX, Point(-box.x, -box.y));

            Rect core = (t.label == "Mustika")
                         ? Rect(0, 0, box.width, box.height)
                         : tall_core(blob);
            if (core.width < 12 || core.height < 12 ||
                (double)core.height / core.width < t.min_aspect ||
                (double)core.height / core.width > t.max_aspect) continue;

            double fill = (t.label == "Mustika")
                          ? 1.0
                          : countNonZero(blob(core)) / (double)core.area();
            if (fill < t.min_fill) continue;

            Rect r = core + box.tl();
            double range;
            bool from_lidar;
            if (check_measure(r, t, range, from_lidar) &&
                range > 0.0 && range <= max_dist_ && range < best.range) {
                best.valid = true;
                best.rect = r;
                best.range = range;
                best.from_lidar = from_lidar;
            }
        }
        return best;
    }

    void img_cb(const Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (...) { return; }

        Mat img = cv_ptr->image;
        cx_ = img.cols / 2.0;
        f_px_ = cx_ / std::tan(cam_fov_ / 2.0);

        Mat hsv, debug;
        cvtColor(img, hsv, COLOR_BGR2HSV);
        if (debug_) debug = Mat::zeros(img.size(), CV_8UC3);

        const size_t n = targets_.size();

        // 1) Mask mentah semua target
        std::vector<Mat> raw(n), masks(n);
        for (size_t i = 0; i < n; ++i) {
            raw[i] = build_mask(hsv, targets_[i]);
            masks[i] = raw[i].clone();
        }

        // 2) Buang piksel milik target prioritas dari mask target ini
        for (size_t i = 0; i < n; ++i) {
            for (const auto &ex_label : targets_[i].exclude) {
                int j = find_idx(ex_label);
                if (j < 0) continue;
                Mat ex;
                threshold(raw[j], ex, 127, 255, THRESH_BINARY);
                dilate(ex, ex, k_excl_); // ikut buang tepi/bayangan objek prioritas
                masks[i].setTo(0, ex);
            }
            if (!targets_[i].exclude.empty())
                morphologyEx(masks[i], masks[i], MORPH_OPEN, k_open_); // bersihin sisa tipis
            if (debug_) debug.setTo(targets_[i].color, masks[i]);
        }

        // 3) Deteksi
        std::vector<Detection> dets(n);
        for (size_t i = 0; i < n; ++i) dets[i] = detect_target(masks[i], targets_[i]);

        // 4) Jaring pengaman: kalau kotak tumpang tindih dengan target prioritas, buang
        for (size_t i = 0; i < n; ++i) {
            if (!dets[i].valid) continue;
            for (const auto &ex_label : targets_[i].exclude) {
                int j = find_idx(ex_label);
                if (j < 0 || !dets[j].valid) continue;
                double inter = (dets[i].rect & dets[j].rect).area();
                if (inter / std::max(1, dets[i].rect.area()) > overlap_thr_) {
                    dets[i].valid = false;
                    break;
                }
            }
        }

        // 5) Gambar
        for (size_t i = 0; i < n; ++i)
            if (dets[i].valid)
                draw_and_log(img, dets[i].rect, targets_[i],
                             dets[i].range, dets[i].from_lidar);

        pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg());
        if (debug_)
            pub_mask_->publish(*cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg());
    }

    std::vector<Target> targets_;
    Mat k_close_, k_open_, k_excl_;
    std::shared_ptr<PointCloud2> last_cloud_;

    double cam_fov_, lidar_y_, max_dist_, max_ang_, grasp_dist_, min_area_, overlap_thr_;
    bool fallback_, flip_y_, debug_;
    double cx_ = 320.0, f_px_ = 343.5;

    rclcpp::Subscription<Image>::SharedPtr sub_img_;
    rclcpp::Subscription<PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Publisher<Image>::SharedPtr pub_, pub_mask_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectorBR>());
    rclcpp::shutdown();
    return 0;
}