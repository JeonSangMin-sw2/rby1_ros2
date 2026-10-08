// camera_publisher: a webcam (OpenCV), or an image or video file, as ROS topics for
// Isaac ROS AprilTag in the container -- the camera stays on the host, only images
// cross over. Settings: config/webcam.yaml.
//
//   ros2 launch rby1_additional_tools camera.launch.py camera:=webcam [source:=/dev/video0]
//   ros2 launch rby1_additional_tools camera.launch.py camera:=file source:=/path/tag.png
//
// Publishes the image (bgr8) and camera_info on `image_topic` / `info_topic`. The
// camera model comes from `intrinsics_file` when `use_custom_intrinsics` (a
// calibration, camera_ws or ROS layout), else from `horizontal_fov` as a
// distortion-free pinhole.
#include <chrono>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rby1_additional_tools/camera_model.hpp"

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : rclcpp::Node("camera_publisher") {
    const auto source = declare_parameter("source", "0");
    width_ = static_cast<int>(declare_parameter("width", 1280));
    height_ = static_cast<int>(declare_parameter("height", 720));
    const double fps = declare_parameter("fps", 30.0);
    frame_id_ = declare_parameter("frame_id", "camera_optical_frame");
    const double fov = declare_parameter("horizontal_fov", 69.0);
    const bool custom = declare_parameter("use_custom_intrinsics", false);
    const auto intrinsics_file = declare_parameter("intrinsics_file", "");
    const bool auto_exposure = declare_parameter("auto_exposure", true);
    const double exposure = declare_parameter("exposure", 100.0);  // the driver's units (V4L2: 100 us)
    // Reliable: Isaac ROS subscribes reliably and would get nothing from a best-effort
    // publisher; a reliable one serves best-effort subscribers too.
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(declare_parameter("image_topic", "/camera/image_raw"), 5);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(declare_parameter("info_topic", "/camera/camera_info"),
                                                               5);

    source_ = rby1_additional_tools::parse_source(source);
    if (source_.kind == rby1_additional_tools::Source::Kind::image) {
      still_ = cv::imread(source_.path, cv::IMREAD_COLOR);
      if (still_.empty()) throw std::runtime_error("cannot read image " + source_.path);
      width_ = still_.cols;
      height_ = still_.rows;
    } else {
      const bool opened = source_.kind == rby1_additional_tools::Source::Kind::device
                            ? capture_.open(source_.index, cv::CAP_V4L2) : capture_.open(source_.path);
      if (!opened) {
        throw std::runtime_error("cannot open " + source_.path +
                                 (source_.kind == rby1_additional_tools::Source::Kind::device
                                    ? " -- is a camera plugged in (ls /dev/video*), and is this user in the "
                                      "video group?" : ""));
      }
      if (source_.kind == rby1_additional_tools::Source::Kind::device) {
        capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        capture_.set(cv::CAP_PROP_FPS, fps);
        // V4L2 through OpenCV: 3 = auto (aperture priority), 1 = manual.
        if (!capture_.set(cv::CAP_PROP_AUTO_EXPOSURE, auto_exposure ? 3 : 1) ||
            (!auto_exposure && !capture_.set(cv::CAP_PROP_EXPOSURE, exposure))) {
          RCLCPP_WARN(get_logger(), "this camera did not take the exposure setting (auto_exposure %s, exposure %.0f)",
                      auto_exposure ? "true" : "false", exposure);
        }
      }
      width_ = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
      height_ = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
    }

    if (custom) {
      if (intrinsics_file.empty()) {
        throw std::runtime_error("use_custom_intrinsics is true but intrinsics_file is empty: give the calibration "
                                 "YAML (camera_ws or ROS layout)");
      }
      info_ = rby1_additional_tools::camera_info(rby1_additional_tools::read_intrinsics(intrinsics_file), width_,
                                                 height_, frame_id_);
      RCLCPP_INFO(get_logger(), "camera model from %s (fx %.1f fy %.1f)", intrinsics_file.c_str(), info_.k[0],
                  info_.k[4]);
    } else {
      info_ = rby1_additional_tools::approximate_camera_info(width_, height_, fov, frame_id_);
      RCLCPP_WARN(get_logger(), "no calibration (use_custom_intrinsics false): a pinhole from horizontal_fov %.1f "
                                "deg; marker distances are approximate until the camera is calibrated", fov);
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / fps), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "publishing %s (%dx%d) at %.0f Hz as %s in %s", source_.path.c_str(), width_,
                height_, fps, image_pub_->get_topic_name(), frame_id_.c_str());
  }

private:
  void tick() {
    cv::Mat frame;
    if (source_.kind == rby1_additional_tools::Source::Kind::image) {
      frame = still_;
    } else if (!capture_.read(frame) || frame.empty()) {
      if (source_.kind == rby1_additional_tools::Source::Kind::video) {
        capture_.set(cv::CAP_PROP_POS_FRAMES, 0);  // loop the file
        return;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no frame from %s", source_.path.c_str());
      return;
    }
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = frame_id_;
    image_pub_->publish(*cv_bridge::CvImage(header, "bgr8", frame).toImageMsg());
    info_.header = header;
    info_pub_->publish(info_);
  }

  rby1_additional_tools::Source source_;
  cv::VideoCapture capture_;
  cv::Mat still_;
  int width_, height_;
  std::string frame_id_;
  sensor_msgs::msg::CameraInfo info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<CameraPublisher>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("camera_publisher"), "%s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
