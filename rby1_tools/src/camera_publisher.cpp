// camera_publisher: a webcam (OpenCV), or an image or video file, as ROS topics for
// Isaac ROS AprilTag in the container -- the camera stays on the host, only images
// cross over. Settings: config/camera/webcam.yaml (`source` picks the device or file).
//
//   ros2 launch rby1_tools camera.launch.py camera:=webcam [camera_name:=camera] [mount:=head_default]
//
// Publishes the image (bgr8) on /<camera_name>/rgb/raw -- with `compressed`, as JPEG on
// /<camera_name>/rgb/compressed instead -- and its CameraInfo on /<camera_name>/rgb/info,
// in <camera_name>_optical_frame; the mount (`mount` in camera_mount.yaml) as static TF.
// The camera model comes from `intrinsics_file` when one is named (a calibration in
// config/camera/intrinsics/, camera_ws or ROS layout), else from `horizontal_fov` as a
// distortion-free pinhole.
#include <chrono>
#include <memory>
#include <string>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>   // Jazzy
#else
#include <cv_bridge/cv_bridge.h>     // Humble
#endif
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "rby1_tools/camera_model.hpp"
#include "rby1_tools/camera_config.hpp"
#include "rby1_tools/image_publisher.hpp"

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : rclcpp::Node("camera_publisher") {
    const auto source = declare_parameter("source", "0");
    width_ = static_cast<int>(declare_parameter("width", 1280));
    height_ = static_cast<int>(declare_parameter("height", 720));
    const double fps = declare_parameter("fps", 30.0);
    const auto camera_name = declare_parameter("camera_name", "camera");
    const auto mount = declare_parameter("mount", "head_default");
    frame_id_ = rby1_tools::frame_name(camera_name, "optical_frame");
    const double fov = declare_parameter("horizontal_fov", 69.0);
    const auto intrinsics_file = rby1_tools::intrinsics_path(declare_parameter("intrinsics_file", ""));
    const bool auto_exposure = declare_parameter("auto_exposure", true);
    const double exposure = declare_parameter("exposure", 100.0);  // the driver's units (V4L2: 100 us)
    // Reliable: Isaac ROS subscribes reliably and would get nothing from a best-effort
    // publisher; a reliable one serves best-effort subscribers too.
    const bool compressed = declare_parameter("compressed", false);
    const auto stream = rby1_tools::stream_topic(camera_name, "rgb");
    image_pub_ = rby1_tools::image_publisher(this, stream, compressed);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(stream + "/info", 5);
    tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    tf_->sendTransform(rby1_tools::mount_transforms(this, camera_name, mount));

    source_ = rby1_tools::parse_source(source);
    if (source_.kind == rby1_tools::Source::Kind::image) {
      still_ = cv::imread(source_.path, cv::IMREAD_COLOR);
      if (still_.empty()) throw std::runtime_error("cannot read image " + source_.path);
      width_ = still_.cols;
      height_ = still_.rows;
    } else {
      const bool opened = source_.kind == rby1_tools::Source::Kind::device
                            ? capture_.open(source_.index, cv::CAP_V4L2) : capture_.open(source_.path);
      if (!opened) {
        throw std::runtime_error("cannot open " + source_.path +
                                 (source_.kind == rby1_tools::Source::Kind::device
                                    ? " as a webcam -- is one plugged in (ls /dev/video*), and is this user in "
                                      "the video group? A RealSense has /dev/video* too but does not open this "
                                      "way: start it with camera:=realsense" : ""));
      }
      if (source_.kind == rby1_tools::Source::Kind::device) {
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

    if (!intrinsics_file.empty()) {
      info_ = rby1_tools::camera_info(rby1_tools::read_intrinsics(intrinsics_file), width_,
                                                 height_, frame_id_);
      RCLCPP_INFO(get_logger(), "camera model from %s (fx %.1f fy %.1f)", intrinsics_file.c_str(), info_.k[0],
                  info_.k[4]);
    } else {
      info_ = rby1_tools::approximate_camera_info(width_, height_, fov, frame_id_);
      RCLCPP_WARN(get_logger(), "no calibration (intrinsics_file empty): a pinhole from horizontal_fov %.1f "
                                "deg; marker distances are approximate until the camera is calibrated", fov);
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / fps), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "publishing %s (%dx%d) at %.0f Hz as %s and %s/info in %s (mount: %s)",
                source_.path.c_str(), width_, height_, fps, rby1_tools::image_topic(stream, compressed).c_str(),
                stream.c_str(), frame_id_.c_str(), rby1_tools::mounted(mount) ? mount.c_str() : "none");
  }

private:
  void tick() {
    cv::Mat frame;
    if (source_.kind == rby1_tools::Source::Kind::image) {
      frame = still_;
    } else if (!capture_.read(frame) || frame.empty()) {
      if (source_.kind == rby1_tools::Source::Kind::video) {
        capture_.set(cv::CAP_PROP_POS_FRAMES, 0);  // loop the file
        return;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no frame from %s", source_.path.c_str());
      return;
    }
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = frame_id_;
    image_pub_.publish(cv_bridge::CvImage(header, "bgr8", frame).toImageMsg());
    info_.header = header;
    info_pub_->publish(info_);
  }

  rby1_tools::Source source_;
  cv::VideoCapture capture_;
  cv::Mat still_;
  int width_, height_;
  std::string frame_id_;
  sensor_msgs::msg::CameraInfo info_;
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_;
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
