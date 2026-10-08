// Camera sources and a pinhole model for cameras nobody has calibrated yet.
#pragma once

#include <array>
#include <string>
#include <variant>
#include <vector>

#include <sensor_msgs/msg/camera_info.hpp>

namespace rby1_additional_tools {

// What `source` names: a video device (index "0" or "/dev/video0") or a file -- an
// image, republished as it is, or a video.
struct Source {
  enum class Kind { device, image, video } kind;
  int index = -1;     // device
  std::string path;   // device node or file
};
Source parse_source(const std::string & source);

// A distortion-free pinhole CameraInfo from the horizontal field of view -- enough to
// start with; AprilTag distances are only as good as this guess until the camera is
// calibrated (use_custom_intrinsics with an intrinsics_file).
sensor_msgs::msg::CameraInfo approximate_camera_info(int width, int height, double horizontal_fov_deg,
                                                     const std::string & frame_id);

// A calibrated camera: 3x3 camera matrix (row-major) and plumb_bob distortion
// (k1, k2, p1, p2, k3) at the image size it was measured for.
struct Intrinsics {
  int width = 0, height = 0;
  std::array<double, 9> k{};
  std::vector<double> d;
  std::string model = "plumb_bob";
};

// An intrinsics YAML file, in either layout:
//   camera_ws (OpenCV):  width, height, camera_matrix: [[fx,0,cx],[0,fy,cy],[0,0,1]], dist_coeffs: [...]
//   ROS calibration:     image_width, image_height, camera_matrix: {data: [9]},
//                        distortion_model, distortion_coefficients: {data: [...]}
Intrinsics read_intrinsics(const std::string & path);

// The CameraInfo for an image of `width` x `height`: the camera matrix scaled if the
// file was measured at another size with the same aspect ratio (refused otherwise).
sensor_msgs::msg::CameraInfo camera_info(const Intrinsics & intrinsics, int width, int height,
                                         const std::string & frame_id);

}  // namespace rby1_additional_tools
