#include "rby1_additional_tools/camera_model.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace rby1_additional_tools {

namespace {

std::string lower_extension(const std::string & path) {
  const auto dot = path.find_last_of('.');
  if (dot == std::string::npos) return "";
  std::string ext = path.substr(dot + 1);
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
  return ext;
}

}  // namespace

Source parse_source(const std::string & source) {
  if (source.empty()) throw std::invalid_argument("source is empty: give a device (0, /dev/video0) or a file");
  if (std::all_of(source.begin(), source.end(), [](unsigned char c) { return std::isdigit(c); })) {
    return {Source::Kind::device, std::stoi(source), "/dev/video" + source};
  }
  if (source.rfind("/dev/video", 0) == 0) {
    const std::string number = source.substr(10);
    if (number.empty() || !std::all_of(number.begin(), number.end(), [](unsigned char c) { return std::isdigit(c); })) {
      throw std::invalid_argument("not a video device: " + source);
    }
    return {Source::Kind::device, std::stoi(number), source};
  }
  const auto ext = lower_extension(source);
  if (ext == "png" || ext == "jpg" || ext == "jpeg" || ext == "bmp") return {Source::Kind::image, -1, source};
  return {Source::Kind::video, -1, source};
}

sensor_msgs::msg::CameraInfo approximate_camera_info(int width, int height, double horizontal_fov_deg,
                                                     const std::string & frame_id) {
  if (width <= 0 || height <= 0) throw std::invalid_argument("image size must be positive");
  if (!(horizontal_fov_deg > 1.0 && horizontal_fov_deg < 179.0)) {
    throw std::invalid_argument("horizontal_fov must be between 1 and 179 degrees");
  }
  const double f = width / 2.0 / std::tan(horizontal_fov_deg * M_PI / 360.0);
  const double cx = (width - 1) / 2.0, cy = (height - 1) / 2.0;
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.width = static_cast<uint32_t>(width);
  info.height = static_cast<uint32_t>(height);
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

Intrinsics read_intrinsics(const std::string & path) {
  YAML::Node file;
  try {
    file = YAML::LoadFile(path);
  } catch (const std::exception & error) {
    throw std::runtime_error("cannot read intrinsics file " + path + ": " + error.what());
  }
  Intrinsics result;
  try {
    const auto matrix = file["camera_matrix"];
    if (!matrix) throw std::runtime_error("no camera_matrix");
    if (matrix.IsMap()) {  // ROS calibration
      result.width = file["image_width"].as<int>();
      result.height = file["image_height"].as<int>();
      const auto data = matrix["data"].as<std::vector<double>>();
      if (data.size() != 9) throw std::runtime_error("camera_matrix.data needs 9 values");
      std::copy(data.begin(), data.end(), result.k.begin());
      if (file["distortion_coefficients"]) {
        result.d = file["distortion_coefficients"]["data"].as<std::vector<double>>();
      }
      if (file["distortion_model"]) result.model = file["distortion_model"].as<std::string>();
    } else {  // camera_ws: nested rows
      result.width = file["width"].as<int>();
      result.height = file["height"].as<int>();
      const auto rows = matrix.as<std::vector<std::vector<double>>>();
      if (rows.size() != 3 || rows[0].size() != 3 || rows[1].size() != 3 || rows[2].size() != 3) {
        throw std::runtime_error("camera_matrix must be 3x3");
      }
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) result.k[r * 3 + c] = rows[r][c];
      }
      if (file["dist_coeffs"]) result.d = file["dist_coeffs"].as<std::vector<double>>();
    }
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("intrinsics file " + path + ": " + error.what());
  } catch (const std::runtime_error & error) {
    throw std::runtime_error("intrinsics file " + path + ": " + error.what());
  }
  if (result.width <= 0 || result.height <= 0 || !(result.k[0] > 0.0) || !(result.k[4] > 0.0)) {
    throw std::runtime_error("intrinsics file " + path + ": image size and focal lengths must be positive");
  }
  result.d.resize(std::max<size_t>(result.d.size(), 5), 0.0);
  return result;
}

sensor_msgs::msg::CameraInfo camera_info(const Intrinsics & intrinsics, int width, int height,
                                         const std::string & frame_id) {
  const double sx = static_cast<double>(width) / intrinsics.width;
  const double sy = static_cast<double>(height) / intrinsics.height;
  if (std::abs(sx - sy) > 1e-3 * std::max(sx, sy)) {
    throw std::runtime_error("the intrinsics are for " + std::to_string(intrinsics.width) + "x" +
                             std::to_string(intrinsics.height) + ", the image is " + std::to_string(width) + "x" +
                             std::to_string(height) + " (another aspect ratio): calibrate at this size");
  }
  const auto & k = intrinsics.k;
  const double fx = k[0] * sx, fy = k[4] * sy, cx = k[2] * sx, cy = k[5] * sy, skew = k[1] * sx;
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.width = static_cast<uint32_t>(width);
  info.height = static_cast<uint32_t>(height);
  info.distortion_model = intrinsics.model;
  info.d = intrinsics.d;
  info.k = {fx, skew, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {fx, skew, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

}  // namespace rby1_additional_tools
