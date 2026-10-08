// Camera sources and the uncalibrated pinhole; no ROS graph, no camera.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "rby1_additional_tools/camera_model.hpp"

using namespace rby1_additional_tools;

TEST(Source, DevicesAndFilesAreTold) {
  EXPECT_EQ(parse_source("0").kind, Source::Kind::device);
  EXPECT_EQ(parse_source("2").index, 2);
  EXPECT_EQ(parse_source("/dev/video4").index, 4);
  EXPECT_EQ(parse_source("/tmp/tag.PNG").kind, Source::Kind::image);
  EXPECT_EQ(parse_source("/tmp/walk.mp4").kind, Source::Kind::video);
  EXPECT_THROW(parse_source(""), std::invalid_argument);
  EXPECT_THROW(parse_source("/dev/videoX"), std::invalid_argument);
}

TEST(Pinhole, FocalLengthFollowsTheFieldOfView) {
  const auto info = approximate_camera_info(1280, 720, 90.0, "camera_optical_frame");
  EXPECT_NEAR(info.k[0], 640.0, 1e-9);  // 90 deg across 1280 px: f = 640
  EXPECT_NEAR(info.k[4], 640.0, 1e-9);
  EXPECT_NEAR(info.k[2], 639.5, 1e-9);
  EXPECT_NEAR(info.k[5], 359.5, 1e-9);
  EXPECT_EQ(info.p[0], info.k[0]);
  EXPECT_EQ(info.header.frame_id, "camera_optical_frame");
  EXPECT_EQ(info.width, 1280u);
  EXPECT_THROW(approximate_camera_info(1280, 720, 0.5, "c"), std::invalid_argument);
  EXPECT_THROW(approximate_camera_info(0, 720, 60.0, "c"), std::invalid_argument);
}

namespace {

std::string write_file(const std::string & name, const std::string & text) {
  const auto path = (std::filesystem::temp_directory_path() / name).string();
  std::ofstream(path) << text;
  return path;
}

}  // namespace

TEST(Intrinsics, CameraWsLayoutIsRead) {
  // As camera_ws writes it (config/camera_intrinsics_d435.yaml).
  const auto path = write_file("rby1_intrinsics_camera_ws.yaml", R"(camera_matrix:
- - 906.5679246553677
  - 0.0
  - 639.3426898074739
- - 0.0
  - 907.2519543479634
  - 383.9957721342101
- - 0.0
  - 0.0
  - 1.0
device_name: D435
dist_coeffs:
- 0.12183897591222062
- -0.37092493575481705
- 0.0010067941803011953
- 0.001299501989543505
- 0.33076464260693217
height: 720
rms_error: 0.19824150874733465
width: 1280
)");
  const auto intrinsics = read_intrinsics(path);
  EXPECT_EQ(intrinsics.width, 1280);
  EXPECT_NEAR(intrinsics.k[0], 906.5679246553677, 1e-12);
  EXPECT_NEAR(intrinsics.k[5], 383.9957721342101, 1e-12);
  ASSERT_EQ(intrinsics.d.size(), 5u);
  EXPECT_NEAR(intrinsics.d[4], 0.33076464260693217, 1e-12);
  const auto info = camera_info(intrinsics, 1280, 720, "camera_optical_frame");
  EXPECT_EQ(info.distortion_model, "plumb_bob");
  EXPECT_NEAR(info.p[0], info.k[0], 1e-12);
}

TEST(Intrinsics, RosCalibrationLayoutIsRead) {
  const auto path = write_file("rby1_intrinsics_ros.yaml", R"(image_width: 640
image_height: 480
camera_name: webcam
camera_matrix:
  rows: 3
  cols: 3
  data: [600.0, 0.0, 320.0, 0.0, 601.0, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.1, -0.2, 0.0, 0.0, 0.05]
)");
  const auto intrinsics = read_intrinsics(path);
  EXPECT_EQ(intrinsics.height, 480);
  EXPECT_NEAR(intrinsics.k[4], 601.0, 1e-12);
  EXPECT_NEAR(intrinsics.d[1], -0.2, 1e-12);
}

TEST(Intrinsics, ScaledToAnotherSizeOfTheSameShape) {
  Intrinsics intrinsics;
  intrinsics.width = 1280;
  intrinsics.height = 720;
  intrinsics.k = {900.0, 0.0, 640.0, 0.0, 902.0, 360.0, 0.0, 0.0, 1.0};
  intrinsics.d = {0.1, 0.0, 0.0, 0.0, 0.0};
  const auto half = camera_info(intrinsics, 640, 360, "c");
  EXPECT_NEAR(half.k[0], 450.0, 1e-12);
  EXPECT_NEAR(half.k[5], 180.0, 1e-12);
  EXPECT_NEAR(half.d[0], 0.1, 1e-12);  // distortion does not scale
  EXPECT_THROW(camera_info(intrinsics, 640, 480, "c"), std::runtime_error);  // 16:9 file, 4:3 image
}

TEST(Intrinsics, BadFilesSayWhy) {
  EXPECT_THROW(read_intrinsics("/nonexistent/intrinsics.yaml"), std::runtime_error);
  const auto path = write_file("rby1_intrinsics_bad.yaml", "width: 1280\nheight: 720\ncamera_matrix: [[1, 0], [0, 1]]\n");
  EXPECT_THROW(read_intrinsics(path), std::runtime_error);
}
