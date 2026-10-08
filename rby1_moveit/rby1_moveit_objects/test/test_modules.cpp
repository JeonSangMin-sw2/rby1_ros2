// Module files and the planning-scene diffs they become; no ROS graph.
#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "rby1_moveit_objects/modules.hpp"

using namespace rby1_moveit_objects;

namespace {

std::string write(const std::string & name, const std::string & text) {
  const std::string path = testing::TempDir() + name;
  std::ofstream(path) << text;
  return path;
}

std::string failure(const std::string & text) {
  try {
    modules_from_file(write("bad_objects.yaml", text));
  } catch (const std::invalid_argument & error) {
    return error.what();
  }
  return "";
}

}  // namespace

TEST(Modules, TheShippedExampleLoadsWithItsMesh) {
  const auto modules = modules_from_file(EXAMPLE_OBJECTS);
  ASSERT_EQ(modules.size(), 3u);
  EXPECT_EQ(modules[0].spec.name, "head_camera");
  EXPECT_EQ(modules[0].mesh.rfind("file://", 0), 0u);  // relative path resolved
  EXPECT_TRUE(modules[0].attach);
  EXPECT_FALSE(modules[2].attach);
  const auto camera = collision_object(modules[0]);
  ASSERT_EQ(camera.meshes.size(), 1u);
  EXPECT_EQ(camera.meshes[0].triangles.size(), 12u);
  EXPECT_TRUE(camera.primitives.empty());
  EXPECT_EQ(camera.header.frame_id, "link_head_2");
  EXPECT_DOUBLE_EQ(camera.mesh_poses[0].position.x, 0.06);
}

TEST(Modules, ScaleShrinksTheMesh) {
  const auto mesh_file = (std::filesystem::path(EXAMPLE_OBJECTS).parent_path() / "meshes" / "camera_box.stl").string();
  const auto path = write("scaled.yaml", "objects:\n  - {name: m, frame: base, type: mesh, mesh: '" + mesh_file +
                                         "', scale: [0.5, 0.5, 0.5]}\n");
  const auto mesh = collision_object(modules_from_file(path)[0]).meshes[0];
  double top = 0.0;
  for (const auto & vertex : mesh.vertices) top = std::max(top, vertex.z);
  EXPECT_NEAR(top, 0.01, 1e-6);  // half of the 2 cm half-height
  std::remove(path.c_str());
}

TEST(Modules, AttachedGoOnTheirLinkWorldOnesIntoTheWorld) {
  const auto diff = add_diff(modules_from_file(EXAMPLE_OBJECTS));
  ASSERT_EQ(diff.robot_state.attached_collision_objects.size(), 2u);
  ASSERT_EQ(diff.world.collision_objects.size(), 1u);
  const auto & tool = diff.robot_state.attached_collision_objects[1];
  EXPECT_EQ(tool.link_name, "ee_right");
  EXPECT_EQ(tool.object.id, "tool_tip");
  EXPECT_EQ(tool.touch_links, (std::vector<std::string>{"ee_right", "gripper_right", "gripper_finger_r1",
                                                       "gripper_finger_r2", "link_right_arm_6"}));
  EXPECT_EQ(diff.robot_state.attached_collision_objects[0].touch_links,
            std::vector<std::string>{"link_head_2"});  // default: its own link
  EXPECT_EQ(diff.world.collision_objects[0].id, "workbench");
}

TEST(Modules, RemovingDetachesAndRemoves) {
  const auto diff = remove_diff(modules_from_file(EXAMPLE_OBJECTS));
  EXPECT_EQ(diff.robot_state.attached_collision_objects.size(), 2u);
  EXPECT_EQ(diff.world.collision_objects.size(), 3u);
  for (const auto & object : diff.world.collision_objects) {
    EXPECT_EQ(object.operation, moveit_msgs::msg::CollisionObject::REMOVE);
  }
}

TEST(Modules, MissingOnesAreFound) {
  const auto modules = modules_from_file(EXAMPLE_OBJECTS);
  moveit_msgs::msg::PlanningScene scene;
  moveit_msgs::msg::CollisionObject bench;
  bench.id = "workbench";
  scene.world.collision_objects.push_back(bench);
  moveit_msgs::msg::AttachedCollisionObject camera;
  camera.object.id = "head_camera";
  scene.robot_state.attached_collision_objects.push_back(camera);
  EXPECT_EQ(missing(modules, scene), std::vector<std::string>{"tool_tip"});
}

TEST(Modules, BadEntriesSayWhatIsWrong) {
  EXPECT_NE(failure("objects: []\n").find("non-empty"), std::string::npos);
  EXPECT_NE(failure("objects:\n  - {name: a, type: box, size: [1, 1, 1]}\n").find("needs frame"), std::string::npos);
  EXPECT_NE(failure("objects:\n  - {name: a, frame: base, type: mesh}\n").find("needs mesh"), std::string::npos);
  EXPECT_NE(failure("objects:\n  - {name: a, frame: base, type: mesh, mesh: nothing.stl}\n").find("not found"),
            std::string::npos);
  EXPECT_NE(failure("objects:\n  - {name: a, frame: base, type: cone}\n").find("type must be"), std::string::npos);
  EXPECT_NE(failure("objects:\n  - {name: a, frame: base, type: sphere, radius: 0.1}\n"
                    "  - {name: a, frame: base, type: sphere, radius: 0.1}\n").find("twice"),
            std::string::npos);
  EXPECT_NE(failure("objects:\n  - {name: a, frame: base, type: sphere, radius: 0.1, attach: maybe}\n").find("'a'"),
            std::string::npos);
}
