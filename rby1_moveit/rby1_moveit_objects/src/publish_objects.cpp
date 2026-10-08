// publish_objects: put the modules of a YAML file into the MoveIt planning scene and
// keep them there.
//
//   ros2 run rby1_moveit_objects publish_objects --ros-args -p config:=/path/objects.yaml
//   ros2 launch rby1_moveit_objects objects.launch.py [config:=...]
//
// Every `check_period` s it looks whether they are all still in the scene (a restarted
// move_group starts empty) and puts them back if not. On Ctrl+C they are taken away
// again (remove_on_exit), so modules exist only while this runs.
#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rby1_moveit_objects/modules.hpp"
#include "rby1_moveit_objects/scene.hpp"

namespace {
std::atomic<bool> stop{false};
}  // namespace

int main(int argc, char ** argv) {
  using namespace rby1_moveit_objects;
  using moveit_msgs::msg::PlanningSceneComponents;
  // Own the Ctrl+C: the modules must still be taken away through ROS afterwards.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) { stop = true; });
  std::signal(SIGTERM, [](int) { stop = true; });
  auto node = rclcpp::Node::make_shared("rby1_moveit_objects");
  const auto config = node->declare_parameter(
    "config", ament_index_cpp::get_package_share_directory("rby1_moveit_objects") + "/config/objects.yaml");
  const double period = node->declare_parameter("check_period", 2.0);
  const bool remove_on_exit = node->declare_parameter("remove_on_exit", true);

  int code = 0;
  std::vector<Module> modules;
  Scene scene(node);
  try {
    modules = modules_from_file(config);
    scene.apply_diff(add_diff(modules));
    std::string names;
    for (const auto & m : modules) names += (names.empty() ? "" : ", ") + m.spec.name + (m.attach ? " on " : " at ") + m.spec.frame;
    RCLCPP_INFO(node->get_logger(), "%zu object(s) from %s: %s", modules.size(), config.c_str(), names.c_str());
    auto next = std::chrono::steady_clock::now();
    while (!stop) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (std::chrono::steady_clock::now() < next) continue;
      next = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                  std::chrono::duration<double>(period));
      try {
        const auto gone = missing(modules, scene.current(PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                                         PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS));
        if (!gone.empty()) {
          RCLCPP_WARN(node->get_logger(), "%zu object(s) missing from the scene (move_group restarted?); putting "
                                          "them back", gone.size());
          scene.apply_diff(add_diff(modules));
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(node->get_logger(), "planning scene check: %s", error.what());
      }
    }
    if (remove_on_exit) {
      scene.apply_diff(remove_diff(modules));
      RCLCPP_INFO(node->get_logger(), "removed %zu object(s)", modules.size());
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "rby1_moveit_objects: %s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
