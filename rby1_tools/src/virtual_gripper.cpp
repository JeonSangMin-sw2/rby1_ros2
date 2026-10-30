// virtual_gripper: a stand-in for the grippers, which the RB-Y1 driver does not drive. It
// takes a command per hand and reports the fingers moving there, so what follows a
// gripper's state -- the gripper's model on TF and its modules in the planning scene
// (gripper.launch.py) -- can be used without a gripper driver. Settings:
// config/virtual_gripper.yaml. The same topics as gripper_driver, which moves the real one.
//
//   ros2 launch rby1_tools gripper.launch.py                 # controller:=virtual is the default
//   ros2 topic pub --once /rby1/right_gripper/grasp std_msgs/msg/Bool "{data: true}"          # close
//   ros2 topic pub --once /rby1/right_gripper/command std_msgs/msg/Float64 "{data: 0.02}"    # 2 cm
//
// Per hand in `hands`: <topic_prefix>/<hand>_gripper/command (std_msgs/Float64, each
// finger's opening in m: 0 closed .. max_opening open), <topic_prefix>/<hand>_gripper/grasp
// (std_msgs/Bool: true closes, false opens) and <topic_prefix>/<hand>_gripper/state
// (sensor_msgs/JointState at `rate` Hz: gripper_finger_<r|l>1 and _2, both at the opening,
// moving toward the command at `speed`). A command outside that range is refused with a
// warning. Nothing on the robot moves.
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rby1_tools/virtual_gripper.hpp"

class VirtualGripper : public rclcpp::Node {
public:
  VirtualGripper() : rclcpp::Node("virtual_gripper") {
    const auto hands = declare_parameter("hands", std::vector<std::string>{"right", "left"});
    const auto prefix = declare_parameter("topic_prefix", "/rby1");
    max_opening_ = declare_parameter("max_opening", 0.05);
    speed_ = declare_parameter("speed", 0.05);
    const double rate = declare_parameter("rate", 50.0);
    const double initial = declare_parameter("initial_opening", 0.05);
    // The finger joints of the gripper's URDF -- gripper_finger_<r|l>1_joint (0..max_opening)
    // and _2_joint (-max_opening..0) -- for robot_state_publisher: gripper.launch.py's own
    // (/gripper/joint_states), or the robot's /joint_states on an RB-Y1 v1.3, whose URDF
    // carries the gripper. Empty: not published.
    const auto joint_states_topic = declare_parameter("joint_states_topic", "");
    if (!joint_states_topic.empty()) {
      joint_states_ = create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, 10);
    }
    if (!(max_opening_ > 0.0) || !(speed_ > 0.0) || !(rate > 0.0)) {
      throw std::invalid_argument("max_opening, speed and rate must be positive");
    }
    if (const auto why = rby1_tools::refused(initial, max_opening_)) {
      throw std::invalid_argument("initial_opening " + std::to_string(initial) + ": " + *why);
    }
    dt_ = 1.0 / rate;
    for (const auto & name : hands) {
      if (name != "right" && name != "left") {
        throw std::invalid_argument("hands are right and/or left, got '" + name + "'");
      }
      auto hand = std::make_shared<Hand>();
      hand->name = name;
      hand->opening = hand->goal = initial;
      const std::string letter = name == "right" ? "r" : "l";
      hand->joints = {"gripper_finger_" + letter + "1", "gripper_finger_" + letter + "2"};
      const auto base = prefix + "/" + name + "_gripper/";
      hand->state = create_publisher<sensor_msgs::msg::JointState>(base + "state", 10);
      hand->command = create_subscription<std_msgs::msg::Float64>(
        base + "command", 10, [this, hand](const std_msgs::msg::Float64 & message) {
          if (const auto why = rby1_tools::refused(message.data, max_opening_)) {
            RCLCPP_WARN(get_logger(), "%s gripper: refused command %g: %s", hand->name.c_str(), message.data,
                        why->c_str());
            return;
          }
          hand->goal = message.data;
          RCLCPP_INFO(get_logger(), "%s gripper: to %.4f m", hand->name.c_str(), hand->goal);
        });
      hand->grasp = create_subscription<std_msgs::msg::Bool>(
        base + "grasp", 10, [this, hand](const std_msgs::msg::Bool & message) {
          hand->goal = message.data ? 0.0 : max_opening_;
          RCLCPP_INFO(get_logger(), "%s gripper: %s", hand->name.c_str(), message.data ? "closing" : "opening");
        });
      hands_.push_back(hand);
      RCLCPP_INFO(get_logger(), "%s gripper: %scommand, %sgrasp -> %sstate, open %.4f m (virtual, nothing moves)",
                  name.c_str(), base.c_str(), base.c_str(), base.c_str(), initial);
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this] { publish(); });
  }

private:
  struct Hand {
    std::string name;
    std::vector<std::string> joints;
    double opening = 0.0;
    double goal = 0.0;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp;
  };

  void publish() {
    for (auto & hand : hands_) {
      const double before = hand->opening;
      hand->opening = rby1_tools::step_toward(hand->opening, hand->goal, speed_, dt_);
      sensor_msgs::msg::JointState state;
      state.header.stamp = now();
      state.name = hand->joints;
      state.position = {hand->opening, hand->opening};
      const double velocity = (hand->opening - before) / dt_;
      state.velocity = {velocity, velocity};
      hand->state->publish(state);
    }
    if (joint_states_) {
      sensor_msgs::msg::JointState joints;
      joints.header.stamp = now();
      for (const auto & hand : hands_) {
        for (size_t i = 0; i < hand->joints.size(); ++i) {
          const double sign = i == 0 ? 1.0 : -1.0;   // finger 2 moves along -x in the URDF
          joints.name.push_back(hand->joints[i] + "_joint");
          joints.position.push_back(sign * hand->opening);
        }
      }
      joint_states_->publish(joints);
    }
  }

  std::vector<std::shared_ptr<Hand>> hands_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
  double max_opening_ = 0.05;
  double speed_ = 0.05;
  double dt_ = 0.02;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<VirtualGripper>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("virtual_gripper"), "%s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
