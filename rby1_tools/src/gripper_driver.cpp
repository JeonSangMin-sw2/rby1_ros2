// gripper_driver: the RB-Y1 gripper itself, through the SDK's Dynamixel bus -- one motor per
// hand on /dev/rby1_gripper, as SDK example 35 drives it. The same topics as virtual_gripper,
// so what listens to a gripper's state (gripper.launch.py's model and scene) does not care
// which runs. Settings: config/gripper_driver.yaml. Built only where the SDK is (RBY1_SDK_PATH).
//
//   ros2 launch rby1_tools gripper.launch.py controller:=real
//
// At start each motor is homed as the SDK example does: pushed with `homing_current` to one
// end and the other until the encoder stops, which gives its travel; THE FINGERS MOVE. Then
// current-based position control with `hold_current` as the current limit: a command sets
// the goal, the state is read back from the encoder at `rate` Hz. Per hand in `hands`:
// <topic_prefix>/<hand>_gripper/command (std_msgs/Float64, each finger's opening in m:
// 0 closed .. max_opening open), /grasp (std_msgs/Bool: true closes, false opens) and
// /state (sensor_msgs/JointState: gripper_finger_<r|l>1 and _2, both at the opening read).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rby1-sdk/base/dynamixel_bus.h"
#include "rby1-sdk/upc/device.h"
#include "rby1_tools/virtual_gripper.hpp"

class GripperDriver : public rclcpp::Node {
public:
  GripperDriver() : rclcpp::Node("gripper_driver") {
    const auto device = declare_parameter("device", std::string(rb::upc::kGripperDeviceName));
    const auto hands = declare_parameter("hands", std::vector<std::string>{"right", "left"});
    const int64_t right_id = declare_parameter("right_id", 0);
    const int64_t left_id = declare_parameter("left_id", 1);
    const auto prefix = declare_parameter("topic_prefix", "/rby1");
    max_opening_ = declare_parameter("max_opening", 0.05);
    const double rate = declare_parameter("rate", 50.0);
    const bool homing = declare_parameter("homing", true);
    homing_current_ = declare_parameter("homing_current", 0.3);
    hold_current_ = declare_parameter("hold_current", 5.0);
    inverted_ = declare_parameter("inverted", false);
    const auto joint_states_topic = declare_parameter("joint_states_topic", "");
    if (hands.empty()) throw std::invalid_argument("hands is empty");
    if (!(max_opening_ > 0.0) || !(rate > 0.0)) {
      throw std::invalid_argument("max_opening and rate must be positive");
    }
    if (!joint_states_topic.empty()) {
      joint_states_ = create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, 10);
    }

    bus_ = std::make_unique<rb::DynamixelBus>(device);
    if (!bus_->OpenPort()) throw std::runtime_error("could not open " + device);
    if (!bus_->SetBaudRate(rb::DynamixelBus::kDefaultBaudrate)) throw std::runtime_error("could not set the baud rate of " + device);
    std::vector<int> all_ids;
    for (size_t i = 0; i < hands.size(); ++i) {
      if (hands[i] != "right" && hands[i] != "left") {
        throw std::invalid_argument("hands are right and/or left, got '" + hands[i] + "'");
      }
      auto hand = std::make_shared<Hand>();
      hand->name = hands[i];
      hand->id = static_cast<int>(hands[i] == "right" ? right_id : left_id);
      if (!bus_->Ping(hand->id)) {
        throw std::runtime_error(hand->name + " gripper: Dynamixel id " + std::to_string(hand->id) + " does not answer on " + device);
      }
      const std::string letter = hand->name == "right" ? "r" : "l";
      hand->joints = {"gripper_finger_" + letter + "1", "gripper_finger_" + letter + "2"};
      const auto base = prefix + "/" + hand->name + "_gripper/";
      hand->state = create_publisher<sensor_msgs::msg::JointState>(base + "state", 10);
      hand->command = create_subscription<std_msgs::msg::Float64>(
        base + "command", 10, [this, hand](const std_msgs::msg::Float64 & message) {
          if (const auto why = rby1_tools::refused(message.data, max_opening_)) {
            RCLCPP_WARN(get_logger(), "%s gripper: refused command %g: %s", hand->name.c_str(), message.data, why->c_str());
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
      all_ids.push_back(hand->id);
      RCLCPP_INFO(get_logger(), "%s gripper: Dynamixel id %d on %s; %scommand, %sgrasp -> %sstate",
                  hand->name.c_str(), hand->id, device.c_str(), base.c_str(), base.c_str(), base.c_str());
    }
    bus_->SetTorqueConstant(std::vector<double>(*std::max_element(all_ids.begin(), all_ids.end()) + 1, 1.0));
    bus_->GroupSyncWriteTorqueEnable(all_ids, rb::DynamixelBus::kTorqueEnable);
    if (homing) home(all_ids);
    for (const auto & hand : hands_) {
      if (!std::isfinite(hand->min) || !std::isfinite(hand->max) || !(hand->max > hand->min)) {
        throw std::runtime_error(hand->name + " gripper: its travel is not known (homing: true learns it)");
      }
    }
    set_mode(all_ids, rb::DynamixelBus::kCurrentBasedPositionControlMode);
    std::vector<std::pair<int, double>> limits;
    for (const auto & hand : hands_) limits.emplace_back(hand->id, hold_current_);
    bus_->GroupSyncWriteSendTorque(limits);
    for (const auto & hand : hands_) hand->goal = hand->opening;   // stay where the homing left it
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { cycle(); });
  }

  ~GripperDriver() override {
    if (!bus_) return;
    std::vector<int> ids;
    for (const auto & hand : hands_) ids.push_back(hand->id);
    try {
      bus_->GroupSyncWriteTorqueEnable(ids, rb::DynamixelBus::kTorqueDisable);
    } catch (...) {
    }
  }

private:
  struct Hand {
    std::string name;
    int id = 0;
    std::vector<std::string> joints;
    double min = std::numeric_limits<double>::infinity();    // encoder at one end, rad
    double max = -std::numeric_limits<double>::infinity();   // and the other
    double opening = 0.0;   // m, read back
    double goal = 0.0;      // m
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp;
  };

  void set_mode(const std::vector<int> & ids, int mode) {
    bus_->GroupSyncWriteTorqueEnable(ids, rb::DynamixelBus::kTorqueDisable);
    std::vector<std::pair<int, int>> modes;
    for (int id : ids) modes.emplace_back(id, mode);
    bus_->GroupSyncWriteOperatingMode(modes);
    bus_->GroupSyncWriteTorqueEnable(ids, rb::DynamixelBus::kTorqueEnable);
  }

  // SDK example 35's homing: push every motor one way, then the other, until the encoders
  // have not moved for 30 reads 0.1 s apart; the extremes read are the travel.
  void home(const std::vector<int> & ids) {
    RCLCPP_WARN(get_logger(), "homing: the fingers move to both ends of their travel");
    set_mode(ids, rb::DynamixelBus::kCurrentControlMode);
    for (int direction = 0; direction < 2; ++direction) {
      std::vector<std::pair<int, double>> push;
      for (int id : ids) push.emplace_back(id, homing_current_ * (direction == 0 ? 1.0 : -1.0));
      std::vector<double> previous(hands_.size(), std::numeric_limits<double>::quiet_NaN());
      int still = 0;
      while (still < 30 && rclcpp::ok()) {
        bus_->GroupSyncWriteSendTorque(push);
        if (const auto read = bus_->GroupFastSyncReadEncoder(ids)) {
          bool moved = false;
          for (size_t i = 0; i < hands_.size(); ++i) {
            for (const auto & [id, encoder] : *read) {
              if (id != hands_[i]->id) continue;
              hands_[i]->min = std::min(hands_[i]->min, encoder);
              hands_[i]->max = std::max(hands_[i]->max, encoder);
              if (encoder != previous[i]) moved = true;
              previous[i] = encoder;
            }
          }
          still = moved ? 0 : still + 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    std::vector<std::pair<int, double>> rest;
    for (int id : ids) rest.emplace_back(id, 0.0);
    bus_->GroupSyncWriteSendTorque(rest);
    for (const auto & hand : hands_) {
      hand->opening = opening_of(*hand, previous_encoder(*hand));
      RCLCPP_INFO(get_logger(), "%s gripper: travel %.3f .. %.3f rad, now %.4f m open", hand->name.c_str(), hand->min, hand->max, hand->opening);
    }
  }

  double previous_encoder(const Hand & hand) {
    if (const auto read = bus_->GroupFastSyncReadEncoder({hand.id})) {
      for (const auto & [id, encoder] : *read) if (id == hand.id) return encoder;
    }
    return hand.min;
  }

  // Each finger's opening (m) from a motor's encoder, and back. Open is `max` unless
  // `inverted` (example 35's GRIPPER_DIRECTION).
  double opening_of(const Hand & hand, double encoder) const {
    double fraction = (encoder - hand.min) / (hand.max - hand.min);
    if (inverted_) fraction = 1.0 - fraction;
    return std::clamp(fraction, 0.0, 1.0) * max_opening_;
  }
  double encoder_of(const Hand & hand, double opening) const {
    double fraction = std::clamp(opening / max_opening_, 0.0, 1.0);
    if (inverted_) fraction = 1.0 - fraction;
    return hand.min + fraction * (hand.max - hand.min);
  }

  void cycle() {
    std::vector<int> ids;
    std::vector<std::pair<int, double>> goals;
    for (const auto & hand : hands_) {
      ids.push_back(hand->id);
      goals.emplace_back(hand->id, encoder_of(*hand, hand->goal));
    }
    bus_->GroupSyncWriteSendPosition(goals);
    const auto read = bus_->GroupFastSyncReadEncoder(ids);
    if (!read) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "the gripper's encoders did not answer");
      return;
    }
    const auto stamp = now();
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    for (const auto & hand : hands_) {
      for (const auto & [id, encoder] : *read) if (id == hand->id) hand->opening = opening_of(*hand, encoder);
      sensor_msgs::msg::JointState state;
      state.header.stamp = stamp;
      state.name = hand->joints;
      state.position = {hand->opening, hand->opening};
      hand->state->publish(state);
      for (size_t i = 0; i < hand->joints.size(); ++i) {
        joints.name.push_back(hand->joints[i] + "_joint");
        joints.position.push_back((i == 0 ? 1.0 : -1.0) * hand->opening);   // finger 2's joint reads minus the opening
      }
    }
    if (joint_states_) joint_states_->publish(joints);
  }

  std::unique_ptr<rb::DynamixelBus> bus_;
  std::vector<std::shared_ptr<Hand>> hands_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
  double max_opening_ = 0.05;
  double homing_current_ = 0.3;
  double hold_current_ = 5.0;
  bool inverted_ = false;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<GripperDriver>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("gripper_driver"), "%s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
