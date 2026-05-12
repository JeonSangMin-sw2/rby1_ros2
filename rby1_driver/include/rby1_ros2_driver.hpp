#pragma once

#include <mutex>
#include <chrono>
#include <thread>
#include <optional>
#include <fstream>
//ros2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rby1_msgs/msg/all_motor_state.hpp"
#include "rby1_msgs/msg/power_state.hpp"
#include "rby1_msgs/msg/tool_flange_state.hpp"
#include "rby1_msgs/msg/torque_velocity_state.hpp"
#include "rby1_msgs/msg/cartesian_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rby1_msgs/srv/state_on_off.hpp"
#include "rby1_msgs/srv/control_mode.hpp"
#include "rby1_msgs/action/multi_joint_command.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rby1_msgs/action/single_joint_command.hpp"
//sdk
#include "rby1-sdk/robot.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot_command_builder.h"
#include "std_srvs/srv/trigger.hpp"
#include "rby1_msgs/action/stream_position.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
//local header file
#include "type.hpp"

using namespace std::placeholders;
namespace rby1_ros2{
    struct BuilderConfig {
        bool is_configured = false;
        uint8_t control_type; // Maps to rby1_msgs::srv::ControlMode enums
        
        // Cartesian Impedance Parameters
        std::string ref_link;
        std::string target_link;
        std::vector<double> translation_weight;
        std::vector<double> rotation_weight;
        
        // Joint Impedance Parameters
        std::vector<double> joint_stiffness;
        
        // Common Impedance Parameters
        double damping_ratio;

        // New parameters for added SDK controllers
        std::vector<std::string> joint_names;
        double linear_velocity_limit = 0.5;
        double angular_velocity_limit = 1.0;
    };

    template <typename ModelType>
    class RBY1_ROS2_DRIVER : public rclcpp::Node {
        private:
            //robot
            RobotParameter robot_parameter_;
            RobotJoint robot_joint_;
            RobotState robot_state_;
            rb::RobotInfo info_;

            std::shared_ptr<rb::Robot<ModelType>> robot_;
            std::shared_ptr<rb::dyn::Robot<ModelType::kRobotDOF>> dynamics_;
            std::shared_ptr<rb::dyn::State<ModelType::kRobotDOF>> dyn_state_;

            std::string address;
            std::string model;
            std::string state_topic_name;
            std::string servo_list_str;
            std::string power_list_str;
            bool fault_reset_trigger;
            bool node_power_off_trigger_;
            bool use_all_motor_state_topic_trigger_;
            double collision_threshold_{0.03};
            bool publish_power_state_{false};
            bool publish_tool_flange_{false};
            bool publish_torque_velocity_{false};
            bool is_control_canceled_{false};

            //utility
            std::mutex mutex_;

            //ros2
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torso_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_arm_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_pub_;
            //rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_pub_;
            rclcpp::Publisher<rby1_msgs::msg::AllMotorState>::SharedPtr all_motor_state_pub_;
            rclcpp::Publisher<rby1_msgs::msg::PowerState>::SharedPtr power_state_pub_;
            rclcpp::Publisher<rby1_msgs::msg::ToolFlangeState>::SharedPtr tool_flange_state_pub_;
            rclcpp::Publisher<rby1_msgs::msg::TorqueVelocityState>::SharedPtr torque_velocity_state_pub_;
            rclcpp::Publisher<rby1_msgs::msg::CartesianPose>::SharedPtr torso_cartesian_pub_;
            rclcpp::Publisher<rby1_msgs::msg::CartesianPose>::SharedPtr right_arm_cartesian_pub_;
            rclcpp::Publisher<rby1_msgs::msg::CartesianPose>::SharedPtr left_arm_cartesian_pub_;
            rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_control_state_;

            // Timer for 100Hz publishing
            rclcpp::TimerBase::SharedPtr joint_state_timer_;

            using MultiJointCommand = rby1_msgs::action::MultiJointCommand;
            using SingleJointCommand = rby1_msgs::action::SingleJointCommand;
            using StreamPosition = rby1_msgs::action::StreamPosition;
            
            rclcpp_action::Server<MultiJointCommand>::SharedPtr multi_position_action_server_;
            rclcpp_action::Server<SingleJointCommand>::SharedPtr single_position_action_server_;
            rclcpp_action::Server<StreamPosition>::SharedPtr stream_position_action_server_;
            
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr power_service_;
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr servo_service_;
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr tool_flange_service_;
            rclcpp::Service<rby1_msgs::srv::ControlMode>::SharedPtr control_mode_service_;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_control_service_;

            void control_mode_callback(const std::shared_ptr<rby1_msgs::srv::ControlMode::Request> request,
                                       std::shared_ptr<rby1_msgs::srv::ControlMode::Response> response);
            
            geometry_msgs::msg::Pose matrix_to_pose(const Eigen::Matrix4d& matrix);

            BuilderConfig torso_builder_;
            BuilderConfig right_arm_builder_;
            BuilderConfig left_arm_builder_;
            BuilderConfig head_builder_;
        public:
            RBY1_ROS2_DRIVER();
            ~RBY1_ROS2_DRIVER();
            bool check_controll_manager();
            void read_joint_state();
            std::string finish_code_to_string(rb::RobotCommandFeedback::FinishCode code);
            
            void apply_body_builder(
                rb::BodyComponentBasedCommandBuilder& body_comp, 
                const std::string& part_name,
                const std::vector<double>& goal_data,
                double min_time);
            
            std::optional<rb::HeadCommandBuilder> apply_head_builder(
                const std::vector<double>& goal_data,
                double min_time);
            
            std::optional<rb::MobilityCommandBuilder> apply_mobile_builder(
                const std::vector<double>& goal_data);
            void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
            std::vector<std::string> dyn_link_names_;

        private:
            void init_parameter();
            void resize_joint_states();
            void publish_joint_states();
            
            void power_control(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                               std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response);
            void servo_control(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                               std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response);
            void tool_flange_control(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                                std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response);

            // Multi-joint Action Handlers
            rclcpp_action::GoalResponse handle_multi_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MultiJointCommand::Goal> goal);
            rclcpp_action::CancelResponse handle_multi_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiJointCommand>> goal_handle);
            void handle_multi_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiJointCommand>> goal_handle);
            void execute_multi_command(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiJointCommand>> goal_handle);

            // Single-joint Action Handlers
            rclcpp_action::GoalResponse handle_single_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SingleJointCommand::Goal> goal);
            rclcpp_action::CancelResponse handle_single_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointCommand>> goal_handle);
            void handle_single_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointCommand>> goal_handle);
            void execute_single_command(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointCommand>> goal_handle);

            // Stream Position Action Handlers
            rclcpp_action::GoalResponse handle_stream_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const StreamPosition::Goal> goal);
            rclcpp_action::CancelResponse handle_stream_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamPosition>> goal_handle);
            void handle_stream_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamPosition>> goal_handle);
            void execute_stream_position(const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamPosition>> goal_handle);

            void cancel_control_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
            
            std::unique_ptr<rb::RobotCommandStreamHandler<ModelType>> stream_handler_;
    };
}