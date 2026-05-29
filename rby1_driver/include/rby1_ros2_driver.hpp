#pragma once

#include <mutex>
#include <chrono>
#include <thread>
#include <optional>
#include <atomic>
#include <fstream>
//ros2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rby1_msgs/msg/brake_state.hpp"
#include "rby1_msgs/msg/robot_state.hpp"
#include "rby1_msgs/msg/tool_flange_state.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rby1_msgs/srv/state_on_off.hpp"
#include "rby1_msgs/srv/get_cartesian_pose.hpp"
#include "rby1_msgs/srv/gravity_compensation.hpp"
#include "rby1_msgs/srv/control_manager_command.hpp"
#include "rby1_msgs/action/rby1_joint_command.hpp"
#include "rby1_msgs/action/rby1_cartesian_command.hpp"
#include "std_msgs/msg/int32.hpp"
//sdk
#include "rby1-sdk/robot.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot_command_builder.h"
#include "std_srvs/srv/trigger.hpp"
#include "rby1_msgs/action/stream_position.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
//local header file
#include "type.hpp"
#include <Eigen/Dense>

using namespace std::placeholders;
namespace rby1_ros2{


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

            // Cached physical joint limits
            Eigen::VectorXd q_lower_;
            Eigen::VectorXd q_upper_;
            Eigen::VectorXd qdot_upper_;
            Eigen::VectorXd qddot_upper_;
            std::vector<std::string> dyn_joint_names_;

            std::string address;
            std::string model;
            std::string state_topic_name;
            std::string joint_position_topic_name;
            std::string cartesian_position_topic_name;
            std::string servo_list_str;
            std::string power_list_str;
            bool fault_reset_trigger;
            bool node_power_off_trigger_;
            double collision_threshold_{0.03};
            bool publish_battery_state_{false};
            bool publish_tool_flange_state_{false};
            std::atomic<bool> is_control_canceled_{false};

            bool gravity_compensation_torso_{false};
            bool gravity_compensation_right_arm_{false};
            bool gravity_compensation_left_arm_{false};
            std::unique_ptr<rb::RobotCommandHandler<ModelType>> gravity_compensation_torso_handler_;
            std::unique_ptr<rb::RobotCommandHandler<ModelType>> gravity_compensation_right_arm_handler_;
            std::unique_ptr<rb::RobotCommandHandler<ModelType>> gravity_compensation_left_arm_handler_;

            //utility
            std::mutex mutex_;
            std::mutex stream_mutex_;

            //ros2
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torso_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_arm_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_pub_;
            //rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_pub_;
            rclcpp::Publisher<rby1_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
            rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
            rclcpp::Publisher<rby1_msgs::msg::ToolFlangeState>::SharedPtr tool_flange_left_pub_;
            rclcpp::Publisher<rby1_msgs::msg::ToolFlangeState>::SharedPtr tool_flange_right_pub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
            bool stream_active_{false};
            bool collision_enable_{false};

            // Timer for 100Hz publishing
            rclcpp::TimerBase::SharedPtr joint_state_timer_;

            using Rby1JointCommand = rby1_msgs::action::Rby1JointCommand;
            using Rby1CartesianCommand = rby1_msgs::action::Rby1CartesianCommand;
            using StreamPosition = rby1_msgs::action::StreamPosition;
            
            rclcpp_action::Server<Rby1JointCommand>::SharedPtr rby1_joint_command_action_server_;
            rclcpp_action::Server<Rby1CartesianCommand>::SharedPtr rby1_cartesian_command_action_server_;
            rclcpp_action::Server<StreamPosition>::SharedPtr stream_position_action_server_;
            
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr power_service_;
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr servo_service_;
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr tool_flange_service_;
            rclcpp::Service<rby1_msgs::srv::GravityCompensation>::SharedPtr gravity_compensation_service_;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_control_service_;
            rclcpp::Service<rby1_msgs::srv::GetCartesianPose>::SharedPtr get_cartesian_pose_service_;
            rclcpp::Service<rby1_msgs::srv::ControlManagerCommand>::SharedPtr control_manager_service_;
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr motor_brake_service_;
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr stream_control_service_;

            void gravity_compensation_callback(const std::shared_ptr<rby1_msgs::srv::GravityCompensation::Request> request,
                                               std::shared_ptr<rby1_msgs::srv::GravityCompensation::Response> response);
            void control_manager_callback(const std::shared_ptr<rby1_msgs::srv::ControlManagerCommand::Request> request,
                                          std::shared_ptr<rby1_msgs::srv::ControlManagerCommand::Response> response);
            void motor_brake_callback(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                                      std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response);
            void stream_control_callback(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                                         std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response);
            void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
            
            geometry_msgs::msg::Pose matrix_to_pose(const Eigen::Matrix4d& matrix);


        public:
            RBY1_ROS2_DRIVER();
            ~RBY1_ROS2_DRIVER();
            bool check_controll_manager();
            void read_joint_state();
            std::string finish_code_to_string(rb::RobotCommandFeedback::FinishCode code);
            

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
            void get_cartesian_pose_callback(const std::shared_ptr<rby1_msgs::srv::GetCartesianPose::Request> request,
                                             std::shared_ptr<rby1_msgs::srv::GetCartesianPose::Response> response);



            // Stream Position Action Handlers
            rclcpp_action::GoalResponse handle_stream_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const StreamPosition::Goal> goal);
            rclcpp_action::CancelResponse handle_stream_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamPosition>> goal_handle);
            void handle_stream_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamPosition>> goal_handle);
            void execute_stream_position(const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamPosition>> goal_handle);

            // Rby1 Joint Command Action Handlers
            rclcpp_action::GoalResponse handle_rby1_joint_goal(
                const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const Rby1JointCommand::Goal> goal);
            
            rclcpp_action::CancelResponse handle_rby1_joint_cancel(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1JointCommand>> goal_handle);
            
            void handle_rby1_joint_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1JointCommand>> goal_handle);
            void execute_rby1_joint_command(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1JointCommand>> goal_handle);

            rclcpp_action::GoalResponse handle_rby1_cartesian_goal(
                const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const Rby1CartesianCommand::Goal> goal);
            
            rclcpp_action::CancelResponse handle_rby1_cartesian_cancel(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1CartesianCommand>> goal_handle);
            
            void handle_rby1_cartesian_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1CartesianCommand>> goal_handle);
            void execute_rby1_cartesian_command(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1CartesianCommand>> goal_handle);

            void cancel_control_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
            
            std::unique_ptr<rb::RobotCommandStreamHandler<ModelType>> stream_handler_;
    };
} 
