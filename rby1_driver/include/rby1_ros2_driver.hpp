#pragma once

#include <mutex>
#include <chrono>
#include <thread>
#include <optional>
#include <atomic>
#include <fstream>
#include <array>
//ros2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rby1_msgs/msg/brake_state.hpp"
#include "rby1_msgs/msg/robot_state.hpp"
#include "rby1_msgs/msg/tool_flange_state.hpp"
#include "rby1_msgs/msg/tool_flange_status.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rby1_msgs/srv/state_on_off.hpp"
#include "rby1_msgs/srv/get_cartesian_pose.hpp"
#include "rby1_msgs/srv/gravity_compensation.hpp"
#include "rby1_msgs/srv/control_manager_command.hpp"
#include "rby1_msgs/srv/set_trajectory_impedance.hpp"
#include "rby1_msgs/action/rby1_joint_command.hpp"
#include "rby1_msgs/action/rby1_cartesian_command.hpp"
#include "rby1_msgs/action/stream_joint.hpp"
#include "rby1_msgs/action/stream_cartesian.hpp"
#include "std_msgs/msg/int32.hpp"
//sdk
#include "rby1-sdk/robot.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot_command_builder.h"
#pragma GCC diagnostic push //remove warning log
#pragma GCC diagnostic ignored "-Wreorder" //remove warning log
#pragma GCC diagnostic ignored "-Wsign-compare" //remove warning log
#include "rby1-sdk/math/optimal_control.h"
#pragma GCC diagnostic pop //remove warning log
#include "std_srvs/srv/trigger.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include <numeric>
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
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
            rclcpp::Publisher<rby1_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
            rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
            // Tool flange classified publishers
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr tool_flange_left_imu_pub_;
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr tool_flange_right_imu_pub_;
            rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tool_flange_left_wrench_pub_;
            rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tool_flange_right_wrench_pub_;
            rclcpp::Publisher<rby1_msgs::msg::ToolFlangeStatus>::SharedPtr tool_flange_left_status_pub_;
            rclcpp::Publisher<rby1_msgs::msg::ToolFlangeStatus>::SharedPtr tool_flange_right_status_pub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

            bool robot_initialize_flag{false};
            bool stream_active_{false};
            bool collision_enable_{true};

            // ── Per-part control flags ──────────────────────────────────────────
            // Indexed by PartIndex: MOBILE=0, TORSO=1, RIGHT_ARM=2, LEFT_ARM=3, HEAD=4
            static constexpr size_t PART_MOBILE    = 0;
            static constexpr size_t PART_TORSO     = 1;
            static constexpr size_t PART_RIGHT_ARM = 2;
            static constexpr size_t PART_LEFT_ARM  = 3;
            static constexpr size_t PART_HEAD      = 4;
            static constexpr size_t PART_COUNT     = 5;
            std::atomic<bool> is_controlling_[PART_COUNT]{}; // true while that part is commanded
            bool any_part_controlling() const;

            std::atomic<uint64_t> command_seq_{0}; // incremented on each new stream cmd


            std::atomic<bool> has_printed_collision_log_{false};
            int get_link_index(const std::string& name);
            std::optional<std::string> get_predicted_collision_reason(const Eigen::VectorXd& target_q);
            std::optional<Eigen::VectorXd> solve_cartesian_ik(const std::vector<typename rb::OptimalControl<ModelType::kRobotDOF>::LinkTarget>& link_targets);

            // ── Builder helpers ────────────────────────────────────────────────
            void make_joint_pos_builder(
                rb::JointPositionCommandBuilder& builder,
                const Eigen::VectorXd& q, double minimum_time, double hold_time,
                double vel_limit = 0.0, double acc_limit = 0.0);
            void make_joint_impedance_builder(
                rb::JointImpedanceControlCommandBuilder& builder,
                const Eigen::VectorXd& q, double minimum_time, double hold_time,
                const Eigen::VectorXd& stiffness, double damping_ratio, double torque_limit,
                double vel_limit = 0.0, double acc_limit = 0.0);
            void build_mobility_cmd(
                rb::MobilityCommandBuilder& builder,
                double vx, double vy, double wz, double minimum_time);

            // Timer for 100Hz publishing
            rclcpp::TimerBase::SharedPtr joint_state_timer_;

            using Rby1JointCommand = rby1_msgs::action::Rby1JointCommand;
            using Rby1CartesianCommand = rby1_msgs::action::Rby1CartesianCommand;
            using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
            using StreamJoint = rby1_msgs::action::StreamJoint;
            using StreamCartesian = rby1_msgs::action::StreamCartesian;
            
            rclcpp_action::Server<Rby1JointCommand>::SharedPtr rby1_joint_command_action_server_;
            rclcpp_action::Server<Rby1CartesianCommand>::SharedPtr rby1_cartesian_command_action_server_;
            rclcpp_action::Server<FollowJointTrajectory>::SharedPtr follow_joint_trajectory_action_server_;
            rclcpp_action::Server<StreamJoint>::SharedPtr stream_joint_action_server_;
            rclcpp_action::Server<StreamCartesian>::SharedPtr stream_cartesian_action_server_;
            
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr power_service_;
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr servo_service_;
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr tool_flange_service_;
            rclcpp::Service<rby1_msgs::srv::GravityCompensation>::SharedPtr gravity_compensation_service_;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_control_service_;
            rclcpp::Service<rby1_msgs::srv::GetCartesianPose>::SharedPtr get_cartesian_pose_service_;
            rclcpp::Service<rby1_msgs::srv::ControlManagerCommand>::SharedPtr control_manager_service_;
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr stream_control_service_;
            rclcpp::Service<rby1_msgs::srv::SetTrajectoryImpedance>::SharedPtr set_trajectory_impedance_service_;
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr hardware_control_service_;

            std::atomic<bool> hardware_control_active_{false};
            std::atomic<uint64_t> last_stream_command_time_ns_{0};
            rclcpp::TimerBase::SharedPtr stream_safety_timer_;

            // Impedance state for follow_joint_trajectory
            bool   trajectory_impedance_enabled_torso_{false};
            bool   trajectory_impedance_enabled_right_arm_{false};
            bool   trajectory_impedance_enabled_left_arm_{false};
            std::vector<double> trajectory_stiffness_torso_;
            std::vector<double> trajectory_stiffness_right_arm_;
            std::vector<double> trajectory_stiffness_left_arm_;
            double trajectory_damping_ratio_{1.0};
            double trajectory_torque_limit_{10.0};
            double stream_hz_{15.0};

            void gravity_compensation_callback(const std::shared_ptr<rby1_msgs::srv::GravityCompensation::Request> request,
                                               std::shared_ptr<rby1_msgs::srv::GravityCompensation::Response> response);
            void control_manager_callback(const std::shared_ptr<rby1_msgs::srv::ControlManagerCommand::Request> request,
                                          std::shared_ptr<rby1_msgs::srv::ControlManagerCommand::Response> response);
            void stream_control_callback(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                                         std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response);
            void set_trajectory_impedance_callback(
                const std::shared_ptr<rby1_msgs::srv::SetTrajectoryImpedance::Request> request,
                std::shared_ptr<rby1_msgs::srv::SetTrajectoryImpedance::Response> response);
            void hardware_control_callback(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                                           std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response);
            void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
            void deactivate_all_streams();
            void check_stream_safety();
            
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

            // Follow Joint Trajectory Action Handlers
            rclcpp_action::GoalResponse handle_follow_joint_trajectory_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
            rclcpp_action::CancelResponse handle_follow_joint_trajectory_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle);
            void handle_follow_joint_trajectory_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle);
            void execute_follow_joint_trajectory(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle);

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

            // Stream Command Actions Handlers
            rclcpp_action::GoalResponse handle_stream_joint_goal(
                const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const StreamJoint::Goal> goal);
            rclcpp_action::CancelResponse handle_stream_joint_cancel(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamJoint>> goal_handle);
            void handle_stream_joint_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamJoint>> goal_handle);
            void execute_stream_joint(const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamJoint>> goal_handle);

            rclcpp_action::GoalResponse handle_stream_cartesian_goal(
                const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const StreamCartesian::Goal> goal);
            rclcpp_action::CancelResponse handle_stream_cartesian_cancel(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamCartesian>> goal_handle);
            void handle_stream_cartesian_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamCartesian>> goal_handle);
            void execute_stream_cartesian(const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamCartesian>> goal_handle);

            void cancel_control_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
            
            std::unique_ptr<rb::RobotCommandStreamHandler<ModelType>> upper_body_stream_handler_;
            std::unique_ptr<rb::RobotCommandStreamHandler<ModelType>> mobility_stream_handler_;
            std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> active_follow_joint_trajectory_goal_{nullptr};

            bool process_joint_part(const rby1_msgs::msg::JointCommand& cmd,
                                    const std::string& part_name,
                                    size_t expected_dof,
                                    rb::BodyComponentBasedCommandBuilder& body_comp,
                                    rb::ComponentBasedCommandBuilder& component_cmd_builder,
                                    bool& use_body,
                                    bool& use_head,
                                    std::string& err_msg);

            bool process_cartesian_part(const rby1_msgs::msg::CartesianCommand& cmd,
                                         const std::string& part_name,
                                         double stop_position_tracking_error,
                                         double stop_orientation_tracking_error,
                                         rb::BodyComponentBasedCommandBuilder& body_comp,
                                         bool& use_body,
                                         std::string& err_msg);
    };
} 
