#include <sensor_msgs/msg/joint_state.hpp>

namespace rby1_ros2{
    using JointState = sensor_msgs::msg::JointState;
    enum ControlState{
        NONE,
        IDLE,
        ENABLE,
        EXECUTING,
        MAJOR_FAULT,
        MINOR_FAULT
    };

    struct RobotState{
        bool is_connected = false;
        bool is_power_on = false;
        bool is_servo_on = false;
        bool is_ready = false;
        ControlState state = IDLE;
    };

    struct RobotJoint{
        JointState joint_torso;      // 6 DOF
        JointState joint_right_arm;  // 7 DOF
        JointState joint_left_arm;   // 7 DOF
        JointState joint_head;       // 2 DOF
        JointState joint_wheel;      // 2 DOF (model A) / 4 DOF (model M) 이건 자료형 다른걸로 해야함
    };

    struct RobotParameter{
        std::vector<std::string> power_on_list;
        std::vector<std::string> servo_on_list;
        double get_state_period;
        double minimum_time;
        double angular_velocity_limit;
        double linear_velocity_limit;
        double acceleration_limit;
        double stop_orientation_tracking_error;
        double stop_position_tracking_error;
    };
}
