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
        bool collision = false;
    };

    struct RobotJoint{
        JointState joint_torso;      // 6 DOF
        JointState joint_right_arm;  // 7 DOF
        JointState joint_left_arm;   // 7 DOF
        JointState joint_head;       // 2 DOF
        JointState joint_wheel;      // 2 DOF (model A) / 4 DOF (model M) 이건 자료형 다른걸로 해야함
    };

    struct RobotParameter{
        double se2_minimum_time;
        double se2_linear_acceleration_limit;
        double se2_angular_acceleration_limit;
    };
}
 
