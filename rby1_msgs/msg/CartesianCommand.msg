# Target Cartesian transformation (pose) command
geometry_msgs/Transform transform

# Reference coordinate frame link name
string ref_link

# Target coordinate frame link name (e.g. tool flange or end effector link)
string target_link

# Enable impedance control mode if True, otherwise standard position control is used
bool use_impedance false

# Minimum duration for executing the trajectory (seconds)
float64 minimum_time 2.0

# Duration to hold position control after reaching target (seconds)
float64 control_hold_time 0.1

# Limit for linear velocity of the end-effector (m/s)
float64 linear_velocity_limit 1.5

# Limit for angular velocity of the end-effector (rad/s)
float64 angular_velocity_limit 4.7

# Scaling factor for acceleration limits [0.0, 1.0]
float64 acceleration_limit_scaling 1.0

# Weights for linear translation degrees of freedom [x, y, z] during QP optimization
float64[] translation_weight

# Weights for rotation degrees of freedom during QP optimization
float64[] rotation_weight

# Flag to specify an additional single joint target along with the Cartesian pose
bool add_joint_position_target false

# Name of the additional joint to move
string add_joint_name

# Target position value for the additional joint (rad or meters)
float64 add_joint_value
 
