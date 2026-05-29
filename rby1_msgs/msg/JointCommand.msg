# Joint names associated with the target command positions
string[] joint_names

# Target joint positions (rad or meters)
float64[] position

# Enable joint impedance control if True, otherwise standard position control is used
bool use_impedance false

# Enable group joint movement logic if True
bool use_group_joint false

# Minimum duration for executing the trajectory (seconds)
float64 minimum_time 2.0

# Duration to hold position control after reaching target (seconds)
float64 control_hold_time 0.1

# Limit for joint velocities (rad/s or m/s)
float64 velocity_limit 4.7

# Limit for joint accelerations
float64 acceleration_limit 1.0

# Stiffness coefficients array for joint impedance control
float64[] stiffness

# Damping ratio for joint impedance control (default = 1.0)
float64 damping_ratio 1.0

# Torque safety limits for impedance control
float64 torque_limit 10.0
 
