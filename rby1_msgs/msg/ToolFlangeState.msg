# Force/Torque Sensor Data
# Dynamic Force components [Fx, Fy, Fz] in Newtons
float64[3] ft_force

# Dynamic Torque components [Tx, Ty, Tz] in Newton-meters
float64[3] ft_torque

# Tool Flange onboard IMU Data
# Gyroscope rates [roll, pitch, yaw] in rad/s
float64[3] gyro

# Accelerometer values [ax, ay, az] in m/s^2
float64[3] acceleration

# Tool Flange physical Switch A state (True if active/triggered, False otherwise)
bool switch_a

# Supplied output voltage to the tool (millivolts)
int32 output_voltage

# State of general-purpose digital input A (True if High, False if Low)
bool digital_input_a

# State of general-purpose digital input B (True if High, False if Low)
bool digital_input_b

# State of general-purpose digital output A (True if High, False if Low)
bool digital_output_a

# State of general-purpose digital output B (True if High, False if Low)
bool digital_output_b
 
