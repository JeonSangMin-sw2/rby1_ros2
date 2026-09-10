# Tool Flange Status and Digital I/O
std_msgs/Header header

# Supplied output voltage to the tool (millivolts, e.g., 0, 12000, 24000)
int32 output_voltage

# Tool Flange physical Switch A state (True if active/pressed, False otherwise)
bool switch_a

# State of general-purpose digital input A (True if High, False if Low)
bool digital_input_a

# State of general-purpose digital input B (True if High, False if Low)
bool digital_input_b

# State of general-purpose digital output A (True if High, False if Low)
bool digital_output_a

# State of general-purpose digital output B (True if High, False if Low)
bool digital_output_b
