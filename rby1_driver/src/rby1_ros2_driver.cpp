#include "rby1_ros2_driver.hpp"
namespace rby1_ros2{
    //using namespace rb;


    template <typename ModelType>
    RBY1_ROS2_DRIVER<ModelType>::RBY1_ROS2_DRIVER()
        : Node("rby1_ros2_driver"){
            //declare parameter from yaml
            init_parameter();
            try{
                robot_ = rb::Robot<ModelType>::Create(address);
                if(robot_->Connect()){
                    robot_->SetParameter("default.acceleration_limit_scaling", std::to_string(robot_parameter_.acceleration_limit));
                    robot_->SetParameter("joint_position_command.cutoff_frequency", std::to_string(robot_parameter_.angular_velocity_limit));
                    robot_->SetParameter("cartesian_command.cutoff_frequency", std::to_string(robot_parameter_.linear_velocity_limit));
                    robot_->SetParameter("default.linear_acceleration_limit", std::to_string(robot_parameter_.acceleration_limit));
                    // Fetch robot info once and cache it
                    info_ = robot_->GetRobotInfo();

                    // Auto-reset MAJOR/MINOR faults on startup if fault_reset_trigger is true
                    if (fault_reset_trigger) {
                        const auto& control_manager_state = robot_->GetControlManagerState();
                        if (control_manager_state.state == rb::ControlManagerState::State::kMajorFault ||
                            control_manager_state.state == rb::ControlManagerState::State::kMinorFault) {
                            RCLCPP_INFO(this->get_logger(), "Auto-resetting active fault on startup as requested by fault_reset_trigger...");
                            if (robot_->ResetFaultControlManager()) {
                                RCLCPP_INFO(this->get_logger(), "Fault auto-reset completed successfully.");
                            } else {
                                RCLCPP_WARN(this->get_logger(), "Failed to auto-reset fault on startup.");
                            }
                        }
                    }

                    RCLCPP_INFO(this->get_logger(), "Robot Info: Model=%s, Version=%s, Compile-time DOF=%zu", 
                                info_.robot_model_name.c_str(), info_.robot_model_version.c_str(), ModelType::kRobotDOF);
                    RCLCPP_INFO(this->get_logger(), "Joint counts: torso=%zu, right_arm=%zu, left_arm=%zu, head=%zu, mobility=%zu", 
                                info_.torso_joint_idx.size(), info_.right_arm_joint_idx.size(), 
                                info_.left_arm_joint_idx.size(), info_.head_joint_idx.size(), 
                                info_.mobility_joint_idx.size());
                    resize_joint_states();
                    try {
                        RCLCPP_INFO(this->get_logger(), "Loading Dynamics model...");
                        dynamics_ = robot_->GetDynamics();
                        RCLCPP_INFO(this->get_logger(), "Dynamics model loaded. Creating DynState...");
                        
                        // Register ALL links from dynamics model
                        dyn_link_names_ = dynamics_->GetLinkNames();
                        
                        dyn_state_ = dynamics_->template MakeState(
                            dyn_link_names_,
                            dynamics_->GetJointNames()
                        );
                        RCLCPP_INFO(this->get_logger(), "DynState created.");

                        q_lower_ = dynamics_->GetLimitQLower(dyn_state_);
                        q_upper_ = dynamics_->GetLimitQUpper(dyn_state_);
                        qdot_upper_ = dynamics_->GetLimitQdotUpper(dyn_state_);
                        qddot_upper_ = dynamics_->GetLimitQddotUpper(dyn_state_);
                        auto joint_names = dyn_state_->GetJointNames();
                        dyn_joint_names_.clear();
                        for (size_t i = 0; i < joint_names.size(); ++i) {
                            dyn_joint_names_.push_back(std::string(joint_names[i]));
                        }
                        
                        //RCLCPP_INFO(this->get_logger(), "=== ROBOT JOINT PHYSICAL LIMITS ===");
                        // for (size_t i = 0; i < dyn_joint_names_.size(); ++i) {
                        //     RCLCPP_INFO(this->get_logger(), "Joint [%zu] %s: Pos=[%.3f, %.3f] rad, Vel_Max=%.3f rad/s, Accel_Max=%.3f rad/s^2",
                        //                 i, dyn_joint_names_[i].c_str(), q_lower_[i], q_upper_[i], qdot_upper_[i], qddot_upper_[i]);
                        // }
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(this->get_logger(), "\033[1;33m[DYNAMICS WARNING] Failed to load dynamics model: %s. Cartesian poses and impedance features will be disabled.\033[0m", e.what());
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "\033[1;31m====================================================================\033[0m");
                    RCLCPP_ERROR(this->get_logger(), "\033[1;31m[CONNECTION ERROR] Failed to connect to robot at address: %s\033[0m", address.c_str());
                    RCLCPP_ERROR(this->get_logger(), "\033[1;31mPlease check:\033[0m");
                    RCLCPP_ERROR(this->get_logger(), "\033[1;31m  1. Is the robot's physical power/servo switched on?\033[0m");
                    RCLCPP_ERROR(this->get_logger(), "\033[1;31m  2. Is the network ethernet/Wi-Fi connection active?\033[0m");
                    RCLCPP_ERROR(this->get_logger(), "\033[1;31m  3. Is the robot_ip parameter '%s' correct?\033[0m", address.c_str());
                    RCLCPP_ERROR(this->get_logger(), "\033[1;31m====================================================================\033[0m");
                    throw std::runtime_error("Failed to connect to RBY1 robot at address: " + address);
                }
                
                // robot state publisher
                torso_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic_name + "/torso", 10);
                right_arm_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic_name + "/right_arm", 10);
                left_arm_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic_name + "/left_arm", 10);
                head_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic_name + "/head", 10);
                
                // consolidated robot state publisher
                robot_state_pub_ = this->create_publisher<rby1_msgs::msg::RobotState>("robot_state", 10);
                
                if (publish_battery_state_) {
                    battery_state_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
                }
                if (publish_tool_flange_state_) {
                    tool_flange_left_pub_ = this->create_publisher<rby1_msgs::msg::ToolFlangeState>("tool_flange/left", 10);
                    tool_flange_right_pub_ = this->create_publisher<rby1_msgs::msg::ToolFlangeState>("tool_flange/right", 10);
                }
                // loop for reading joint state
                // loop for reading joint state
                joint_state_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(robot_parameter_.get_state_period*1000.0)), std::bind(&RBY1_ROS2_DRIVER<ModelType>::read_joint_state, this));
                
                // service for important robot action
                power_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
                    "robot_power", std::bind(&RBY1_ROS2_DRIVER<ModelType>::power_control, this, _1, _2));
                servo_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
                    "robot_servo", std::bind(&RBY1_ROS2_DRIVER<ModelType>::servo_control, this, _1, _2));
                tool_flange_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
                    "tool_flange_power", std::bind(&RBY1_ROS2_DRIVER<ModelType>::tool_flange_control, this, _1, _2));
                gravity_compensation_service_ = this->create_service<rby1_msgs::srv::GravityCompensation>(
                    "gravity_compensation", std::bind(&RBY1_ROS2_DRIVER<ModelType>::gravity_compensation_callback, this, _1, _2));
                cancel_control_service_ = this->create_service<std_srvs::srv::Trigger>(
                    "cancel_control", std::bind(&RBY1_ROS2_DRIVER<ModelType>::cancel_control_callback, this, _1, _2));
                get_cartesian_pose_service_ = this->create_service<rby1_msgs::srv::GetCartesianPose>(
                    "get_cartesian_pose", std::bind(&RBY1_ROS2_DRIVER<ModelType>::get_cartesian_pose_callback, this, _1, _2));
                control_manager_service_ = this->create_service<rby1_msgs::srv::ControlManagerCommand>(
                    "control_manager_command", std::bind(&RBY1_ROS2_DRIVER<ModelType>::control_manager_callback, this, _1, _2));
                motor_brake_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
                    "set_motor_brake", std::bind(&RBY1_ROS2_DRIVER<ModelType>::motor_brake_callback, this, _1, _2));
                
                stream_control_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
                    "stream_control", std::bind(&RBY1_ROS2_DRIVER<ModelType>::stream_control_callback, this, _1, _2));
 
                odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
                tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
                cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                    "cmd_vel", 10, std::bind(&RBY1_ROS2_DRIVER<ModelType>::cmd_vel_callback, this, _1));
 
                /* we need to add tool flange on/off service*/
 
                stream_position_action_server_ = rclcpp_action::create_server<StreamPosition>(
                    this,
                    "stream_position_command",
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_stream_goal, this, _1, _2),
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_stream_cancel, this, _1),
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_stream_accepted, this, _1));

                if (!joint_position_topic_name.empty()) {
                    rby1_joint_command_action_server_ = rclcpp_action::create_server<Rby1JointCommand>(
                        this,
                        joint_position_topic_name,
                        std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_goal, this, _1, _2),
                        std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_cancel, this, _1),
                        std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_accepted, this, _1));
                }

                if (!cartesian_position_topic_name.empty()) {
                    rby1_cartesian_command_action_server_ = rclcpp_action::create_server<Rby1CartesianCommand>(
                        this,
                        cartesian_position_topic_name,
                        std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_goal, this, _1, _2),
                        std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_cancel, this, _1),
                        std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_accepted, this, _1));
                }
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "\033[1;31m[INITIALIZATION ERROR] Exception caught during driver setup: %s\033[0m", e.what());
                throw;
            }
    }

    template <typename ModelType>
    RBY1_ROS2_DRIVER<ModelType>::~RBY1_ROS2_DRIVER(){
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::init_parameter(){
        RCLCPP_INFO(this->get_logger(), "Declaring parameters...");
        this->declare_parameter<std::string>("robot_ip", "127.0.0.1:50051");
        this->declare_parameter<std::string>("model", "a");
        this->declare_parameter<std::vector<std::string>>("power_on", {"all"});
        this->declare_parameter<std::vector<std::string>>("servo_on", {"all"});

        this->declare_parameter<double>("get_state_period", 0.01);
        this->declare_parameter<double>("minimum_time", 2.0);
        this->declare_parameter<double>("angular_velocity_limit", 4.712388);
        this->declare_parameter<double>("linear_velocity_limit", 1.5);
        this->declare_parameter<double>("acceleration_limit", 1.0);
        this->declare_parameter<double>("stop_orientation_tracking_error", 1e-5);
        this->declare_parameter<double>("stop_position_tracking_error", 1e-5);
        this->declare_parameter<double>("se2_minimum_time", 1.0);
        this->declare_parameter<double>("se2_linear_acceleration_limit", 0.5);
        this->declare_parameter<double>("se2_angular_acceleration_limit", 0.5);
        
        this->declare_parameter<bool>("fault_reset_trigger", true);
        this->declare_parameter<bool>("node_power_off_trigger", false);
        this->declare_parameter<double>("collision_threshold", 0.01);
        this->declare_parameter<bool>("publish_battery_state", true);
        this->declare_parameter<bool>("publish_tool_flange_state", true);
        this->declare_parameter<bool>("collision_enable", false);
        
        this->get_parameter("robot_ip", address);
        this->get_parameter("model", model);
        
        state_topic_name = "joint_states";
        joint_position_topic_name = "robot_joint";
        cartesian_position_topic_name = "robot_cartesian";
 
        this->get_parameter("power_on", robot_parameter_.power_on_list);
        this->get_parameter("servo_on", robot_parameter_.servo_on_list);
        this->get_parameter("get_state_period", robot_parameter_.get_state_period);
        this->get_parameter("minimum_time", robot_parameter_.minimum_time);
        this->get_parameter("angular_velocity_limit", robot_parameter_.angular_velocity_limit);
        this->get_parameter("linear_velocity_limit", robot_parameter_.linear_velocity_limit);
        this->get_parameter("acceleration_limit", robot_parameter_.acceleration_limit);
        this->get_parameter("stop_orientation_tracking_error", robot_parameter_.stop_orientation_tracking_error);
        this->get_parameter("stop_position_tracking_error", robot_parameter_.stop_position_tracking_error);
        this->get_parameter("se2_minimum_time", robot_parameter_.se2_minimum_time);
        this->get_parameter("se2_linear_acceleration_limit", robot_parameter_.se2_linear_acceleration_limit);
        this->get_parameter("se2_angular_acceleration_limit", robot_parameter_.se2_angular_acceleration_limit);
        this->get_parameter("fault_reset_trigger", fault_reset_trigger);
        this->get_parameter("node_power_off_trigger", node_power_off_trigger_);
        this->get_parameter("collision_threshold", collision_threshold_);
        this->get_parameter("publish_battery_state", publish_battery_state_);
        this->get_parameter("publish_tool_flange_state", publish_tool_flange_state_);
        this->get_parameter("collision_enable", collision_enable_);



        if (address == "" || model == ""){
            RCLCPP_ERROR(this->get_logger(), "address or model isn't declared");
            rclcpp::shutdown();
        }
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::power_control(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                                                    std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response) {
        
        // Parse parameters string
        std::vector<std::string> param_list;
        std::stringstream ss(request->parameters);
        std::string token;
        while(std::getline(ss, token, ',')) {
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (!token.empty()) param_list.push_back(token);
        }
        if (param_list.empty()) param_list.push_back("all");

        std::string power_list_str = "";
        if (address == "127.0.0.1:50051"){
            power_list_str = ".*";
        }else{
            for (size_t i = 0; i < param_list.size(); i++) {
                if (param_list[i] == "all" || param_list[i] == ".*") {
                    power_list_str = ".*";
                    break;
                }
                power_list_str += param_list[i];
                if (param_list[i].find('v') == std::string::npos && param_list[i] != ".*") {
                    power_list_str += "v";
                }
                if (i != param_list.size() - 1){
                    power_list_str += "|";
                }
            }
        }

        if (request->state) {
            if (robot_->IsPowerOn(power_list_str)) {
                RCLCPP_INFO(this->get_logger(), "Power is already ON [%s], skipping.", power_list_str.c_str());
                response->success = true;
                response->message = "Power already ON";
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Power ON [%s]...", power_list_str.c_str());
            if (!robot_->PowerOn(power_list_str)) {
                response->success = false;
                response->message = "Failed to power on";
                return;
            }
        } else {
            // Check control manager state and disable it if enabled before powering off
            const auto& cm_state = robot_->GetControlManagerState();
            if (cm_state.state == rb::ControlManagerState::State::kEnabled) {
                RCLCPP_INFO(this->get_logger(), "Control Manager is enabled. Disabling Control Manager first before powering off...");
                if (!robot_->DisableControlManager()) {
                    RCLCPP_WARN(this->get_logger(), "Failed to disable Control Manager, proceeding with power off anyway.");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Control Manager successfully disabled.");
                }
            }

            RCLCPP_INFO(this->get_logger(), "Power OFF [%s]...", power_list_str.c_str());
            if (!robot_->PowerOff(power_list_str)) {
                response->success = false;
                response->message = "Failed to power off";
                return;
            }
        }
        response->success = true;
        response->message = "Power control success";
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::servo_control(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                                                    std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response) {
        // Parse parameters string
        std::vector<std::string> param_list;
        std::stringstream ss(request->parameters);
        std::string token;
        while(std::getline(ss, token, ',')) {
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (!token.empty()) param_list.push_back(token);
        }
        if (param_list.empty()) param_list.push_back("all");

        std::string servo_list_str = "";
        for (size_t i = 0; i < param_list.size(); i++) {
            std::string name = param_list[i];
            if(name == "right")      servo_list_str += "^right_arm_.*";
            else if(name == "left")  servo_list_str += "^left_arm_.*";
            else if(name == "head")  servo_list_str += "^head_.*";
            else if(name == "torso") servo_list_str += "^torso_.*";
            else if(name == "all") {
                servo_list_str = ".*";
                break;
            } else {
                servo_list_str += name;
            }
            if (i != param_list.size() - 1){
                servo_list_str += "|";
            }
        }

        if (request->state) {
            bool already_on = false;
            try {
                if (robot_->IsServoOn(servo_list_str)) {
                    already_on = true;
                }
            } catch (...) {}

            // Fallback check: If the Control Manager is already Enabled, the servos must be ON.
            auto cm_state_val = robot_->GetControlManagerState().state;
            if (cm_state_val == rb::ControlManagerState::State::kEnabled) {
                already_on = true;
            }

            if (already_on) {
                RCLCPP_INFO(this->get_logger(), "Servo is already ON [%s] (Control Manager: %d), skipping.", 
                            servo_list_str.c_str(), static_cast<int>(cm_state_val));
                check_controll_manager();
                response->success = true;
                response->message = "Servo already ON";
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Servo ON [%s]...", servo_list_str.c_str());
            if (!robot_->ServoOn(servo_list_str)) {
                RCLCPP_ERROR(this->get_logger(), "SDK ServoOn failed for [%s]", servo_list_str.c_str());
                response->success = false;
                response->message = "SDK_SERVO_ON_FAILED";
                return;
            }
            
            // Give SDK a bit of time to settle after servo on
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            { // sync info 
                std::lock_guard<std::mutex> lock(mutex_);
                info_ = robot_->GetRobotInfo();
                resize_joint_states();
            }

            RCLCPP_INFO(this->get_logger(), "Checking Control Manager after Servo ON...");
            if (!check_controll_manager()) {
                RCLCPP_ERROR(this->get_logger(), "Control Manager check failed after Servo ON.");
                response->success = false;
                if(robot_->IsPowerOn("48v")){
                    response->message = "Control Manager failed to enable (Power is ON)";
                }else{
                    response->message = "Control Manager failed: Robot power is OFF";
                }
                return;
            }
            response->success = true;
            response->message = "Servo control success";
        } else {
            RCLCPP_INFO(this->get_logger(), "Servo OFF [%s]...", servo_list_str.c_str());
            robot_->DisableControlManager();
            if (!robot_->ServoOff(servo_list_str)) {
                RCLCPP_ERROR(this->get_logger(), "SDK ServoOff failed for [%s]", servo_list_str.c_str());
                response->success = false;
                response->message = "SDK ServoOff call failed";
                return;
            }
            response->success = true;
            response->message = "Servo control success";
        }
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::tool_flange_control(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                                                    std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response) {
        // Parse parameters string
        int tool_flange_vol = 0;
        if (request->state) {
            if(request->parameters == "12" || request->parameters == "12v"){
                tool_flange_vol = 12;
            }else if(request->parameters == "24" || request->parameters == "24v"){
                tool_flange_vol = 24;
            }else{
                RCLCPP_INFO(this->get_logger(), "Invalid parameters: %s", request->parameters.c_str());
                response->success = false;
                response->message = "Invalid parameters (must be '12', '12v', '24', or '24v')";
                return;
            }
            RCLCPP_INFO(this->get_logger(), "set tool flange output voltage [%d]", tool_flange_vol);
            bool right_success = robot_->SetToolFlangeOutputVoltage("right",tool_flange_vol);
            bool left_success = robot_->SetToolFlangeOutputVoltage("left",tool_flange_vol);
            if (!right_success || !left_success) {
                response->success = false;
                response->message = "Failed to set tool flange output voltage";
                return;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "turn off tool flange");
            bool right_success = robot_->SetToolFlangeOutputVoltage("right",0);
            bool left_success = robot_->SetToolFlangeOutputVoltage("left",0);
            if (!right_success || !left_success) {
                response->success = false;
                response->message = "Failed to turn off tool flange";
                return;
            }
        }
        response->success = true;
        response->message = "Tool flange control success";
    }
    template <typename ModelType>
    geometry_msgs::msg::Pose RBY1_ROS2_DRIVER<ModelType>::matrix_to_pose(const Eigen::Matrix4d& matrix) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = matrix(0, 3);
        pose.position.y = matrix(1, 3);
        pose.position.z = matrix(2, 3);
        Eigen::Quaterniond q(matrix.block<3, 3>(0, 0));
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        return pose;
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::gravity_compensation_callback(const std::shared_ptr<rby1_msgs::srv::GravityCompensation::Request> request,
                                                                    std::shared_ptr<rby1_msgs::srv::GravityCompensation::Response> response) {
        if (request->part_name != "torso" && request->part_name != "right_arm" && request->part_name != "left_arm") {
            response->response = false;
            RCLCPP_ERROR(this->get_logger(), "GravityCompensation: Invalid part_name: %s", request->part_name.c_str());
            return;
        }

        // 1. Collision check: Robot must NOT be moving (executing/switching)
        const auto& cm_state = robot_->GetControlManagerState();
        bool is_any_gravity_compensation_active = 
            gravity_compensation_torso_ || 
            gravity_compensation_right_arm_ || 
            gravity_compensation_left_arm_;

        // We only enforce the active execution check if:
        // - We are enabling gravity compensation (request->state == true)
        // - AND gravity compensation is not already active on any body part.
        if (request->state && !is_any_gravity_compensation_active) {
            if (cm_state.control_state == rb::ControlManagerState::ControlState::kExecuting ||
                cm_state.control_state == rb::ControlManagerState::ControlState::kSwitching) {
                response->response = false;
                RCLCPP_WARN(this->get_logger(), "GravityCompensation: Robot is active in execution/switching state. Rejecting request.");
                return;
            }
        }

        // 2. Build and Send Gravity Compensation command
        auto b = rb::GravityCompensationCommandBuilder();
        b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(10.0)); // standard control hold time
        b.SetOn(request->state);

        auto body_comp = rb::BodyComponentBasedCommandBuilder();
        if (request->part_name == "torso") {
            body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
            gravity_compensation_torso_ = request->state;
        } else if (request->part_name == "right_arm") {
            body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
            gravity_compensation_right_arm_ = request->state;
        } else if (request->part_name == "left_arm") {
            body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
            gravity_compensation_left_arm_ = request->state;
        }

        auto component_cmd_builder = rb::ComponentBasedCommandBuilder();
        component_cmd_builder.SetBodyCommand(rb::BodyCommandBuilder(body_comp));

        try {
            if (request->state) {
                auto cmd_handler = robot_->SendCommand(rb::RobotCommandBuilder().SetCommand(component_cmd_builder));
                if (request->part_name == "torso") {
                    gravity_compensation_torso_handler_ = std::move(cmd_handler);
                } else if (request->part_name == "right_arm") {
                    gravity_compensation_right_arm_handler_ = std::move(cmd_handler);
                } else if (request->part_name == "left_arm") {
                    gravity_compensation_left_arm_handler_ = std::move(cmd_handler);
                }
            } else {
                // 1. Reset/cancel the original ON command handler first
                if (request->part_name == "torso") {
                    gravity_compensation_torso_handler_.reset();
                } else if (request->part_name == "right_arm") {
                    gravity_compensation_right_arm_handler_.reset();
                } else if (request->part_name == "left_arm") {
                    gravity_compensation_left_arm_handler_.reset();
                }

                // 2. Send explicit command to turn gravity compensation OFF and wait for completion
                auto cmd_handler = robot_->SendCommand(rb::RobotCommandBuilder().SetCommand(component_cmd_builder));
                if (cmd_handler) {
                    cmd_handler->WaitFor(1000); // Wait up to 1 second for transition to complete
                }
            }
            response->response = true;
            RCLCPP_INFO(this->get_logger(), "GravityCompensation: Successfully set %s gravity compensation to %s",
                request->part_name.c_str(), request->state ? "ON" : "OFF");
        } catch (const std::exception& e) {
            response->response = false;
            RCLCPP_ERROR(this->get_logger(), "GravityCompensation: Failed to send command: %s", e.what());
        }
    }

    template <typename ModelType>
    bool RBY1_ROS2_DRIVER<ModelType>::check_controll_manager(){
        const auto& control_manager_state = robot_->GetControlManagerState();

        if (control_manager_state.state == rb::ControlManagerState::State::kEnabled) {
            return true;
        }

        if (control_manager_state.state == rb::ControlManagerState::State::kMajorFault ||
            control_manager_state.state == rb::ControlManagerState::State::kMinorFault)
        {
            RCLCPP_WARN(this->get_logger(), "Detected a %s fault in the Control Manager. Attempting automatic reset...", 
                    (control_manager_state.state == rb::ControlManagerState::State::kMajorFault ? "Major" : "Minor"));
        
            if (!robot_->ResetFaultControlManager()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to reset the fault in the Control Manager.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Fault reset successfully.");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Control Manager state is normal. No faults detected.");
        }
        
        RCLCPP_INFO(this->get_logger(), "Enabling Control Manager...");
        if (!robot_->EnableControlManager()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable the Control Manager.");
            return false;
        }

        // Wait for control ready to ensure SendCommand doesn't fail with kUnknown immediately
        if (!robot_->WaitForControlReady(2000)) {
            RCLCPP_WARN(this->get_logger(), "Control Manager enabled, but timed out waiting for Control Ready status.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Control Manager enabled and ready.");
        }
        
        return true;
    }

    template <typename ModelType>
    std::string RBY1_ROS2_DRIVER<ModelType>::finish_code_to_string(rb::RobotCommandFeedback::FinishCode code) {
        switch (code) {
            case rb::RobotCommandFeedback::FinishCode::kUnknown: return "kUnknown";
            case rb::RobotCommandFeedback::FinishCode::kOk: return "kOk";
            case rb::RobotCommandFeedback::FinishCode::kCanceled: return "kCanceled";
            case rb::RobotCommandFeedback::FinishCode::kPreempted: return "kPreempted";
            case rb::RobotCommandFeedback::FinishCode::kInitializationFailed: return "kInitializationFailed";
            case rb::RobotCommandFeedback::FinishCode::kControlManagerIdle: return "kControlManagerIdle";
            case rb::RobotCommandFeedback::FinishCode::kControlManagerFault: return "kControlManagerFault";
            case rb::RobotCommandFeedback::FinishCode::kUnexpectedState: return "kUnexpectedState";
            default: return "Unknown";
        }
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::read_joint_state(){
        if (info_.joint_infos.empty()) return; // info가 아직 오지 않았으면 리턴
        try {
            auto state = robot_->GetState();
        auto cm_state = robot_->GetControlManagerState();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto now = this->now();
            
            if (cm_state.state == rb::ControlManagerState::State::kMinorFault) {
                robot_state_.state = MINOR_FAULT;
            } else if (cm_state.state == rb::ControlManagerState::State::kMajorFault) {
                robot_state_.state = MAJOR_FAULT;
            } else if (cm_state.state == rb::ControlManagerState::State::kIdle) {
                robot_state_.state = IDLE;
            } else if (cm_state.state == rb::ControlManagerState::State::kEnabled) {
                if (cm_state.control_state == rb::ControlManagerState::ControlState::kExecuting) {
                    robot_state_.state = EXECUTING;
                } else {
                    robot_state_.state = ENABLE;
                }
            } else {
                robot_state_.state = NONE;
            }

            // Collision check (respect collision_enable_)
            if (collision_enable_ && !state.collisions.empty()) {
                double min_dist = 1e9;
                std::string link1, link2;
                for (const auto& col : state.collisions) {
                    if (col.distance < min_dist) {
                        min_dist = col.distance;
                        link1 = col.link1;
                        link2 = col.link2;
                    }
                }
                if (min_dist < collision_threshold_) {
                    RCLCPP_ERROR(this->get_logger(), "Collision detected! Distance: %.4f (Links: %s <-> %s). Canceling control.", 
                                 min_dist, link1.c_str(), link2.c_str());
                    robot_->CancelControl();
                    if (stream_handler_) {
                        stream_handler_.reset();
                    }
                    is_control_canceled_ = true;
                    stream_active_ = false;
                }
            }
            
            auto fill = [&](JointState& js, const std::vector<unsigned int>& idx_vec){
                js.header.stamp = now;
                for (size_t i = 0; i < idx_vec.size(); ++i) {
                    unsigned int idx = idx_vec[i];
                    js.name[i]     = info_.joint_infos[idx].name;
                    js.position[i] = state.position[idx];
                    js.velocity[i] = state.velocity[idx];
                    js.effort[i]   = state.torque[idx];
                }
            };

            // Update internal joint state
            fill(this->robot_joint_.joint_torso,      this->info_.torso_joint_idx);
            fill(this->robot_joint_.joint_right_arm,  this->info_.right_arm_joint_idx);
            fill(this->robot_joint_.joint_left_arm,   this->info_.left_arm_joint_idx);
            fill(this->robot_joint_.joint_head,       this->info_.head_joint_idx);
            fill(this->robot_joint_.joint_wheel,      this->info_.mobility_joint_idx);

            this->torso_pub_->publish(this->robot_joint_.joint_torso);
            this->right_arm_pub_->publish(this->robot_joint_.joint_right_arm);
            this->left_arm_pub_->publish(this->robot_joint_.joint_left_arm);
            this->head_pub_->publish(this->robot_joint_.joint_head);

            // Base Mobility Odometry publishing
            {
                static double last_x = 0.0;
                static double last_y = 0.0;
                static double last_theta = 0.0;
                static bool first_odom = true;

                double x = state.odometry(0, 2);
                double y = state.odometry(1, 2);
                double theta = std::atan2(state.odometry(1, 0), state.odometry(0, 0));

                double vx = 0.0;
                double vy = 0.0;
                double wz = 0.0;

                double dt = robot_parameter_.get_state_period;
                if (dt > 0.0 && !first_odom) {
                    vx = (x - last_x) / dt;
                    vy = (y - last_y) / dt;
                    wz = (theta - last_theta) / dt;
                    if (wz > M_PI / dt) wz -= 2.0 * M_PI / dt;
                    if (wz < -M_PI / dt) wz += 2.0 * M_PI / dt;
                }
                first_odom = false;
                last_x = x;
                last_y = y;
                last_theta = theta;

                double cy = std::cos(theta * 0.5);
                double sy = std::sin(theta * 0.5);

                geometry_msgs::msg::Quaternion odom_quat;
                odom_quat.x = 0.0;
                odom_quat.y = 0.0;
                odom_quat.z = sy;
                odom_quat.w = cy;

                // TF frame prefixing is namespace-safe because ROS 2 handles base frames relative to node
                auto odom_msg = nav_msgs::msg::Odometry();
                odom_msg.header.stamp = now;
                odom_msg.header.frame_id = "odom";
                odom_msg.child_frame_id = "base_footprint";

                odom_msg.pose.pose.position.x = x;
                odom_msg.pose.pose.position.y = y;
                odom_msg.pose.pose.position.z = 0.0;
                odom_msg.pose.pose.orientation = odom_quat;

                odom_msg.twist.twist.linear.x = vx;
                odom_msg.twist.twist.linear.y = vy;
                odom_msg.twist.twist.angular.z = wz;

                odom_pub_->publish(odom_msg);

                geometry_msgs::msg::TransformStamped odom_tf;
                odom_tf.header.stamp = now;
                odom_tf.header.frame_id = "odom";
                odom_tf.child_frame_id = "base_footprint";

                odom_tf.transform.translation.x = x;
                odom_tf.transform.translation.y = y;
                odom_tf.transform.translation.z = 0.0;
                odom_tf.transform.rotation = odom_quat;

                tf_broadcaster_->sendTransform(odom_tf);
            }
            // Publish RobotState
            rby1_msgs::msg::RobotState robot_state_msg;
            robot_state_msg.control_manager_state = static_cast<int32_t>(robot_state_.state);

            auto fill_brakes = [&](std::vector<bool>& brake_vec, const std::vector<unsigned int>& idx_vec) {
                brake_vec.resize(idx_vec.size());
                for (size_t i = 0; i < idx_vec.size(); ++i) {
                    unsigned int idx = idx_vec[i];
                    bool has_brake = info_.joint_infos[idx].has_brake;
                    brake_vec[i] = has_brake && !state.joint_states[idx].is_ready;
                }
            };

            fill_brakes(robot_state_msg.brake_state.torso,     info_.torso_joint_idx);
            fill_brakes(robot_state_msg.brake_state.right_arm, info_.right_arm_joint_idx);
            fill_brakes(robot_state_msg.brake_state.left_arm,  info_.left_arm_joint_idx);
            fill_brakes(robot_state_msg.brake_state.head,       info_.head_joint_idx);

            bool left_connected = (state.tool_flange_left.time_since_last_update.tv_sec != 0 || state.tool_flange_left.time_since_last_update.tv_nsec != 0);
            bool right_connected = (state.tool_flange_right.time_since_last_update.tv_sec != 0 || state.tool_flange_right.time_since_last_update.tv_nsec != 0);
            robot_state_msg.tool_flange_state = {left_connected, right_connected};

            robot_state_msg.emo_state = false;
            if (!state.emo_states.empty()) {
                robot_state_msg.emo_state = (state.emo_states[0].state == rb::EMOState::State::kPressed);
            }

            robot_state_msg.center_of_mass[0] = state.center_of_mass[0];
            robot_state_msg.center_of_mass[1] = state.center_of_mass[1];
            robot_state_msg.center_of_mass[2] = state.center_of_mass[2];

            robot_state_pub_->publish(robot_state_msg);

            // Publish Battery State
            if (publish_battery_state_) {
                sensor_msgs::msg::BatteryState bat_msg;
                bat_msg.header.stamp = now;
                bat_msg.voltage = state.battery_state.voltage;
                bat_msg.current = state.battery_state.current;
                bat_msg.percentage = state.battery_state.level_percent / 100.0;
                battery_state_pub_->publish(bat_msg);
            }

            // Publish Tool Flange State split
            if (publish_tool_flange_state_) {
                auto fill_vec3 = [](std::array<double, 3>& dest, const Eigen::Vector<double, 3>& src, bool valid) {
                    if (valid) { dest[0] = src[0]; dest[1] = src[1]; dest[2] = src[2]; }
                    else        { dest[0] = 0.0;   dest[1] = 0.0;   dest[2] = 0.0; }
                };

                // left flange
                bool lf_valid = (state.tool_flange_left.time_since_last_update.tv_sec != 0 ||
                                 state.tool_flange_left.time_since_last_update.tv_nsec != 0);
                bool lf_ft_valid = (state.ft_sensor_left.time_since_last_update.tv_sec != 0 ||
                                    state.ft_sensor_left.time_since_last_update.tv_nsec != 0);
                rby1_msgs::msg::ToolFlangeState tf_left;
                fill_vec3(tf_left.ft_force,      state.ft_sensor_left.force,       lf_ft_valid);
                fill_vec3(tf_left.ft_torque,     state.ft_sensor_left.torque,      lf_ft_valid);
                fill_vec3(tf_left.gyro,          state.tool_flange_left.gyro,       lf_valid);
                fill_vec3(tf_left.acceleration,  state.tool_flange_left.acceleration, lf_valid);
                tf_left.switch_a        = lf_valid ? state.tool_flange_left.switch_A        : false;
                tf_left.output_voltage  = lf_valid ? state.tool_flange_left.output_voltage  : 0;
                tf_left.digital_input_a = lf_valid ? state.tool_flange_left.digital_input_A : false;
                tf_left.digital_input_b = lf_valid ? state.tool_flange_left.digital_input_B : false;
                tf_left.digital_output_a= lf_valid ? state.tool_flange_left.digital_output_A: false;
                tf_left.digital_output_b= lf_valid ? state.tool_flange_left.digital_output_B: false;
                tool_flange_left_pub_->publish(tf_left);

                // right flange
                bool rf_valid = (state.tool_flange_right.time_since_last_update.tv_sec != 0 ||
                                 state.tool_flange_right.time_since_last_update.tv_nsec != 0);
                bool rf_ft_valid = (state.ft_sensor_right.time_since_last_update.tv_sec != 0 ||
                                    state.ft_sensor_right.time_since_last_update.tv_nsec != 0);
                rby1_msgs::msg::ToolFlangeState tf_right;
                fill_vec3(tf_right.ft_force,      state.ft_sensor_right.force,        rf_ft_valid);
                fill_vec3(tf_right.ft_torque,     state.ft_sensor_right.torque,       rf_ft_valid);
                fill_vec3(tf_right.gyro,          state.tool_flange_right.gyro,        rf_valid);
                fill_vec3(tf_right.acceleration,  state.tool_flange_right.acceleration,rf_valid);
                tf_right.switch_a        = rf_valid ? state.tool_flange_right.switch_A        : false;
                tf_right.output_voltage  = rf_valid ? state.tool_flange_right.output_voltage  : 0;
                tf_right.digital_input_a = rf_valid ? state.tool_flange_right.digital_input_A : false;
                tf_right.digital_input_b = rf_valid ? state.tool_flange_right.digital_input_B : false;
                tf_right.digital_output_a= rf_valid ? state.tool_flange_right.digital_output_A: false;
                tf_right.digital_output_b= rf_valid ? state.tool_flange_right.digital_output_B: false;
                tool_flange_right_pub_->publish(tf_right);
            }

        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Error in read_joint_state loop: %s", e.what());
    }
}




    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::cancel_control_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        RCLCPP_INFO(this->get_logger(), "Cancel Control service called");
        is_control_canceled_ = true;
        robot_->CancelControl();
        if (stream_handler_) {
            stream_handler_->Cancel();
            stream_handler_.reset();
        }
        response->success = true;
        response->message = "Control cancelled";
    }

    // --- StreamPosition Handlers ---
    template <typename ModelType>
    rclcpp_action::GoalResponse RBY1_ROS2_DRIVER<ModelType>::handle_stream_goal(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const StreamPosition::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received StreamPosition request");
        (void)uuid;
        if (goal->trajectory.points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory has no points.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    template <typename ModelType>
    rclcpp_action::CancelResponse RBY1_ROS2_DRIVER<ModelType>::handle_stream_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamPosition>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel StreamPosition goal");
        (void)goal_handle;
        robot_->CancelControl();
        if (stream_handler_) {
            stream_handler_->Cancel();
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::handle_stream_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamPosition>> goal_handle) {
        using namespace std::placeholders;
        RCLCPP_INFO(this->get_logger(), "StreamPosition Goal accepted. Detaching execution thread...");
        std::thread{std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_stream_position, this, _1), goal_handle}.detach();
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::execute_stream_position(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamPosition>> goal_handle) {
        is_control_canceled_ = false;
        RCLCPP_INFO(this->get_logger(), "Starting StreamPosition execution thread");
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<StreamPosition::Result>();

        const auto& trajectory = goal->trajectory;
        std::vector<int> joint_mapping(trajectory.joint_names.size(), -1);
        
        for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
            for (size_t j = 0; j < info_.joint_infos.size(); ++j) {
                if (trajectory.joint_names[i] == info_.joint_infos[j].name) {
                    joint_mapping[i] = j;
                    break;
                }
            }
        }

        if (dynamics_ && dyn_state_ && q_lower_.size() > 0) {
            for (size_t i = 0; i < trajectory.points.size(); ++i) {
                const auto& point = trajectory.points[i];
                for (size_t j = 0; j < joint_mapping.size(); ++j) {
                    if (joint_mapping[j] != -1 && j < point.positions.size()) {
                        double pos = point.positions[j];
                        std::string joint_name = info_.joint_infos[joint_mapping[j]].name;

                        int pinocchio_idx = -1;
                        for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
                            if (dyn_joint_names_[k] == joint_name) {
                                pinocchio_idx = k;
                                break;
                            }
                        }
                        if (pinocchio_idx == -1) continue;

                        double min_p = q_lower_[pinocchio_idx];
                        double max_p = q_upper_[pinocchio_idx];

                        if (pos < min_p || pos > max_p) {
                            RCLCPP_ERROR(this->get_logger(), 
                                         "\033[1;31m[SAFETY ERROR] StreamPosition trajectory point %lu violates limit: "
                                         "joint '%s' position %.3f out of range [%.3f, %.3f] rad.\033[0m",
                                         i, joint_name.c_str(), pos, min_p, max_p);
                            result->success = false;
                            result->finish_code = "Safety Limit Exceeded: Commanded position for joint '" + joint_name + 
                                                  "' (" + std::to_string(pos) + " rad) is out of safety range [" + 
                                                  std::to_string(min_p) + ", " + std::to_string(max_p) + "] rad.";
                            try { goal_handle->abort(result); } catch (...) {}
                            return;
                        }
                    }
                }
            }
        }

        if (!stream_active_ || !stream_handler_) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory stream command rejected: stream is not active. Call /stream_control service first.");
            result->success = false;
            result->finish_code = "kRejected";
            try { goal_handle->abort(result); } catch (...) {}
            return;
        }

        // 1. Ensure robot is ready before creating stream
        this->check_controll_manager();
        usleep(100000); // 100ms safety gap

        auto current_state = robot_->GetState();
        Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(current_state.position.data(), current_state.position.size());

        try {
            for (size_t i = 0; i < trajectory.points.size(); ++i) {
                const auto& point = trajectory.points[i];
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->finish_code = "kCanceled";
                    try { goal_handle->canceled(result); } catch (...) {}
                    return;
                }

                // Update q with trajectory points - q accumulates changes over points
                for (size_t j = 0; j < joint_mapping.size(); ++j) {
                    if (joint_mapping[j] != -1 && j < point.positions.size()) {
                        q[joint_mapping[j]] = point.positions[j];
                    }
                }

                // Calculate timing from trajectory message (Delta between points)
                double pt_time = 0.01;
                if (i > 0) {
                    pt_time = (trajectory.points[i].time_from_start.sec + trajectory.points[i].time_from_start.nanosec * 1e-9) - 
                            (trajectory.points[i-1].time_from_start.sec + trajectory.points[i-1].time_from_start.nanosec * 1e-9);
                } else {
                    pt_time = trajectory.points[0].time_from_start.sec + trajectory.points[0].time_from_start.nanosec * 1e-9;
                }
                if (pt_time <= 0.0) pt_time = 0.01;

                double cmd_min_time = pt_time * 1.1;

                // Build Component-based commands for each part to ensure full-body synchronization
                // Build Component-based commands for each part using CORRECT indices
                Eigen::VectorXd torso_q(6);
                for (int i = 0; i < 6; ++i) torso_q[i] = q[info_.torso_joint_idx[i]];
                rb::TorsoCommandBuilder torso_builder;
                torso_builder.SetCommand(rb::JointPositionCommandBuilder().SetPosition(torso_q).SetMinimumTime(cmd_min_time));

                Eigen::VectorXd right_arm_q(7);
                for (int i = 0; i < 7; ++i) right_arm_q[i] = q[info_.right_arm_joint_idx[i]];
                rb::ArmCommandBuilder right_arm_builder;
                right_arm_builder.SetCommand(rb::JointPositionCommandBuilder().SetPosition(right_arm_q).SetMinimumTime(cmd_min_time));

                Eigen::VectorXd left_arm_q(7);
                for (int i = 0; i < 7; ++i) left_arm_q[i] = q[info_.left_arm_joint_idx[i]];
                rb::ArmCommandBuilder left_arm_builder;
                left_arm_builder.SetCommand(rb::JointPositionCommandBuilder().SetPosition(left_arm_q).SetMinimumTime(cmd_min_time));

                Eigen::VectorXd head_q(2);
                for (int i = 0; i < 2; ++i) head_q[i] = q[info_.head_joint_idx[i]];
                rb::HeadCommandBuilder head_builder;
                head_builder.SetCommand(rb::JointPositionCommandBuilder().SetPosition(head_q).SetMinimumTime(cmd_min_time));

                rb::BodyComponentBasedCommandBuilder body_comp_builder;
                body_comp_builder.SetTorsoCommand(torso_builder);
                body_comp_builder.SetRightArmCommand(right_arm_builder);
                body_comp_builder.SetLeftArmCommand(left_arm_builder);

                rb::ComponentBasedCommandBuilder comp_builder;
                comp_builder.SetBodyCommand(rb::BodyCommandBuilder(std::move(body_comp_builder)));
                comp_builder.SetHeadCommand(head_builder);

                rb::RobotCommandFeedback feedback;
                {
                    std::lock_guard<std::mutex> lock(stream_mutex_);
                    if (!stream_handler_) {
                        throw std::runtime_error("Stream was deactivated during transmission.");
                    }
                    feedback = stream_handler_->SendCommand(
                        rb::RobotCommandBuilder().SetCommand(comp_builder)
                    );
                }

                auto action_feedback = std::make_shared<StreamPosition::Feedback>();
                action_feedback->current_state = "streaming point " + std::to_string(i + 1) + "/" + std::to_string(trajectory.points.size());
                try { goal_handle->publish_feedback(action_feedback); } catch (...) {}

                // Wait for the duration of the point before sending the next one
                // to match the requested trajectory timing.
                auto sleep_duration = std::chrono::microseconds(static_cast<long long>(pt_time * 1e6));
                std::this_thread::sleep_for(sleep_duration);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error during stream transmission: %s", e.what());
            result->success = false;
            result->finish_code = "kError";
            try { goal_handle->abort(result); } catch (...) {}
            return;
        }

        double total_duration = 0.0;
        if (!trajectory.points.empty()) {
            total_duration = trajectory.points.back().time_from_start.sec + 
                             trajectory.points.back().time_from_start.nanosec * 1e-9;
        }
        uint32_t wait_ms = static_cast<uint32_t>((total_duration + 2.0) * 1000.0);
        RCLCPP_INFO(this->get_logger(), "Stream sent. Waiting for completion (%u ms)...", wait_ms);
        try {
            bool wait_success = false;
            {
                std::lock_guard<std::mutex> lock(stream_mutex_);
                if (!stream_handler_) {
                    throw std::runtime_error("Stream was deactivated while waiting.");
                }
                wait_success = stream_handler_->WaitFor(wait_ms);
            }
            if (!wait_success) {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for stream completion (%u ms).", wait_ms);
                result->success = false;
                result->finish_code = "kTimeout";
                try { goal_handle->abort(result); } catch (...) {}
                this->check_controll_manager();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Stream execution finished.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error during stream wait: %s", e.what());
            result->success = false;
            result->finish_code = "kError";
            try { goal_handle->abort(result); } catch (...) {}
            this->check_controll_manager();
            return;
        }

        result->success = true;
        result->finish_code = "kOk";
        try { goal_handle->succeed(result); } catch (...) {}
    }

    // --- Rby1 Joint Command Handlers ---
    template <typename ModelType>
    rclcpp_action::GoalResponse RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_goal(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Rby1JointCommand::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received Rby1JointCommand request");
        (void)uuid;
        (void)goal; // We will do validation in execute
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    template <typename ModelType>
    rclcpp_action::CancelResponse RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1JointCommand>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel Rby1JointCommand goal");
        (void)goal_handle;
        robot_->CancelControl();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1JointCommand>> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_rby1_joint_command, this, _1), goal_handle}.detach();
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::execute_rby1_joint_command(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1JointCommand>> goal_handle) {
        is_control_canceled_ = false;
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<Rby1JointCommand::Result>();

        try {
            auto cm_state_val = robot_->GetControlManagerState().state;
            bool is_ready = (cm_state_val == rb::ControlManagerState::State::kEnabled);
            if (!is_ready) {
                try {
                    is_ready = robot_->IsPowerOn(".*") && robot_->IsServoOn(".*");
                } catch (...) {
                    is_ready = false;
                }
            }

            if (!is_ready) {
                result->success = false;
                result->finish_code = "Robot is disabled (power/servo is off)";
                RCLCPP_WARN(this->get_logger(), "\033[1;33m[CONTROL REJECTED] Joint command rejected: Robot is disabled (power/servo is off).\033[0m");
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }

            if (!goal->torso.position.empty() && gravity_compensation_torso_) {
                result->success = false;
                result->finish_code = "Torso is in Gravity Compensation mode!";
                RCLCPP_WARN(this->get_logger(), "\033[1;33m[CONTROL REJECTED] Joint command rejected: Torso is in Gravity Compensation mode!\033[0m");
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }
            if (!goal->right_arm.position.empty() && gravity_compensation_right_arm_) {
                result->success = false;
                result->finish_code = "Right arm is in Gravity Compensation mode!";
                RCLCPP_WARN(this->get_logger(), "\033[1;33m[CONTROL REJECTED] Joint command rejected: Right arm is in Gravity Compensation mode!\033[0m");
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }
            if (!goal->left_arm.position.empty() && gravity_compensation_left_arm_) {
                result->success = false;
                result->finish_code = "Left arm is in Gravity Compensation mode!";
                RCLCPP_WARN(this->get_logger(), "\033[1;33m[CONTROL REJECTED] Joint command rejected: Left arm is in Gravity Compensation mode!\033[0m");
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }

            if (!robot_->HasEstablishedTimeSync()) robot_->SyncTime();

            if (!this->check_controll_manager()) {
                RCLCPP_ERROR(this->get_logger(), "\033[1;31m[CONTROL MANAGER ERROR] Control Manager not ready for Rby1JointCommand.\033[0m");
                result->success = false;
                result->finish_code = "Control Manager Error";
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }

            rb::ComponentBasedCommandBuilder component_cmd_builder;
            rb::BodyComponentBasedCommandBuilder body_comp;
            bool use_body = false;
            bool use_head = false;

            auto process_part = [&](const rby1_msgs::msg::JointCommand& cmd, const std::string& part_name, size_t expected_dof, std::string& err_msg) -> bool {
                if (cmd.position.empty()) return true; // Ignore empty parts

                Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(cmd.position.data(), cmd.position.size());
                Eigen::VectorXd clamped_vel_limits(cmd.position.size());
                Eigen::VectorXd clamped_accel_limits(cmd.position.size());

                for (size_t i = 0; i < cmd.position.size(); ++i) {
                    clamped_vel_limits[i] = cmd.velocity_limit;
                    clamped_accel_limits[i] = cmd.acceleration_limit;
                }

                if (dynamics_ && dyn_state_ && q_lower_.size() > 0) {
                    if (cmd.use_group_joint) {
                        if (cmd.position.size() == cmd.joint_names.size()) {
                            for (size_t i = 0; i < cmd.joint_names.size(); ++i) {
                                std::string commanded_name = cmd.joint_names[i];
                                
                                int pinocchio_idx = -1;
                                for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
                                    if (dyn_joint_names_[k] == commanded_name) {
                                        pinocchio_idx = k;
                                        break;
                                    }
                                }
                                if (pinocchio_idx == -1) continue;

                                double pos = cmd.position[i];
                                double min_p = q_lower_[pinocchio_idx];
                                double max_p = q_upper_[pinocchio_idx];

                                // 1. Strict Position Limit Validation
                                if (pos < min_p || pos > max_p) {
                                    err_msg = "Safety Limit Exceeded: Commanded position for joint '" + commanded_name + 
                                              "' (" + std::to_string(pos) + " rad) is out of safety range [" + 
                                              std::to_string(min_p) + ", " + std::to_string(max_p) + "] rad.";
                                    RCLCPP_ERROR(this->get_logger(), "\033[1;31m[SAFETY ERROR] %s\033[0m", err_msg.c_str());
                                    return false;
                                }


                            }
                        }
                    } else {
                        const std::vector<unsigned int>* part_indices = nullptr;
                        if (part_name == "torso") part_indices = &info_.torso_joint_idx;
                        else if (part_name == "right_arm") part_indices = &info_.right_arm_joint_idx;
                        else if (part_name == "left_arm") part_indices = &info_.left_arm_joint_idx;
                        else if (part_name == "head") part_indices = &info_.head_joint_idx;

                        if (part_indices && cmd.position.size() == part_indices->size()) {
                            for (size_t i = 0; i < cmd.position.size(); ++i) {
                                size_t global_idx = (*part_indices)[i];
                                std::string joint_name = info_.joint_infos[global_idx].name;

                                int pinocchio_idx = -1;
                                for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
                                    if (dyn_joint_names_[k] == joint_name) {
                                        pinocchio_idx = k;
                                        break;
                                    }
                                }
                                if (pinocchio_idx == -1) continue;

                                double pos = cmd.position[i];
                                double min_p = q_lower_[pinocchio_idx];
                                double max_p = q_upper_[pinocchio_idx];

                                // 1. Strict Position Limit Validation
                                if (pos < min_p || pos > max_p) {
                                    err_msg = "Safety Limit Exceeded: Commanded position for joint '" + joint_name + 
                                              "' (" + std::to_string(pos) + " rad) is out of safety range [" + 
                                              std::to_string(min_p) + ", " + std::to_string(max_p) + "] rad.";
                                    RCLCPP_ERROR(this->get_logger(), "\033[1;31m[SAFETY ERROR] %s\033[0m", err_msg.c_str());
                                    return false;
                                }


                            }
                        }
                    }
                }

                if (cmd.use_group_joint && part_name == "torso") {
                    if (cmd.joint_names.empty()) {
                        err_msg = "Empty Joint Names: " + part_name + " joint_names cannot be empty when use_group_joint is True.";
                        RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
                        return false;
                    }
                    if (cmd.position.size() != cmd.joint_names.size()) {
                        err_msg = "Invalid Array Size: " + part_name + " position size (" + std::to_string(cmd.position.size()) + ") must match joint_names size (" + std::to_string(cmd.joint_names.size()) + ") for Joint Group Control.";
                        RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
                        return false;
                    }
                    auto b = rb::JointGroupPositionCommandBuilder();
                    b.SetJointNames(cmd.joint_names);
                    b.SetPosition(q);
                    b.SetMinimumTime(cmd.minimum_time);
                    if (cmd.velocity_limit > 0.0) {
                        b.SetVelocityLimit(clamped_vel_limits);
                    }
                    if (cmd.acceleration_limit > 0.0) {
                        b.SetAccelerationLimit(clamped_accel_limits);
                    }
                    body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
                    use_body = true;
                    return true;
                }

                if (cmd.use_group_joint) {
                    RCLCPP_WARN(this->get_logger(), "Joint group control is only supported for torso. Falling back to standard control for %s", part_name.c_str());
                }

                if (cmd.position.size() != expected_dof) {
                    err_msg = "Invalid Array Size: " + part_name + " position requires " + std::to_string(expected_dof) + " elements, got " + std::to_string(cmd.position.size());
                    RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
                    return false;
                }

                if (cmd.use_impedance) {
                    if (part_name == "head") {
                        RCLCPP_WARN(this->get_logger(), "Head does not support impedance control. Falling back to joint position control.");
                        auto b = rb::JointPositionCommandBuilder();
                        b.SetMinimumTime(cmd.minimum_time);
                        b.SetPosition(q);
                        component_cmd_builder.SetHeadCommand(rb::HeadCommandBuilder(b));
                        use_head = true;
                    } else {
                        auto b = rb::JointImpedanceControlCommandBuilder();
                        b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(cmd.control_hold_time));
                        b.SetPosition(q);
                        b.SetMinimumTime(cmd.minimum_time);

                        if (cmd.velocity_limit > 0.0) {
                            b.SetVelocityLimit(Eigen::VectorXd::Constant(expected_dof, cmd.velocity_limit));
                        }
                        if (cmd.acceleration_limit > 0.0) {
                            b.SetAccelerationLimit(Eigen::VectorXd::Constant(expected_dof, cmd.acceleration_limit));
                        }
                        
                        std::vector<double> stiffness = cmd.stiffness;
                        if (stiffness.empty()) stiffness.assign(expected_dof, 100.0);
                        else if (stiffness.size() != expected_dof) {
                            err_msg = "Invalid Array Size: " + part_name + " stiffness requires " + std::to_string(expected_dof) + " elements, got " + std::to_string(stiffness.size());
                            RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
                            return false;
                        }
                        b.SetStiffness(Eigen::Map<const Eigen::VectorXd>(stiffness.data(), stiffness.size()));
                        b.SetDampingRatio(cmd.damping_ratio);
                        b.SetTorqueLimit(Eigen::VectorXd::Constant(expected_dof, cmd.torque_limit));

                        if (part_name == "torso") body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
                        else if (part_name == "right_arm") body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
                        else if (part_name == "left_arm") body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
                        use_body = true;
                    }
                } else {
                    auto b = rb::JointPositionCommandBuilder();
                    b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(cmd.control_hold_time));
                    b.SetPosition(q);
                    b.SetMinimumTime(cmd.minimum_time);

                    if (cmd.velocity_limit > 0.0) {
                        b.SetVelocityLimit(Eigen::VectorXd::Constant(expected_dof, cmd.velocity_limit));
                    }
                    if (cmd.acceleration_limit > 0.0) {
                        b.SetAccelerationLimit(Eigen::VectorXd::Constant(expected_dof, cmd.acceleration_limit));
                    }
                    
                    if (part_name == "torso") body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
                    else if (part_name == "right_arm") body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
                    else if (part_name == "left_arm") body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
                    else if (part_name == "head") {
                        component_cmd_builder.SetHeadCommand(rb::HeadCommandBuilder(b));
                        use_head = true;
                        return true;
                    }
                    use_body = true;
                }
                return true;
            };

            std::string err_msg;

            if (!process_part(goal->torso, "torso", info_.torso_joint_idx.size(), err_msg)) {
                result->success = false;
                result->finish_code = err_msg;
                goal_handle->abort(result);
                return;
            }
            if (!process_part(goal->right_arm, "right_arm", info_.right_arm_joint_idx.size(), err_msg)) {
                result->success = false;
                result->finish_code = err_msg;
                goal_handle->abort(result);
                return;
            }
            if (!process_part(goal->left_arm, "left_arm", info_.left_arm_joint_idx.size(), err_msg)) {
                result->success = false;
                result->finish_code = err_msg;
                goal_handle->abort(result);
                return;
            }
            if (!process_part(goal->head, "head", info_.head_joint_idx.size(), err_msg)) {
                result->success = false;
                result->finish_code = err_msg;
                goal_handle->abort(result);
                return;
            }

            if (!use_body && !use_head) {
                RCLCPP_WARN(this->get_logger(), "Received empty Rby1JointCommand goal.");
                result->success = false;
                result->finish_code = "Empty arrays";
                goal_handle->abort(result);
                return;
            }

            if (use_body) {
                component_cmd_builder.SetBodyCommand(rb::BodyCommandBuilder(body_comp));
            }



            auto cmd_handler = robot_->SendCommand(rb::RobotCommandBuilder().SetCommand(component_cmd_builder), goal->priority);
            
            rclcpp::Rate rate(10);
            while (rclcpp::ok() && !cmd_handler->IsDone()) {
                if (goal_handle->is_canceling()) {
                    cmd_handler->Cancel();
                    result->success = false;
                    result->finish_code = "kCanceled";
                    try { goal_handle->canceled(result); } catch (...) {}
                    return;
                }

                auto cm_state = robot_->GetControlManagerState();
                if (cm_state.state == rb::ControlManagerState::State::kMajorFault ||
                    cm_state.state == rb::ControlManagerState::State::kMinorFault) {
                    cmd_handler->Cancel();
                    result->success = false;
                    result->finish_code = "Fault Detected";
                    try { goal_handle->abort(result); } catch (...) {}
                    return;
                }

                auto feedback = std::make_shared<Rby1JointCommand::Feedback>();
                feedback->current_state = "excuting";
                try { goal_handle->publish_feedback(feedback); } catch (...) {}
                rate.sleep();
            }

            if (rclcpp::ok()) {
                auto rv = cmd_handler->Get();
                result->finish_code = this->finish_code_to_string(rv.finish_code());
                
                if (is_control_canceled_) result->finish_code = "kCanceled";
                
                RCLCPP_INFO(this->get_logger(), "Rby1JointCommand finished with code: %s", result->finish_code.c_str());
                result->success = (result->finish_code == "kOk");
                
                if (result->success) {
                    try { goal_handle->succeed(result); } catch (...) {}
                } else {
                    try { goal_handle->abort(result); } catch (...) {}
                    RCLCPP_INFO(this->get_logger(), "Rby1JointCommand failed. Attempting to recover control manager...");
                    this->check_controll_manager();
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in Rby1JointCommand: %s", e.what());
            result->success = false;
            result->finish_code = "kError";
            try { goal_handle->abort(result); } catch (...) {}
            this->check_controll_manager();
        }
    }

    // --- Rby1 Cartesian Command Handlers ---
    template <typename ModelType>
    rclcpp_action::GoalResponse RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_goal(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Rby1CartesianCommand::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received Rby1CartesianCommand request");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    template <typename ModelType>
    rclcpp_action::CancelResponse RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1CartesianCommand>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel Rby1CartesianCommand goal");
        (void)goal_handle;
        robot_->CancelControl();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1CartesianCommand>> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_rby1_cartesian_command, this, _1), goal_handle}.detach();
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::execute_rby1_cartesian_command(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1CartesianCommand>> goal_handle) {
        is_control_canceled_ = false;
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<Rby1CartesianCommand::Result>();

        try {
            auto cm_state_val = robot_->GetControlManagerState().state;
            bool is_ready = (cm_state_val == rb::ControlManagerState::State::kEnabled);
            if (!is_ready) {
                try {
                    is_ready = robot_->IsPowerOn(".*") && robot_->IsServoOn(".*");
                } catch (...) {
                    is_ready = false;
                }
            }

            if (!is_ready) {
                result->success = false;
                result->finish_code = "Robot is disabled (power/servo is off)";
                RCLCPP_WARN(this->get_logger(), "\033[1;33m[CONTROL REJECTED] Cartesian command rejected: Robot is disabled (power/servo is off).\033[0m");
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }

            if ((!goal->torso.ref_link.empty() || !goal->torso.target_link.empty()) && gravity_compensation_torso_) {
                result->success = false;
                result->finish_code = "Torso is in Gravity Compensation mode!";
                RCLCPP_WARN(this->get_logger(), "\033[1;33m[CONTROL REJECTED] Cartesian command rejected: Torso is in Gravity Compensation mode!\033[0m");
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }
            if ((!goal->right_arm.ref_link.empty() || !goal->right_arm.target_link.empty()) && gravity_compensation_right_arm_) {
                result->success = false;
                result->finish_code = "Right arm is in Gravity Compensation mode!";
                RCLCPP_WARN(this->get_logger(), "\033[1;33m[CONTROL REJECTED] Cartesian command rejected: Right arm is in Gravity Compensation mode!\033[0m");
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }
            if ((!goal->left_arm.ref_link.empty() || !goal->left_arm.target_link.empty()) && gravity_compensation_left_arm_) {
                result->success = false;
                result->finish_code = "Left arm is in Gravity Compensation mode!";
                RCLCPP_WARN(this->get_logger(), "\033[1;33m[CONTROL REJECTED] Cartesian command rejected: Left arm is in Gravity Compensation mode!\033[0m");
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }

            if (!robot_->HasEstablishedTimeSync()) robot_->SyncTime();

            if (!this->check_controll_manager()) {
                RCLCPP_ERROR(this->get_logger(), "\033[1;31m[CONTROL MANAGER ERROR] Control Manager not ready for Rby1CartesianCommand.\033[0m");
                result->success = false;
                result->finish_code = "Control Manager Error";
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }

            rb::ComponentBasedCommandBuilder component_cmd_builder;
            rb::BodyComponentBasedCommandBuilder body_comp;
            bool use_body = false;

            auto process_part = [&](const rby1_msgs::msg::CartesianCommand& cmd, const std::string& part_name, std::string& err_msg) -> bool {
                if (cmd.ref_link.empty() && cmd.target_link.empty()) return true; // Ignore empty parts

                if (cmd.ref_link.empty() || cmd.target_link.empty()) {
                    err_msg = "Invalid CartesianCommand: Both ref_link and target_link must be provided for " + part_name;
                    RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
                    return false;
                }

                auto get_weight = [](const std::vector<double>& input, double default_val, const std::string& err_prefix, std::string& err) -> std::optional<Eigen::Vector3d> {
                    if (input.empty()) return Eigen::Vector3d(default_val, default_val, default_val);
                    if (input.size() != 3) {
                        err = err_prefix + " requires 3 elements, got " + std::to_string(input.size());
                        return std::nullopt;
                    }
                    return Eigen::Vector3d(input[0], input[1], input[2]);
                };

                auto t_weight = get_weight(cmd.translation_weight, 1000.0, "Invalid Array Size: " + part_name + " translation_weight", err_msg);
                if (!t_weight) return false;

                auto r_weight = get_weight(cmd.rotation_weight, 100.0, "Invalid Array Size: " + part_name + " rotation_weight", err_msg);
                if (!r_weight) return false;

                Eigen::Quaterniond q(cmd.transform.rotation.w, cmd.transform.rotation.x, cmd.transform.rotation.y, cmd.transform.rotation.z);
                Eigen::Vector3d t(cmd.transform.translation.x, cmd.transform.translation.y, cmd.transform.translation.z);
                
                rb::math::SE3::MatrixType T = rb::math::SE3::MatrixType::Identity();
                T.block(0, 0, 3, 3) = q.toRotationMatrix();
                T.block(0, 3, 3, 1) = t;

                if (cmd.use_impedance) {
                    auto b = rb::ImpedanceControlCommandBuilder();
                    b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(cmd.control_hold_time));
                    b.SetReferenceLinkName(cmd.ref_link);
                    b.SetLinkName(cmd.target_link);
                    b.SetTranslationWeight(*t_weight);
                    b.SetRotationWeight(*r_weight);
                    b.SetTransformation(T);

                    if (cmd.add_joint_position_target) {
                        RCLCPP_WARN(this->get_logger(), "add_joint_position_target is not supported for Cartesian Impedance Control in this SDK version. Ignoring for %s", part_name.c_str());
                    }
                    
                    if (part_name == "right_arm") body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
                    else if (part_name == "left_arm") body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
                    else if (part_name == "torso") body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
                    
                    use_body = true;
                    return true;
                } else {
                    auto b = rb::CartesianCommandBuilder();
                    b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(cmd.control_hold_time));
                    b.SetMinimumTime(cmd.minimum_time);

                    double lin_vel = (cmd.linear_velocity_limit <= 0.0) ? robot_parameter_.linear_velocity_limit : cmd.linear_velocity_limit;
                    double ang_vel = (cmd.angular_velocity_limit <= 0.0) ? robot_parameter_.angular_velocity_limit : cmd.angular_velocity_limit;
                    double acc_scale = (cmd.acceleration_limit_scaling <= 0.0) ? robot_parameter_.acceleration_limit : cmd.acceleration_limit_scaling;

                    b.AddTarget(cmd.ref_link, cmd.target_link, T, lin_vel, ang_vel, acc_scale);
                    b.SetStopPositionTrackingError(goal->stop_position_tracking_error);
                    b.SetStopOrientationTrackingError(goal->stop_orientation_tracking_error);

                    if (cmd.add_joint_position_target) {
                        int pinocchio_idx = -1;
                        for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
                            if (dyn_joint_names_[k] == cmd.add_joint_name) {
                                pinocchio_idx = k;
                                break;
                            }
                        }
                        if (pinocchio_idx != -1) {
                            double min_p = q_lower_[pinocchio_idx];
                            double max_p = q_upper_[pinocchio_idx];
                            if (cmd.add_joint_value < min_p || cmd.add_joint_value > max_p) {
                                err_msg = "Safety Limit Exceeded: Commanded additional joint '" + cmd.add_joint_name + 
                                          "' value (" + std::to_string(cmd.add_joint_value) + " rad) is out of safety range [" + 
                                          std::to_string(min_p) + ", " + std::to_string(max_p) + "] rad.";
                                RCLCPP_ERROR(this->get_logger(), "\033[1;31m[SAFETY ERROR] %s\033[0m", err_msg.c_str());
                                return false;
                            }
                        }
                        b.AddJointPositionTarget(cmd.add_joint_name, cmd.add_joint_value);
                    }
                    
                    if (part_name == "right_arm") body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
                    else if (part_name == "left_arm") body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
                    else if (part_name == "torso") body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
                    
                    use_body = true;
                    return true;
                }
            };

            std::string err_msg;

            if (!process_part(goal->torso, "torso", err_msg)) {
                result->success = false;
                result->finish_code = err_msg;
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }
            if (!process_part(goal->right_arm, "right_arm", err_msg)) {
                result->success = false;
                result->finish_code = err_msg;
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }
            if (!process_part(goal->left_arm, "left_arm", err_msg)) {
                result->success = false;
                result->finish_code = err_msg;
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }

            if (!use_body) {
                RCLCPP_WARN(this->get_logger(), "Received empty Rby1CartesianCommand goal.");
                result->success = false;
                result->finish_code = "Empty targets";
                try { goal_handle->abort(result); } catch (...) {}
                return;
            }

            component_cmd_builder.SetBodyCommand(rb::BodyCommandBuilder(body_comp));



            auto cmd_handler = robot_->SendCommand(rb::RobotCommandBuilder().SetCommand(component_cmd_builder), goal->priority);
            
            rclcpp::Rate rate(10);
            while (rclcpp::ok() && !cmd_handler->IsDone()) {
                if (goal_handle->is_canceling()) {
                    cmd_handler->Cancel();
                    result->success = false;
                    result->finish_code = "kCanceled";
                    try { goal_handle->canceled(result); } catch (...) {}
                    return;
                }

                auto cm_state = robot_->GetControlManagerState();
                if (cm_state.state == rb::ControlManagerState::State::kMajorFault ||
                    cm_state.state == rb::ControlManagerState::State::kMinorFault) {
                    cmd_handler->Cancel();
                    result->success = false;
                    result->finish_code = "Fault Detected";
                    try { goal_handle->abort(result); } catch (...) {}
                    return;
                }

                rate.sleep();
            }

            if (rclcpp::ok()) {
                auto rv = cmd_handler->Get();
                result->finish_code = this->finish_code_to_string(rv.finish_code());
                
                if (is_control_canceled_) result->finish_code = "kCanceled";
                
                RCLCPP_INFO(this->get_logger(), "Rby1CartesianCommand finished with code: %s", result->finish_code.c_str());
                result->success = (result->finish_code == "kOk");
                
                if (result->success) {
                    try { goal_handle->succeed(result); } catch (...) {}
                } else {
                    try { goal_handle->abort(result); } catch (...) {}
                    RCLCPP_INFO(this->get_logger(), "Rby1CartesianCommand failed. Attempting to recover control manager...");
                    this->check_controll_manager();
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in Rby1CartesianCommand: %s", e.what());
            result->success = false;
            result->finish_code = "kError";
            try { goal_handle->abort(result); } catch (...) {}
            this->check_controll_manager();
        }
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::resize_joint_states(){
        constexpr size_t kTorsoDOF    = 6;
        constexpr size_t kArmDOF      = 7;
        constexpr size_t kHeadDOF     = 2;

        auto resize_js = [](JointState& js, size_t n) {
            js.name.assign(n, "");
            js.position.assign(n, 0.0);
            js.velocity.assign(n, 0.0);
            js.effort.assign(n, 0.0);
        };

        resize_js(robot_joint_.joint_torso,     kTorsoDOF);
        resize_js(robot_joint_.joint_right_arm,  kArmDOF);
        resize_js(robot_joint_.joint_left_arm,   kArmDOF);
        resize_js(robot_joint_.joint_head,       kHeadDOF);
        
        size_t kMobilityDOF = info_.mobility_joint_idx.size();
        if (kMobilityDOF == 0) {
            if (model == "a" || model == "A") kMobilityDOF = 2;
            else if (model == "m" || model == "M") kMobilityDOF = 4;
        }
        resize_js(robot_joint_.joint_wheel, kMobilityDOF);
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::get_cartesian_pose_callback(
        const std::shared_ptr<rby1_msgs::srv::GetCartesianPose::Request> request,
        std::shared_ptr<rby1_msgs::srv::GetCartesianPose::Response> response) {
        
        if (!dynamics_ || !dyn_state_) {
            RCLCPP_ERROR(this->get_logger(), "Dynamics model not loaded. Cannot get cartesian pose.");
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        try {
            auto state = robot_->GetState();
            Eigen::Vector<double, ModelType::kRobotDOF> q = Eigen::Vector<double, ModelType::kRobotDOF>::Zero();
            auto dyn_joint_names = dyn_state_->GetJointNames();
            for (int i = 0; i < (int)dyn_joint_names.size(); ++i) {
                std::string name(dyn_joint_names[i]);
                for (size_t j = 0; j < info_.joint_infos.size(); ++j) {
                    if (info_.joint_infos[j].name == name) {
                        q[i] = state.position[j];
                        break;
                    }
                }
            }
            
            dyn_state_->SetQ(q);
            dynamics_->ComputeForwardKinematics(dyn_state_);

            std::string ref = request->ref_link;
            std::string target = request->target_link;

            auto it_ref = std::find(dyn_link_names_.begin(), dyn_link_names_.end(), ref);
            auto it_target = std::find(dyn_link_names_.begin(), dyn_link_names_.end(), target);
            
            if (it_ref != dyn_link_names_.end() && it_target != dyn_link_names_.end()) {
                int ref_idx = std::distance(dyn_link_names_.begin(), it_ref);
                int target_idx = std::distance(dyn_link_names_.begin(), it_target);
                
                auto T = dynamics_->ComputeTransformation(dyn_state_, ref_idx, target_idx);
                
                geometry_msgs::msg::Transform transform;
                transform.translation.x = T(0, 3);
                transform.translation.y = T(1, 3);
                transform.translation.z = T(2, 3);
                Eigen::Matrix3d R = T.block(0, 0, 3, 3);
                Eigen::Quaterniond q_rot(R);
                transform.rotation.x = q_rot.x();
                transform.rotation.y = q_rot.y();
                transform.rotation.z = q_rot.z();
                transform.rotation.w = q_rot.w();
                
                response->transform = transform;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Link name not found in dynamics model. Ref: %s, Target: %s", ref.c_str(), target.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in get_cartesian_pose: %s", e.what());
        }
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::control_manager_callback(
        const std::shared_ptr<rby1_msgs::srv::ControlManagerCommand::Request> request,
        std::shared_ptr<rby1_msgs::srv::ControlManagerCommand::Response> response) {
        
        try {
            if (request->command == rby1_msgs::srv::ControlManagerCommand::Request::CMD_ENABLE) {
                RCLCPP_INFO(this->get_logger(), "Received Control Manager ENABLE request");
                if (robot_->EnableControlManager()) {
                    response->success = true;
                    response->message = "Control Manager enabled successfully.";
                } else {
                    response->success = false;
                    response->message = "Failed to enable Control Manager.";
                }
            } else if (request->command == rby1_msgs::srv::ControlManagerCommand::Request::CMD_DISABLE) {
                RCLCPP_INFO(this->get_logger(), "Received Control Manager DISABLE request");
                if (robot_->DisableControlManager()) {
                    response->success = true;
                    response->message = "Control Manager disabled successfully.";
                } else {
                    response->success = false;
                    response->message = "Failed to disable Control Manager.";
                }
            } else if (request->command == rby1_msgs::srv::ControlManagerCommand::Request::CMD_RESET) {
                RCLCPP_INFO(this->get_logger(), "Received Control Manager RESET request");
                if (robot_->ResetFaultControlManager()) {
                    response->success = true;
                    response->message = "Control Manager reset successfully.";
                } else {
                    response->success = false;
                    response->message = "Failed to reset Control Manager.";
                }
            } else {
                response->success = false;
                response->message = "Invalid command type provided.";
                RCLCPP_ERROR(this->get_logger(), "Control Manager command: Invalid command type %d", request->command);
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Exception occurred: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Control Manager exception: %s", e.what());
        }
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::motor_brake_callback(
        const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
        std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response) {
        
        try {
            // Check control manager state (Brakes can only be changed when IDLE)
            const auto& cm_state = robot_->GetControlManagerState();
            if (cm_state.state != rb::ControlManagerState::State::kIdle) {
                response->success = false;
                response->message = "Brakes can only be engaged/released when the Control Manager is in IDLE/DISABLED state!";
                RCLCPP_WARN(this->get_logger(), "\033[1;33m[BRAKE REJECTED] Brake command rejected: Robot is not in IDLE state.\033[0m");
                return;
            }

            // Safety Enhancement: Absolutely reject brake operations when 48V power is active
            if (robot_->IsPowerOn("48v")) {
                response->success = false;
                response->message = "Brakes cannot be engaged/released while 48V power is active!";
                RCLCPP_WARN(this->get_logger(), "\033[1;33m[BRAKE REJECTED] Brake command rejected: 48V power is active.\033[0m");
                return;
            }

            if (request->parameters.empty()) {
                response->success = false;
                response->message = "Joint/Device name parameter is required!";
                RCLCPP_ERROR(this->get_logger(), "[BRAKE ERROR] Joint/Device name is empty.");
                return;
            }

            bool op_success = false;
            bool has_brake_hardware = false;
            for (const auto& joint : info_.joint_infos) {
                if (joint.name == request->parameters) {
                    has_brake_hardware = joint.has_brake;
                    break;
                }
            }

            if (!has_brake_hardware) {
                RCLCPP_WARN(this->get_logger(), "Joint '%s' does not have brake hardware (or running in simulation). Mocking success.", request->parameters.c_str());
                op_success = true;
            } else {
                if (request->state) {
                    RCLCPP_INFO(this->get_logger(), "Received Brake ENGAGE request for joint: %s", request->parameters.c_str());
                    op_success = robot_->BrakeEngage(request->parameters);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Received Brake RELEASE request for joint: %s", request->parameters.c_str());
                    op_success = robot_->BrakeRelease(request->parameters);
                }
            }

            if (op_success) {
                response->success = true;
                response->message = std::string("Brake command executed successfully for ") + request->parameters;
            } else {
                response->success = false;
                response->message = std::string("Failed to execute brake command for ") + request->parameters;
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Exception occurred: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Brake command exception: %s", e.what());
        }
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        try {
            double linear_y = msg->linear.y;
            Eigen::Vector2d linear_vel(msg->linear.x, linear_y);
            double angular_vel = msg->angular.z;

            rb::SE2VelocityCommandBuilder se2_cmd;
            se2_cmd.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(0.5));
            se2_cmd.SetMinimumTime(robot_parameter_.se2_minimum_time);
            se2_cmd.SetVelocity(linear_vel, angular_vel);
            se2_cmd.SetAccelerationLimit(
                Eigen::Vector2d(robot_parameter_.se2_linear_acceleration_limit, robot_parameter_.se2_linear_acceleration_limit),
                robot_parameter_.se2_angular_acceleration_limit
            );

            rb::MobilityCommandBuilder mobility_cmd;
            mobility_cmd.SetCommand(se2_cmd);

            rb::ComponentBasedCommandBuilder component_cmd;
            component_cmd.SetMobilityCommand(mobility_cmd);

            rb::RobotCommandBuilder robot_cmd;
            robot_cmd.SetCommand(component_cmd);

            robot_->SendCommand(robot_cmd);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in cmd_vel_callback: %s", e.what());
        }
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::stream_control_callback(
        const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
        std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (request->state) {
            std::lock_guard<std::mutex> stream_lock(stream_mutex_);
            if (stream_active_ && stream_handler_) {
                response->success = true;
                response->message = "Persistent stream control is already active.";
                RCLCPP_INFO(this->get_logger(), "Persistent stream control is already active. Ignoring request.");
                return;
            }
            try {
                // When stream is ON, create command stream
                stream_handler_ = robot_->CreateCommandStream();
                stream_active_ = true;
                response->success = true;
                response->message = "Persistent stream control activated successfully.";
                RCLCPP_INFO(this->get_logger(), "Persistent stream control activated.");
            } catch (const std::exception& e) {
                response->success = false;
                response->message = std::string("Failed to activate stream control: ") + e.what();
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            }
        } else {
            std::lock_guard<std::mutex> stream_lock(stream_mutex_);
            if (stream_handler_) {
                stream_handler_.reset();
            }
            stream_active_ = false;
            response->success = true;
            response->message = "Persistent stream control deactivated.";
            RCLCPP_INFO(this->get_logger(), "Persistent stream control deactivated.");
        }
    }

    // Explicit template instantiations
    template class RBY1_ROS2_DRIVER<rb::y1_model::A>;
    template class RBY1_ROS2_DRIVER<rb::y1_model::M>;
}