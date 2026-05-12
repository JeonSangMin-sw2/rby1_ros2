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
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to load dynamics: %s. Cartesian poses will not be published.", e.what());
                    }
                }
                
                // robot state publisher
                if (use_all_motor_state_topic_trigger_){
                    all_motor_state_pub_ = this->create_publisher<rby1_msgs::msg::AllMotorState>(state_topic_name + "/all_motor_state", 10);
                }else{
                    torso_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic_name + "/torso", 10);
                    right_arm_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic_name + "/right_arm", 10);
                    left_arm_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic_name + "/left_arm", 10);
                    head_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic_name + "/head", 10);
                }
                // robot control state checker
                pub_control_state_ = this->create_publisher<std_msgs::msg::Int32>(state_topic_name + "/control_state", 10);
                
                if (publish_power_state_) {
                    power_state_pub_ = this->create_publisher<rby1_msgs::msg::PowerState>(state_topic_name + "/power_state", 10);
                }
                if (publish_tool_flange_) {
                    tool_flange_state_pub_ = this->create_publisher<rby1_msgs::msg::ToolFlangeState>(state_topic_name + "/tool_flange_state", 10);
                }
                if (publish_torque_velocity_) {
                    torque_velocity_state_pub_ = this->create_publisher<rby1_msgs::msg::TorqueVelocityState>(state_topic_name + "/torque_velocity_state", 10);
                }
                
                torso_cartesian_pub_ = this->create_publisher<rby1_msgs::msg::CartesianPose>(state_topic_name + "/torso_cartesian", 10);
                right_arm_cartesian_pub_ = this->create_publisher<rby1_msgs::msg::CartesianPose>(state_topic_name + "/right_cartesian", 10);
                left_arm_cartesian_pub_ = this->create_publisher<rby1_msgs::msg::CartesianPose>(state_topic_name + "/left_cartesian", 10);
                
                // loop for reading joint state
                joint_state_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(robot_parameter_.get_state_period*1000.0)), std::bind(&RBY1_ROS2_DRIVER<ModelType>::read_joint_state, this));
                
                // service for important robot action
                power_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
                    "robot_power", std::bind(&RBY1_ROS2_DRIVER<ModelType>::power_control, this, _1, _2));
                servo_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
                    "robot_servo", std::bind(&RBY1_ROS2_DRIVER<ModelType>::servo_control, this, _1, _2));
                tool_flange_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
                    "tool_flange_power", std::bind(&RBY1_ROS2_DRIVER<ModelType>::tool_flange_control, this, _1, _2));
                control_mode_service_ = this->create_service<rby1_msgs::srv::ControlMode>(
                    "set_control_mode", std::bind(&RBY1_ROS2_DRIVER<ModelType>::control_mode_callback, this, _1, _2));
                cancel_control_service_ = this->create_service<std_srvs::srv::Trigger>(
                    "cancel_control", std::bind(&RBY1_ROS2_DRIVER<ModelType>::cancel_control_callback, this, _1, _2));
                
                /* we need to add tool flange on/off service*/

                multi_position_action_server_ = rclcpp_action::create_server<MultiJointCommand>(
                    this,
                    state_topic_name + "/multi_position_command",
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_multi_goal, this, _1, _2),
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_multi_cancel, this, _1),
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_multi_accepted, this, _1));

                single_position_action_server_ = rclcpp_action::create_server<SingleJointCommand>(
                    this,
                    state_topic_name + "/single_position_command",
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_single_goal, this, _1, _2),
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_single_cancel, this, _1),
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_single_accepted, this, _1));

                stream_position_action_server_ = rclcpp_action::create_server<StreamPosition>(
                    this,
                    state_topic_name + "/stream_position_command",
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_stream_goal, this, _1, _2),
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_stream_cancel, this, _1),
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_stream_accepted, this, _1));
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "error occured: %s", e.what());
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
        this->declare_parameter<std::string>("state_topic_name", "joint_states");
        this->declare_parameter<std::vector<std::string>>("power_on", {"all"});
        this->declare_parameter<std::vector<std::string>>("servo_on", {"all"});

        this->declare_parameter<double>("get_state_period", 0.01);
        this->declare_parameter<double>("minimum_time", 2.0);
        this->declare_parameter<double>("angular_velocity_limit", 4.712388);
        this->declare_parameter<double>("linear_velocity_limit", 1.5);
        this->declare_parameter<double>("acceleration_limit", 1.0);
        this->declare_parameter<double>("stop_orientation_tracking_error", 1e-5);
        this->declare_parameter<double>("stop_position_tracking_error", 1e-5);
        
        this->declare_parameter<bool>("fault_reset_trigger", false);
        this->declare_parameter<bool>("node_power_off_trigger",false);
        this->declare_parameter<bool>("use_all_motor_state_topic_trigger",false);
        this->declare_parameter<double>("collision_threshold", 0.03);
        this->declare_parameter<bool>("publish_power_state", false);
        this->declare_parameter<bool>("publish_tool_flange", false);
        this->declare_parameter<bool>("publish_torque_velocity", false);
        
        this->get_parameter("robot_ip", address);
        this->get_parameter("model", model);
        this->get_parameter("state_topic_name", state_topic_name);
        

        this->get_parameter("power_on", robot_parameter_.power_on_list);
        this->get_parameter("servo_on", robot_parameter_.servo_on_list);
        this->get_parameter("get_state_period", robot_parameter_.get_state_period);
        this->get_parameter("minimum_time", robot_parameter_.minimum_time);
        this->get_parameter("angular_velocity_limit", robot_parameter_.angular_velocity_limit);
        this->get_parameter("linear_velocity_limit", robot_parameter_.linear_velocity_limit);
        this->get_parameter("acceleration_limit", robot_parameter_.acceleration_limit);
        this->get_parameter("stop_orientation_tracking_error", robot_parameter_.stop_orientation_tracking_error);
        this->get_parameter("stop_position_tracking_error", robot_parameter_.stop_position_tracking_error);
        this->get_parameter("fault_reset_trigger", fault_reset_trigger);
        this->get_parameter("node_power_off_trigger", node_power_off_trigger_);
        this->get_parameter("use_all_motor_state_topic_trigger", use_all_motor_state_topic_trigger_);
        this->get_parameter("collision_threshold", collision_threshold_);
        this->get_parameter("publish_power_state", publish_power_state_);
        this->get_parameter("publish_tool_flange", publish_tool_flange_);
        this->get_parameter("publish_torque_velocity", publish_torque_velocity_);

        auto init_bcfg = [](BuilderConfig& bcfg, const std::string& mode, const std::string& ref, const std::string& target, int dof) {
            bcfg.is_configured = false;
            if (mode == "joint_impedance") bcfg.control_type = rby1_msgs::srv::ControlMode::Request::JOINT_IMPEDANCE;
            else if (mode == "cartesian_position" || mode == "cartesian") bcfg.control_type = rby1_msgs::srv::ControlMode::Request::CARTESIAN_POSITION;
            else if (mode == "cartesian_impedance") bcfg.control_type = rby1_msgs::srv::ControlMode::Request::CARTESIAN_IMPEDANCE;
            else if (mode == "gravity_compensation") bcfg.control_type = rby1_msgs::srv::ControlMode::Request::GRAVITY_COMPENSATION;
            else if (mode == "joint_group_position") bcfg.control_type = rby1_msgs::srv::ControlMode::Request::JOINT_GROUP_POSITION;
            else bcfg.control_type = rby1_msgs::srv::ControlMode::Request::JOINT_POSITION;
            
            bcfg.ref_link = ref;
            bcfg.target_link = target;
            bcfg.translation_weight = {1000.0, 1000.0, 1000.0};
            bcfg.rotation_weight = {100.0, 100.0, 100.0};
            bcfg.joint_stiffness = std::vector<double>(dof, 1000.0);
            bcfg.damping_ratio = 0.85;
            bcfg.linear_velocity_limit = 0.5;
            bcfg.angular_velocity_limit = 1.0;
        };

        init_bcfg(torso_builder_, "joint", "base", "link_torso_5", 6);
        init_bcfg(right_arm_builder_, "joint", "link_torso_5", "link_right_arm_6", 7);
        init_bcfg(left_arm_builder_, "joint", "link_torso_5", "link_left_arm_6", 7);
        init_bcfg(head_builder_, "joint", "link_torso_5", "link_head_2", 2);

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
            if (robot_->IsServoOn(servo_list_str)) {
                RCLCPP_INFO(this->get_logger(), "Servo is already ON [%s], skipping.", servo_list_str.c_str());
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
        //if (address != "127.0.0.1:50051"){
            if(request->parameters == "12"){
                tool_flange_vol = 12;
            }else if(request->parameters == "24"){
                tool_flange_vol = 24;
            }else{
                RCLCPP_INFO(this->get_logger(), "Invalid parameters: %s", request->parameters.c_str());
                response->success = false;
                response->message = "Invalid parameters";
                return;
            }
        //}
        if (request->state) {
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
    void RBY1_ROS2_DRIVER<ModelType>::control_mode_callback(const std::shared_ptr<rby1_msgs::srv::ControlMode::Request> request,
                                                            std::shared_ptr<rby1_msgs::srv::ControlMode::Response> response) {
        BuilderConfig* target_bcfg = nullptr;
        if (request->part_name == "torso") target_bcfg = &torso_builder_;
        else if (request->part_name == "right_arm") target_bcfg = &right_arm_builder_;
        else if (request->part_name == "left_arm") target_bcfg = &left_arm_builder_;
        else if (request->part_name == "head") target_bcfg = &head_builder_;
        
        if (!target_bcfg) {
            response->success = false;
            response->message = "Invalid part_name: " + request->part_name;
            return;
        }

        const auto& cm_state = robot_->GetControlManagerState();
        if (cm_state.control_state == rb::ControlManagerState::ControlState::kExecuting ||
            cm_state.control_state == rb::ControlManagerState::ControlState::kSwitching) {
            RCLCPP_WARN(this->get_logger(), "Robot is moving. Canceling current control before changing mode...");
            robot_->CancelControl();
        }

        target_bcfg->is_configured = true;
        target_bcfg->control_type = request->control_type;
        target_bcfg->ref_link = request->ref_link;
        target_bcfg->target_link = request->target_link;
        target_bcfg->translation_weight = request->translation_weight;
        target_bcfg->rotation_weight = request->rotation_weight;
        target_bcfg->joint_stiffness = request->joint_stiffness;
        target_bcfg->damping_ratio = request->damping_ratio;
        target_bcfg->linear_velocity_limit = (request->linear_velocity_limit > 0.0) ? request->linear_velocity_limit : robot_parameter_.linear_velocity_limit;
        target_bcfg->angular_velocity_limit = (request->angular_velocity_limit > 0.0) ? request->angular_velocity_limit : robot_parameter_.angular_velocity_limit;
        target_bcfg->joint_names = request->joint_names;

        response->success = true;
        response->message = "Control mode updated for " + request->part_name;
        RCLCPP_INFO(this->get_logger(), "Updated control mode for %s to type %d (Ref: %s, Target: %s, V_lim: %f, W_lim: %f)", 
            request->part_name.c_str(), request->control_type, 
            request->ref_link.c_str(), request->target_link.c_str(),
            target_bcfg->linear_velocity_limit, target_bcfg->angular_velocity_limit);
    }

    template <typename ModelType>
    bool RBY1_ROS2_DRIVER<ModelType>::check_controll_manager(){
        robot_->ResetFaultControlManager();
        if (!robot_->EnableControlManager()) return false;
        
        const auto& control_manager_state = robot_->GetControlManagerState();

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

            // Collision check (always on if objects detected)
            if (!state.collisions.empty()) {
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

            if(this->use_all_motor_state_topic_trigger_){
                rby1_msgs::msg::AllMotorState all_motor_state;
                all_motor_state.torso = this->robot_joint_.joint_torso.position;
                all_motor_state.right_arm = this->robot_joint_.joint_right_arm.position;
                all_motor_state.left_arm = this->robot_joint_.joint_left_arm.position;
                all_motor_state.head = this->robot_joint_.joint_head.position;
                this->all_motor_state_pub_->publish(all_motor_state);
            }else{
                this->torso_pub_->publish(this->robot_joint_.joint_torso);
                this->right_arm_pub_->publish(this->robot_joint_.joint_right_arm);
                this->left_arm_pub_->publish(this->robot_joint_.joint_left_arm);
                this->head_pub_->publish(this->robot_joint_.joint_head);
            }
            
            std_msgs::msg::Int32 state_msg;
            state_msg.data = static_cast<int32_t>(robot_state_.state);
            pub_control_state_->publish(state_msg);

            // Publish rich status
            if (publish_power_state_) {
                rby1_msgs::msg::PowerState ps_msg;
                ps_msg.battery_voltage = state.battery_state.voltage;
                ps_msg.battery_current = state.battery_state.current;
                ps_msg.battery_level_percent = state.battery_state.level_percent;
                
                if (!state.emo_states.empty()) ps_msg.emo_state = rb::to_string(state.emo_states[0].state);
                if (!state.power_states.empty()) {
                    ps_msg.power_voltage = state.power_states[0].voltage;
                    ps_msg.power_state = rb::to_string(state.power_states[0].state);
                }
                for (const auto& js : state.joint_states) ps_msg.motor_currents.push_back(js.current);
                power_state_pub_->publish(ps_msg);
            }

            auto fill_arr = [](double* dest, const Eigen::Vector<double, 3>& src) {
                dest[0] = src[0]; dest[1] = src[1]; dest[2] = src[2];
            };

            // Cartesian Poses publishing
            if (dynamics_ && dyn_state_) {
                try {
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

                    auto get_pose = [&](const std::string& part_name, BuilderConfig& cfg, const std::string& default_ref, const std::string& default_target) {
                        std::string ref = cfg.ref_link.empty() ? default_ref : cfg.ref_link;
                        std::string target = cfg.target_link.empty() ? default_target : cfg.target_link;
                        
                        rby1_msgs::msg::CartesianPose msg;
                        msg.reference_link = ref;
                        msg.target_link = target;
                        
                        auto it_ref = std::find(dyn_link_names_.begin(), dyn_link_names_.end(), ref);
                        auto it_target = std::find(dyn_link_names_.begin(), dyn_link_names_.end(), target);
                        
                        if (it_ref != dyn_link_names_.end() && it_target != dyn_link_names_.end()) {
                            int ref_idx = std::distance(dyn_link_names_.begin(), it_ref);
                            int target_idx = std::distance(dyn_link_names_.begin(), it_target);
                            try {
                                auto T = dynamics_->ComputeTransformation(dyn_state_, ref_idx, target_idx);
                                std::copy(T.data(), T.data() + 16, msg.matrix.begin());
                            } catch (...) {}
                        }
                        return msg;
                    };

                    // Torso
                    torso_cartesian_pub_->publish(get_pose("torso", torso_builder_, "base", "link_torso_5"));

                    // Right Arm
                    right_arm_cartesian_pub_->publish(get_pose("right_arm", right_arm_builder_, "base", "link_right_arm_6"));

                    // Left Arm
                    left_arm_cartesian_pub_->publish(get_pose("left_arm", left_arm_builder_, "base", "link_left_arm_6"));

                } catch (const std::exception& e) {
                    // FK might fail if link names are wrong or robot not initialized
                }
            }

            if (publish_tool_flange_) {
                rby1_msgs::msg::ToolFlangeState tf_msg;
                
                // FT Sensor
                fill_arr(tf_msg.ft_force_right.data(), state.ft_sensor_right.force);
                fill_arr(tf_msg.ft_torque_right.data(), state.ft_sensor_right.torque);
                fill_arr(tf_msg.ft_force_left.data(), state.ft_sensor_left.force);
                fill_arr(tf_msg.ft_torque_left.data(), state.ft_sensor_left.torque);

                // Right Tool Flange
                fill_arr(tf_msg.tool_flange_right_gyro.data(), state.tool_flange_right.gyro);
                fill_arr(tf_msg.tool_flange_right_acceleration.data(), state.tool_flange_right.acceleration);
                tf_msg.tool_flange_right_switch_a = state.tool_flange_right.switch_A;
                tf_msg.tool_flange_right_output_voltage = state.tool_flange_right.output_voltage;
                tf_msg.tool_flange_right_digital_input_a = state.tool_flange_right.digital_input_A;
                tf_msg.tool_flange_right_digital_input_b = state.tool_flange_right.digital_input_B;
                tf_msg.tool_flange_right_digital_output_a = state.tool_flange_right.digital_output_A;
                tf_msg.tool_flange_right_digital_output_b = state.tool_flange_right.digital_output_B;

                // Left Tool Flange
                fill_arr(tf_msg.tool_flange_left_gyro.data(), state.tool_flange_left.gyro);
                fill_arr(tf_msg.tool_flange_left_acceleration.data(), state.tool_flange_left.acceleration);
                tf_msg.tool_flange_left_switch_a = state.tool_flange_left.switch_A;
                tf_msg.tool_flange_left_output_voltage = state.tool_flange_left.output_voltage;
                tf_msg.tool_flange_left_digital_input_a = state.tool_flange_left.digital_input_A;
                tf_msg.tool_flange_left_digital_input_b = state.tool_flange_left.digital_input_B;
                tf_msg.tool_flange_left_digital_output_a = state.tool_flange_left.digital_output_A;
                tf_msg.tool_flange_left_digital_output_b = state.tool_flange_left.digital_output_B;
                
                tool_flange_state_pub_->publish(tf_msg);
            }

            if (publish_torque_velocity_) {
                rby1_msgs::msg::TorqueVelocityState tv_msg;
                for (int i = 0; i < state.velocity.size(); ++i) tv_msg.velocity.push_back(state.velocity[i]);
                for (int i = 0; i < state.torque.size(); ++i) tv_msg.torque.push_back(state.torque[i]);
                for (int i = 0; i < state.target_feedforward_torque.size(); ++i) tv_msg.target_feedforward_torque.push_back(state.target_feedforward_torque[i]);
                tv_msg.center_of_mass[0] = state.center_of_mass[0];
                tv_msg.center_of_mass[1] = state.center_of_mass[1];
                tv_msg.center_of_mass[2] = state.center_of_mass[2];
                torque_velocity_state_pub_->publish(tv_msg);
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Error in read_joint_state loop: %s", e.what());
    }
}


    // --- MultiJointCommand Handlers ---
    template <typename ModelType>
    rclcpp_action::GoalResponse RBY1_ROS2_DRIVER<ModelType>::handle_multi_goal(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MultiJointCommand::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received MultiJointCommand request");
        (void)uuid;
        
        if (!goal->torso.empty() && !torso_builder_.is_configured) {
            RCLCPP_ERROR(this->get_logger(), "Torso control mode is not configured. Call SetControlMode service first.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (!goal->right_arm.empty() && !right_arm_builder_.is_configured) {
            RCLCPP_ERROR(this->get_logger(), "Right arm control mode is not configured. Call SetControlMode service first.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (!goal->left_arm.empty() && !left_arm_builder_.is_configured) {
            RCLCPP_ERROR(this->get_logger(), "Left arm control mode is not configured. Call SetControlMode service first.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (!goal->head.empty() && !head_builder_.is_configured) {
            RCLCPP_ERROR(this->get_logger(), "Head control mode is not configured. Call SetControlMode service first.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    template <typename ModelType>
    rclcpp_action::CancelResponse RBY1_ROS2_DRIVER<ModelType>::handle_multi_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiJointCommand>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel MultiJointCommand goal");
        (void)goal_handle;
        robot_->CancelControl();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::handle_multi_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiJointCommand>> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_multi_command, this, _1), goal_handle}.detach();
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::apply_body_builder(
        rb::BodyComponentBasedCommandBuilder& body_comp, 
        const std::string& part_name,
        const std::vector<double>& goal_data,
        double min_time) {
        
        BuilderConfig* bcfg = nullptr;
        if (part_name == "torso") bcfg = &torso_builder_;
        else if (part_name == "right_arm") bcfg = &right_arm_builder_;
        else if (part_name == "left_arm") bcfg = &left_arm_builder_;
        else return;

        if (bcfg->control_type == rby1_msgs::srv::ControlMode::Request::GRAVITY_COMPENSATION) {
            auto b = rb::GravityCompensationCommandBuilder();
            b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(min_time * 0.1));
            b.SetOn(true);
            if (part_name == "torso") body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
            else if (part_name == "right_arm") body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
            else if (part_name == "left_arm") body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
        } else if (bcfg->control_type == rby1_msgs::srv::ControlMode::Request::CARTESIAN_POSITION || 
                   bcfg->control_type == rby1_msgs::srv::ControlMode::Request::CARTESIAN_IMPEDANCE) {
            
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            if (goal_data.size() == 16) {
                T = Eigen::Map<const Eigen::Matrix4d>(goal_data.data());
            } else {
                RCLCPP_WARN(this->get_logger(), "Cartesian control %s requires exactly 16 float elements (4x4 matrix), got %zu", part_name.c_str(), goal_data.size());
                return;
            }

            if (bcfg->control_type == rby1_msgs::srv::ControlMode::Request::CARTESIAN_IMPEDANCE) {
                auto b = rb::ImpedanceControlCommandBuilder();
                b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(min_time * 0.1));
                b.SetReferenceLinkName(bcfg->ref_link);
                b.SetLinkName(bcfg->target_link);
                
                Eigen::Vector3d t_weight(bcfg->translation_weight.size()>=3 ? bcfg->translation_weight[0] : 1000.0, 
                                         bcfg->translation_weight.size()>=3 ? bcfg->translation_weight[1] : 1000.0, 
                                         bcfg->translation_weight.size()>=3 ? bcfg->translation_weight[2] : 1000.0);
                Eigen::Vector3d r_weight(bcfg->rotation_weight.size()>=3 ? bcfg->rotation_weight[0] : 100.0, 
                                         bcfg->rotation_weight.size()>=3 ? bcfg->rotation_weight[1] : 100.0, 
                                         bcfg->rotation_weight.size()>=3 ? bcfg->rotation_weight[2] : 100.0);
                b.SetRotationWeight(r_weight);
                b.SetDampingRatio(bcfg->damping_ratio > 0.0 ? bcfg->damping_ratio : 1.0);
                b.SetTransformation(T);

                if (part_name == "torso") body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
                else if (part_name == "right_arm") body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
                else if (part_name == "left_arm") body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
            } else {
                auto b = rb::CartesianCommandBuilder();
                b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(min_time * 0.1));
                b.AddTarget(bcfg->ref_link, bcfg->target_link, T, bcfg->linear_velocity_limit, bcfg->angular_velocity_limit);
                b.SetMinimumTime(min_time);
                
                b.SetStopPositionTrackingError(robot_parameter_.stop_position_tracking_error);
                b.SetStopOrientationTrackingError(robot_parameter_.stop_orientation_tracking_error);
                
                if (part_name == "torso") body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
                else if (part_name == "right_arm") body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
                else if (part_name == "left_arm") body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
            }
        } else {
            if (part_name == "torso" && bcfg->control_type == rby1_msgs::srv::ControlMode::Request::JOINT_GROUP_POSITION) {
                auto b = rb::JointGroupPositionCommandBuilder();
                if (!bcfg->joint_names.empty()) {
                    b.SetJointNames(bcfg->joint_names);
                }
                Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(goal_data.data(), goal_data.size());
                b.SetPosition(q);
                b.SetMinimumTime(min_time);
                body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
            } else if (bcfg->control_type == rby1_msgs::srv::ControlMode::Request::JOINT_IMPEDANCE) {
                size_t expected_dof = (part_name == "torso") ? 6 : (part_name == "head" ? 2 : 7);
                if (goal_data.size() != expected_dof) {
                    RCLCPP_WARN(this->get_logger(), "Joint Impedance control %s requires %zu float elements, got %zu", part_name.c_str(), expected_dof, goal_data.size());
                    return;
                }
                auto b = rb::JointImpedanceControlCommandBuilder();
                b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(min_time * 0.1));
                Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(goal_data.data(), goal_data.size());
                b.SetPosition(q);
                Eigen::VectorXd stiffness = Eigen::Map<const Eigen::VectorXd>(bcfg->joint_stiffness.data(), bcfg->joint_stiffness.size());
                if (stiffness.size() == q.size()) {
                    b.SetStiffness(stiffness);
                }
                b.SetDampingRatio(bcfg->damping_ratio);
                
                if (part_name == "torso") body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
                else if (part_name == "right_arm") body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
                else if (part_name == "left_arm") body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
            } else {
                size_t expected_dof = 0;
                if (part_name == "torso") expected_dof = info_.torso_joint_idx.size();
                else if (part_name == "head") expected_dof = info_.head_joint_idx.size();
                else if (part_name == "right_arm") expected_dof = info_.right_arm_joint_idx.size();
                else if (part_name == "left_arm") expected_dof = info_.left_arm_joint_idx.size();

                if (goal_data.size() != expected_dof) {
                    RCLCPP_WARN(this->get_logger(), "Joint Position control %s requires %zu float elements, got %zu", part_name.c_str(), expected_dof, goal_data.size());
                    return;
                }
                auto b = rb::JointPositionCommandBuilder();
                b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(min_time * 0.1));
                b.SetMinimumTime(min_time);
                Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(goal_data.data(), goal_data.size());
                b.SetPosition(q);
                if (part_name == "torso") body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
                else if (part_name == "right_arm") body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
                else if (part_name == "left_arm") body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
            }
        }
    }

    template <typename ModelType>
    std::optional<rb::HeadCommandBuilder> RBY1_ROS2_DRIVER<ModelType>::apply_head_builder(
        const std::vector<double>& goal_data,
        double min_time) {
        
        BuilderConfig* bcfg = &head_builder_;
        if (bcfg->control_type == rby1_msgs::srv::ControlMode::Request::JOINT_POSITION || 
            bcfg->control_type == rby1_msgs::srv::ControlMode::Request::JOINT_IMPEDANCE) { // head mostly uses joint position
            auto b = rb::JointPositionCommandBuilder();
            b.SetMinimumTime(min_time);
            Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(goal_data.data(), goal_data.size());
            b.SetPosition(q);
            return std::make_optional<rb::HeadCommandBuilder>(b);
        } else {
            RCLCPP_WARN(this->get_logger(), "Head currently only supports joint control.");
            return std::nullopt;
        }
    }

    template <typename ModelType>
    std::optional<rb::MobilityCommandBuilder> RBY1_ROS2_DRIVER<ModelType>::apply_mobile_builder(
        const std::vector<double>& goal_data) {
        
        RCLCPP_WARN(this->get_logger(), "Mobility control is not yet fully implemented in apply_mobile_builder.");
        return std::nullopt;
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::execute_multi_command(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiJointCommand>> goal_handle) {
        is_control_canceled_ = false;
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<MultiJointCommand::Result>();

        try {
            bool use_torso = !goal->torso.empty();
            bool use_right_arm = !goal->right_arm.empty();
            bool use_left_arm = !goal->left_arm.empty();
            bool use_head = !goal->head.empty();

            if (!use_torso && !use_right_arm && !use_left_arm && !use_head) {
                RCLCPP_WARN(this->get_logger(), "Received empty MultiJointCommand goal.");
                result->success = false;
                result->finish_code = "Empty arrays";
                goal_handle->abort(result);
                return;
            }

            if (!robot_->HasEstablishedTimeSync()) robot_->SyncTime();
            double min_time = (goal->minimum_time > 0.01) ? goal->minimum_time : robot_parameter_.minimum_time;

            // Ensure Control Manager is ready before sending any command
            if (!this->check_controll_manager()) {
                RCLCPP_ERROR(this->get_logger(), "Control Manager not ready for MultiJointCommand.");
                result->success = false;
                result->finish_code = "Control Manager Error";
                goal_handle->abort(result);
                return;
            }

            rb::ComponentBasedCommandBuilder component_cmd_builder;
            rb::BodyComponentBasedCommandBuilder body_comp;
            
            if (use_torso) apply_body_builder(body_comp, "torso", goal->torso, min_time);
            if (use_right_arm) apply_body_builder(body_comp, "right_arm", goal->right_arm, min_time);
            if (use_left_arm) apply_body_builder(body_comp, "left_arm", goal->left_arm, min_time);

            if (use_torso || use_right_arm || use_left_arm) {
                rb::BodyCommandBuilder body_builder;
                body_builder.SetCommand(body_comp);
                component_cmd_builder.SetBodyCommand(body_builder);
            }

            if (use_head) {
                if (auto head_comp = apply_head_builder(goal->head, min_time)) {
                    component_cmd_builder.SetHeadCommand(*head_comp);
                }
            }

            auto cmd_handler = robot_->SendCommand(rb::RobotCommandBuilder().SetCommand(component_cmd_builder));
            
            rclcpp::Rate rate(10);
            while (rclcpp::ok() && !cmd_handler->IsDone()) {
                if (goal_handle->is_canceling()) {
                    cmd_handler->Cancel();
                    result->success = false;
                    result->finish_code = "kCanceled";
                    goal_handle->canceled(result);
                    return;
                }

                auto cm_state = robot_->GetControlManagerState();
                if (cm_state.state == rb::ControlManagerState::State::kMajorFault ||
                    cm_state.state == rb::ControlManagerState::State::kMinorFault) {
                    cmd_handler->Cancel();
                    result->success = false;
                    result->finish_code = "Fault Detected";
                    goal_handle->abort(result);
                    return;
                }

                auto feedback = std::make_shared<MultiJointCommand::Feedback>();
                feedback->current_state = "excuting";
                goal_handle->publish_feedback(feedback);
                rate.sleep();
            }

            if (rclcpp::ok()) {
                auto rv = cmd_handler->Get();
                result->finish_code = this->finish_code_to_string(rv.finish_code());
                
                // If we manually canceled via service, override the potentially vague SDK code
                if (is_control_canceled_) {
                    result->finish_code = "kCanceled";
                }
                
                RCLCPP_INFO(this->get_logger(), "MultiJointCommand finished with code: %s", result->finish_code.c_str());
                result->success = (result->finish_code == "kOk");
                
                if (result->success) {
                    goal_handle->succeed(result);
                } else {
                    goal_handle->abort(result);
                    RCLCPP_INFO(this->get_logger(), "MultiJointCommand failed. Attempting to recover control manager...");
                    this->check_controll_manager();
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in MultiJointCommand: %s", e.what());
            result->success = false;
            result->finish_code = "kError";
            goal_handle->abort(result);
            this->check_controll_manager();
        }
    }

    // --- SingleJointCommand Handlers ---
    template <typename ModelType>
    rclcpp_action::GoalResponse RBY1_ROS2_DRIVER<ModelType>::handle_single_goal(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SingleJointCommand::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received SingleJointCommand request");
        (void)uuid;

        BuilderConfig* target_bcfg = nullptr;
        if (goal->target_name == "torso") target_bcfg = &torso_builder_;
        else if (goal->target_name == "right_arm") target_bcfg = &right_arm_builder_;
        else if (goal->target_name == "left_arm") target_bcfg = &left_arm_builder_;
        else if (goal->target_name == "head") target_bcfg = &head_builder_;
        else if (goal->target_name == "mobile") {
            // Mobile does not use BuilderConfig currently
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown target_name: %s", goal->target_name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (target_bcfg && !target_bcfg->is_configured) {
            RCLCPP_ERROR(this->get_logger(), "Control mode for %s is not configured. Call SetControlMode service first.", goal->target_name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    template <typename ModelType>
    rclcpp_action::CancelResponse RBY1_ROS2_DRIVER<ModelType>::handle_single_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointCommand>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel SingleJointCommand goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::handle_single_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointCommand>> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_single_command, this, _1), goal_handle}.detach();
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::execute_single_command(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointCommand>> goal_handle) {
        is_control_canceled_ = false;
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<SingleJointCommand::Result>();

        if (goal->position.empty() || goal->target_name.empty()) {
            RCLCPP_WARN(this->get_logger(), "Invalid SingleJointCommand data.");
            result->success = false;
            result->finish_code = "Invalid arguments";
            goal_handle->abort(result);
            return;
        }

        if (!robot_->HasEstablishedTimeSync()) robot_->SyncTime();

        double min_time = (goal->minimum_time > 0.01) ? goal->minimum_time : robot_parameter_.minimum_time;

        if (goal->target_name != "torso" && goal->target_name != "right_arm" && 
            goal->target_name != "left_arm" && goal->target_name != "head") {
            RCLCPP_WARN(this->get_logger(), "Unknown target name in SingleJointCommand: %s", goal->target_name.c_str());
            result->success = false;
            result->finish_code = "Unknown target name";
            goal_handle->abort(result);
            return;
        }

        rb::ComponentBasedCommandBuilder component_cmd_builder;
        rb::BodyComponentBasedCommandBuilder body_comp;
        
        if (goal->target_name == "head") {
            if (auto head_comp = apply_head_builder(goal->position, min_time)) {
                component_cmd_builder.SetHeadCommand(*head_comp);
            }
        } else if (goal->target_name == "torso" || goal->target_name == "right_arm" || goal->target_name == "left_arm") {
            apply_body_builder(body_comp, goal->target_name, goal->position, min_time);
            component_cmd_builder.SetBodyCommand(rb::BodyCommandBuilder(body_comp));
        }

        auto cmd_handler = robot_->SendCommand(rb::RobotCommandBuilder().SetCommand(component_cmd_builder));
        
        rclcpp::Rate rate(10);
        while (rclcpp::ok() && !cmd_handler->IsDone()) {
            if (goal_handle->is_canceling()) {
                cmd_handler->Cancel();
                result->success = false;
                result->finish_code = "kCanceled";
                goal_handle->canceled(result);
                return;
            }

            auto cm_state = robot_->GetControlManagerState();
            if (cm_state.state == rb::ControlManagerState::State::kMajorFault ||
                cm_state.state == rb::ControlManagerState::State::kMinorFault) {
                cmd_handler->Cancel();
                result->success = false;
                result->finish_code = "Fault Detected";
                goal_handle->abort(result);
                return;
            }

            auto feedback = std::make_shared<SingleJointCommand::Feedback>();
            feedback->current_state = "excuting";
            goal_handle->publish_feedback(feedback);
            rate.sleep();
        }

        if (rclcpp::ok()) {
            auto rv = cmd_handler->Get();
            result->finish_code = this->finish_code_to_string(rv.finish_code());
            
            if (is_control_canceled_) {
                result->finish_code = "kCanceled";
            }
            
            result->success = (result->finish_code == "kOk");
            if (result->success) {
                goal_handle->succeed(result);
            } else {
                goal_handle->abort(result);
                RCLCPP_INFO(this->get_logger(), "SingleJointCommand failed. Attempting to recover control manager...");
                this->check_controll_manager();
            }
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

        // 1. Ensure robot is ready before creating stream
        this->check_controll_manager();
        usleep(100000); // 100ms safety gap

        // 2. Create stream ONLY when we are ready to send
        std::unique_ptr<rb::RobotCommandStreamHandler<ModelType>> stream_handler;
        try {
            stream_handler = robot_->CreateCommandStream();
            if (!stream_handler) {
                throw std::runtime_error("Stream handler is null");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create CommandStream: %s", e.what());
            result->success = false;
            result->finish_code = "kError";
            goal_handle->abort(result);
            return;
        }

        auto current_state = robot_->GetState();
        Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(current_state.position.data(), current_state.position.size());

        try {
            for (size_t i = 0; i < trajectory.points.size(); ++i) {
                const auto& point = trajectory.points[i];
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->finish_code = "kCanceled";
                    goal_handle->canceled(result);
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

                // Build Component-based commands for each part to ensure full-body synchronization
                // Build Component-based commands for each part using CORRECT indices
                Eigen::VectorXd torso_q(6);
                for (int i = 0; i < 6; ++i) torso_q[i] = q[info_.torso_joint_idx[i]];
                rb::TorsoCommandBuilder torso_builder;
                torso_builder.SetCommand(rb::JointPositionCommandBuilder().SetPosition(torso_q).SetMinimumTime(pt_time));

                Eigen::VectorXd right_arm_q(7);
                for (int i = 0; i < 7; ++i) right_arm_q[i] = q[info_.right_arm_joint_idx[i]];
                rb::ArmCommandBuilder right_arm_builder;
                right_arm_builder.SetCommand(rb::JointPositionCommandBuilder().SetPosition(right_arm_q).SetMinimumTime(pt_time));

                Eigen::VectorXd left_arm_q(7);
                for (int i = 0; i < 7; ++i) left_arm_q[i] = q[info_.left_arm_joint_idx[i]];
                rb::ArmCommandBuilder left_arm_builder;
                left_arm_builder.SetCommand(rb::JointPositionCommandBuilder().SetPosition(left_arm_q).SetMinimumTime(pt_time));

                Eigen::VectorXd head_q(2);
                for (int i = 0; i < 2; ++i) head_q[i] = q[info_.head_joint_idx[i]];
                rb::HeadCommandBuilder head_builder;
                head_builder.SetCommand(rb::JointPositionCommandBuilder().SetPosition(head_q).SetMinimumTime(pt_time));

                rb::BodyComponentBasedCommandBuilder body_comp_builder;
                body_comp_builder.SetTorsoCommand(torso_builder);
                body_comp_builder.SetRightArmCommand(right_arm_builder);
                body_comp_builder.SetLeftArmCommand(left_arm_builder);

                rb::ComponentBasedCommandBuilder comp_builder;
                comp_builder.SetBodyCommand(rb::BodyCommandBuilder(std::move(body_comp_builder)));
                comp_builder.SetHeadCommand(head_builder);

                auto feedback = stream_handler->SendCommand(
                    rb::RobotCommandBuilder().SetCommand(comp_builder)
                );

                auto action_feedback = std::make_shared<StreamPosition::Feedback>();
                action_feedback->current_state = "streaming point " + std::to_string(i + 1) + "/" + std::to_string(trajectory.points.size());
                goal_handle->publish_feedback(action_feedback);

                // Wait for the duration of the point before sending the next one
                // to match the requested trajectory timing.
                auto sleep_duration = std::chrono::microseconds(static_cast<long long>(pt_time * 1e6));
                std::this_thread::sleep_for(sleep_duration);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error during stream transmission: %s", e.what());
            result->success = false;
            result->finish_code = "kError";
            goal_handle->abort(result);
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
            if (!stream_handler->WaitFor(wait_ms)) {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for stream completion (%u ms).", wait_ms);
                result->success = false;
                result->finish_code = "kTimeout";
                goal_handle->abort(result);
                this->check_controll_manager();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Stream execution finished.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error during stream wait: %s", e.what());
            result->success = false;
            result->finish_code = "kError";
            goal_handle->abort(result);
            this->check_controll_manager();
            return;
        }

        result->success = true;
        result->finish_code = "kOk";
        goal_handle->succeed(result);
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

    // Explicit template instantiations
    template class RBY1_ROS2_DRIVER<rb::y1_model::A>;
    template class RBY1_ROS2_DRIVER<rb::y1_model::M>;
}