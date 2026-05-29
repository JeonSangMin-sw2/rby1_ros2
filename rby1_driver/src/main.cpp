#include "rclcpp/rclcpp.hpp"
#include "rby1_ros2_driver.hpp"

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    
    // Create a temporary node to parse parameters from the launch file targeting rby1_ros2_driver
    std::string model = "a";
    {
        auto temp_node = std::make_shared<rclcpp::Node>("rby1_ros2_driver");
        temp_node->declare_parameter<std::string>("model", "a");
        temp_node->get_parameter("model", model);
    }

    std::shared_ptr<rclcpp::Node> node;
    try {
        if (model == "a" || model == "A") {
            node = std::make_shared<rby1_ros2::RBY1_ROS2_DRIVER<rb::y1_model::A>>();
        } else if (model == "m" || model == "M") {
            node = std::make_shared<rby1_ros2::RBY1_ROS2_DRIVER<rb::y1_model::M>>();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "\033[1;31m[FATAL] Invalid or unsupported robot model: %s\033[0m", model.c_str());
            rclcpp::shutdown();
            return 1;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "\033[1;31m[FATAL] Driver initialization failed: %s. Shutting down.\033[0m", e.what());
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
