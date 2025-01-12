#include <rclcpp/rclcpp.hpp>
#include "camera_handler_cpp/CameraHandler.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraHandler>());
    rclcpp::shutdown();
    return 0;
}