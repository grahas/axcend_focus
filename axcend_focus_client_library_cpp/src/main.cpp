#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "axcend_focus_client_library_cpp/axcend_focus_client.h"

int main(int argc, char* argv[])
{


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<axcend_focus_client_library_cpp::MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
