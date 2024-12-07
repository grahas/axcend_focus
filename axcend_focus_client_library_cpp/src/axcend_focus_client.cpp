#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "axcend_focus_client_library_cpp/axcend_focus_client.h"

using std::placeholders::_1;

namespace axcend_focus_client_library_cpp {

    MinimalSubscriber::MinimalSubscriber() : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/FocusLC_SN3000/front_panel_button", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

    MinimalSubscriber::~MinimalSubscriber()
    {
    }

    void MinimalSubscriber::topic_callback(const std_msgs::msg::String& msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }

}  // namespace axcend_focus_client_library_cpp

