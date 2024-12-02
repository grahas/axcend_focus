#ifndef AXCEND_FOCUS_CLIENT_LIBRARY_CPP__AXCEND_FOCUS_CLIENT_H_
#define AXCEND_FOCUS_CLIENT_LIBRARY_CPP__AXCEND_FOCUS_CLIENT_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "axcend_focus_client_library_cpp/visibility_control.h"

namespace axcend_focus_client_library_cpp {

	class AXCEND_FOCUS_CLIENT_LIBRARY_CPP_PUBLIC MinimalSubscriber : public rclcpp::Node
	{
	public:
		MinimalSubscriber();
		~MinimalSubscriber();

	private:
		void topic_callback(const std_msgs::msg::String& msg) const;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
	};

}  // namespace axcend_focus_client_library_cpp

#endif  // AXCEND_FOCUS_CLIENT_LIBRARY_CPP__AXCEND_FOCUS_CLIENT_H_
