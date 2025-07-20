/******************************************************************
class of RC bridge

Features:
- IIC
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-11-20: Initial version
2025-07-19: Migrated from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_rc_bridge/bridge_base.h"
#include <whi_interfaces/msg/whi_rc_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace whi_rc_bridge
{
	class RcBridge
	{
    public:
        enum Hardware { HARDWARE_I2C = 0, HARDWARE_SERIAL, HARDWARE_SUM };
        static constexpr const char* hardware[HARDWARE_SUM] = { "i2c", "serial" };

    public:
        RcBridge() = delete;
        RcBridge(std::shared_ptr<rclcpp::Node>& NodeHandle);
        ~RcBridge();

    protected:
        void init();
        void update();
        int indexOf(const std::string& Name);
        void cancelNaviGoal();

    protected:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
        rclcpp::TimerBase::SharedPtr non_realtime_loop_{ nullptr };
        std::unique_ptr<BaseBridge> bridge_{ nullptr };
        // publisher
        using Twist = geometry_msgs::msg::TwistStamped;
        rclcpp::Publisher<Twist>::SharedPtr pub_twist_{ nullptr };
        rclcpp::Publisher<whi_interfaces::msg::WhiRcState>::SharedPtr pub_rc_state_{ nullptr };
        // nav2 client
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_nav_to_pose_{ nullptr };

        double max_linear_{ 1.0 };
        double max_angular_{ 1.57 };
        std::vector<std::string> channel_names_;
        std::vector<int64_t> channel_offsets_;
        double angular_range_{ 50.0 };
		bool print_raw_{ false };
	};
} // namespace whi_rc_bridge
