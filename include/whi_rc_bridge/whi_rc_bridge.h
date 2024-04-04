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
2023-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>

#include "whi_rc_bridge/bridge_base.h"

namespace whi_rc_bridge
{
	class RcBridge
	{
    public:
        enum Hardware { HARDWARE_I2C = 0, HARDWARE_SERIAL, HARDWARE_SUM };
        static constexpr const char* hardware[HARDWARE_SUM] = { "i2c", "serial" };

    public:
        RcBridge() = delete;
        RcBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~RcBridge();

    protected:
        void init();
        void update(const ros::TimerEvent& Event);
        int indexOf(const std::string& Name);
        void cancelNaviGoal();

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
        ros::Duration elapsed_time_;
        std::unique_ptr<BaseBridge> bridge_{ nullptr };
        std::unique_ptr<ros::Publisher> pub_twist_{ nullptr };
        std::unique_ptr<ros::Publisher> pub_rc_state_{ nullptr };
        std::unique_ptr<ros::Publisher> pub_cancel_goal_{ nullptr };
        double max_linear_{ 1.0 };
        double max_angular_{ 1.57 };
        std::vector<std::string> channels_name_;
        std::vector<int> channels_offset_;
        double angular_range_{ 50.0 };
		bool print_raw_{ false };
	};
} // namespace whi_rc_bridge
