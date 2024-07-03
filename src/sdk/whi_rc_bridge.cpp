/******************************************************************
class of RC bridge

Features:
- IIC
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rc_bridge/whi_rc_bridge.h"
#include "whi_rc_bridge/bridge_iic.h"
#include "whi_rc_bridge/bridge_sbus.h"
#include "whi_interfaces/WhiRcState.h"

#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <algorithm>

namespace whi_rc_bridge
{
    RcBridge::RcBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    RcBridge::~RcBridge()
    {
        bridge_->close();

        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
		msg.angular.z = 0.0;
		pub_twist_->publish(msg);
    }

    void RcBridge::init()
    {
        // params
        double frequency = 10.0;
        node_handle_->param("frequency", frequency, 10.0);
        std::string hardwareStr;
		node_handle_->param("hardware", hardwareStr, std::string(hardware[HARDWARE_I2C]));
        node_handle_->param("max_linear", max_linear_, 1.0);
        node_handle_->param("max_angular", max_angular_, 1.57);
        node_handle_->getParam("channels_name", channels_name_);
        node_handle_->getParam("channels_offset", channels_offset_);
        bool damp = true;
        node_handle_->param("damp_angular", damp, true);
        if (damp)
        {
            angular_range_ = pow(50.0, 3.0);
        }
		node_handle_->param("print_raw", print_raw_, false);

        // twist publisher
        std::string topicTwist;
        node_handle_->param("twist_topic", topicTwist, std::string("cmd_vel"));
        pub_twist_ = std::make_unique<ros::Publisher>(
            node_handle_->advertise<geometry_msgs::Twist>(topicTwist, 50));
        // rc state publisher
        std::string topicRcState;
        node_handle_->param("rc_state_topic", topicRcState, std::string("rc_state"));
        pub_rc_state_ = std::make_unique<ros::Publisher>(
            node_handle_->advertise<whi_interfaces::WhiRcState>(topicRcState, 50));        
        // cancel goal publisher
        std::string topicCancel;
        node_handle_->param("cancel_goal_topic", topicCancel, std::string("move_base/cancel"));
		pub_cancel_goal_ = std::make_unique<ros::Publisher>(
				node_handle_->advertise<actionlib_msgs::GoalID>(topicCancel, 10));

        // bridge instance
        if (hardwareStr == hardware[HARDWARE_I2C])
        {
            int busAddr = 0;
            int deviceAddr = 0;
            node_handle_->param("i2c/bus_addr", busAddr, -1);
            node_handle_->param("i2c/device_addr", deviceAddr, -1);
            bridge_ = std::make_unique<I2cBridge>(busAddr, deviceAddr);
        }
        else if (hardwareStr == hardware[HARDWARE_SERIAL])
        {
            std::string devAddr;
            node_handle_->param("serial/device", devAddr, std::string("/dev/ttyUSB0"));
            bridge_ = std::make_unique<SbusBridge>(devAddr);
        }

        ros::Duration updateFreq = ros::Duration(1.0 / frequency);
		non_realtime_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(
            updateFreq, std::bind(&RcBridge::update, this, std::placeholders::_1)));
    }

    void RcBridge::update(const ros::TimerEvent& Event)
    {
		elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);

        auto values = bridge_->readChannels();
		if (print_raw_)
		{
			std::cout << "channel values: ";
			for (const auto& it : values)
			{
				std::cout << int(it) << ",";
			}
			std::cout << std::endl;
		}

        whi_interfaces::WhiRcState msgState;
        int indexActive = indexOf("active");
        if (indexActive >= 0 &&
            values[indexActive] >= 0 && values[indexActive] < 100 + channels_offset_[indexActive])
        {
            // neutralize the navigation's goal
            cancelNaviGoal();

            // set remote mode
            msgState.state = whi_interfaces::WhiRcState::STA_REMOTE;
            pub_rc_state_->publish(msgState);
            // clear error
            int indexClear = indexOf("clear_error");
            if (indexClear >= 0 && values[indexClear] > 0)
            {
                msgState.state = whi_interfaces::WhiRcState::STA_CLEAR_FAULT;
                pub_rc_state_->publish(msgState);
            }

            int valForthBack = values[indexOf("forth_back")];
            int offsetForthBack = channels_offset_[indexOf("forth_back")];
            int dirBackForth = 0;
            if (valForthBack < 50 + offsetForthBack)
            {
                dirBackForth = 1;
            }
            else if (valForthBack > 50 - offsetForthBack)
            {
                dirBackForth = -1;
            }

            geometry_msgs::Twist msgTwist;
            int valThrottle = values[indexOf("throttle")];
            msgTwist.linear.x = dirBackForth * max_linear_ * valThrottle / 100.0;
            double angularRatio = 50 + channels_offset_[indexOf("left_right")] - values[indexOf("left_right")];
            angularRatio = angular_range_ > 2500.0 ? pow(angularRatio, 3.0) / angular_range_ : angularRatio / angular_range_;
            msgTwist.angular.z = valThrottle > channels_offset_[indexOf("throttle")] ? max_angular_ * angularRatio : 0.0;
            pub_twist_->publish(msgTwist);
        }
        else
        {
            msgState.state = whi_interfaces::WhiRcState::STA_AUTO;
            pub_rc_state_->publish(msgState);
        }
    }

    int RcBridge::indexOf(const std::string& Name)
    {
        auto found = std::find(channels_name_.begin(), channels_name_.end(), Name);
        if (found != channels_name_.end())
        {
            return std::distance(channels_name_.begin(), found);
        }
        else
        {
            return -1;
        }
    }

	void RcBridge::cancelNaviGoal()
	{
		actionlib_msgs::GoalID cancelID; // must be an empty goal msg
		pub_cancel_goal_->publish(cancelID);
	}
} // namespace whi_rc_bridge
