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
#include <whi_interfaces/msg/whi_rc_state.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>

namespace whi_rc_bridge
{
    RcBridge::RcBridge(std::shared_ptr<rclcpp::Node>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    RcBridge::~RcBridge()
    {
        bridge_->close();

        geometry_msgs::msg::Twist msgUnstamped;
        msgUnstamped.linear.x = 0.0;
		msgUnstamped.angular.z = 0.0;
        if (pub_twist_)
        {
            Twist msg;
            msg.header.stamp = node_handle_->get_clock()->now();
            msg.twist.linear = msgUnstamped.linear;
		    msg.twist.angular = msgUnstamped.angular;
		    pub_twist_->publish(msg);
        }
        else
        {
            pub_twist_unstamped_->publish(msgUnstamped);
        }
    }

    void RcBridge::init()
    {
        // params
        node_handle_->declare_parameter<std::string>("hardware", std::string(hardware[HARDWARE_I2C]));
        auto hardwareStr = node_handle_->get_parameter("hardware").as_string();

        node_handle_->declare_parameter<double>("max_linear", max_linear_);
        max_linear_ = node_handle_->get_parameter("max_linear").as_double();
        node_handle_->declare_parameter<double>("max_angular", max_angular_);
        max_angular_ = node_handle_->get_parameter("max_angular").as_double();
        node_handle_->declare_parameter<std::vector<std::string>>("channels_name", std::vector<std::string>());
        channel_names_ = node_handle_->get_parameter("channels_name").as_string_array();
        node_handle_->declare_parameter<std::vector<int64_t>>("channels_offset", std::vector<int64_t>());
        channel_offsets_ = node_handle_->get_parameter("channels_offset").as_integer_array();

        node_handle_->declare_parameter<bool>("damp_angular", true);
        bool damp = node_handle_->get_parameter("damp_angular").as_bool();
        if (damp)
        {
            angular_range_ = pow(50.0, 3.0);
        }
        node_handle_->declare_parameter<bool>("print_raw", print_raw_);
        print_raw_ = node_handle_->get_parameter("print_raw").as_bool();

        // twist publisher
        node_handle_->declare_parameter<std::string>("twist_topic", std::string("cmd_vel"));
        auto topicTwist = node_handle_->get_parameter("twist_topic").as_string();
        node_handle_->declare_parameter<bool>("use_stamped_vel", true);
        bool useStamped = node_handle_->get_parameter("use_stamped_vel").as_bool();
        if (useStamped)
        {
            pub_twist_ = node_handle_->create_publisher<Twist>(topicTwist, 50);
        }
        else
        {
            pub_twist_unstamped_ = node_handle_->create_publisher<geometry_msgs::msg::Twist>(topicTwist, 50);
        }
        // rc state publisher
        node_handle_->declare_parameter<std::string>("rc_state_topic", std::string("rc_state"));
        auto topicRcState = node_handle_->get_parameter("rc_state_topic").as_string();
        pub_rc_state_ = node_handle_->create_publisher<whi_interfaces::msg::WhiRcState>(topicRcState, 50);
        // cancel goal client // TODO::leave for future that the action name need be configured
        // client_nav_to_pose_ = rclcpp_action::create_client<NavigateToPose>(node_handle_, pose_action_);

        // bridge instance
        if (hardwareStr == hardware[HARDWARE_I2C])
        {
            node_handle_->declare_parameter<int>("i2c.bus_addr", -1);
            int busAddr = node_handle_->get_parameter("i2c.bus_addr").as_int();
            node_handle_->declare_parameter<int>("i2c.device_addr", -1);
            int deviceAddr = node_handle_->get_parameter("i2c.device_addr").as_int();

            bridge_ = std::make_unique<I2cBridge>(busAddr, deviceAddr);
        }
        else if (hardwareStr == hardware[HARDWARE_SERIAL])
        {
            node_handle_->declare_parameter<std::string>("serial.device", std::string("/dev/ttyUSB0"));
            auto devAddr = node_handle_->get_parameter("serial.device").as_string();

            bridge_ = std::make_unique<SbusBridge>(devAddr);
        }

        node_handle_->declare_parameter<double>("frequency", 10.0);
        double frequency = node_handle_->get_parameter("frequency").as_double();
        auto period = std::chrono::duration<double>(1.0 / frequency);
        non_realtime_loop_ = node_handle_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&RcBridge::update, this));
    }

    void RcBridge::update()
    {
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

        auto currentTime = node_handle_->get_clock()->now();

        whi_interfaces::msg::WhiRcState msgState;
        msgState.header.stamp = currentTime;
        int indexActive = indexOf("active");
        if (indexActive >= 0 &&
            values[indexActive] >= 0 && values[indexActive] < 100 + channel_offsets_[indexActive])
        {
            // neutralize the navigation's goal
            cancelNaviGoal();

            // set remote mode
            msgState.state = whi_interfaces::msg::WhiRcState::STA_REMOTE;
            pub_rc_state_->publish(msgState);
            // clear error
            int indexClear = indexOf("clear_error");
            if (indexClear >= 0 && values[indexClear] > 0)
            {
                msgState.state = whi_interfaces::msg::WhiRcState::STA_CLEAR_FAULT;
                pub_rc_state_->publish(msgState);
            }

            int valForthBack = values[indexOf("forth_back")];
            int offsetForthBack = channel_offsets_[indexOf("forth_back")];
            int dirBackForth = 0;
            if (valForthBack < 50 + offsetForthBack)
            {
                dirBackForth = 1;
            }
            else if (valForthBack > 50 - offsetForthBack)
            {
                dirBackForth = -1;
            }

            geometry_msgs::msg::Twist msgUnstamped;
            int valThrottle = values[indexOf("throttle")];
            msgUnstamped.linear.x = dirBackForth * max_linear_ * valThrottle / 100.0;
            double angularRatio = 50 + channel_offsets_[indexOf("left_right")] - values[indexOf("left_right")];
            angularRatio = angular_range_ > 2500.0 ? pow(angularRatio, 3.0) / angular_range_ : angularRatio / angular_range_;
            msgUnstamped.angular.z = valThrottle > channel_offsets_[indexOf("throttle")] ? max_angular_ * angularRatio : 0.0;
            if (pub_twist_)
            {
                Twist msg;
                msg.header.stamp = currentTime;
                msg.twist.linear = msgUnstamped.linear;
                msg.twist.angular = msgUnstamped.angular;
                pub_twist_->publish(msg);
            }
            else
            {
                pub_twist_unstamped_->publish(msgUnstamped);
            }
        }
        else
        {
            msgState.state = whi_interfaces::msg::WhiRcState::STA_AUTO;
            pub_rc_state_->publish(msgState);
        }
    }

    int RcBridge::indexOf(const std::string& Name)
    {
        auto found = std::find(channel_names_.begin(), channel_names_.end(), Name);
        if (found != channel_names_.end())
        {
            return std::distance(channel_names_.begin(), found);
        }
        else
        {
            return -1;
        }
    }

	void RcBridge::cancelNaviGoal()
	{
		if (!client_nav_to_pose_)
		{
			// TODO::check the action name
			client_nav_to_pose_ = rclcpp_action::create_client<NavigateToPose>(node_handle_, "navigate_to_pose");
		}
		client_nav_to_pose_->async_cancel_all_goals();
	}
} // namespace whi_rc_bridge
