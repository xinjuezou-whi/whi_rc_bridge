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
        node_handle_->declare_parameter<double>("min_angular", min_angular_);
        min_angular_ = node_handle_->get_parameter("min_angular").as_double();
        node_handle_->declare_parameter<double>("max_angular", max_angular_);
        max_angular_ = node_handle_->get_parameter("max_angular").as_double();
        node_handle_->declare_parameter<std::vector<std::string>>("channels_name", std::vector<std::string>());
        channel_names_ = node_handle_->get_parameter("channels_name").as_string_array();
        node_handle_->declare_parameter<std::vector<std::string>>("rotary_joints", std::vector<std::string>());
        joints_rotary_ = node_handle_->get_parameter("rotary_joints").as_string_array();
        node_handle_->declare_parameter<std::vector<std::string>>("lift_joints", std::vector<std::string>());
        joints_lift_linear_ = node_handle_->get_parameter("lift_joints").as_string_array();
        node_handle_->declare_parameter<std::vector<double>>("rotary_position_limits", std::vector<double>{ -3.1415926, 3.1415926 });
        auto rotaryPosArray = node_handle_->get_parameter("rotary_position_limits").as_double_array();
        if (rotaryPosArray.size() == 2)
        {
            rotary_position_limits_.first = rotaryPosArray[0];
            rotary_position_limits_.second = rotaryPosArray[1];
        }
        node_handle_->declare_parameter<std::vector<double>>("rotary_velocity_limits", std::vector<double>{ 0.0, 0.12 });
        auto rotaryVelArray = node_handle_->get_parameter("rotary_velocity_limits").as_double_array();
        if (rotaryVelArray.size() == 2)
        {
            rotary_velocity_limits_.first = rotaryVelArray[0];
            rotary_velocity_limits_.second = rotaryVelArray[1];
        }
        node_handle_->declare_parameter<std::vector<double>>("lift_position_limits", std::vector<double>{ 0.0, 0.05 });
        auto leftPosArray = node_handle_->get_parameter("lift_position_limits").as_double_array();
        if (leftPosArray.size() == 2)
        {
            lift_position_limits_.first = leftPosArray[0];
            lift_position_limits_.second = leftPosArray[1];
        }
        node_handle_->declare_parameter<std::vector<double>>("lift_velocity_limits", std::vector<double>{ 0.0, 0.015 });
        auto leftVelArray = node_handle_->get_parameter("lift_velocity_limits").as_double_array();
        if (leftVelArray.size() == 2)
        {
            lift_velocity_limits_.first = leftVelArray[0];
            lift_velocity_limits_.second = leftVelArray[1];
        }
        node_handle_->declare_parameter<std::vector<int64_t>>("channels_offset", std::vector<int64_t>());
        channel_offsets_ = node_handle_->get_parameter("channels_offset").as_integer_array();

        node_handle_->declare_parameter<bool>("damp_angular", true);
        bool damp = node_handle_->get_parameter("damp_angular").as_bool();
        if (damp)
        {
            angular_range_ = pow(50.0, 3.0);
        }

        std::vector<int64_t> ioAddr, val2Addr;
        node_handle_->declare_parameter<std::vector<int64_t>>("io_request.addr", std::vector<int64_t>());
        ioAddr = node_handle_->get_parameter("io_request.addr").as_integer_array();
        node_handle_->declare_parameter<std::vector<int64_t>>("io_request.value_to_addr", std::vector<int64_t>());
        val2Addr = node_handle_->get_parameter("io_request.value_to_addr").as_integer_array();
        for (int i = 0; i < std::min(ioAddr.size(), val2Addr.size()); ++i)
        {
            io_maps_.emplace(std::make_pair<int, int>(int(val2Addr[i]), int(ioAddr[i])));
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
        // io request publisher
        node_handle_->declare_parameter<std::string>("io_request.topic", std::string("modbus_io_request"));
        auto topicIoRequest = node_handle_->get_parameter("io_request.topic").as_string();
        pub_io_ = node_handle_->create_publisher<whi_interfaces::msg::WhiIo>(topicIoRequest, 50);
        // rotary lift publisher
        node_handle_->declare_parameter<std::string>("rotary_lift_topic", std::string("cmd_rotary_lift"));
        auto topicRotaryLift = node_handle_->get_parameter("rotary_lift_topic").as_string();
        pub_rotary_lift_ = node_handle_->create_publisher<whi_interfaces::msg::WhiRotaryLiftPoseStamped>(topicRotaryLift, 50);
        // sw estop subscriber
        node_handle_->declare_parameter<std::string>("sw_estop_topic", std::string("estop"));
        auto topicSwEstop = node_handle_->get_parameter("sw_estop_topic").as_string();
        sub_sw_estop_ = node_handle_->create_subscription<std_msgs::msg::Bool>(
      		topicSwEstop, 10, std::bind(&RcBridge::callbackSwEstop, this, std::placeholders::_1));
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
        if (indexActive >= 0)
        {
            if ((values[indexActive] <= 50 + abs(channel_offsets_[indexActive])) &&
                (values[indexActive] >= 50 - abs(channel_offsets_[indexActive])))
            {
                /// active
                // neutralize the navigation's goal
                cancelNaviGoal();

                // set remote mode
                msgState.state = whi_interfaces::msg::WhiRcState::STA_ACTIVE;
                pub_rc_state_->publish(msgState);

                if (!sw_estopped_)
                {
                    // io and clear error
                    int indexIo = indexOf("io");
                    if (indexIo >= 0)
                    {
                        if (values[indexIo] < 100 - channel_offsets_[indexIo])
                        {
                            // io control
                            whi_interfaces::msg::WhiIo msg;
                            msg.operation = whi_interfaces::msg::WhiIo::OPER_WRITE;
                            if (auto found = io_maps_.find(values[indexIo]); found != io_maps_.end())
                            {
                                msg.addr = found->second;

                                int indexTrigger = indexOf("clear_error");
                                if (indexTrigger >= 0)
                                {
                                    msg.level = values[indexTrigger] > 0 - channel_offsets_[indexTrigger] ? 1 : 0;

                                    pub_io_->publish(msg);
                                }
                            }
                        }
                        else
                        {
                            // clear error
                            int indexClear = indexOf("clear_error");
                            if (indexClear >= 0 && values[indexClear] > 0 - channel_offsets_[indexClear])
                            {
                                msgState.state = whi_interfaces::msg::WhiRcState::STA_CLEAR_FAULT;
                                pub_rc_state_->publish(msgState);
                            }
                        }
                    }

                    // twist
                    int indexForthBack = indexOf("forth_back");
                    int indexLeftRight = indexOf("left_right");
                    int indexThrottle = indexOf("throttle");
                    if (indexForthBack >=0 && indexLeftRight >= 0 && indexThrottle >= 0)
                    {
                        int dirBackForth = 0;
                        if (values[indexForthBack] < 50 + channel_offsets_[indexForthBack])
                        {
                            dirBackForth = 1;
                        }
                        else if (values[indexForthBack] > 50 - channel_offsets_[indexForthBack])
                        {
                            dirBackForth = -1;
                        }

                        geometry_msgs::msg::Twist msgUnstamped;
                        int valThrottle = values[indexThrottle];
                        msgUnstamped.linear.x = dirBackForth * max_linear_ * valThrottle / 100.0;
                        double angularRatio = 50 + channel_offsets_[indexLeftRight] - values[indexLeftRight];
                        angularRatio = angular_range_ > 2500.0 ? pow(angularRatio, 3.0) / angular_range_ : angularRatio / angular_range_;
                        msgUnstamped.angular.z = valThrottle > channel_offsets_[indexThrottle] ? max_angular_ * angularRatio : 0.0;
                        msgUnstamped.angular.z = fabs(msgUnstamped.angular.z) < min_angular_ ? 0.0 : msgUnstamped.angular.z;
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
                }
            }
            else if ((values[indexActive] <= 0 + abs(channel_offsets_[indexActive])) &&
                (values[indexActive] >= 0 - abs(channel_offsets_[indexActive])))
            {
                /// other layer: rotary and lift command
                // motion
                int indexForthBack = indexOf("forth_back");
                int indexLeftRight = indexOf("left_right");
                if (indexForthBack >=0 && indexLeftRight >= 0 &&
                    (!joints_rotary_.empty() || !joints_lift_linear_.empty()))
                {
                    int dirBackForth = 0;
                    if (values[indexForthBack] < 50 + channel_offsets_[indexForthBack])
                    {
                        dirBackForth = 1;
                    }
                    else if (values[indexForthBack] > 50 - channel_offsets_[indexForthBack])
                    {
                        dirBackForth = -1;
                    }
                    int dirLeftRight = 0;
                    if (values[indexLeftRight] < 50 + channel_offsets_[indexLeftRight])
                    {
                        dirLeftRight = 1;
                    }
                    else if (values[indexLeftRight] > 50 - channel_offsets_[indexLeftRight])
                    {
                        dirLeftRight = -1;
                    }

                    whi_interfaces::msg::WhiRotaryLiftPoseStamped msg;
                    msg.header.stamp = currentTime;
                    for (const auto& it : joints_rotary_)
                    {
                        msg.joints.push_back(it);
                        if (dirLeftRight == 1)
                        {
                            msg.positions.push_back(rotary_position_limits_.second);
                        }
                        else
                        {
                            msg.positions.push_back(rotary_position_limits_.first);
                        }
                        msg.velocities.push_back(abs(values[indexLeftRight] - 50 - channel_offsets_[indexLeftRight]) * rotary_velocity_limits_.second / 50.0);
                    }
                    for (const auto& it : joints_lift_linear_)
                    {
                        msg.joints.push_back(it);
                        if (dirBackForth == 1)
                        {
                            msg.positions.push_back(lift_position_limits_.second);
                        }
                        else
                        {
                            msg.positions.push_back(lift_position_limits_.first);
                        }
                        msg.velocities.push_back(abs(values[indexForthBack] - 50 - channel_offsets_[indexForthBack]) * lift_velocity_limits_.second / 50.0);
                    }
                    pub_rotary_lift_->publish(msg);
                }

                // homing
                int indexTrigger = indexOf("clear_error");
                if (indexTrigger >= 0 && values[indexTrigger] > 0 - channel_offsets_[indexTrigger])
                {
                    whi_interfaces::msg::WhiRotaryLiftPoseStamped msg;
                    msg.header.stamp = currentTime;
                    for (const auto& it : joints_rotary_)
                    {
                        msg.joints.push_back(it);
                        msg.positions.push_back(0.0);
                        msg.velocities.push_back(rotary_velocity_limits_.second);
                    }
                    for (const auto& it : joints_lift_linear_)
                    {
                        msg.joints.push_back(it);
                        msg.positions.push_back(lift_position_limits_.first);
                        msg.velocities.push_back(lift_velocity_limits_.second);
                    }
                    pub_rotary_lift_->publish(msg);
                }
            }
            else
            {
                /// inactive
                msgState.state = whi_interfaces::msg::WhiRcState::STA_INACTIVE;
                pub_rc_state_->publish(msgState);
            }
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

    void RcBridge::callbackSwEstop(const std_msgs::msg::Bool::SharedPtr Msg)
    {
        sw_estopped_ = Msg->data;
    }

	void RcBridge::cancelNaviGoal()
	{
		if (!client_nav_to_pose_)
		{
			client_nav_to_pose_ = rclcpp_action::create_client<NavigateToPose>(node_handle_, "navigate_to_pose");
		}
		client_nav_to_pose_->async_cancel_all_goals();
	}
} // namespace whi_rc_bridge
