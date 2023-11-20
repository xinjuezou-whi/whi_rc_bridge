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
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>

namespace whi_rc_bridge
{
	class RcBridge
	{
    public:
        RcBridge() = delete;
        RcBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~RcBridge();

    protected:
        void init();

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
	};
} // namespace whi_rc_bridge
