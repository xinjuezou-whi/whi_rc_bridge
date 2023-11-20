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

namespace whi_rc_bridge
{
    RcBridge::RcBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    RcBridge::~RcBridge()
    {
    }

    void RcBridge::init()
    {
        // params
    }
} // namespace whi_rc_bridge
