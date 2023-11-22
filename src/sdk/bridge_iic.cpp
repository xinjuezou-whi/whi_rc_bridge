/******************************************************************
class of IIC bridge

Features:
- IIC bu
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-11-22: Initial version
2022-xx-xx: xxx
******************************************************************/
#include "whi_rc_bridge/bridge_iic.h"

#include <iostream>

namespace whi_rc_bridge
{
    I2cBridge::I2cBridge(int BusAddr, int DeviceAddr)
        : bus_(std::make_unique<I2cDriver>(BusAddr, DeviceAddr)) {}

    int I2cBridge::readChannel(int ChannelIndex)
    {
        auto values = readChannels();
        if (ChannelIndex < values.size())
        {
            return values[ChannelIndex];
        }
        else
        {
            return 0;
        }
    }

    std::vector<int> I2cBridge::readChannels()
    {
        uint8_t rbuff[4] = { 0, 0, 0, 0 };
        if (bus_->connected() && bus_->readBytes(rbuff, sizeof(rbuff)))
        {
            std::vector<int> values;
            for (const auto& it : rbuff)
            {
                values.push_back(it);
            }

            return values;
        }
        else
        {
            return std::vector<int>();
        }
    }
} // namespace whi_rc_bridge
