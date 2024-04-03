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
2023-xx-xx: xxx
******************************************************************/
#pragma once
#include "bridge_base.h"
#include "i2c.h"

#include <memory>

namespace whi_rc_bridge
{
    class I2cBridge : public BaseBridge
    {
    public:
        I2cBridge() = default;
        I2cBridge(int BusAddr, int DeviceAddr);
        virtual ~I2cBridge() = default;

    public:
        int readChannel(int ChannelIndex) override;
        std::vector<int> readChannels() override;
        void close() override;

    protected:
        std::unique_ptr<I2cDriver> bus_{ nullptr };
    };
} // namespace whi_rc_bridge
