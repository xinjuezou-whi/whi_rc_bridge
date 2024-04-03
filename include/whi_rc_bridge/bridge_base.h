/******************************************************************
class of base bridge

Features:
- virtual interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-11-22: Initial version
2023-xx-xx: xxx
******************************************************************/
#pragma once
#include <vector>

namespace whi_rc_bridge
{
    class BaseBridge
    {
    public:
        BaseBridge() = default;
        virtual ~BaseBridge() = default;

    public:
        virtual int readChannel(int ChannelIndex) = 0;
        virtual std::vector<int> readChannels() = 0;
        virtual void close() = 0;
    };
} // namespace whi_rc_bridge
