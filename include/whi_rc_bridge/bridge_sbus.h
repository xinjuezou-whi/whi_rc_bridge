/******************************************************************
class of SBUS bridge

Features:
- custom baud rate 100000 serial
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-11-27: Initial version
2023-xx-xx: xxx
******************************************************************/
#pragma once
#include "bridge_base.h"

#include <string>
#include <thread>
#include <atomic>

namespace whi_rc_bridge
{
    class SbusBridge : public BaseBridge
    {
    public:
        SbusBridge() = default;
        SbusBridge(const std::string& DeviceAddr);
        virtual ~SbusBridge() = default;

    public:
        int readChannel(int ChannelIndex) override;
        std::vector<int> readChannels() override;

    protected:
        bool openSerial(const std::string& DeviceAddr, int Baudrate);
        int serialAvailable();
        int serialRead(char* Buf, unsigned int Len);
        void threadReadSerial();

    protected:
        int serial_handle_{ -1 };
        std::thread th_read_;
	    std::atomic_bool terminated_{ false };
    };
} // namespace whi_rc_bridge
