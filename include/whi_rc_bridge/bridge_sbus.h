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
#include <mutex>
#include <atomic>

namespace whi_rc_bridge
{
    struct SbusData
    {
        // SBUS message definitions
        static constexpr int8_t PAYLOAD_LEN = 23;
        static constexpr int8_t HEADER_LEN = 1;
        static constexpr int8_t FOOTER_LEN = 1;
        static constexpr int8_t NUM_SBUS_CH = 16;
        static constexpr uint8_t HEADER = 0x0F;
        static constexpr uint8_t FOOTER = 0x00;
        static constexpr uint8_t FOOTER2 = 0x04;
        static constexpr uint8_t CH17_MASK = 0x01;
        static constexpr uint8_t CH18_MASK = 0x02;
        static constexpr uint8_t LOST_FRAME_MASK = 0x04;
        static constexpr uint8_t FAILSAFE_MASK = 0x08;
        static constexpr int8_t NUM_CH = 16;

        bool lost_frame_;
        bool failsafe_;
        bool ch17_;
        bool ch18_;
        int16_t ch_[NUM_CH];
    };
    
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
        int serialRead(uint8_t* Buf, unsigned int Len);
        void threadReadSerial();

    protected:
        int serial_handle_{ -1 };
        std::thread th_read_;
	    std::atomic_bool terminated_{ false };
        uint8_t pre_byte_{ SbusData::FOOTER };
        uint8_t msg_buf_[SbusData::PAYLOAD_LEN + SbusData::HEADER_LEN + SbusData::FOOTER_LEN];
        int index_{ 0 };
        SbusData data_;
        std::mutex mtx_;
    };
} // namespace whi_rc_bridge
