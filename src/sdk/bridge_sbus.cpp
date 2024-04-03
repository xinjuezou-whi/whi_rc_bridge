/******************************************************************
class of SBUS bridge

Features:
- custom baud rate 100000 serial
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rc_bridge/bridge_sbus.h"

#include <ros/ros.h>

#include <asm/termbits.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <functional>
#include <iostream>
#include <cstring>

namespace whi_rc_bridge
{
    SbusBridge::SbusBridge(const std::string& DeviceAddr)
    {
        if (openSerial(DeviceAddr, 100000))
        {
		    // spawn the read thread
		    th_read_ = std::thread(std::bind(&SbusBridge::threadReadSerial, this));
        }
    }

    SbusBridge::~SbusBridge()
    {
        close();
    }

    int SbusBridge::readChannel(int ChannelIndex)
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

    static int mapRange(int Value, int SrcMin, int SrcMax, int DstMin, int DstMax)
    {
        return (Value - SrcMin) * (DstMax - DstMin) / (SrcMax - SrcMin) + DstMin;
    }

    std::vector<int> SbusBridge::readChannels()
    {
        int16_t buff[SbusData::NUM_CH] = { 0 };
        {
            const std::lock_guard<std::mutex> lock(mtx_);
            memcpy(buff, data_.ch_, sizeof(int16_t) * SbusData::NUM_CH);
        }

        std::vector<int> chData;
        chData.resize(SbusData::NUM_CH);
        for (size_t i = 0; i < std::min(chData.size(), sizeof(buff)); ++i)
        {
            chData[i] = mapRange(buff[i], 200, 1800, 0, 100);

            // validate
            if (chData[i] < 0 || chData[i] > 100)
            {
                for (auto& it : chData)
                {
                    it = 65535;
                }
                break;
            }
        }
        return chData;
    }

    void SbusBridge::close()
    {
        terminated_.store(true);
        if (th_read_.joinable())
        {
            th_read_.join();
        }

        if (serial_handle_ > 0)
        {
            ::close(serial_handle_);
        }
    }

    bool SbusBridge::openSerial(const std::string& DeviceAddr, int Baudrate)
    {
        serial_handle_ = open(DeviceAddr.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_handle_ <= 0)
        {
            ROS_WARN_STREAM("failed to open serial " << DeviceAddr);
            return false;
        }
        else
        {
            struct termios2 tio;
            int rc = ioctl(serial_handle_, TCGETS2, &tio);
            if (rc)
            {
                ::close(serial_handle_);
                ROS_WARN_STREAM("failed to get termios2");

                return false;
            }
            else
            {
                // Clear the current output baud rate and fill a new value
                tio.c_cflag &= ~CBAUD;
                tio.c_cflag |= BOTHER;
                tio.c_ospeed = Baudrate;
                // Clear the current input baud rate and fill a new value
                tio.c_cflag &= ~(CBAUD << IBSHIFT);
                tio.c_cflag |= BOTHER << IBSHIFT;
                tio.c_ispeed = Baudrate;
                // SBUS
                tio.c_cflag |= PARENB; // enable parity
                tio.c_cflag &= ~PARODD; // even parity
                tio.c_cflag |= CSTOPB; // 2 stop bits
                //
                tio.c_cflag &= ~CSIZE ;
                tio.c_cflag |= CS8 ;
                tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
                tio.c_oflag &= ~OPOST ;
                tio.c_cc[VMIN] = 0;
	            tio.c_cc[VTIME] = 0;
                // Set new serial port settings via supported ioctl
                rc = ioctl(serial_handle_, TCSETS2, &tio);

                return true;
            }
        }
    }

    int SbusBridge::serialAvailable()
    {
        if (serial_handle_ > 0)
        {
            int result = 0;
            if (ioctl(serial_handle_, FIONREAD, &result) == -1)
            {
                return 0;
            }
            else
            {
                return result;
            }
        }
        else
        {
            return 0;
        }
    }

    int SbusBridge::serialRead(uint8_t* Buf, unsigned int Len)
    {
        return serial_handle_ > 0 ? (int)read(serial_handle_, Buf, Len) : 0;
    }

    void SbusBridge::threadReadSerial()
    {
        while (!terminated_.load())
        {
            size_t count = serialAvailable();
            if (count > 0)
            {
                uint8_t rbuff[count];
                int readNum = serialRead(rbuff, count);
#ifdef DEBUG
                std::cout << "read from SBUS: ";
                for (const auto& it : rbuff)
                {
                    std::cout << int(it) << ",";
                }
                std::cout << std::endl;
#endif
                for (const auto& it : rbuff)
                {
                    if (index_ == 0)
                    {
                        if ((it == SbusData::HEADER) &&
                            (pre_byte_ == SbusData::FOOTER || (pre_byte_ & 0x0F) == SbusData::FOOTER2))
                        {
                            msg_buf_[index_++] = it;
                        }
                        else
                        {
                            index_ = 0;
                        }
                    }
                    else if (index_ < SbusData::PAYLOAD_LEN + SbusData::HEADER_LEN)
                    {
                        msg_buf_[index_++] = it;
                    }
                    else if (index_ < SbusData::PAYLOAD_LEN + SbusData::HEADER_LEN + SbusData::FOOTER_LEN)
                    {
                        index_ = 0;
                        pre_byte_ = it;
                        if (it == SbusData::FOOTER || (it & 0x0F) == SbusData::FOOTER2)
                        {
                            const std::lock_guard<std::mutex> lock(mtx_);
                            // grab the channel data
                            data_.ch_[0] = static_cast<int16_t>(msg_buf_[1] | ((msg_buf_[2] << 8) & 0x07FF));
                            data_.ch_[1] = static_cast<int16_t>((msg_buf_[2] >> 3) | ((msg_buf_[3] << 5) & 0x07FF));
                            data_.ch_[2] = static_cast<int16_t>((msg_buf_[3] >> 6) | (msg_buf_[4] << 2) |
                                ((msg_buf_[5] << 10) & 0x07FF));
                            data_.ch_[3] = static_cast<int16_t>((msg_buf_[5] >> 1) | ((msg_buf_[6] << 7) & 0x07FF));
                            data_.ch_[4] = static_cast<int16_t>((msg_buf_[6] >> 4) | ((msg_buf_[7] << 4) & 0x07FF));
                            data_.ch_[5] = static_cast<int16_t>((msg_buf_[7] >> 7) | (msg_buf_[8] << 1) |
                                ((msg_buf_[9] << 9) & 0x07FF));
                            data_.ch_[6] = static_cast<int16_t>((msg_buf_[9] >> 2) | ((msg_buf_[10] << 6) & 0x07FF));
                            data_.ch_[7] = static_cast<int16_t>((msg_buf_[10] >> 5) | ((msg_buf_[11] << 3) & 0x07FF));
                            data_.ch_[8] = static_cast<int16_t>(msg_buf_[12] | ((msg_buf_[13] << 8) & 0x07FF));
                            data_.ch_[9] = static_cast<int16_t>((msg_buf_[13] >> 3) | ((msg_buf_[14] << 5) & 0x07FF));
                            data_.ch_[10] = static_cast<int16_t>((msg_buf_[14] >> 6) | (msg_buf_[15] << 2) |
                                ((msg_buf_[16] << 10) & 0x07FF));
                            data_.ch_[11] = static_cast<int16_t>((msg_buf_[16] >> 1) | ((msg_buf_[17] << 7) & 0x07FF));
                            data_.ch_[12] = static_cast<int16_t>((msg_buf_[17] >> 4) | ((msg_buf_[18] << 4) & 0x07FF));
                            data_.ch_[13] = static_cast<int16_t>((msg_buf_[18] >> 7) | (msg_buf_[19] << 1) |
                                ((msg_buf_[20] << 9) & 0x07FF));
                            data_.ch_[14] = static_cast<int16_t>((msg_buf_[20] >> 2) | ((msg_buf_[21] << 6) & 0x07FF));
                            data_.ch_[15] = static_cast<int16_t>((msg_buf_[21] >> 5) | ((msg_buf_[22] << 3) & 0x07FF));
                            // CH 17
                            data_.ch17_ = msg_buf_[23] & SbusData::CH17_MASK;
                            // CH 18
                            data_.ch18_ = msg_buf_[23] & SbusData::CH18_MASK;
                            // grab the lost frame
                            data_.lost_frame_ = msg_buf_[23] & SbusData::LOST_FRAME_MASK;
                            // grab the failsafe
                            data_.failsafe_ = msg_buf_[23] & SbusData::FAILSAFE_MASK;
                        }
                    }
                    else
                    {
                        index_ = 0;
                    }
                    pre_byte_ = it;
                }
            }
        }
    }
} // namespace whi_rc_bridge
