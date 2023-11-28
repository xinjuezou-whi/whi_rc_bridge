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

#include <asm/termbits.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <functional>
#include <iostream>

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

    std::vector<int> SbusBridge::readChannels()
    {
        return std::vector<int>();
    }

    bool SbusBridge::openSerial(const std::string& DeviceAddr, int Baudrate)
    {
        serial_handle_ = open(DeviceAddr.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_handle_ <= 0)
        {
            return false;
        }
        else
        {
            struct termios2 tio;
            int rc = ioctl(serial_handle_, TCGETS2, &tio);
            if (rc)
            {
                close(serial_handle_);
                std::cout << "failed to get termios2" << std::endl;

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
                        if ((it == HEADER) && ((pre_byte_ == FOOTER) || ((pre_byte_ & 0x0F) == FOOTER2)))
                        {
                            msg_buf_[index_++] = it;
                        }
                        else
                        {
                            index_ = 0;
                        }
                    }
                    else if (index_ < PAYLOAD_LEN + HEADER_LEN)
                    {
                        msg_buf_[index_++] = it;
                    }
                    else if (index_ < PAYLOAD_LEN + HEADER_LEN + FOOTER_LEN)
                    {
                        index_ = 0;
                        pre_byte_ = it;
                        if (it == FOOTER || (it & 0x0F) == FOOTER2)
                        {
                            // // Grab the channel data
                            auto ch0 = static_cast<int16_t>(msg_buf_[1] | ((msg_buf_[2] << 8) & 0x07FF));
                            auto ch1 = static_cast<int16_t>((msg_buf_[2] >> 3) | ((msg_buf_[3] << 5) & 0x07FF));
                            auto ch2 = static_cast<int16_t>((msg_buf_[3] >> 6) | (msg_buf_[4] << 2) |
                                ((msg_buf_[5] << 10) & 0x07FF));
                            auto ch3 = static_cast<int16_t>((msg_buf_[5] >> 1) | ((msg_buf_[6] << 7) & 0x07FF));
std::cout << "ch0 " << int(ch0) << ", ch1 " << int(ch1) << ", ch2 " << ch2 << ", ch3 " << ch3 << std::endl;
                            // data_.ch[0] = static_cast<int16_t>(buf_[1] | ((buf_[2] << 8) & 0x07FF));
                            // data_.ch[1] = static_cast<int16_t>((buf_[2] >> 3) | ((buf_[3] << 5) & 0x07FF));
                            // data_.ch[2] = static_cast<int16_t>((buf_[3] >> 6) | (buf_[4] << 2) | ((buf_[5] << 10) & 0x07FF));
                            // data_.ch[3] = static_cast<int16_t>((buf_[5] >> 1) | ((buf_[6] << 7) & 0x07FF));
                            // data_.ch[4] = static_cast<int16_t>((buf_[6] >> 4) | ((buf_[7] << 4) & 0x07FF));
                            // data_.ch[5] = static_cast<int16_t>((buf_[7] >> 7) | (buf_[8] << 1) | ((buf_[9] << 9) & 0x07FF));
                            // data_.ch[6] = static_cast<int16_t>((buf_[9] >> 2) | ((buf_[10] << 6) & 0x07FF));
                            // data_.ch[7] = static_cast<int16_t>((buf_[10] >> 5) | ((buf_[11] << 3) & 0x07FF));
                            // data_.ch[8] = static_cast<int16_t>(buf_[12] | ((buf_[13] << 8) & 0x07FF));
                            // data_.ch[9] = static_cast<int16_t>((buf_[13] >> 3) | ((buf_[14] << 5) & 0x07FF));
                            // data_.ch[10] = static_cast<int16_t>((buf_[14] >> 6) | (buf_[15] << 2) | ((buf_[16] << 10) & 0x07FF));
                            // data_.ch[11] = static_cast<int16_t>((buf_[16] >> 1) | ((buf_[17] << 7) & 0x07FF));
                            // data_.ch[12] = static_cast<int16_t>((buf_[17] >> 4) | ((buf_[18] << 4) & 0x07FF));
                            // data_.ch[13] = static_cast<int16_t>((buf_[18] >> 7) | (buf_[19] << 1) | ((buf_[20] << 9) & 0x07FF));
                            // data_.ch[14] = static_cast<int16_t>((buf_[20] >> 2) | ((buf_[21] << 6) & 0x07FF));
                            // data_.ch[15] = static_cast<int16_t>((buf_[21] >> 5) | ((buf_[22] << 3) & 0x07FF));
                            // // CH 17
                            // data_.ch17 = buf_[23] & CH17_MASK;
                            // // CH 18
                            // data_.ch18 = buf_[23] & CH18_MASK;
                            // // Grab the lost frame
                            // data_.lost_frame = buf_[23] & LOST_FRAME_MASK;
                            // // Grab the failsafe
                            // data_.failsafe = buf_[23] & FAILSAFE_MASK;
                            // //return true;
                        }
                        else
                        {
                            //return false;
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
