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

#include <fcntl.h>
#include <stropts.h>
#include <asm/termios.h>
#include <asm/ioctl.h>
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
        serial_handle_ = open(DeviceAddr.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
        if (serial_handle_ <= 0)
        {
            return false;
        }
        else
        {
            struct termios2 tio;
            ioctl(serial_handle_, TCGETS2, &tio);
            tio.c_cflag &= ~CBAUD;
            tio.c_cflag |= BOTHER;
            tio.c_ispeed = Baudrate;
            tio.c_ospeed = Baudrate;
            ioctl(serial_handle_, TCSETS2, &tio);

            return true;
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

    int SbusBridge::serialRead(char* Buf, unsigned int Len)
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
                char rbuff[count];
                int readNum = serialRead(rbuff, count);
#ifndef DEBUG
                std::cout << "read from SBUS: ";
                for (int i = 0; i < readNum; ++i)
                {
                    std::cout << int(rbuff[i]) << ",";
                }
                std::cout << std::endl;
#endif
                // fetchData(rbuff, readNum);
            }
        }
    }
} // namespace whi_rc_bridge
