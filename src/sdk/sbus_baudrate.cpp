/******************************************************************
separate include file to solve struct confliction

Features:
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rc_bridge/sbus_baudrate.h"

#include <asm/termbits.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>

namespace whi_rc_bridge
{
    bool SbusBaudrate::sbusBaudrate(int Handle, int Baudrate)
    {
        struct termios2 tio;
        int rc = ioctl(Handle, TCGETS2, &tio);
        if (rc)
        {
            close(Handle);

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
            tio.c_cflag &= ~CSIZE;
            tio.c_cflag |= CS8;
            tio.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
            tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            // tio.c_oflag &= ~OPOST;
            tio.c_oflag = 0; // no remapping, no delays
            tio.c_cc[VMIN] = 0;
            tio.c_cc[VTIME] = 0;
            // Set new serial port settings via supported ioctl
            rc = ioctl(Handle, TCSETS2, &tio);

            return (rc == 0);
        }
    }
} // namespace whi_rc_bridge
