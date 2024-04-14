/******************************************************************
separate include file to solve struct confliction

Features:
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-04-14: Initial version
2024-xx-xx: xxx
******************************************************************/

namespace whi_rc_bridge
{
    class SbusBaudrate
    {
    public:
        SbusBaudrate() = delete;
        ~SbusBaudrate() = delete;

    public:
        static bool sbusBaudrate(int Handle, int Baudrate);
    };
} // namespace whi_rc_bridge
