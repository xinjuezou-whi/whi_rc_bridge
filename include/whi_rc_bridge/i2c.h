/******************************************************************
I2C bus driver

Features:
- i2c bus operation
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-03-06: Initial version
2022-11-05: fix read with register
2023-xx-xx: xxx
******************************************************************/
#pragma once
#include <cstdint>

#define SMBUS 0

class I2cDriver
{
public:
	I2cDriver() = delete;
	I2cDriver(uint8_t BusAddr, uint8_t DeviceAddr);
	~I2cDriver();

protected:
	void connect();

public:
	bool connected();
	bool readBytes(uint8_t* ReadBuf, std::size_t BufSize);
	bool readBytes(uint8_t RegAddr, uint8_t* ReadBuf, std::size_t BufSize);
	bool writeBytes(const uint8_t* DataBuf, std::size_t BufSize);
	bool writeBytes(uint8_t RegAddr, const uint8_t* DataBuf, std::size_t BufSize);

protected:
	uint8_t bus_{ 0 };
	uint8_t device_addr_{ 0 };
	int file_handle_{ -1 };
	bool connected_{ false };
};
