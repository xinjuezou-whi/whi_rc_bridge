/******************************************************************
I2C bus driver

Features:
- i2c bus operation
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rc_bridge/i2c.h"

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#if SMBUS
extern "C" {
#include <i2c/smbus.h>
}
#endif
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>

I2cDriver::I2cDriver(uint8_t Bus, uint8_t DeviceAddr)
	: bus_(Bus), device_addr_(DeviceAddr)
{
	connect();
}

I2cDriver::~I2cDriver()
{
	if (file_handle_ > 0)
	{
		close(file_handle_);
	}
}

void I2cDriver::connect()
{
	connected_ = false;

	if ((file_handle_ = open(("/dev/i2c-" + std::to_string(bus_)).c_str(), O_RDWR)) < 0)
	{
		// uint8_t is a typedef for unsigned char. plus the ostream class has a special overload for unsigned char,
		// so it prints the character with the number 5, which is non-visible
		std::cout << "failed to open I2C bus "  << (int)bus_ << std::endl;
	}
	else
	{
		if (ioctl(file_handle_, I2C_SLAVE, device_addr_) < 0)
		{
			std::cout << "failed to open i2c device " << (int)device_addr_ << " on bus " << (int)bus_ << std::endl;
		}
		else
		{
			connected_ = true;
		}
	}
}

bool I2cDriver::connected()
{
	return connected_;
}

bool I2cDriver::readBytes(uint8_t* ReadBuf, std::size_t BufSize)
{
	return readBytes(0, ReadBuf, BufSize);
}

bool I2cDriver::readBytes(uint8_t RegAddr, uint8_t* ReadBuf, std::size_t BufSize)
{
	if (file_handle_ != -1)
	{
#if SMBUS
		if (i2c_smbus_read_i2c_block_data(file_handle_, RegAddr, BufSize, ReadBuf) == int(BufSize))
		{
			return true;
		}
		else
		{
			std::cout << "failed to read from i2c reg addr " << (int)RegAddr << " of device " << (int)device_addr_ << " on bus " << (int)bus_ << std::endl;
			return false;
		}
#else
		struct i2c_smbus_ioctl_data args;
		args.read_write = I2C_SMBUS_READ;
		args.command = RegAddr;
		args.size = BufSize;
		union i2c_smbus_data smbusData;
		args.data = &smbusData;
		if (ioctl(file_handle_, I2C_SMBUS, &args) != -1)
		{
			std::vector<uint8_t> buf;
			buf.resize(BufSize);
			if (read(file_handle_, buf.data(), BufSize) != (ssize_t)BufSize)
			{
				std::cout << "failed to read from i2c reg addr " << (int)RegAddr << " of device " << (int)device_addr_ << " on bus " << (int)bus_ << std::endl;
				return false;
			}
			else
			{
				for (std::size_t i = 0; i < BufSize; ++i)
				{
					ReadBuf[i] = buf[i];
				}
				return true;
			}
		}
		else
		{
			return false;
		}
#endif
	}
	else
	{
		std::cout << "failed to read from i2c reg addr " << (int)RegAddr << " since it is not available" << std::endl;
		return false;
	}
}

bool I2cDriver::writeBytes(const uint8_t* DataBuf, std::size_t BufSize)
{
	return writeBytes(0, DataBuf, BufSize);
}

bool I2cDriver::writeBytes(uint8_t RegAddr, const uint8_t* DataBuf, std::size_t BufSize)
{
	if (file_handle_ != -1)
	{
		std::vector<uint8_t> buf;
		buf.push_back(RegAddr);
		for (std::size_t i = 0; i < BufSize; ++i)
		{
			buf.push_back(DataBuf[i]);
		}

		if (write(file_handle_, buf.data(), buf.size()) != (ssize_t)buf.size())
		{
			std::cout << "failed to write to i2c reg addr " << (int)RegAddr << " of device " << (int)device_addr_ << " on bus " << (int)bus_ << std::endl;
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		std::cout << "failed to write to i2c reg addr " << (int)RegAddr << " since it is not available" << std::endl;
		return false;
	}
}
