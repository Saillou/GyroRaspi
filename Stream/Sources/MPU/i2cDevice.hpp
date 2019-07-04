#pragma once

#include <string>
#include <iostream>
#include <vector>

#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <byteswap.h>

#include "i2c_smbus.h"

// Class to ease the i2c writting
class i2cDevice {
public:
	// --------------- Ctors ------------------
	i2cDevice() : _fd(-1), _id(-1) {
		
	}
	virtual ~i2cDevice() {
		release();
	}
	
	// ----------------------------------------
	// --------------- Methods ----------------
	// ----------------------------------------
	virtual bool open(const std::string path, const int idSlave) {
		if(_fd > 0)
			return true;
			
		// Open i2c bus
		_fd = ::open(path.c_str(), O_RDWR);
		if(_fd < 0)
			return false;
			
		// Check address
		_id = idSlave;
		if(ioctl(_fd, I2C_SLAVE, _id) < 0)
			return false;
		
		std::vector<unsigned char> _buffer;
		if(!readBytes(4, _buffer)) 
			return false;

		// Everything ok
		return true;
	}
	virtual void release() {
		if(_fd > -1)
			::close(_fd);
			
		_fd = -1;
		_id = -1;
	}
	
protected:
	// ------------------------------
	// -- Smbus io --
	// ------------------------------
	// Reading
	int8_t read8t(const __u8 cmd) {
		if(_fd < 0)
			return 0;
			
		return i2c_smbus_read_byte_data(_fd, cmd);
	}
	int16_t read16t(const __u8 cmd) {
		if(_fd < 0)
			return 0;
			
		return ((read8t(cmd) << 8) | read8t(cmd+1));
	}
	void readBytes(const __u8 cmd, const __u8 length, __u8 *values) {
		if(_fd < 0)
			return;
			
		i2c_smbus_read_i2c_block_data(_fd, cmd, length, values);
	}
	
	// Writting
	bool write8t(const __u8 cmd, const __u8 value) {
		if(_fd < 0)
			return false;
			
		return i2c_smbus_write_byte_data(_fd, cmd, value) != -1;
	}
	void writeBit(int register_address, int nth_bit, int value) {
		int8_t reg = read8t(register_address);
		
		if (value == 1)
			reg = reg | (1 << nth_bit);
		else
			reg = reg & ~(1 << nth_bit);
		
		write8t(register_address, reg);
	}
	
	// ------------------------------
	// -- System io --
	// ------------------------------
	bool readBytes(int length, std::vector<unsigned char>& buffer) {
		// Check
		if(length < 1 || _fd < 0) {
			buffer.clear();
			return false;
		}
		
		// Read
		buffer.resize((size_t)length);
		int totalRead = ::read(_fd, &buffer[0], length);
		
		return length == totalRead;
	}
	
	bool writeBytes(const std::vector<unsigned char>& buffer) {
		// Check
		if(buffer.empty() || _fd < 0) 
			return false;
		
		// Write
		int totalWrote = ::write(_fd, &buffer[0], (int)buffer.size());
		
		return (int)buffer.size() == totalWrote;
	}
	
private:	
	// Members
	int _fd;
	int _id;
};
