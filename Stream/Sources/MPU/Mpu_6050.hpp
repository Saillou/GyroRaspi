#pragma once

#include "i2cDevice.hpp"

#include <iostream>

class Mpu_6050 : public i2cDevice {
public:
	// -- Converted data --
	struct vec3 {
		double x;
		double y;
		double z;
	};
	
	struct Data {
		double temperature; // celsius
		vec3 accel;			 // LSB/g
		vec3 gyro;			 // LSB/deg/second
	};
	
private:
	// -- Raw data --
	struct vec3_16 {
		int16_t x;
		int16_t y;
		int16_t z;
	};
	
	struct RawData {
		vec3_16 accel;
		int16_t temperature;
		vec3_16 gyro;
	};
	
public:
	// Constructor
	Mpu_6050() : fifoBuffer {0} {
		// Wait for open();
	}
	
	// Methods
	void start() {
		int8_t power = read8t(MPU_POWER1);
		
       // Wake up
        write8t(MPU_POWER1, 2); 
        write8t(MPU_POWER2, 0);
        
        // Settings
        write8t(ACCEL_CONFIG, ACCEL_SCALE_RANGE_2G);  // Set accel 2G
        write8t(GYRO_CONFIG, GYRO_SCALE_RANGE_250DEG);// Set Gyro 250 deg

		// Set fifo sample rate
		int sampleRate = 40; // /s
		int8_t irate = (8000 / sampleRate) -1;
		write8t(SAMPLE_RATE, irate);

		// Enable fifo
        writeBit(MPU_POWER0, 6, 1); // Enable fifo operations
        write8t(FIFO_EN, 0xf8); 	// Enable all sensors
		writeBit(MPU_POWER0, 2, 1); // Reset fifo
	}
	
	bool acquireData(Data& data) {
		// Check fifo
		int16_t countFifo = read16t(FIFO_COUNT);
		if(countFifo == 1024) // Overflow
			writeBit(0x6a, 2, 1); 	// Reset fifo
		else if(countFifo < 14) // Not enough data
			return false;

		// Read fifo register
		readBytes(FIFO_RW, 14, (__u8 *)fifoBuffer);
		
		for(int8_t i = 0; i < 7; i++)
			fifoBuffer[i] = __bswap_16(fifoBuffer[i]);
		
		RawData rawdata;
		
		rawdata.accel.x = fifoBuffer[0];
		rawdata.accel.y = fifoBuffer[1];
		rawdata.accel.z = fifoBuffer[2];
		
		rawdata.temperature = fifoBuffer[3];
		
		rawdata.gyro.x = fifoBuffer[4];
		rawdata.gyro.y = fifoBuffer[5];
		rawdata.gyro.z = fifoBuffer[6];
		
		// Convert
		scaledData(rawdata, data);
		
		return true;
	}
	
	void scaledData(const RawData& rawdata, Data& data) {
		data.temperature = _scaledTemp(rawdata.temperature);
		
		data.accel.x = _scaledAccel(rawdata.accel.x);
		data.accel.y = _scaledAccel(rawdata.accel.y);
		data.accel.z = _scaledAccel(rawdata.accel.z);
		
		data.gyro.x = _scaledGyro(rawdata.gyro.x);
		data.gyro.y = _scaledGyro(rawdata.gyro.y);
		data.gyro.z = _scaledGyro(rawdata.gyro.z);
	}

private:
	
	// -- Address registers
	enum Command {
		MPU_ACCEL_X = 0x3b,
		MPU_ACCEL_Y = 0x3d,
		MPU_ACCEL_Z = 0x3f,
		
		MPU_TEMP   = 0x41,

		MPU_GYRO_X = 0x43,
		MPU_GYRO_Y = 0x45,
		MPU_GYRO_Z = 0x47,

		MPU_POWER0 = 0x6a,
		MPU_POWER1 = 0x6b,
		MPU_POWER2 = 0x6c,
	};
	
	enum Fifo {
		// FIFO Enable Register
		FIFO_EN = 0x23,
		
		// FIFO Count Register
		FIFO_COUNT  = 0x72,
		FIFO_RW     = 0x74,
		SAMPLE_RATE = 0x19
	};
	
	enum Config {
		// Accel
		ACCEL_CONFIG 		  = 0x1c,
		ACCEL_SCALE_RANGE_2G  = 0x00,
		ACCEL_SCALE_RANGE_4G  = 0x08,
		ACCEL_SCALE_RANGE_8G  = 0x10,
		ACCEL_SCALE_RANGE_16G = 0x18,

		// Gyro
		GYRO_CONFIG 			 = 0x1b,
		GYRO_SCALE_RANGE_250DEG  = 0x00,
		GYRO_SCALE_RANGE_500DEG  = 0x08,
		GYRO_SCALE_RANGE_1000DEG = 0x10,
		GYRO_SCALE_RANGE_2000DEG = 0x18,
	};
	
	double _scaledTemp(const int16_t rawTemp) const {
		return (rawTemp / 340.0) + 36.53;
	}
	double _scaledAccel(const int16_t rawAccel) const {
		return 9.80665 * _signed(rawAccel) / 16384.0;
	}
	double _scaledGyro(const int16_t rawGyro) const {
		return _signed(rawGyro) / 131.0;
	}
	int _signed(const int16_t val) const {
		int signedVal = (val >= 0x8000) ? -(65536 - val) : val;
		return signedVal;
	}
	
	// Members
	int16_t fifoBuffer[7];
};
