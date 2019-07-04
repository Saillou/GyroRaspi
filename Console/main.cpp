#define USE_CAMERA_0 0
#define USE_MPU_6050 1

// --------- General includes ---------
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
// -----------------------------------

// --------- Camera includes ---------
#if USE_CAMERA_0 > 0
#include <opencv2/opencv.hpp>
#endif
// -----------------------------------

// ----------- Mpu includes ----------
#if USE_MPU_6050 > 0
#include "Mpu_6050.hpp"
#endif
// -----------------------------------


int main() {
	// ------- Camera sources --------
#if USE_CAMERA_0 > 0
  cv::VideoCapture cap;
  if(!cap.open(0)) {
	  std::cout << "Could not open the camera 0" << std::endl;
	  return 1;
  }
  
  cv::Mat frame;

  for(;cv::waitKey(30) != 27;) {
     cap >> frame;
     if(frame.empty())
	continue;

     cv::imshow("frame", frame);
  }
#endif
	// ---------------------------------
	
	// --------------- Gyro sources ------------
#if USE_MPU_6050 > 0
	gz::Mpu_6050 mpu;
	if(!mpu.open("/dev/i2c-1", 0x68)) {
		std::cout << "Could not open the i2c slave" << std::endl;
		return 2;
	}
	
	mpu.start();
	gz::Mpu_6050::Data data;
	
	for(;;) {
		if(mpu.acquireData(data)) {			
			std::cout << "Temperature: " << data.temperature << std::endl;
			std::cout << "Accel: " << data.accel.x << ", " << data.accel.y << ", " << data.accel.z << std::endl;
			std::cout << "Gyro: " << data.gyro.x << ", " << data.gyro.y << ", " << data.gyro.z << std::endl;
			std::cout << " ----- " << std::endl;
		}
		
		std::this_thread::sleep_for(std::chrono::microseconds(1000));
	}
	
#endif

  return 0;
}
