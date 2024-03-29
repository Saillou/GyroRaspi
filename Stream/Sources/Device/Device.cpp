#include "Device.hpp"

// Standard
#include <string>
#include <iostream>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>

// ----------  Implementation linux ----------
#ifdef __linux__

	#include "Device_impl_linux.hpp"

// ---------- Implementation windows ----------
#elif _WIN32

	#include "Device_impl_windows.hpp"
	
#endif

// --------- Interface publique ------------
// Constructors
Device::Device(const std::string& pathVideo) : _impl(new _Impl(pathVideo)) {
	// Wait open
}
Device::~Device() {
	delete _impl;
}

// Flow
bool Device::open() {
	return _impl->open();
}
bool Device::close() {
	return _impl->close();
}

// Method
bool Device::grab() {
	return _impl->grab();
}
bool Device::retrieve(Gb::Frame& frame) {
	return _impl->retrieve(frame);
}
bool Device::read(Gb::Frame& frame) {
	return _impl->read(frame);
}

// Setters
bool Device::setFormat(int width, int height, PixelFormat formatPix) {
	return _impl->setFormat(width, height, formatPix);
}
bool Device::set(Param code, double value) {
	return _impl->set(code, value);
}

// Getters
const Device::FrameFormat Device::getFormat() const {
	return _impl->getFormat();
}
double Device::get(Param code) {
	return _impl->get(code);
}




