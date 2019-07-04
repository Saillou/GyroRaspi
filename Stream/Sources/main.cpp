#include <iostream>
#include <csignal>
#include <atomic>
#include <map>
#include <deque>

#include "Device/DeviceMt.hpp"
#include "Network/Server.hpp"
#include "Network/Message.hpp"
#include "Timer.hpp"

#include "MPU/Mpu_6050.hpp"

namespace Globals {
	// Constantes
	const int PORT = 8888;
	const std::string PATH_CAMERA = "/dev/video0";
	
	// Variables
	volatile std::sig_atomic_t signalStatus = 0;
}

// --- Structures ---
struct ClientRequest {
	bool play;
};

// --- Signals ---
static void sigintHandler(int signal) {
	Globals::signalStatus = signal;
}

// --- Entry point ---
int main() {
	// -- Install signal handler
	std::signal(SIGINT, sigintHandler);
	
	// Variables
	Server server;
	DeviceMt device;
	std::map<SOCKET, ClientRequest> mapRequests;
	
	// -- Connect server --
	server.connectAt(Globals::PORT);
	
	server.onClientConnect([&](const Server::ClientInfo& client) {
		std::cout << "New client, client_" << client.id << std::endl;
		mapRequests[client.id].play = false;
	});
	server.onClientDisconnect([&](const Server::ClientInfo& client) {
		std::cout << "Client quit, client_" << client.id << std::endl;
		mapRequests[client.id].play = false;
	});
	server.onError([&](const Error& error) {
		std::cout << "Error : " << error.msg() << std::endl;
	});
	
	server.onInfo([&](const Server::ClientInfo& client, const Message& message) {
		std::cout << "Info received from client_" << client.id << ": [Code:" << message.code() << "] " << message.str() << std::endl;
		if(message.code() == Message::TEXT && message.str() == "Send") {
			mapRequests[client.id].play = true;
		}
	});
	server.onData([&](const Server::ClientInfo& client, const Message& message) {
		std::cout << "Data received from client_" << client.id << ": [Code:" << message.code() << "] " << message.str() << std::endl;
	});
	
	
	Timer t;
	std::deque<double> freq(100, 0.0);
	
	// -- Open devices --	
	if(device.open(Globals::PATH_CAMERA)) {
		// Params
		device.setFormat(640, 480, Device::MJPG);	
		
		// Events		
		device.onFrame([&](const Gb::Frame& frame) {		
			// Send camera frame
			for(auto& client: server.getClients()) {
				if(client.connected && mapRequests[client.id].play) {
					server.sendData(client, Message(Message::CAMERA, reinterpret_cast<const char*>(frame.start()), frame.length()));
				}
			}
		});
	}
	

	
	// -------- Main loop --------  
	// Connect mpu
	Mpu_6050 mpu;
	if(!mpu.open("/dev/i2c-1", 0x68)) {
		std::cout << "Could not open the i2c slave" << std::endl;
	}
	
	Mpu_6050::Data data;
	for(Timer timer; Globals::signalStatus != SIGINT; timer.wait(1)) {
		if(mpu.acquireData(data)) {	
			// Create message
			MessageFormat msgMpu;
			msgMpu.add("temperature", data.temperature);
			msgMpu.add("accel_x", data.accel.x);
			msgMpu.add("accel_y", data.accel.y);
			msgMpu.add("accel_z", data.accel.z);
			msgMpu.add("gyro_x", data.gyro.x);
			msgMpu.add("gyro_y", data.gyro.y);
			msgMpu.add("gyro_z", data.gyro.z);
			
			// Send Mpu
			for(auto& client: server.getClients()) {
				if(client.connected && mapRequests[client.id].play) {
					server.sendData(client, Message(Message::MPU, msgMpu.str()));
				}
			}
		}
	}
		
	// -- End
	device.release();
	server.disconnect();
	
	std::cout << "Clean exit" << std::endl;
	return 0;
}

