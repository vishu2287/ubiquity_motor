/**
Copyright (c) 2016, Ubiquity Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of ubiquity_motor nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include <ubiquity_motor/motor_hardware.h>
#include <ubiquity_motor/motor_message.h>
#include <string>
#include <boost/thread.hpp>
#include <time.h>
#include "controller_manager/controller_manager.h"
#include <ros/ros.h>

#include <ubiquity_motor/Leds.h>

static const double BILLION = 1000000000.0;
struct timespec last_time;
struct timespec current_time;

// void controlLoop(
// 	MotorHardware &robot,
// 	controller_manager::ControllerManager &cm,
// 	timespec &last_time,
// 	timespec &current_time){
	
// 	clock_gettime(CLOCK_MONOTONIC, &current_time);
// 	ros::Duration elapsed = ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
// 	last_time = current_time;
// 	robot.sendPid();
// 	robot.readInputs();
// 	cm.update(ros::Time::now(), elapsed);
// 	robot.writeSpeeds();	

// }

main(int argc, char* argv[]) {
	ros::init(argc, argv, "motor_node");
	ros::NodeHandle nh;


	std::string sPort;
	int sBaud;

	double sLoopRate;

	if (!nh.getParam("ubiquity_motor/serial_port", sPort))
	{
		sPort.assign("/dev/ttyS0");
		nh.setParam("ubiquity_motor/serial_port", sPort);
	}

	if (!nh.getParam("ubiquity_motor/serial_baud", sBaud))
	{
		sBaud = 9600;
		nh.setParam("ubiquity_motor/serial_baud", sBaud);
	}

	if (!nh.getParam("ubiquity_motor/serial_loop_rate", sLoopRate))
	{
		sLoopRate = 100;
		nh.setParam("ubiquity_motor/serial_loop_rate", sLoopRate);
	}

	MotorSerial motor_serial(sPort,sBaud,sLoopRate);

	ros::Rate r(5); 
	while (ros::ok()){
		MotorMessage left_vel;
		left_vel.setRegister(MotorMessage::REG_LEFT_SPEED_SET);
		left_vel.setType(MotorMessage::TYPE_WRITE);
		left_vel.setData(100);
		motor_serial.transmitCommand(left_vel);
		r.sleep();
	}


	ros::spin();
}
