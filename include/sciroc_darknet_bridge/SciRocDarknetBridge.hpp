/*
 * SciRocDarknetBridge.hpp
 *
 *  Created on: August 16, 2021
 *      Author: Marco Gabriele Fedozzi
 *   Institute: University of Genoa, MSc Robotics Engineering
 */

#pragma once
#ifndef SCIROC_DARKNET_BRIDGE_H
#define SCIROC_DARKNET_BRIDGE_H

// ROS
#include <ros/ros.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
//#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
//#include <std_msgs/Header.h>

// Darknet_ROS
#include <darknet_ros_msgs/CheckForObjectsAction.h>

// SciRoc_ObjDet_msgs
#include "sciroc_objdet/ObjectEnumerationAction.h"
#include "sciroc_objdet/ObjectClassificationAction.h"
#include "sciroc_objdet/ObjectComparisonAction.h"

// C++
#include <pthread.h>
#include <chrono>
// #include <cmath>
// #include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

namespace sciroc_darknet_bridge
{
template <typename T>
class SciRocDarknetBridge
{
	public:
		SciRocDarknetBridge(ros::NodeHandle nh_, std::string action_server_name);
		// static_assert(std::is_base_of<BaseClass, Derived>::value, "Derived not derived from BaseClass");
		~SciRocDarknetBridge();

	// 1 generic SciRoc actionserver
	// 1 Darknet_ROS actionclient
	private:
		/* -- METHODS -- */
		
		void waitForServer();
		/*	SciRoc ObjDet Action Server side	*/
		void goalCB();
		void preemptCB();
		virtual void resultCB() = 0; // has to be implemented by the specific subclasses
		
		/*	Darknet_ROS Action Client side	*/
		void sendGoal();
		void clockCB(const ros::TimerEvent&);

		/*
			Subscribe to the camera topic and keep track of the latest
			image received (which will be sent to the darknet_ros_as)
		*/
		void cameraCallback(const sensor_msgs::ImageConstPtr &msg);

		/* -- MEMBERS -- */
		ros::NodeHandle node_handle_;

		typedef actionlib::SimpleActionServer<T> ASType;
		typedef std::shared_ptr<ASType> ASTypePtr;
		typedef std::shared_ptr<actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> > ACTypePtr;

		std::string as_name_;
		ASTypePtr as_; //-> generic ActionServer
		ACTypePtr ac_; //-> darknet_ros ActionClient
		ros::Timer as_clock;
		const double as_clock_period = 0.2; // expect a reply 5 times per second, adjust if need be
		
		// Camera readings
		std::shared_ptr<image_transport::ImageTransport> it;
		std::shared_ptr<image_transport::Subscriber> camera_sub_;
		
		// Image 
		/* use this to access the image for reading it (shared_lock)
		 or modifying it (unique_lock)
		*/
		sensor_msgs::Image current_img_;
		boost::shared_mutex mutexCurrentImage_; 
		/*
			Base template class, needs to be implemented by the three different ActionServers
			The client part will be the same, but the resultCB will be pure virtual
		*/
};
} // namespace
#endif // SCIROC_DARKNET_BRIDGE_H