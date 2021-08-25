#pragma once

#ifndef BRIDGE_UTILS_HPP
#define BRIDGE_UTILS_HPP

#include <ros/ros.h>

#include <darknet_ros_msgs/BoundingBox.h>

using BoundingBoxes2D = std::vector<std::vector<darknet_ros_msgs::BoundingBox>>;

namespace sciroc_darknet_bridge
{
	namespace sciroc_darknet_bridge_utils
	{
		std::map<std::string, u_int8_t> setResAvg(std::vector<std::string> &tags, const BoundingBoxes2D& detectedBoxes);
		std::map<std::string, u_int8_t> setResMode(std::vector<std::string> &tags, const BoundingBoxes2D& detectedBoxes);
		std::map<std::string, u_int8_t> setResMax(std::vector<std::string> &tags, const BoundingBoxes2D& detectedBoxes);

		int setResAvg(const BoundingBoxes2D& detectedBoxes);
		int setResMode(const BoundingBoxes2D& detectedBoxes);
		int setResMax(const BoundingBoxes2D& detectedBoxes);
	}
}

#endif // BRIDGE_UTILS_HPP