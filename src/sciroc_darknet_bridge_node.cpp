#include <ros/ros.h>
#include "sciroc_darknet_bridge/SciRocDarknetBridge.hpp"

#include "EnumBridge.cpp"
#include "ClasBridge.cpp"
#include "CompBridge.cpp"

using namespace sciroc_darknet_bridge;

int main(int argc, char** argv) {
  ros::init(argc, argv, "sciroc_darknet_bridge_node");
  ros::NodeHandle nodeHandle;
	// TODO: get action names from rosparam
	std::string enum_name, clas_name, comp_name;
	nodeHandle.param("objdet/actions/enumeration/topic", enum_name, std::string("/enum_bridge_as"));
	nodeHandle.param("objdet/actions/classification/topic", clas_name, std::string("/clas_bridge_as"));
	nodeHandle.param("objdet/actions/comparison/topic", comp_name, std::string("/comp_bridge_as"));
	EnumBridge enum_bridge(nodeHandle, enum_name);
	ClasBridge clas_bridge(nodeHandle, clas_name);
  CompBridge comp_bridge(nodeHandle, comp_name);

  ros::spin();
  return 0;
}
