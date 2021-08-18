#include <ros/ros.h>
#include "sciroc_darknet_bridge/SciRocDarknetBridge.hpp"

// SciRoc_ObjDet_msgs
#include "sciroc_objdet/ObjectEnumerationAction.h"
#include "sciroc_objdet/ObjectClassificationAction.h"
#include "sciroc_objdet/ObjectComparisonAction.h"

using EnumAS = sciroc_darknet_bridge::SciRocDarknetBridge<sciroc_objdet::ObjectEnumerationAction>;

class EnumBridge : public EnumAS
{
	public:
		EnumBridge(ros::NodeHandle nh_, std::string action_server_name)
		: EnumAS(nh_, action_server_name){}
	private:
		void resultCB(){}
		void saveGoalData(){}
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sciroc_darknet_bridge");
  ros::NodeHandle nodeHandle("~");
  EnumBridge enum_bridge(nodeHandle, "bridge_as");

  ros::spin();
  return 0;
}