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
		: EnumAS(nh_, action_server_name),
			detThreshold_(0)
			{
				node_handle_.param("objdet/specs/detection_threshold", detThreshold_, float(0));
			}
	private:
		void saveGoalDataImp(){}
		void setResultImp()
		{
			float foundBoxes = 0;
			for (auto imageBoxes : detectedBoxes)
			{
				for (auto box : imageBoxes.bounding_boxes)
				{
					if (box.probability >= detThreshold_)	foundBoxes++;
				}
			}
			foundBoxes /= detectedBoxes.size();
			action_.action_result.result.n_found_tags = std::round(foundBoxes);
		}

		float detThreshold_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sciroc_darknet_bridge");
  ros::NodeHandle nodeHandle("~");
  EnumBridge enum_bridge(nodeHandle, "bridge_as");

  ros::spin();
  return 0;
}