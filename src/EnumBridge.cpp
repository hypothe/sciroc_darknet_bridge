#ifndef ENUM_BRIDGE_CPP
#define ENUM_BRIDGE_CPP

#include <ros/ros.h>
#include "sciroc_darknet_bridge/SciRocDarknetBridge.hpp"

// SciRoc_ObjDet_msgs
#include "sciroc_objdet/ObjectEnumerationAction.h"
#include "sciroc_darknet_bridge/bridge_utils.hpp"

namespace sciroc_darknet_bridge
{
using EnumAS = sciroc_darknet_bridge::SciRocDarknetBridge<sciroc_objdet::ObjectEnumerationAction>;

class EnumBridge : public EnumAS
{
	public:
		EnumBridge(ros::NodeHandle nh_, std::string action_server_name)
		: EnumAS(nh_, action_server_name)
		{
			node_handle_.param("objdet/detection/threshold/enumeration", det_threshold_, det_threshold_);
			std::string tmp_mode;
			node_handle_.param("objdet/detection/selection_mode/enumeration", tmp_mode, std::string("MODE"));
			setSelectionMode(tmp_mode);
		}
	private:
		void saveGoalDataImp(){}
		void setResultImp()
		{
			int foundBoxes = 0;

			switch (selection_mode_)
			{
				case SelectionMode::AVG:
					foundBoxes = sciroc_darknet_bridge_utils::setResAvg(detectedBoxes);
					break;
				case SelectionMode::MODE:
					foundBoxes = sciroc_darknet_bridge_utils::setResMode(detectedBoxes);
					break;
				case SelectionMode::MAX:
					foundBoxes = sciroc_darknet_bridge_utils::setResMax(detectedBoxes);
					break;
				default:
					ROS_WARN("[enum]: unexisting SelectionMode case");
			}
			
			action_.action_result.result.n_found_tags = foundBoxes;
			ROS_DEBUG_NAMED("result", "[enum:%d]: %ld detectedBoxesSize\n%d boxes found", static_cast<int>(selection_mode_), detectedBoxes.size(), foundBoxes);
		}
};

}

#endif // ENUM_BRIDGE_CPP