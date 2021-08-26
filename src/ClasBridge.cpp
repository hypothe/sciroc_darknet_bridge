#ifndef CLAS_BRIDGE_CPP
#define CLAS_BRIDGE_CPP

#include <ros/ros.h>
#include "sciroc_darknet_bridge/SciRocDarknetBridge.hpp"

// SciRoc_ObjDet_msgs
#include "sciroc_objdet/ObjectClassificationAction.h"

#include "sciroc_darknet_bridge/bridge_utils.hpp"

namespace sciroc_darknet_bridge
{
using ClasAS = sciroc_darknet_bridge::SciRocDarknetBridge<sciroc_objdet::ObjectClassificationAction>;

class ClasBridge : public ClasAS
{
	public:
		ClasBridge(ros::NodeHandle nh_, std::string action_server_name)
		: ClasAS(nh_, action_server_name)
		{
			node_handle_.param("objdet/detection/threshold/classification", det_threshold_, det_threshold_);
			std::string tmp_mode;
			node_handle_.param("objdet/detection/selection_mode/classification", tmp_mode, std::string("MAX"));
			setSelectionMode(tmp_mode);
		}
	private:
		void saveGoalDataImp(){}
		void setResultImp()
		{
			action_.action_result.result.found_tags.clear();
			std::vector<std::string> tags;
			switch (selection_mode_)
			{
				case SelectionMode::AVG:
					sciroc_darknet_bridge_utils::setResAvg(tags, detectedBoxes);
					break;
				case SelectionMode::MODE:
					sciroc_darknet_bridge_utils::setResMode(tags, detectedBoxes);
					break;
				case SelectionMode::MAX:
					sciroc_darknet_bridge_utils::setResMax(tags, detectedBoxes);
					break;
				default:
					ROS_WARN("[clas]: unexisting SelectionMode case");
					break;
			}
			
			// TODO: should I return only the 3 most frequent classes?
			ROS_DEBUG_NAMED("result", "[clas]: found classes");
			for (std::string tag : tags)
			{
				ROS_DEBUG_NAMED("result", "[clas]:\t %s", tag.c_str());
			}
			action_.action_result.result.found_tags = tags;
			
		}
};

}
#endif // CLAS_BRIDGE_CPP