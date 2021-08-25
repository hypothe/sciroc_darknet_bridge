#ifndef COMP_BRIDGE_CPP
#define COMP_BRIDGE_CPP

#include <ros/ros.h>
#include "sciroc_darknet_bridge/SciRocDarknetBridge.hpp"

// SciRoc_ObjDet_msgs
#include "sciroc_objdet/ObjectComparisonAction.h"
#include "sciroc_darknet_bridge/bridge_utils.hpp"


namespace sciroc_darknet_bridge
{
using CompAS = sciroc_darknet_bridge::SciRocDarknetBridge<sciroc_objdet::ObjectComparisonAction>;


class CompBridge : public CompAS
{
	public:
		CompBridge(ros::NodeHandle nh_, std::string action_server_name)
		: CompAS(nh_, action_server_name)
		{
			node_handle_.param("objdet/detection/threshold/comparison", det_threshold_, det_threshold_);
			std::string tmp_mode;
			node_handle_.param("objdet/detection/selection_mode/comparison", tmp_mode, std::string("MAX"));
			setSelectionMode(tmp_mode);
		}
	private:
		void saveGoalDataImp()
		{
			exp_classes.clear();
			for (auto expected_tag : action_.action_goal.goal.expected_tags)
			{
				++exp_classes[expected_tag];
			}
		}
		void setResultImp()
		{
			action_.action_result.result.found_tags.clear();
			std::vector<std::string> tags;
			std::map<std::string, u_int8_t> tag_map;

			bool found = false;
			switch (selection_mode_)
			{
				case SelectionMode::AVG:
					tag_map = sciroc_darknet_bridge_utils::setResAvg(tags, detectedBoxes);
					found = true;
					for (auto tag : tag_map)
					{
						found = found && (tag.second - exp_classes[tag.first]) > 0;
					}
					break;

				case SelectionMode::MODE:
					tag_map = sciroc_darknet_bridge_utils::setResMode(tags, detectedBoxes);
					found = true;
					
					for (auto tag : tag_map)
					{
						found = found && (tag.second - exp_classes[tag.first]) > 0;
					}

					break;

				case SelectionMode::MAX:
					tag_map = sciroc_darknet_bridge_utils::setResMode(tags, detectedBoxes);
					found = true;
					
					for (auto const& cls : exp_classes)
					{
						found = found && tag_map[cls.first];
					}
					break;
				default:
					ROS_WARN("[comp]: unexisting SelectionMode case");
					break;
			}


			action_.action_result.result.found_tags = tags;
			action_.action_result.result.match = found;
			

			ROS_DEBUG_NAMED("result", "[comp]: found classes");
			for (std::string tag : tags)
			{
				ROS_DEBUG_NAMED("result", "[comp]:\t %s", tag.c_str());
			}

		}
		
		std::map<std::string, size_t> exp_classes;
};

}

#endif // COMP_BRIDGE_CPP