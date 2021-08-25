#ifndef ENUM_BRIDGE_CPP
#define ENUM_BRIDGE_CPP

#include <ros/ros.h>
#include "sciroc_darknet_bridge/SciRocDarknetBridge.hpp"

// SciRoc_ObjDet_msgs
#include "sciroc_objdet/ObjectEnumerationAction.h"

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
					setResAvg(foundBoxes);
					break;
				case SelectionMode::MODE:
					setResMode(foundBoxes);
					break;
				case SelectionMode::MAX:
					setResMax(foundBoxes);
					break;
				default:
					ROS_WARN("[enum]: unexisting SelectionMode case");
			}
			
			action_.action_result.result.n_found_tags = foundBoxes;
			ROS_DEBUG_NAMED("result", "[enum:%d]: %ld detectedBoxesSize\n%d boxes found", static_cast<int>(selection_mode_), detectedBoxes.size(), foundBoxes);
		}

	void setResAvg(int& foundBoxes)
	{
		float tmp_found_boxes = 0;
		for (auto imageBoxes : detectedBoxes)
		{
			tmp_found_boxes += imageBoxes.size();
			ROS_DEBUG_NAMED("result", "[enum]: %ld imageBoxesSize", imageBoxes.size());
		}
		foundBoxes = static_cast<int>(tmp_found_boxes / detectedBoxes.size());
	}

	void setResMode(int& foundBoxes)
	{
		std::vector<int> freq_found(256, 0);
		std::vector<int>::iterator res;
		for (auto imageBoxes : detectedBoxes)
		{
			++freq_found[imageBoxes.size()];
			ROS_DEBUG_NAMED("result", "[enum]: %ld imageBoxesSize", imageBoxes.size());
		}
		for (int i = 0; i < 10; i++)
		{
			ROS_DEBUG_NAMED("result", "\t[enum]: freq_found[%d]: %d", i, freq_found[i]);
		}
		res = std::max_element(freq_found.begin(), freq_found.end(),
														[](int a, int b)
														{ return a <= b; }
													);

		foundBoxes = std::distance(freq_found.begin(), res);
	}

	void setResMax(int& foundBoxes)
	{
		int tmp_found_boxes;
		for (auto imageBoxes : detectedBoxes)
		{
			tmp_found_boxes = imageBoxes.size();
			
			if (tmp_found_boxes > foundBoxes)
				foundBoxes = tmp_found_boxes;
			ROS_DEBUG_NAMED("result", "[enum]: %ld imageBoxesSize", imageBoxes.size());
		}
	}
};

#endif // ENUM_BRIDGE_CPP