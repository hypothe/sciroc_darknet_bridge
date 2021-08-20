#include <ros/ros.h>
#include "sciroc_darknet_bridge/SciRocDarknetBridge.hpp"

// SciRoc_ObjDet_msgs
#include "sciroc_objdet/ObjectEnumerationAction.h"
#include "sciroc_objdet/ObjectClassificationAction.h"
#include "sciroc_objdet/ObjectComparisonAction.h"

using EnumAS = sciroc_darknet_bridge::SciRocDarknetBridge<sciroc_objdet::ObjectEnumerationAction>;
using ClasAS = sciroc_darknet_bridge::SciRocDarknetBridge<sciroc_objdet::ObjectClassificationAction>;
using CompAS = sciroc_darknet_bridge::SciRocDarknetBridge<sciroc_objdet::ObjectComparisonAction>;

class EnumBridge : public EnumAS
{
	public:
		EnumBridge(ros::NodeHandle nh_, std::string action_server_name)
		: EnumAS(nh_, action_server_name)
		{
			node_handle_.param("objdet/detection/threshold/enumeration", det_threshold_, det_threshold_);
		}
	private:
		void saveGoalDataImp(){}
		void setResultImp()
		{
			float foundBoxes = 0;
			for (auto imageBoxes : detectedBoxes)
			{
				for (auto box : imageBoxes)
				{
					// if (box.probability >= detThreshold_)
					++foundBoxes;
				}
			}
			foundBoxes /= detectedBoxes.size();
			action_.action_result.result.n_found_tags = std::round(foundBoxes);
		}
};

class ClasBridge : public ClasAS
{
	public:
		ClasBridge(ros::NodeHandle nh_, std::string action_server_name)
		: ClasAS(nh_, action_server_name)
		{
			node_handle_.param("objdet/detection/threshold/classification", det_threshold_, det_threshold_);
		}
	private:
		void saveGoalDataImp(){}
		void setResultImp()
		{
			for (auto imageBoxes : detectedBoxes)
			{
				for (auto box : imageBoxes)
				{
					++found_classes[box.Class];
				}
			}
			// NOTE: this returns all classess perceived at least once in one
			// of the analyzed images
			// TODO: should I return only the 3 most frequent classes?
			for (auto const &cls : found_classes)
			{
				
				action_.action_result.result.found_tags.push_back(cls.first);
			}
		}
		std::map<std::string, size_t> found_classes;
};

class CompBridge : public CompAS
{
	public:
		CompBridge(ros::NodeHandle nh_, std::string action_server_name)
		: CompAS(nh_, action_server_name)
		{
			node_handle_.param("objdet/detection/threshold/comparison", det_threshold_, det_threshold_);
		}
	private:
		void saveGoalDataImp()
		{
			for (auto expected_tag : action_.action_goal.goal.expected_tags)
			{
				exp_classes[expected_tag] = 0;
			}
		}
		void setResultImp()
		{
			for (auto imageBoxes : detectedBoxes)
			{
				for (auto box : imageBoxes)
				{
					// Only count the number of instances for expected objects
					if (exp_classes.count(box.Class))
						++exp_classes[box.Class];
				}
			}
			// NOTE: it does count one of the expected classes as "found" even
			// if it appears only once among all the images analyzed.
			// TODO: might be better to set a threshold (eg. appears at least 0.8
			// times per object expected)
			// (remember to keep track of multiplicity in expected tags!!!)

			bool found = true;
			for (auto const& cls : exp_classes)
			{
				found = found && cls.second;
				// if one of the expected classes has 0 elements set the flag to false
				action_.action_result.result.found_tags.push_back(cls.first);
			}
			action_.action_result.result.match = found;
		}
		std::map<std::string, size_t> exp_classes;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sciroc_darknet_bridge");
  ros::NodeHandle nodeHandle("~");
  EnumBridge enum_bridge(nodeHandle, "bridge_as");

  ros::spin();
  return 0;
}