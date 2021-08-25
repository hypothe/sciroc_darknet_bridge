#ifndef CLAS_BRIDGE_CPP
#define CLAS_BRIDGE_CPP

#include <ros/ros.h>
#include "sciroc_darknet_bridge/SciRocDarknetBridge.hpp"

// SciRoc_ObjDet_msgs
#include "sciroc_objdet/ObjectClassificationAction.h"

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
					setResAvg(tags);
					break;
				case SelectionMode::MODE:
					setResMode(tags);
					break;
				case SelectionMode::MAX:
					setResMax(tags);
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
		
		void setResAvg(std::vector<std::string>& tags)
		{
			std::map<std::string, float> found_classes;
			for (auto imageBoxes : detectedBoxes)
			{
				for (auto box : imageBoxes)
				{
					found_classes[box.Class] += 1.0/detectedBoxes.size();
				}
			}
			int avg;
			for (auto tag_freq : found_classes)
			{
				ROS_DEBUG_NAMED("result", "[clas]: [%s]: %f", tag_freq.first.c_str(), tag_freq.second);
				avg = static_cast<int>(std::round(tag_freq.second));
				for (int i = 0; i < avg; ++i)
				{
					tags.push_back(tag_freq.first);
				}
			}
		}

		void setResMode(std::vector<std::string>& tags)
		{
			std::map<std::string, std::vector<int> > found_classes;
			int frame_n = 0;
			int num_frames = detectedBoxes.size();
			std::vector<darknet_ros_msgs::BoundingBox> imageBoxes;

			for (int frame_n = 0; frame_n < num_frames; ++frame_n)				
			{
				imageBoxes = detectedBoxes[frame_n];
				for (auto box : imageBoxes)
				{
					// dynamically allocate only for needed classes
					if (found_classes[box.Class].empty())
						found_classes[box.Class].resize(num_frames);

					++found_classes[box.Class][frame_n];
				}
			}

			// TODO: add a map <str, vec<int>>, the elements of each vector
			// have the # of items as index and the # of frames having that number
			// of that item in them

			std::map<std::string, std::vector<int>> class_cardinality_distr;
			// how many instances of the same tag I expect at max
			// in the same frame
			const int max_tag_rep = 16;

			for (auto cls : found_classes)
			{
				class_cardinality_distr[cls.first].resize(max_tag_rep);
				for (int frame_n = 0; frame_n < num_frames; ++frame_n)
				{
					++class_cardinality_distr[cls.first][cls.second[frame_n]];
				}
			}

			std::vector<int>::iterator res;
			int mode;
			for (auto tag_found : class_cardinality_distr)
			{
				for (int i = 0; i < max_tag_rep; ++i)
				{
					ROS_DEBUG_NAMED("result", "[clas]: [%s]: %d-> %d", tag_found.first.c_str(), i, tag_found.second[i]);	
				}
				res = std::max_element(tag_found.second.begin(), tag_found.second.end(),
															 [](int a, int b)
															 { return a <= b; });
				mode = std::distance(tag_found.second.begin(), res);
				ROS_DEBUG_NAMED("result", "[clas] mode %d", mode);

				for (int i = 0; i < mode; ++i)
				{
					tags.push_back(tag_found.first);
				}
			}
		}

		void setResMax(std::vector<std::string>& tags)
		{
			std::map<std::string, size_t> found_classes;
			for (auto imageBoxes : detectedBoxes)
			{
				for (auto box : imageBoxes)
				{
					// Only count the number of instances for expected objects
					++found_classes[box.Class];
				}
			}

			for (auto const& cls : found_classes)
				tags.push_back(cls.first);
			
		}
};

#endif // CLAS_BRIDGE_CPP