#include "sciroc_darknet_bridge/bridge_utils.hpp"

namespace sciroc_darknet_bridge
{
	namespace sciroc_darknet_bridge_utils
	{
		std::map<std::string, u_int8_t> setResAvg(std::vector<std::string> &tags, const BoundingBoxes2D& detectedBoxes)
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
			std::map<std::string, u_int8_t> tag_avg;
			for (auto tag_freq : found_classes)
			{
				ROS_DEBUG_NAMED("result", "[%s]: %f", tag_freq.first.c_str(), tag_freq.second);
				avg = static_cast<int>(std::round(tag_freq.second));
				tag_avg[tag_freq.first] = avg;
				for (int i = 0; i < avg; ++i)
				{
					tags.push_back(tag_freq.first);
				}
			}
			return tag_avg;
		}

		std::map<std::string, u_int8_t> setResMode(std::vector<std::string> &tags, const BoundingBoxes2D& detectedBoxes)
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
			std::map<std::string, u_int8_t> tags_mode;
			int mode;
			for (auto tag_found : class_cardinality_distr)
			{
				for (int i = 0; i < max_tag_rep; ++i)
				{
					ROS_DEBUG_NAMED("result", "[%s]: %d-> %d", tag_found.first.c_str(), i, tag_found.second[i]);	
				}
				res = std::max_element(tag_found.second.begin(), tag_found.second.end(),
															 [](int a, int b)
															 { return a <= b; });
				mode = std::distance(tag_found.second.begin(), res);
				tags_mode[tag_found.first] = mode;
				ROS_DEBUG_NAMED("result", "mode %d", mode);

				for (int i = 0; i < mode; ++i)
				{
					tags.push_back(tag_found.first);
				}
			}
			return tags_mode;
		}
		std::map<std::string, u_int8_t> setResMax(std::vector<std::string> &tags, const BoundingBoxes2D& detectedBoxes)
		{
			std::map<std::string, u_int8_t> tags_all;
			for (auto imageBoxes : detectedBoxes)
			{
				for (auto box : imageBoxes)
				{
					// Only count the number of instances for expected objects
					++tags_all[box.Class];
				}
			}

			for (auto const& cls : tags_all)
				tags.push_back(cls.first);

			return tags_all;
		}

		int setResAvg(const BoundingBoxes2D& detectedBoxes)
		{
			float tmp_found_boxes = 0;
			for (auto imageBoxes : detectedBoxes)
			{
				tmp_found_boxes += imageBoxes.size();
				ROS_DEBUG_NAMED("result", "%ld imageBoxesSize", imageBoxes.size());
			}
			return static_cast<int>(tmp_found_boxes / detectedBoxes.size());
		}
		int setResMode(const BoundingBoxes2D& detectedBoxes)
		{
			std::vector<int> freq_found(256, 0);
			std::vector<int>::iterator res;
			for (auto imageBoxes : detectedBoxes)
			{
				++freq_found[imageBoxes.size()];
				ROS_DEBUG_NAMED("result", "%ld imageBoxesSize", imageBoxes.size());
			}
			for (int i = 0; i < 10; i++)
			{
				ROS_DEBUG_NAMED("result", "\tfreq_found[%d]: %d", i, freq_found[i]);
			}
			res = std::max_element(freq_found.begin(), freq_found.end(),
															[](int a, int b)
															{ return a <= b; }
														);

			return std::distance(freq_found.begin(), res);
		}
		int setResMax(const BoundingBoxes2D& detectedBoxes)
		{
			int max_found_boxes = 0;
			int tmp_found_boxes;
			for (auto imageBoxes : detectedBoxes)
			{
				tmp_found_boxes = imageBoxes.size();
				
				if (tmp_found_boxes > max_found_boxes)
					max_found_boxes = tmp_found_boxes;
				ROS_DEBUG_NAMED("result", "%ld imageBoxesSize", imageBoxes.size());
			}
			return max_found_boxes;
		}
	}
}