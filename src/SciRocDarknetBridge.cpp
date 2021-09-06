#include "sciroc_darknet_bridge/SciRocDarknetBridge.hpp"
#include <random>

using namespace std::literals::chrono_literals;

namespace sciroc_darknet_bridge
{
template <typename T>
SciRocDarknetBridge<T>::SciRocDarknetBridge(ros::NodeHandle nh_, std::string action_server_name)
: node_handle_(nh_),
	as_clock_period(0.2),
	enable_head_movement_(false),
	as_name_(action_server_name),
	as_(std::make_shared<ASType>(nh_, action_server_name, false)),
	image_sent_id_(0), image_detected_id_(0)
{
	ROS_INFO("[%s]: Booting.", as_name_.c_str());
	node_handle_.param("objdet/detection/display_image", display_image_, false);
	// set frequency of clock (how frequently to sample & detect images)
	node_handle_.param("objdet/detection/period/yolo", as_clock_period, double(0.2));
	// detection threshold (redundant, can be expressed directly in darknet_ros)
	node_handle_.param("objdet/detection/threshold/yolo", det_threshold_, float(0));
	// Selection Mode: how to aggregate the data received over multiple frames
	std::string tmp_mode;
	node_handle_.param("objdet/detection/selection_mode/yolo", tmp_mode, std::string("AVG"));
	setSelectionMode(tmp_mode);
	// Clock
	as_clock = node_handle_.createTimer(ros::Duration(as_clock_period), &SciRocDarknetBridge<T>::clockCB, this, false, false);
	// ActionClient
	std::string checkForObjectsActionName;
  node_handle_.param("objdet/actions/detection/topic", checkForObjectsActionName, std::string("/yolov5_as/check_for_objects"));
	ac_ = std::make_shared<ACType>(checkForObjectsActionName, true);
	// Head movement ActionClient
	std::string headMovementActionName;
  node_handle_.param("objdet/actions/head_movement/topic", headMovementActionName, std::string("/head_controller/point_head_action"));
  node_handle_.param("objdet/actions/head_movement/enable", enable_head_movement_, false);
	node_handle_.param("objdet/actions/head_movement/frame_id", common_head_movement_traits_.target.header.frame_id, std::string("/map"));
	node_handle_.param("objdet/actions/head_movement/pointing_frame", common_head_movement_traits_.pointing_frame, std::string("/xtion_rgb_optical_frame"));
	node_handle_.param("objdet/actions/head_movement/min_duration/s", common_head_movement_traits_.min_duration.sec, 1);
	node_handle_.param("objdet/actions/head_movement/max_velocity", common_head_movement_traits_.max_velocity, double(2.0));
	common_head_movement_traits_.pointing_axis.z = 1.0;
	// ActionServer
	as_->registerGoalCallback(boost::bind(&SciRocDarknetBridge<T>::goalCB, this));
	as_->registerPreemptCallback(boost::bind(&SciRocDarknetBridge<T>::preemptCB, this));
	// Subscriber
	std::string cameraTopicName;
	int cameraQueueSize;
	node_handle_.param("objdet/subscribers/camera_reading/topic", cameraTopicName, std::string("/camera/image_raw"));
	node_handle_.param("objdet/subscribers/camera_reading/queue_size", cameraQueueSize, 1);
	it = std::make_shared<image_transport::ImageTransport>(node_handle_);
	// Start the components
	camera_sub_ = std::make_shared<image_transport::Subscriber>(it->subscribe(cameraTopicName, cameraQueueSize,
																																						&SciRocDarknetBridge<T>::cameraCallback, this));
	waitForServer(checkForObjectsActionName, ac_);
	
	if (enable_head_movement_) // only create the AC if movement has been enabled
	{
		head_movement_ac_ = std::make_shared<HeadACType>(headMovementActionName, true);
		waitForServer(headMovementActionName, head_movement_ac_); 
	}
	as_->start();

	// color vector
	if (display_image_)
	{
		for (uint16_t i = 0; i < 256; i += 64)
		{
			for (uint16_t j = 0; j < 256; j += 64)
			{
				for (uint16_t k = 0; k < 256; k += 64)
				{
					if (i == j && i == k )
						continue;
					colors_.push_back(cv::Scalar(i, j, k));
				}
			}
		}
		std::shuffle(colors_.begin(), colors_.end(), std::default_random_engine());
		ROS_DEBUG_NAMED("display", "[%s] colors vector filled with %ld elements", as_name_.c_str(), colors_.size());
	}
	ROS_INFO("[%s]: Up and running.", as_name_.c_str());
}

template <typename T>
SciRocDarknetBridge<T>::~SciRocDarknetBridge()
{
	// ActoinServer
	as_->setAborted();
	as_->shutdown();
	// ActionClient
	ac_->stopTrackingGoal();
	head_movement_ac_->stopTrackingGoal();
	// Clock
	as_clock.stop();
	// Thread
	if (move_head_thread.joinable())
		move_head_thread.join();
}

/*	-- METHODS -- */

template <typename T>
void SciRocDarknetBridge<T>::setSelectionMode(int mode)
{
	selection_mode_ = (mode <= static_cast<int>(SelectionMode::MAX) && mode >= 0) ? static_cast<SelectionMode>(mode) : SelectionMode::AVG;
}

template <typename T>
void SciRocDarknetBridge<T>::setSelectionMode(std::string mode)
{
	for (auto & c : mode)
		c = (char)toupper(c);

	if (mode == std::string("MODE"))
		selection_mode_ = SelectionMode::MODE;
	else if (mode == std::string("MAX"))
		selection_mode_ = SelectionMode::MAX;
	else
		selection_mode_ = SelectionMode::AVG;
}

template <typename T>
template <typename ClientTypePtr>
void SciRocDarknetBridge<T>::waitForServer(std::string server_name, ClientTypePtr action_client)
{
	int failedConnectionAttempt = 0;
	while (!action_client->waitForServer(ros::Duration(1.0)) && failedConnectionAttempt < maxFailedConnectionAttempts)
	{
		ROS_WARN("Waiting for Action Server %s", server_name.c_str());
		failedConnectionAttempt++;
	}
	if (failedConnectionAttempt > maxFailedConnectionAttempts)
	{
		ROS_ERROR("NO SERVER %s FOUND AFTER %d attempts, shutting down.", \
							server_name.c_str(), maxFailedConnectionAttempts);
		node_handle_.shutdown();
	}
	return ;
}

// TODO: does it make sense to subscribe/unsubscribe at runtime when the action is called?
template <typename T>
void SciRocDarknetBridge<T>::cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lockCurrentImage(mutexCurrentImage_);

	current_img_ = *msg; // update the last stored frame
}

template <typename T>
geometry_msgs::Point SciRocDarknetBridge<T>::retrieveTablePos()
{
	/*	Temporary definition!	*/
	// TODO: actually retrieve the info from the server
	ROS_DEBUG("[%s]: retrieving table position.", as_name_.c_str());
	geometry_msgs::Point table_pos;

	table_pos.x = action_.action_goal.goal.table_pos.x;
	table_pos.y = action_.action_goal.goal.table_pos.y;

	if (!node_handle_.getParam("objdet/actions/head_movement/pointing_axis/z", table_pos.z))
  {
    ROS_ERROR("[%s]: no table height detected", as_name_.c_str());
  }

	return table_pos;
}

template <typename T>
void SciRocDarknetBridge<T>::moveHead(geometry_msgs::Point table_pos) // in thread
{
	// ask to an action server to move the downward then upward
	{
		boost::unique_lock<boost::shared_mutex> lockAcquisitionStatus(mutexAcquisitionStatus_);
    acquisition_status_ = AcquisitionStatus::ONGOING;
	}
	ROS_DEBUG_NAMED("head", "[%s]: Head started moving\n looking at [%f, %f, %f]", 	as_name_.c_str(),
																																		table_pos.x, table_pos.y, table_pos.z);

	geometry_msgs::Point look_up = table_pos;
	look_up.z += 0.2; //0.1;
	geometry_msgs::Point look_down = table_pos;
	look_down.z -= 0.2; // 0.1;
	// Sequence of points for a "down - up - back to start" movement
	std::vector<geometry_msgs::Point> look_points {look_down, table_pos, look_up, table_pos};
	control_msgs::PointHeadGoal head_goal = common_head_movement_traits_;
	std::chrono::duration<double> head_move_duration(static_cast<int>(head_goal.min_duration.sec));

	for (auto point : look_points)
	{
		if (enable_head_movement_)
		{
			auto rqst_start = std::chrono::steady_clock::now();
			head_goal.target.point = point;
			head_movement_ac_->sendGoal(head_goal);
			ROS_DEBUG_NAMED("head", "[%s]: Head movement action request", as_name_.c_str());

			head_movement_ac_->waitForResult(ros::Duration(2*head_goal.min_duration.sec));
			actionlib::SimpleClientGoalState state = head_movement_ac_->getState();

			ROS_DEBUG_NAMED("head", "[%s]: Head movement action terminated with %s", as_name_.c_str(), state.toString().c_str());
			// this is here (hopefully temporarily, delaing with the head action server returning at seemingly
			// random delays)
			std::this_thread::sleep_until(rqst_start + head_move_duration);
		}
		else
			std::this_thread::sleep_for(std::chrono::duration<int>(head_goal.min_duration.sec));
	}
	//
	{
		boost::unique_lock<boost::shared_mutex> lockAcquisitionStatus(mutexAcquisitionStatus_);
    acquisition_status_ = AcquisitionStatus::END;
	}
}

/*	-- SCIROC_OBJDET ACTION SERVER -- */
template <typename T>
void SciRocDarknetBridge<T>::goalCB()
{
	// start moving the head
	// start the clock
	ROS_INFO("[%s]: new goal received.", as_name_.c_str());
	// Pure virtual goal function, which can be used to store data received in the goal field
	action_.action_goal.goal = *(as_->acceptNewGoal());
	saveGoalDataImp();

	if (as_->isPreemptRequested())
	{
		preemptCB();
	}

	{
		boost::unique_lock<boost::shared_mutex> lockImageDetectedId(mutexImageDetectedId_);
		image_detected_id_ = 0; // reset the id of the last received image
	}
	image_sent_id_ = 0; // reset the id of the last sent image
	clock_lost_callbacks = 0;

	detectedBoxes.clear(); // remove previous detections

	if (move_head_thread.joinable())
		move_head_thread.join();

	if(display_thread.joinable())
		display_thread.join();
	// TODO: make head reset to standard position
	{
		boost::unique_lock<boost::shared_mutex> lockAcquisitionStatus(mutexAcquisitionStatus_);
    acquisition_status_ = AcquisitionStatus::START;
	}
	move_head_thread = std::thread(&SciRocDarknetBridge<T>::moveHead, this, retrieveTablePos());

	as_clock.start();
	ROS_DEBUG("[%s]: clock started.", as_name_.c_str());
}
template <typename T>
void SciRocDarknetBridge<T>::preemptCB()
{
	as_clock.stop();
	{
		boost::unique_lock<boost::shared_mutex> lockAcquisitionStatus(mutexAcquisitionStatus_);
    acquisition_status_ = AcquisitionStatus::NONE;
	}
	ROS_INFO("[%s] preemption request received.", as_name_.c_str());

	ac_->cancelAllGoals();
	while (!(ac_->getState().isDone()))
  {
    ros::Duration(0.1).sleep();
  }
  as_->setPreempted();

	detectedBoxes.clear();
}

/*	-- DARKNET_ROS ACTION CLIENT -- */
template <typename T>
void SciRocDarknetBridge<T>::sendGoal()
{
	darknet_ros_msgs::CheckForObjectsGoal goal;
	{
		boost::shared_lock<boost::shared_mutex> lockCurrentImage(mutexCurrentImage_);
		goal.image = current_img_; // retrieve last image perceived
		last_img_ = current_img_;
	}
	goal.id = image_sent_id_;

	ac_->sendGoal(goal, boost::bind(&SciRocDarknetBridge<T>::yoloDoneCB, this, _1, _2),
								ACType::SimpleActiveCallback(), ACType::SimpleFeedbackCallback());
}

template <typename T>
void SciRocDarknetBridge<T>::yoloDoneCB(const actionlib::SimpleClientGoalState &state, const darknet_ros_msgs::CheckForObjectsResultConstPtr &result)
{
	if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		{
			boost::unique_lock<boost::shared_mutex> lockImageDetectedId(mutexImageDetectedId_);
			image_detected_id_ = result->id; // update the id of the last received image
		}
		// TODO: should I move this at the end? Does it slow down the callback?
		std::vector<darknet_ros_msgs::BoundingBox> th_boxes;
		for (auto box : result->bounding_boxes.bounding_boxes)
		{
			if (box.probability > det_threshold_)
				th_boxes.push_back(box);
		}
		detectedBoxes.push_back(th_boxes);
	}
	else
		ROS_DEBUG("[%s]: yolo detection returned failed state %s", as_name_.c_str(), state.getText().c_str());
}

template <typename T>
void SciRocDarknetBridge<T>::clockCB(const ros::TimerEvent&)
{
	AcquisitionStatus tempAS;
	{
		boost::shared_lock<boost::shared_mutex> lockAcquisitionStatus(mutexAcquisitionStatus_);
    tempAS = acquisition_status_;
	}
	switch (tempAS)
	{
	case AcquisitionStatus::NONE:
		ROS_WARN("Clock callback called while no acquisition is running.\n"
							"The clock might be running longer than it should.");
		break;
	case AcquisitionStatus::START:
		ROS_DEBUG("Clock callback before head movement started");
		break;
	case AcquisitionStatus::ONGOING:
		/*	check if the previous has returned*/
		int tmp_last_detected_image_id;
		{
			boost::shared_lock<boost::shared_mutex> lockImageDetectedId(mutexImageDetectedId_);
			tmp_last_detected_image_id = image_detected_id_; // retrieve id of the last image received
		}
		if (tmp_last_detected_image_id < image_sent_id_)
		{
			ROS_DEBUG("[%s]: Attempting to send an image while the previous one hasn't come back\n"
								"#Lost callbacks:\t%d\n"
								"Clock rate might be too fast.", as_name_.c_str(), clock_lost_callbacks);
			clock_lost_callbacks++;
			return;
		}

		image_sent_id_++;
		sendGoal();

		// YES -> get new image and send it
		// NO -> skip this image (keep a counter on the number of failed calls for debugging)
		break;
	case AcquisitionStatus::END:
		/*	call the response CB	*/
		ROS_DEBUG("[%s]: head movement ended, calling the resultCB", as_name_.c_str());
		resultCB();
		break;
	default:
		ROS_WARN("Unexpected case inside the clockCB.");
		break;
	}
	return;
}

template <typename T>
void SciRocDarknetBridge<T>::resultCB()
{
	// Stop the callback clock
	as_clock.stop();
	ROS_DEBUG("[%s]: image gathering completed.", as_name_.c_str());
	// join the head moving thread
	if(display_thread.joinable())
		display_thread.join();

	if(move_head_thread.joinable())
		move_head_thread.join();
	else
		ROS_WARN("[%s]: move_head_thread not joinable when it should.", as_name_.c_str());
	{
		boost::unique_lock<boost::shared_mutex> lockAcquisitionStatus(mutexAcquisitionStatus_);
    acquisition_status_ = AcquisitionStatus::NONE;
	}
	/* 	in this function, to be implemented in the children,
			fill a result var appropriately and publish it.
	*/
	setResultImp(); // to be implemented in the children classes
	as_->setSucceeded(action_.action_result.result);

	if (!display_image_)
		return;

	display_thread = std::thread(&SciRocDarknetBridge<T>::displayLastDetection, this);
	
	

}

template <typename T>
void SciRocDarknetBridge<T>::displayLastDetection()
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(last_img_, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	int i = 0;
	for (auto box : detectedBoxes.back())
	{
		auto color = colors_[++i % colors_.size()];
		cv::rectangle(cv_ptr->image,
									cv::Point(box.xmin, box.ymin), cv::Point(box.xmax, box.ymax),
									color,
									2, cv::LINE_8);
		cv::putText(cv_ptr->image, box.Class, cv::Point(box.xmin, box.ymin), 3, 1, color, 2);
	}

	cv::imshow("YOLOv3", cv_ptr->image);
	cv::waitKey(3000);
}

// Reference to correctly link the cpp

template class SciRocDarknetBridge<sciroc_objdet::ObjectEnumerationAction>;
template void SciRocDarknetBridge<sciroc_objdet::ObjectEnumerationAction>::waitForServer(std::string server_name, ACTypePtr action_client);
template void SciRocDarknetBridge<sciroc_objdet::ObjectEnumerationAction>::waitForServer(std::string server_name, HeadACTypePtr action_client);

template class SciRocDarknetBridge<sciroc_objdet::ObjectClassificationAction>;
template void SciRocDarknetBridge<sciroc_objdet::ObjectClassificationAction>::waitForServer(std::string server_name, ACTypePtr action_client);
template void SciRocDarknetBridge<sciroc_objdet::ObjectClassificationAction>::waitForServer(std::string server_name, HeadACTypePtr action_client);

template class SciRocDarknetBridge<sciroc_objdet::ObjectComparisonAction>;
template void SciRocDarknetBridge<sciroc_objdet::ObjectComparisonAction>::waitForServer(std::string server_name, ACTypePtr action_client);
template void SciRocDarknetBridge<sciroc_objdet::ObjectComparisonAction>::waitForServer(std::string server_name, HeadACTypePtr action_client);

} // namespace