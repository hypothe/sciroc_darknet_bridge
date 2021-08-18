#include "sciroc_darknet_bridge/SciRocDarknetBridge.hpp"

using namespace std::chrono_literals;

namespace sciroc_darknet_bridge
{
template <typename T>
SciRocDarknetBridge<T>::SciRocDarknetBridge(ros::NodeHandle nh_, std::string action_server_name)
: node_handle_(nh_),
	as_name_(action_server_name),
	as_(std::make_shared<ASType>(nh_, action_server_name, false)),
	image_sent_id_(0), image_detected_id_(0)
{
	// Clock
	as_clock = node_handle_.createTimer(ros::Duration(as_clock_period), &SciRocDarknetBridge<T>::clockCB, this, false, false);
	// ActionClient
	std::string checkForObjectsActionName;
  node_handle_.param("actions/camera_reading/topic", checkForObjectsActionName, std::string("/darknet_ros_as/check_for_objects"));
	ac_ = std::make_shared<ACType>(checkForObjectsActionName, true);
	// ActionServer
	as_->registerGoalCallback(boost::bind(&SciRocDarknetBridge<T>::goalCB, this));
	as_->registerPreemptCallback(boost::bind(&SciRocDarknetBridge<T>::preemptCB, this);
	// Subscriber
	std::string cameraTopicName;
	int cameraQueueSize;
	node_handle_.param("subscribers/camera_reading/topic", cameraTopicName, std::string("/camera/image_raw"));
	node_handle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);
	it = std::make_shared<image_transport::ImageTransport>(node_handle_);
	// Start the components
	camera_sub_ = std::make_shared<image_transport::Subscriber>(it->subscribe(cameraTopicName, cameraQueueSize,
																																						&SciRocDarknetBridge<T>::cameraCallback, this));
	waitForServer(checkForObjectsActionName);
	as_->start();
}

template <typename T>
SciRocDarknetBridge<T>::~SciRocDarknetBridge()
{
	// ActoinServer
	as_->setAborted();
	as_->shutdown();
	// ActionClient
	ac_->stopTrackingGoal();
	// Clock
	as_clock->stop();
	// Thread
	if (move_head_thread.joinable())
		move_head_thread.join();
}

/*	-- METHODS -- */
template <typename T>
void SciRocDarknetBridge<T>::waitForServer(std::string server_name)
{
	int failedConnectionAttempt = 0;
	while (!ac_->waitForServer(ros::Duration(1s)) && failedConnectionAttempt)
	{
		ROS_WARN("Waiting for Action Server %s", server_name);
		failedConnectionAttempt++;
	}
	if (failedConnectionAttempt > maxFailedConnectionAttempts)
	{
		ROS_ERROR("NO SERVER %s FOUND AFTER %d attempts, shutting down.", \
							server_name.c_Str(), maxFailedConnectionAttempts);
		node_handle_.shutdown();
	}
	return ;
}

// TODO: does it make sense to subscribe/unsubscribe at runtime when the action is called?
template <typename T>
void SciRocDarknetBridge<T>::cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lockCurrentImage(mutexCurrentImage_);
	current_img_ = msg.get(); // update the last stored frame
}

template <typename T>
geometry_msgs::Point SciRocDarknetBridge<T>::retrieveTablePos()
{
	/*	Temporary definition!	*/
	// TODO: actually retrieve the info from the server
	std::vector<float> table_point {1.0, 0.0, 0.8};
}

template <typename T>
void SciRocDarknetBridge<T>::moveHead(geometry_msgs::Point table_pos) // in thread
{
	/*	this will be runned in a thread	*/
	/*	Temporary definition!	*/
	// ask to an action server to move the downward then upward
	{
		boost::unique_lock<boost::shared_mutex> lockAcquisitionStatus(mutexAcquisitionStatus_);
    acquisition_status_ = AcquisitionStatus::ONGOING;
	}

	geometry_msgs::Point look_up = table_pos;
	look_up.z += 0.1;
	geometry_msgs::Point look_down = table_pos;
	look_down.z -= 0.1;
	std::vector<geometry_msgs::Point> look_points {look_down, table_pos, look_up, table_pos};
	for (auto point : look_points)
	{
		// TODO: actually call the server
		std::this_thread::sleep_for(1.5s);
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
	ROS_INFO("[%s]: new goal received.", as_name_.c_Str());
	// Pure virtual goal function, which can be used to store data received in the goal field
	saveGoalData();

	{
		boost::unique_lock<boost::shared_mutex> lockImageDetectedId(mutexImageDetectedId_);
		image_detected_id_ = 0; // reset the id of the last received image
	}
	image_sent_id_ = 0; // reset the id of the last sent image
	clock_lost_callbacks = 0;

	detectedBoxes.clear(); // remove previous detections

	if (move_head_thread.joinable())
		move_head_thread.join();
	// TODO: make head reset to standard position
	{
		boost::unique_lock<boost::shared_mutex> lockAcquisitionStatus(mutexAcquisitionStatus_);
    acquisition_status_ = AcquisitionStatus::START;
	}
	move_head_thread = std::thread(&SciRocDarknetBridge<T>::moveHead, retrieveTablePos(), this);

	as_clock.start();
}
template <typename T>
void SciRocDarknetBridge<T>::preemptCB()
{
	as_clock.stop();
	ROS_DEBUG("[%s] preemption request received.", as_name_.c_str());

	ac_->cancelAllGoals();
	while (!(ac_->getState().isDone()))
  {
    ros::Duration(0.1).sleep();
  }
  as_.setPreempted();

	detectedBoxes.clear();
}

/*	-- DARKNET_ROS ACTION CLIENT -- */
template <typename T>
void SciRocDarknetBridge<T>::sendGoal()
{
	darknet_ros_msgs::CheckForObjectsActionGoal goal;
	{
		boost::shared_lock<boost::shared_mutex> lockCurrentImage(mutexCurrentImage_);
		goal.goal.image = current_img_; // retrieve last image perceived
	}
	goal.goal.id = curent_id_;

	ac_->sendGoal(goal, boost::bind(&SciRocDarknetBridge<T>::yoloDoneCB, std::placeholders::_1, std::placeholders::_2, this),
								ACType::SimpleActiveCallback(), ACType::SimpleFeedbackCallback());
}

template <typename T>
void SciRocDarknetBridge<T>::yoloDoneCB(const actionlib::SimpleClientGoalState &state, const darknet_ros_msgs::CheckForObjectsResultConstPtr &result)
{
	{
		boost::unique_lock<boost::shared_mutex> lockImageDetectedId(mutexImageDetectedId_);
		image_detected_id_ = result->id; // update the id of the last received image
	}
	detectedBoxes.push_back(result->bounding_boxes);
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
	case Acquisition::START:
		ROS_DEBUG("Clock callback before head movement started");
		break;
	case Acquisition::ONGOING:
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
	case Acquisition::END:
		/*	call the response CB	*/
		resultCB();
		break;
	default:
		ROS_WARN("Unexpected case inside the clockCB.");
		break;
	}
	return;
}

template <typename T>
void SciRocDarknetBridge<T>:: resultCB()
{
	// Stop the callback clock
	as_clock.stop();
	// join the head moving thread
	if(move_head_thread.joinable())
		move_head_thread.join();
	else
		ROS_WARN("[%s]: move_head_thread not joinable when it should.", as_name_.c_str());
	{
		boost::unique_lock<boost::shared_mutex> lockAcquisitionStatus(mutexAcquisitionStatus_);
    acquisition_status_ = AcquisitionStatus::NONE;
	}
	fillResult(); // to be implemented in the children classes
	//as_->setSucceeded(result_);
	// TODO: how to correctly declare the result
}

// Reference to correctly link the cpp
template class SciRocDarknetBridge<sciroc_objdet::ObjectEnumerationAction>;
template class SciRocDarknetBridge<sciroc_objdet::ObjectClassificationAction>;
template class SciRocDarknetBridge<sciroc_objdet::ObjectComparisonAction>;

} // namespace