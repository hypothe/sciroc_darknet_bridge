#include "sciroc_darknet_bridge/SciRocDarknetBridge.hpp"

namespace sciroc_darknet_bridge
{
template <typename T>
SciRocDarknetBridge<T>::SciRocDarknetBridge(ros::NodeHandle nh_, std::string action_server_name)
: node_handle_(nh_), as_(std::make_shared<ASType>(nh_, action_server_name, false))
{
	as_clock = node_handle_.createTimer(ros::Duration(as_clock_period), &SciRocDarknetBridge<T>::clockCB, this, false, false);
}
template <typename T>
SciRocDarknetBridge<T>::~SciRocDarknetBridge()
{}

/*	-- METHODS -- */
template <typename T>
void SciRocDarknetBridge<T>::waitForServer()
{}
template <typename T>
void SciRocDarknetBridge<T>::cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{}

template <typename T>
std::vector<float> SciRocDarknetBridge<T>::retrieveTablePos()
{
	/*	Temporary definition!	*/
	std::vector<float> table_point {1.0, 0.0, 0.8};
}

template <typename T>
void SciRocDarknetBridge<T>::moveHead() // in thread
{
	/*	this will be runned in a thread	*/
	/*	Temporary definition!	*/
	// ask to an action server to move the downward then upward
	// thi
}

/*	-- SCIROC_OBJDET ACTION SERVER -- */
template <typename T>
void SciRocDarknetBridge<T>::goalCB()
{
	AcquisitionStatus tempAS;
	{
		boost::shared_lock<boost::shared_mutex> lockAcquisitionStatus(mutexAcquisitionStatus_);
    tempAS = acquisition_status_;
	}
	switch (tempAS)
	{
	case AcquisitionStatus::NONE:
		ROS_WARN("Clock callback called while no acquisition is running.\n" \
							"The clock might be running longer than it should.");
		break;
	case Acquisition::START:
		ROS_DEBUG("Clock callback before head movement started");
		break;
	case Acquisition::ONGOING:
		/*	check if the previous has returned*/
		// YES -> get new image and send it
		// NO -> skip this image (keep a counter on the number of failed calls for debugging)
		break;
	case Acquisition::END:
		/*	call the response CB	*/
		break;
	default:
		ROS_WARN("Unexpected case inside the clockCB.");
		break;
	}
	return;
}
template <typename T>
void SciRocDarknetBridge<T>::preemptCB()
{}

/*	-- DARKNET_ROS ACTION CLIENT -- */
template <typename T>
void SciRocDarknetBridge<T>::sendGoal()
{}
template <typename T>
void SciRocDarknetBridge<T>::clockCB(const ros::TimerEvent&)
{}

// Reference to correctly link the cpp
template class SciRocDarknetBridge<sciroc_objdet::ObjectEnumerationAction>;

} // namespace