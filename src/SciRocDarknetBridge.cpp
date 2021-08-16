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

/*	-- SCIROC_OBJDET ACTION SERVER -- */
template <typename T>
void SciRocDarknetBridge<T>::goalCB()
{}
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