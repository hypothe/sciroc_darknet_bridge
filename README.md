# sciroc_darknet_bridge
Interface between the ObjDet SciRoc ROS package and the darknet_ros ROS package.

## Roadmap

- Aug, 19: 
	- [x] Complete the general SciRocDarknetBridge class definition
- Aug, 20:
	- [x] Write the config file with values used by the class
	- [x] Script a simple component to periodically publish an image as if it were a video stream
	- [x] Write the remaining two children/implementation of the generic bridge Action Server
- Aug, 22:
	- [x] Run tests on the whole infrastructure
	- [x] Fix emerged issues in the bridge
- Aug, 24:
	- [x] Test detection on a pre-recorded video
	- [ ] Select images for the custom yolo database
	- [ ] Start labeling those images
- Aug, 25:
	- [x] Add an **AVG** and **MODE** aggregation modes to both Classification and Comparison, keeping track of the objects cardinality in the scene (aka do not simply consider if an object was detected or not, but insert as many copies of its tag as the AVG/MODE of detected instances)
