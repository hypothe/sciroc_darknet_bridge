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
- Aug, 25:
	- [x] Add an **AVG** and **MODE** aggregation modes to both Classification and Comparison, keeping track of the objects cardinality in the scene (aka do not simply consider if an object was detected or not, but insert as many copies of its tag as the AVG/MODE of detected instances)
- Aug, 27:
	- [x] Add the possibility to display the boxes detected on the last sampled frame (only the last one to make thing easier), to use for debugging purposes
	- [x] Add also the tag labels near its box in the displayed image
- Aug, 28:
	- [x] Start labeling images
	- [x] Update original YOLOR COLAB to deal with YOLOv3 (darknet compatibl version)
- Aug, 29:
	- [x] Take images with morning line conditions
	- [ ] Do a preliminary training on the classes for which there are enough pics
	- [ ] Keep labeling images
- Aug, 30:
	- [ ] Visit Recchiuto lab, test the docker image on a Unix-NVIDIA machine
