# ROS Node: OpenCV Camera

**LibViso2 Author:** Andreas Geiger

**ROS Author:** Kevin Walchko

**License:** GPL Ver. 2

**Language:** C++

**Website:** www.cvlibs.net

Implemented a ROS node which uses libviso2 to do visual odometery. There are also two demo programs that read data sets (you can get them from www.cvlibs.net) and calculate the pose.  

## Command Line

	rosrun libviso2 mono_node "topic_name" _debug:=true/false 

	rosrun libviso2 stereo_node "topic_name" _debug:=true/false 
 
* debug: true or false, opens a window to show captured image
* topic: camera topic 

### Published Topics: 
**Pose:** "/viso_node/pose"
 
### Example:
 	rosrun libviso2 mono_node "/topic/name"

## To Do

* Enable the debug option from command line
* Setup stereo node