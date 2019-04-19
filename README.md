# Follow-me-Drone
An intelligent follow-me drone based on visual inferencing and ROS. This has been tested on Gazebo in conjunction with [SITL](http://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html). This is primarily made for GSoC 2019 but improvements will be done irrespective of GSoC's outcome.


### Things to do

* Evaluate all possible visual tracking algorithms
* Introduce a controller
* Introduce Kalman Filter to make tracking more robust
* Use multiple bots in Gazebo simulator for virtual verification

#### Notes
* Current implementation is based on in built OpenCV trackers
* All implementation is done in ```visual_follow.py``` file
* Python version: 2.7.15
* OpenCV version: 4.0.1
* ROS distro: Kinetic
