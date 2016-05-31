#Yet another ros version of OpenTLD

OpenTLD is the C++ implementation of the TLD Predator

This is (yet another) ROS version of the OpenTLD tracker.

OpenTLD is a C++ implementation of TLD Predator (Tracking. Learning and Detection) implemented by the AIT (Austrian Institute of Technology) that was originally published in MATLAB by Zdenek Kalal. OpenTLD is used for tracking objects in video streams. It doesn't need any training data and is also able to load predefined models (http://gnebehay.github.com/OpenTLD/).

The ROS implementation here is just one node that subscribe to an image topic and publish a polygon which is actually only two points defining to opposite corners of the bounding box. 

## Keyboard shortcuts for the interface (same as OpenTLD)

* `c` stop/resume tracking
* `l` toggle learning
* `a` toggle alternating mode (if true, detector is switched off when tracker is available)
* `e` export model
* `i` import model
* `r` clear model

In the yaml file you can define some parameters use by the node : 
	Graphical_interface => Wether you need one or not to see what's going on.
	ShowTrajectory => Wether you ant to draw the trajectory of the followed point 
		(TODO make it a publisher)
	Trajectory_length => the name speak for itself.
	(TODO add possibility to change topic subscribed)
	(TODO add the possibility to load a model)
	
For now this system is actually used under groovy

Like OpenTLD, ROS_OpenTLD is published under the terms of the GNU General Public License.

A documentation of the internals as well as other possibly helpful information is contained in this [master thesis](https://github.com/downloads/gnebehay/OpenTLD/gnebehay_thesis_msc.pdf).
