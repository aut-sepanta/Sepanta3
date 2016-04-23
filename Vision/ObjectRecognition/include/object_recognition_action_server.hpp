#ifndef _OBJECT_RECOGNITION_ACTION_HPP
#define _OBJECT_RECOGNITION_ACTION_HPP

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <learning_actionlib/AveragingAction.h>
#include <object_recognition/object_recognition.hpp>

class ObjectRecognitionAction: public ObjectRecognition {
public:
    ObjectRecognitionAction(ros::NodeHandle nh, std::string name);
    void goalCallback();
    void preemptCallback();
protected:
    ros::NodeHandle node_handle_;
    actionlib::SimpleActionServer<sepanta_msgs::FindObjectsAction> action_server_;
    ros::Publisher objects_publisher_;
    std::string action_name_;
    sepanta_msgs::FindObjectsFeedback feedback_;
    sepanta_msgs::FindObjectsResult result_;
};

#endif
