#ifndef _OBJECT_RECOGNITION_ACTION_HPP
#define _OBJECT_RECOGNITION_ACTION_HPP

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sepanta_msgs/LookForObjectsAction.h>

#include <actionlib/server/simple_action_server.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <object_pipeline.hpp>
#include <object_recognition.hpp>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> RgbdImagePolicy;

class ObjectRecognitionAction: public ObjectRecognition {
public:
    ObjectRecognitionAction(ros::NodeHandle nh, std::string name);
    void rgbdImageCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud, const sensor_msgs::CameraInfoConstPtr& camera_info);
    void goalCallback();
    void preemptCallback();
protected:
    actionlib::SimpleActionServer<sepanta_msgs::LookForObjectsAction> action_server_;
    std::string action_name_;
    sepanta_msgs::LookForObjectsFeedback feedback_;
    sepanta_msgs::LookForObjectsResult result_;
    ros::NodeHandle node_handle;
};

#endif
