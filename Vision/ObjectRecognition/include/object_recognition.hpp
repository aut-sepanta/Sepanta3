#ifndef _OBJECT_RECOGNITION_HPP
#define _OBJECT_RECOGNITION_HPP

#include <ros/ros.h>

#include <sepanta_msgs/Objects.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <object_pipeline.hpp>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> RgbdImagePolicy;

class ObjectRecognition {
public:
    ObjectRecognition(ros::NodeHandle node_handle);

    void rgbdImageCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud, const sensor_msgs::CameraInfoConstPtr& camera_info);
    void pipeline(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    void loadModels(boost::shared_ptr<std::vector<Object>> objects);
    void publish(boost::shared_ptr<std::vector<Object>> objects);
    bool turnOff(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    bool turnOn(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
protected:
    boost::shared_ptr<ObjectPipeline> object_pipeline;
    ros::Publisher objects_publisher;
    ros::ServiceServer turnOffService;
    ros::ServiceServer turnOnService;
    image_geometry::PinholeCameraModel camera_model;
    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_subscriber;
    message_filters::Subscriber<sensor_msgs::Image> rgb_image_subscriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_subscriber;
    message_filters::Synchronizer<RgbdImagePolicy> rgbd_image_synchronizer;
    ros::NodeHandle node_handle;
};

#endif
