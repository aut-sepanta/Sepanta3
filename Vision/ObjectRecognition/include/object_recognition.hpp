#ifndef _OBJECT_RECOGNITION_HPP
#define _OBJECT_RECOGNITION_HPP

#include <ros/ros.h>

#include <sepanta_msgs/Objects.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <object_pipeline.hpp>

class ObjectRecognition {
public:
    ObjectRecognition(ros::NodeHandle node_handle);

    void rgbdImageCallback(const sensor_msgs::ImageConstPtr& input_image, const sensor_msgs::PointCloud2ConstPtr& input_cloud);
    void pipeline(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    void loadModels(boost::shared_ptr<std::vector<Object>> objects);
    void publish(boost::shared_ptr<std::vector<Object>> objects);
private:
    boost::shared_ptr<ObjectPipeline> object_pipeline;
    ros::Publisher objects_publisher;
};

#endif
