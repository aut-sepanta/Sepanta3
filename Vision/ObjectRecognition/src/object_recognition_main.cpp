#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <object_recognition.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "train_objects");
    ros::NodeHandle node_handle;

    ObjectRecognition object_recognition(node_handle);
    
    ros::spin();

    return 0;
}
