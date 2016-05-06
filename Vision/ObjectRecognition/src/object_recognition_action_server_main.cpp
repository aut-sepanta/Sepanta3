#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <object_recognition_action_server.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_recognition_action");
    ros::NodeHandle node_handle;

    ObjectRecognitionAction object_recognition(node_handle, "look_for_objects");
    
    ros::spin();

    return 0;
}
