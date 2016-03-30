#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

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

    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_subscriber(node_handle, "/camera/depth_registered/points", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_image_subscriber(node_handle, "/camera/rgb/image_color", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> RgbdImagePolicy;

    message_filters::Synchronizer<RgbdImagePolicy> rgbd_image_synchronizer(RgbdImagePolicy(10), rgb_image_subscriber, point_cloud_subscriber);
    ObjectRecognition object_recognition(node_handle);
    rgbd_image_synchronizer.registerCallback(boost::bind(&ObjectRecognition::rgbdImageCallback, object_recognition, _1, _2));
    
    ros::spin();

    return 0;
}
