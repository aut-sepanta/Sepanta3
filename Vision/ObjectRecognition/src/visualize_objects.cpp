#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sepanta_msgs/Object.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/thread/mutex.hpp>

boost::mutex mutex;
sepanta_msgs::ObjectConstPtr object;
bool first_run = true;

void objectCallback(const sepanta_msgs::ObjectConstPtr& obj) {
    mutex.lock();
    object = obj;
    mutex.unlock();
    first_run=false;
}

void rgbImageCallback(const sensor_msgs::ImageConstPtr& input_image) {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::BGR8);

    geometry_msgs::Point position;
    cv::Scalar color;
    if (!first_run) {
        mutex.lock();
        position = object->center_2d;

        if (object->status) {
           color = CV_RGB(255, 0, 0);
        } else {
            color = CV_RGB(0, 255, 0);
        }

        ROS_INFO_STREAM(cv::Point(int(position.x), int(position.y)));
        cv::circle(cv_ptr->image, cv::Point(int(position.x), int(position.y)), 3, color);
        cv::putText(cv_ptr->image, object->label, cv::Point(int(position.x+10), int(position.y+10)), cv::FONT_HERSHEY_PLAIN, 3, color);
        cv::rectangle(cv_ptr->image, cv::Point(int(position.x)-40, int(position.y)-40), cv::Point(int(position.x)+30, int(position.y)+90), color, 3);
        mutex.unlock();
    }
    cv::imshow("Objects Visualizer", cv_ptr->image);
    cv::waitKey(3);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualize_objects");
    ros::NodeHandle node_handle;

    ros::Subscriber rgb_subscriber = node_handle.subscribe("/camera/rgb/image_color", 1, rgbImageCallback);
    ros::Subscriber objs_subscriber = node_handle.subscribe("/behavior/current_object", 1, objectCallback);

    ros::spin();

    return 0;
}
