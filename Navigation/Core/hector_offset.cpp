#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <athomerobot_msgs/slamactionAction.h>

#include "athomerobot_msgs/maptools.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include <iostream>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <pcl/common/time.h>
#include <fstream>
#include "std_msgs/Bool.h"

#include <tf/transform_broadcaster.h>
/////////////TODO////////////////////
// #include <athomerobot_msgs/gesture_detectAction.h>
// #include <athomerobot_msgs/face_detectAction.h>
#include <athomerobot_msgs/grip_partyAction.h>

#include <athomerobot_msgs/sepantaAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/package.h>
#include "athomerobot_msgs/arm.h"
#include "athomerobot_msgs/head.h"

#include "ros/ros.h"
#include <ros/package.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <athomerobot_msgs/arm.h>
#include <athomerobot_msgs/omnidata.h>
#include <athomerobot_msgs/head.h>
#include <athomerobot_msgs/irsensor.h>
#include <athomerobot_msgs/motortorques.h>
#include <std_msgs/Int32.h>


#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <athomerobot_msgs/slamactionAction.h>


#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "std_msgs/String.h"

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "athomerobot_msgs/stop.h" //facestop and manualauto service
#include "athomerobot_msgs/command.h" //command service
#include "athomerobot_msgs/maptools.h" //command service

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
//#include "emergency/position.h"
using namespace std;

#define FD_TIME 5 //TODO
#define GD_TIME 50 //TODO 
#define GF_TIME 10000 //TODO 

typedef actionlib::SimpleActionClient<athomerobot_msgs::slamactionAction> SLAMClient;
typedef actionlib::SimpleActionClient<athomerobot_msgs::grip_partyAction> GFClient;//TODO
typedef athomerobot_msgs::head head_msg;
typedef std_msgs::Int32 int_msg;


SLAMClient *globalSLAM;
GFClient *globalGF;//TODO Bayad avaz she tebghe object
int id_global;
head_msg my_head_msg;
int_msg msg_z;

ros::ServiceClient client;
ros::Publisher chatter_pub2;



inline float Deg2Rad(float deg)
{
    return deg * M_PI / 180;
}

inline float Rad2Deg(float rad)
{
    return rad * 180 / M_PI;
}


void Origin_update()
{
    float ox = (float)(-20) / 100;
    float oy = (float)(880) / 100;
    float oyaw = 0;
    oyaw = Deg2Rad(oyaw);

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(oyaw);
    msg.pose.pose.position.x = ox;
    msg.pose.pose.position.y = oy;

    chatter_pub2.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inspection");

    ros::NodeHandle n2;

    chatter_pub2 = n2.advertise<geometry_msgs::PoseWithCovarianceStamped>("/slam_origin", 1);
   
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    Origin_update();

}


