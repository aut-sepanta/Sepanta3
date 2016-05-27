#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <ctime>
#include <unistd.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <tbb/atomic.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "sepanta_msgs/omnidata.h"

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>

//=============================================================

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;
using namespace ros;

bool App_exit = false;
ros::ServiceClient client_map_save;

ros::Publisher mycmd_vel_pub;

double xSpeed=0;
double ySpeed=0;
double tethaSpeed=0;

double joyAxes[6] = {0};
int joyButtons[12] = {0};


inline double Deg2Rad(double deg)
{
    return deg * M_PI / 180;
}

inline double Rad2Deg(double rad)
{
    return rad * 180 / M_PI;
}

double Quat2Rad(double orientation[])
{
    tf::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void send_omni(double x,double y ,double w)
{
        geometry_msgs::Twist myTwist;
        
        myTwist.linear.x = x;
        myTwist.linear.y = -y;
        myTwist.angular.z = -w;
       
        mycmd_vel_pub.publish(myTwist); 

}


void force_stop()
{
    send_omni(0,0,0);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}



void GetRemote(const sensor_msgs::Joy::ConstPtr &msg)
{
    for(int i=0;i<6;i++)
    {
        joyAxes[i] = msg->axes[i];
    }
    for(int i=0;i<12;i++)
    {
        joyButtons[i] = msg->buttons[i];
    }
    xSpeed = joyAxes[1]*0.3;
    ySpeed = joyAxes[0]*0.3;
    tethaSpeed = joyAxes[2]*0.3;
    send_omni(xSpeed,ySpeed,tethaSpeed);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "remotecontrol");

    ros::NodeHandle node_handles[2];
    ros::Subscriber sub_handles;

    //============================================================================================
    sub_handles = node_handles[0].subscribe("/joy", 10, GetRemote);
    //============================================================================================
    mycmd_vel_pub = node_handles[1].advertise<geometry_msgs::Twist>("sepantamovebase/cmd_vel", 10);
    //============================================================================================

    ros::Rate loop_rate(20);

    while (ros::ok() && App_exit == false)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}