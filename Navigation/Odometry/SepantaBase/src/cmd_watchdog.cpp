#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <stdio.h>
#include <stdlib.h>
#include <tbb/atomic.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sepanta_msgs/arm.h"
#include "sepanta_msgs/omnidata.h"
#include "sepanta_msgs/head.h"
#include "sepanta_msgs/irsensor.h"
#include "sepanta_msgs/motortorques.h"

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <actionlib/server/simple_action_server.h>
#include <sepanta_msgs/sepantaAction.h>

#include "math.h"
using namespace std;
using namespace boost;

nav_msgs::OccupancyGrid static_map;

bool first_init = false;
Eigen::MatrixXf IK_matrix;

float Odometry_raw[3] = {0};

float odom_position_yaw[3] = {0};
float slam_position_yaw[3] = {0};
float odom_speed_xyw[3] = {0};

int speed_Motor[4] = {128,128,128,128};

bool App_exit = false;
int Odometry_speed = 50;

float ratio_X = 1000;
float ratio_Y = 1041;
float ratio_W = 860;

float kp_degree = 3.5;
float ki_degree = 0.03;
float kp_move = 8.5;

float L = 50; //cm
bool isrobotmove = false;
ros::Publisher chatter_pub[20];

float laser_IR[8];
float IR[8];

float pid_i_buffer[20] = {0};

typedef actionlib::SimpleActionServer<sepanta_msgs::sepantaAction> slam_Server;
slam_Server *globalServer;

ros::Publisher mycmd_vel_pub;

 int counter = 0;

void chatterCallback_cmd_vel(const geometry_msgs::Twist &twist_aux)
{
    counter = 0;
}

void force_stop()
{
    geometry_msgs::Twist myTwist;

       
            myTwist.linear.x = 0;
            myTwist.linear.y = 0;
            myTwist.angular.z = 0; 
        
        mycmd_vel_pub.publish(myTwist); 

}


 


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sepanta_watchdog");
  cout << "SEPANTA III Navigation Watchdog started" << endl;

  ros::Time::init();

  
  ros::Rate ros_rate(20);

  ros::NodeHandle node_handles[15];
  ros::Subscriber sub_handles[15];

  sub_handles[1] = node_handles[2].subscribe("sepantamovebase/cmd_vel", 10, chatterCallback_cmd_vel);
  mycmd_vel_pub = node_handles[4].advertise<geometry_msgs::Twist>("sepantamovebase/cmd_vel", 10);

 

  while(ros::ok())
  {
    counter++;
    ros::spinOnce();
    ros_rate.sleep();


    if ( counter > 20 )
    {
        //1s
        counter = 0;
        force_stop();
        
    }
  }

 


  return 0;
} 
