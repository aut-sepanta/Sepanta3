#ifndef _SEPANTA_MOVE_BASE_H
#define _SEPANTA_MOVE_BASE_H

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
#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PolygonStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <dynamixel_controllers/SetComplianceSlope.h>
#include <dynamixel_controllers/SetCompliancePunch.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include <termios.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <sepanta_msgs/command.h>
#include <sepanta_msgs/omnidata.h>
#include <sepanta_msgs/sepantaAction.h> //movex movey turngl turngllocal actions
#include <sepanta_msgs/slamactionAction.h> //slam action
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>
#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tbb/atomic.h>
#include <sepanta_msgs/Person.h>
#include <sepanta_msgs/PersonArray.h>
#include <sepanta_msgs/Leg.h>
#include <sepanta_msgs/LegArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sepanta_msgs/MasterAction.h>
#include <sepanta_msgs/led.h>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;
using namespace ros;

struct person
{
	public:
		int ID;
		geometry_msgs::Pose pose;
};

inline double Deg2Rad(double deg)
{
    return deg * M_PI / 180;
}

inline double Rad2Deg(double rad)
{
    return rad * 180 / M_PI;
}

class SepantaFollowEngine
{
public:
SepantaFollowEngine();
~SepantaFollowEngine();

double Quat2Rad(double orientation[]);
double Quat2Rad2(tf::Quaternion q);
void change_led(int r,int g,int b);
void say_message(string data);
void send_omni(double x,double y ,double w);
void force_stop();
double GetDistance(double x1, double y1, double x2, double y2);
void GetPos(const geometry_msgs::PoseStamped::ConstPtr &msg);
void chatterCallback_laser(const sensor_msgs::LaserScan::ConstPtr &msg);
void chatterCallback_persons(const sepanta_msgs::PersonArray::ConstPtr &msg);
void logic_thread();
bool isidexist(int id);
void scan10hz_thread();
void action_thread();
void init();
void kill();
bool find_user_for_follow();

person target_person;

ros::NodeHandle node_handles[20];
ros::Subscriber sub_handles[5];
boost::thread _thread_Logic;
boost::thread _thread_10hz_publisher;
boost::thread _thread_logic_action;

bool App_exit;
bool say_enable;
bool isttsready;
ros::Publisher led_pub;
ros::Publisher pub_tts;
ros::Publisher marker_pub;
ros::Publisher scan10hz_pub;
ros::Publisher mycmd_vel_pub;
ros::ServiceClient say_service;
int sayMessageId;
double Position[2];
double orientation[4];
double Tetha;
bool scan10hz_can_send;
std::vector<person> list_persons;

};

#endif