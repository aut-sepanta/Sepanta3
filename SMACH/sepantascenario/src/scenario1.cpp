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

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;
using namespace ros;

//=============================================================

std::string coutcolor0 = "\033[0;0m";
std::string coutcolor_red = "\033[0;31m";
std::string coutcolor_green = "\033[0;32m";
std::string coutcolor_blue = "\033[0;34m";
std::string coutcolor_magenta = "\033[0;35m";
std::string coutcolor_brown = "\033[0;33m";
bool App_exit = false;
ros::Publisher pub_tts;
ros::Publisher pub_spr;
bool say_enable = true;

int logic_state = 0;
bool isdooropened = false;
bool isrobotmove = false;
bool isspeechready = false;
bool object_started = false;
bool isobjectready = false;

string speech_last_command = "";
string temp_speech_last_command = "";
ros::ServiceClient client_navigation;
ros::ServiceClient client_startobject;
ros::ServiceClient client_stopobject;

void object_recognition_start()
{
    std_srvs::Empty _srv;
    client_startobject.call(_srv);
    object_started = true;
}

void object_recognition_stop()
{
    std_srvs::Empty _srv;
    client_stopobject.call(_srv);
    object_started = false;
}

void chatterCallback_object(const std_msgs::Bool::ConstPtr &msg)
{
   isobjectready = true;
}

void navigation_go_to(string location)
{
   if ( isrobotmove ) return;
   sepanta_msgs::command _msg;
   _msg.request.id = location;
   _msg.request.command = "exe";
   client_navigation.call(_msg);
}

void navigation_cancel()
{
   if ( isrobotmove == false ) return;
   sepanta_msgs::command _msg;
   _msg.request.command = "cancel";
   client_navigation.call(_msg);
}

void say_message(string data)
{
    if ( say_enable == false ) return;
    std_msgs::String _mes;
    _mes.data = data;
    pub_tts.publish(_mes);
}


void chatterCallback_door(const std_msgs::Bool::ConstPtr &msg)
{
   isdooropened = msg->data;
}

void chatterCallback_move(const std_msgs::Bool::ConstPtr &msg)
{
   isrobotmove = msg->data;
} 

void chatterCallback_speech(const std_msgs::String::ConstPtr &msg)
{
   cout<<coutcolor_brown<<"Speech Get : "<<msg->data<<coutcolor0<<endl;
   if ( temp_speech_last_command != msg->data )
   {
      temp_speech_last_command = msg->data;
      speech_last_command = temp_speech_last_command;
      isspeechready = true;
   }
  
}

void send_feedback_to_speech()
{
   std_msgs::String _msg;
   _msg.data = "start";
   pub_spr.publish(_msg);
}

void show_info()
{

   cout<<coutcolor_magenta<<"isdooropened : "<<isdooropened<<coutcolor0<<endl;
}

void logic_thread()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    say_message("sepanta demo scenario started");

    while(ros::ok() && !App_exit)
    {
         boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

         if ( logic_state == 0 )
         {
           cout<<coutcolor_green<<"idle"<<coutcolor0<<endl;
           isspeechready = false;
           isobjectready = false;
           show_info();
           logic_state = 1;
         }

         if ( logic_state == 1 )
         {
            //init state
            if ( isdooropened == false )
            {
                cout<<coutcolor_green<<"wait for door"<<coutcolor0<<endl;
            }
            else
            {
                cout<<coutcolor_green<<"The door is opened"<<coutcolor0<<endl;
                say_message("The door is opened!");
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                navigation_go_to("roomcenter");
            }
           
         }

         if ( logic_state == 2 )
         {
            //wait for navigation
            cout<<coutcolor_green<<"wait for navigation"<<coutcolor0<<endl;
         }



    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scenario1");
    ros::Time::init();

    ROS_INFO("Sepanta Scenario Started v 1.0");
    
    boost::thread _thread_Logic(&logic_thread);

    ros::NodeHandle node_handles[10];
    ros::Subscriber sub_handles[10];

    pub_tts = node_handles[0].advertise<std_msgs::String>("/texttospeech/message", 10);
    sub_handles[0] = node_handles[1].subscribe("lowerbodycore/isdooropened", 10, chatterCallback_door);
    sub_handles[1] = node_handles[2].subscribe("/speechRec/cmd_spch", 10, chatterCallback_speech);
    pub_spr = node_handles[3].advertise<std_msgs::String>("/speechRec/feedback_spch", 10);
    client_navigation = node_handles[4].serviceClient<sepanta_msgs::command>("sepantamovebase/command");
    sub_handles[2] = node_handles[5].subscribe("lowerbodycore/isrobotmove",10,chatterCallback_move);
    sub_handles[4] = node_handles[6].subscribe("objectrecognition/feedback",10,chatterCallback_object);

    ros::Rate loop_rate(20);

    while (ros::ok() && App_exit == false)
    {
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    _thread_Logic.interrupt();
    _thread_Logic.join();

    App_exit = true;
    return 0;
}

