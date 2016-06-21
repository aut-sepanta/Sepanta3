#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <tbb/atomic.h>
#include <signal.h>

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include "sepanta_msgs/arm.h"
#include "sepanta_msgs/omnidata.h"
#include "sepanta_msgs/head.h"
#include "sepanta_msgs/irsensor.h"
#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/JointState.h>
#include <dynamixel_controllers/SetComplianceSlope.h>
#include <dynamixel_controllers/SetCompliancePunch.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <iostream>


#define FILTER_QUEU 10
//=============================================================

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;

void Omnidrive(float vx, float vy, float w);

bool robot_init = false;
bool App_exit = false;
int control_mode = 0;
ros::Publisher chatter_pub[20];

float L = 50; //cm
int mobileplatform_motors_write[4] = {128, 128, 128, 128};
int mobileplatform_motors_read[4] = {128, 128, 128, 128};
tbb::atomic<int> keypad_status_raw;
tbb::atomic<int> keypad_status;
tbb::atomic<int> last_keypad_value;


float laser_IR[8];
int IR[5] = {0};
int EMS_STOP = 0;

ros::Publisher chatter_pub_motor[20];
tbb::atomic<int> Compass;

int key_pad_reset = 0;
int robot_max_speedx = 300;
int robot_max_speedy = 300;
int robot_max_speedw = 250;

//Status
int control_mode_old = 0;
int voltage_down = 0;
int voltage_up = 0;
bool green_light = false;
bool red_light = false;
bool btn_start = false;
bool btn_stop = false;



void chatterCallback_greenlight(const std_msgs::Bool::ConstPtr &msg)
{
    cout<<msg->data<<endl;
    green_light = msg->data;
}

void chatterCallback_redlight(const std_msgs::Bool::ConstPtr &msg)
{
    cout<<msg->data<<endl;
    red_light = msg->data;
}



float omnidrive_x = 0;
float omnidrive_y = 0;
float omnidrive_w = 0;

void chatterCallback_speech(const std_msgs::String::ConstPtr &msg)
{
   cout<<"Speech Get : "<<msg->data<<endl;

   std_msgs::String data;
   data.data = msg->data;
   //chatter_pub[0].publish(data);
   
}






int main(int argc, char **argv)
{
    ros::init(argc, argv, "speechrecognition");

    ros::NodeHandle node_handles[50];
   ros::Subscriber sub_handles[15];
    //===========================================================================================

    sub_handles[0] = node_handles[0].subscribe("/speechRec/cmd_spch", 10, chatterCallback_speech);
    chatter_pub[0] = node_handles[1].advertise<std_msgs::String>("/texttospeech/message", 10);

   
   
    //============================================================================================

    ros::Rate loop_rate(20);

    while (ros::ok() && App_exit == false)
    {
    
        ros::spinOnce();
        loop_rate.sleep();
    }

    App_exit = true;


    return 0;
}
