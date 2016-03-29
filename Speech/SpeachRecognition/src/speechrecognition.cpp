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

float L = 50; //cm
int mobileplatform_motors_write[4] = {128, 128, 128, 128};
int mobileplatform_motors_read[4] = {128, 128, 128, 128};
tbb::atomic<int> keypad_status_raw;
tbb::atomic<int> keypad_status;
tbb::atomic<int> last_keypad_value;


float laser_IR[8];
int IR[5] = {0};
int EMS_STOP = 0;
ros::Publisher chatter_pub[20];
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

void chatterCallback_laser(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    int val_count = msg->ranges.size(); //512

    float read = 0;
    int valid_count = 0;

    for ( int i = 0 ; i < 8 ; i++)
    {
        read = 0;
        valid_count = 0;
        laser_IR[7 - i] = 0;

        for ( int j = 0 ; j < 64 ; j++)
        {
            read = msg->ranges[i * 64 + j];

            if ( !std::isnan(read) && !isinf(read) )
            {
                laser_IR[7 - i] += read;
                valid_count++;
            }
        }

        if ( valid_count > 0)
            laser_IR[7 - i] = (laser_IR[7 - i] / valid_count) * 100;
        else
            laser_IR[7 - i] = 400; //all segment points are damaged ...

        laser_IR[7 - i] = (int)laser_IR[7 - i];

        //cout<<i<<" "<<valid_count<<endl;
    }

}

float omnidrive_x = 0;
float omnidrive_y = 0;
float omnidrive_w = 0;

void chatterCallback_speech(const std_msgs::String::ConstPtr &msg)
{
    cout<<"Speech Get : "<<msg->data<<endl;
   
}




void Update()
{
    //Publisg Omni Speed
    sepanta_msgs::omnidata omni_info;
    omni_info.d0 = mobileplatform_motors_read[0];
    omni_info.d1 = mobileplatform_motors_read[1];
    omni_info.d2 = mobileplatform_motors_read[2];
    omni_info.d3 = mobileplatform_motors_read[3];
    chatter_pub[4].publish(omni_info);

    //Publisg IR Sensors
    sepanta_msgs::irsensor sensor_info;
    sensor_info.d0 = IR[0];
    sensor_info.d1 = IR[1];
    sensor_info.d2 = IR[2];
    sensor_info.d3 = IR[3];
    sensor_info.d4 = IR[4];
    chatter_pub[6].publish(sensor_info);

    //Publish Laser Sensors
    sepanta_msgs::irsensor sensor_info_laser;
    sensor_info_laser.d0 = (int)laser_IR[0];
    sensor_info_laser.d1 = (int)laser_IR[1];
    sensor_info_laser.d2 = (int)laser_IR[2];
    sensor_info_laser.d3 = (int)laser_IR[3];
    sensor_info_laser.d4 = (int)laser_IR[4];
    sensor_info_laser.d5 = (int)laser_IR[5];
    sensor_info_laser.d6 = (int)laser_IR[6];
    sensor_info_laser.d7 = (int)laser_IR[7];
    chatter_pub[7].publish(sensor_info_laser);

    //Publisg Keypad
    std_msgs::Int32 key_msg;
    key_msg.data = keypad_status;
    chatter_pub[8].publish(key_msg); //keypad

    //Publish EMS_Stop
    std_msgs::Int32 ems_msg;
    ems_msg.data = EMS_STOP;
    chatter_pub[9].publish(ems_msg); //ems stop

    //Publisg Btn_Start
    std_msgs::Bool btn_msg;
    btn_msg.data = btn_start;
    chatter_pub[11].publish(btn_msg);// btn_start

    //Publisg Mode
    std_msgs::Int32 mode_msg;
    mode_msg.data = control_mode;
    chatter_pub[12].publish(mode_msg); //Mode

    //Publish Voltage Down
    std_msgs::Int32 v1_msg;
    v1_msg.data = voltage_down;
    chatter_pub[13].publish(v1_msg); //voltage down

    //Publish Voltage Up
    std_msgs::Int32 v2_msg;
    v2_msg.data = voltage_up;
    chatter_pub[14].publish(v2_msg); //voltage up
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speechrecognition");

    ros::NodeHandle node_handles[50];
     ros::Subscriber sub_handles[15];
    //===========================================================================================

    sub_handles[0] = node_handles[0].subscribe("/speechRec/cmd_spch", 10, chatterCallback_speech);
   
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
