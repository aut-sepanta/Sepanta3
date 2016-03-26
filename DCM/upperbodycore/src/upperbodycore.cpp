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

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/JointState.h>
#include <dynamixel_controllers/SetComplianceSlope.h>
#include <dynamixel_controllers/SetCompliancePunch.h>
#include <dynamixel_controllers/SetComplianceMargin.h>
#include <dynamixel_controllers/SetCompliancep.h>
#include <dynamixel_controllers/SetCompliancei.h>
#include <dynamixel_controllers/SetComplianced.h>
#include <dynamixel_controllers/SetSpeed.h>
#include <dynamixel_controllers/TorqueEnable.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <sepanta_msgs/upperbodymotors.h>
#include <sepanta_msgs/upperbodymotorsfeedback.h>
#include <sepanta_msgs/motorfeedback.h>
#include <sepanta_msgs/motor.h>
#include <sepanta_msgs/motorreset.h>
#include <sepanta_msgs/motortorque.h>
#include <sepanta_msgs/motorpid.h>
#include "upperbodycorefunctions.h"

#include "opencv2/opencv.hpp"
#include <ros/package.h>

//version 1394/1/19 <=full check done
//version 1394/1/19 <=add waist motor

using namespace std;
using namespace boost;
using namespace cv;

bool robot_init = false;
int init_counter = 0;
bool motor_init = false;
bool enable_debug = true;
bool App_exit = false;

int ismotorthere_counter = 0;
bool pid_init = false;

ros::Publisher chatter_pub_motor_right[10];
ros::Publisher chatter_pub_motor_left[10];
ros::Publisher chatter_pub_motor_head[3];

ros::Publisher chatter_pub_ack;
ros::Publisher chatter_pub_log;

ros::Publisher chatter_pub_motors;
ros::ServiceClient service_speed_motor_right[10];
ros::ServiceClient service_speed_motor_left[10];
ros::ServiceClient service_speed_motor_head[3];

ros::ServiceClient service_torque_motor_right[10];
ros::ServiceClient service_torque_motor_left[10];
ros::ServiceClient service_torque_motor_head[3];

ros::ServiceClient service_margin[23];
ros::ServiceClient service_punch[23];
ros::ServiceClient service_slope[23];

ros::ServiceClient service_i[23];
ros::ServiceClient service_d[23];
ros::ServiceClient service_p[23];

ros::ServiceServer service_reset_motor;
ros::ServiceServer service_torque_motor;
ros::ServiceServer service_pid_motor;

int system_state = 0;
bool motor_connected = false;

void send_ack()
{
    std_msgs::String msg_ack;
    msg_ack.data = "ok";
    chatter_pub_ack.publish(msg_ack);
}

void send_log(string msg)
{
    std_msgs::String msg_log;
    msg_log.data = msg;
    chatter_pub_log.publish(msg_log);
}

int save_params_pid()
{
    std::string homedir = getenv("HOME");
    std::string path_points =  homedir + "catkin_ws/settings/pid.yml";
    FileStorage fs(path_points, FileStorage::WRITE);

    for ( int i = 0 ; i < motor_list.size() ; i++ )
    {
        string pp = motor_list.at(i).name + "P";
        string ii = motor_list.at(i).name + "I";
        string dd = motor_list.at(i).name + "D";

        fs << pp <<  motor_list.at(i).p;
        fs << ii <<  motor_list.at(i).i;
        fs << dd <<  motor_list.at(i).d;
    }


    fs.release();
}

void load_params_pid()
{
    std::string homedir = getenv("HOME");
    std::string path_points =  homedir + "/catkin_ws/settings/pid.yml";

    cout<<path_points<<endl;
    FileStorage fs2( path_points, FileStorage::READ);

    for ( int i = 0 ; i < motor_list.size() ; i++ )
    {
        string pp = motor_list.at(i).name + "P";
        string ii = motor_list.at(i).name + "I";
        string dd = motor_list.at(i).name + "D";

        int ppv = (int)fs2[pp];
        int iiv = (int)fs2[ii];
        int ddv = (int)fs2[dd];

        motor_list.at(i).p = ppv;
        motor_list.at(i).i = iiv;
        motor_list.at(i).d = ddv;

        //read
        //cout<<motor_list.at(i).name<<" - "<<motor_list.at(i).p<<" "<<motor_list.at(i).i<<" "<<motor_list.at(i).d<<endl;
    }


    fs2.release();
}

void reset_temp_values()
{
    for ( int i = 0 ; i < left_motor_count ; i++)
    {
        sp_Motortemp_left[i] = -1;
        g_Motortemp_left[i] = -1;
    }

    for ( int i = 0 ; i < right_motor_count ; i++)
    {
        sp_Motortemp_right[i] = -1;
        g_Motortemp_right[i] = -1;
    }

    for ( int i = 0 ; i < head_motor_count ; i++)
    {
        sp_Motortemp_head[i] = -1;
        g_Motortemp_head[i] = -1;
    }
}

void init_motors()
{
    //Init Motors

    reset_temp_values();

    for ( int i = 0 ; i < right_motor_count ; i++ )
    {
        g_Motor_right[i] =  motorconfig_list.at(i).init;
        sp_Motor_right[i] = motorconfig_list.at(i).speed;
    }

    for ( int i = 0 ; i < left_motor_count ; i++ )
    {
        g_Motor_left[i] =  motorconfig_list.at(i + 10).init;
        sp_Motor_left[i] = motorconfig_list.at(i + 10).speed;
    }

    for ( int i = 0 ; i < head_motor_count ; i++ )
    {
        g_Motor_head[i] =  motorconfig_list.at(i + 20).init;
        sp_Motor_head[i] = motorconfig_list.at(i + 20).speed;
    }

    //------------------------------------------------------------------------------------
    //Add motors list [22 motors] [empty field] [we should wait for real motors to fill this list]

    for ( int i = 0 ; i < motorconfig_list.size() ; i++ )
    {
        motor_data motor;
        motor.speed = -1;
        motor.position = -1;
        motor.load = -1;
        motor.voltage = -1;
        motor.id = motorconfig_list.at(i).id;
        motor.temp = -1;
        motor.name = motorconfig_list.at(i).name;
        motor.status = "not found";
        motor.min = motorconfig_list.at(i).min;
        motor.max = motorconfig_list.at(i).max;
        motor.init = motorconfig_list.at(i).init;
        motor.model = motorconfig_list.at(i).model;
        motor.p = 0;
        motor.i = 0;
        motor.d = 0;
        motor_list.push_back(motor);
    }

    load_params_pid();


}

void init_config()
{
    ros::NodeHandle node;
    motor_config motorconfig;
    //Right Motors
    //0
    node.getParam("/rightshoulderYawm_controller/motor/id",motorconfig.id);
    node.getParam("/rightshoulderYawm_controller/joint_name",motorconfig.name);motorconfig.name = "rightshoulderYawm";
    node.getParam("/rightshoulderYawm_controller/motor/init",motorconfig.init);
    node.getParam("/rightshoulderYawm_controller/motor/max",motorconfig.max);
    node.getParam("/rightshoulderYawm_controller/motor/min",motorconfig.min);
    node.getParam("/rightshoulderYawm_controller/joint_speed",motorconfig.speed);
    string arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //1
    node.getParam("/rightshoulderYaws_controller/motor/id",motorconfig.id);     motorconfig.id = 112;
    node.getParam("/rightshoulderYaws_controller/joint_name",motorconfig.name); motorconfig.name = "rightshoulderYaws";
    node.getParam("/rightshoulderYaws_controller/motor/init",motorconfig.init);
    node.getParam("/rightshoulderYaws_controller/motor/max",motorconfig.max);
    node.getParam("/rightshoulderYaws_controller/motor/min",motorconfig.min);
    node.getParam("/rightshoulderYaws_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //2
    node.getParam("/rightshoulderPitchm_controller/motor/id",motorconfig.id);
    node.getParam("/rightshoulderPitchm_controller/joint_name",motorconfig.name);motorconfig.name = "rightshoulderPitchm";
    node.getParam("/rightshoulderPitchm_controller/motor/init",motorconfig.init);
    node.getParam("/rightshoulderPitchm_controller/motor/max",motorconfig.max);
    node.getParam("/rightshoulderPitchm_controller/motor/min",motorconfig.min);
    node.getParam("/rightshoulderPitchm_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //3
    node.getParam("/rightshoulderPitchs_controller/motor/id",motorconfig.id);     motorconfig.id = 114;
    node.getParam("/rightshoulderPitchs_controller/joint_name",motorconfig.name); motorconfig.name = "rightshoulderPitchs";
    node.getParam("/rightshoulderPitchs_controller/motor/init",motorconfig.init);
    node.getParam("/rightshoulderPitchs_controller/motor/max",motorconfig.max);
    node.getParam("/rightshoulderPitchs_controller/motor/min",motorconfig.min);
    node.getParam("/rightshoulderPitchs_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //4
    node.getParam("/rightshoulderRoll_controller/motor/id",motorconfig.id);
    node.getParam("/rightshoulderRoll_controller/joint_name",motorconfig.name);motorconfig.name = "rightshoulderRoll";
    node.getParam("/rightshoulderRoll_controller/motor/init",motorconfig.init);
    node.getParam("/rightshoulderRoll_controller/motor/max",motorconfig.max);
    node.getParam("/rightshoulderRoll_controller/motor/min",motorconfig.min);
    node.getParam("/rightshoulderRoll_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //5
    node.getParam("/rightelbowPitch_controller/motor/id",motorconfig.id);
    node.getParam("/rightelbowPitch_controller/joint_name",motorconfig.name);motorconfig.name = "rightelbowPitch";
    node.getParam("/rightelbowPitch_controller/motor/init",motorconfig.init);
    node.getParam("/rightelbowPitch_controller/motor/max",motorconfig.max);
    node.getParam("/rightelbowPitch_controller/motor/min",motorconfig.min);
    node.getParam("/rightelbowPitch_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //6
    node.getParam("/rightelbowRoll_controller/motor/id",motorconfig.id);
    node.getParam("/rightelbowRoll_controller/joint_name",motorconfig.name);motorconfig.name = "rightelbowRoll";
    node.getParam("/rightelbowRoll_controller/motor/init",motorconfig.init);
    node.getParam("/rightelbowRoll_controller/motor/max",motorconfig.max);
    node.getParam("/rightelbowRoll_controller/motor/min",motorconfig.min);
    node.getParam("/rightelbowRoll_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //7
    node.getParam("/rightwristPitch_controller/motor/id",motorconfig.id);
    node.getParam("/rightwristPitch_controller/joint_name",motorconfig.name);motorconfig.name = "rightwristPitch";
    node.getParam("/rightwristPitch_controller/motor/init",motorconfig.init);
    node.getParam("/rightwristPitch_controller/motor/max",motorconfig.max);
    node.getParam("rightwristPitch_controller/motor/min",motorconfig.min);
    node.getParam("rightwristPitch_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //8
    node.getParam("/rightwristRoll_controller/motor/id",motorconfig.id);
    node.getParam("/rightwristRoll_controller/joint_name",motorconfig.name);motorconfig.name = "rightwristRoll";
    node.getParam("/rightwristRoll_controller/motor/init",motorconfig.init);
    node.getParam("/rightwristRoll_controller/motor/max",motorconfig.max);
    node.getParam("/rightwristRoll_controller/motor/min",motorconfig.min);
    node.getParam("/rightwristRoll_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //9
    node.getParam("/rightgripper_controller/motor/id",motorconfig.id);
    node.getParam("/rightgripper_controller/joint_name",motorconfig.name);motorconfig.name = "rightgripper";
    node.getParam("/rightgripper_controller/motor/init",motorconfig.init);
    node.getParam("/rightgripper_controller/motor/max",motorconfig.max);
    node.getParam("/rightgripper_controller/motor/min",motorconfig.min);
    node.getParam("/rightgripper_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    //---------------------------------------------------------------------------
    //Left Motors
    //0
    node.getParam("/leftshoulderYawm_controller/motor/id",motorconfig.id);
    node.getParam("/leftshoulderYawm_controller/joint_name",motorconfig.name);motorconfig.name = "leftshoulderYawm";
    node.getParam("/leftshoulderYawm_controller/motor/init",motorconfig.init);
    node.getParam("/leftshoulderYawm_controller/motor/max",motorconfig.max);
    node.getParam("/leftshoulderYawm_controller/motor/min",motorconfig.min);
    node.getParam("/leftshoulderYawm_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //1
    node.getParam("/leftshoulderYaws_controller/motor/id",motorconfig.id);motorconfig.id = 102;
    node.getParam("/leftshoulderYaws_controller/joint_name",motorconfig.name);motorconfig.name = "leftshoulderYaws";
    node.getParam("/leftshoulderYaws_controller/motor/init",motorconfig.init);
    node.getParam("/leftshoulderYaws_controller/motor/max",motorconfig.max);
    node.getParam("/leftshoulderYaws_controller/motor/min",motorconfig.min);
    node.getParam("/leftshoulderYaws_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //2
    node.getParam("/leftshoulderPitchm_controller/motor/id",motorconfig.id);
    node.getParam("/leftshoulderPitchm_controller/joint_name",motorconfig.name);motorconfig.name = "leftshoulderPitchm";
    node.getParam("/leftshoulderPitchm_controller/motor/init",motorconfig.init);
    node.getParam("/leftshoulderPitchm_controller/motor/max",motorconfig.max);
    node.getParam("/leftshoulderPitchm_controller/motor/min",motorconfig.min);
    node.getParam("/leftshoulderPitchm_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //3
    node.getParam("/leftshoulderPitchs_controller/motor/id",motorconfig.id);motorconfig.id = 104;
    node.getParam("/leftshoulderPitchs_controller/joint_name",motorconfig.name);motorconfig.name = "leftshoulderPitchs";
    node.getParam("/leftshoulderPitchs_controller/motor/init",motorconfig.init);
    node.getParam("/leftshoulderPitchs_controller/motor/max",motorconfig.max);
    node.getParam("/leftshoulderPitchs_controller/motor/min",motorconfig.min);
    node.getParam("/leftshoulderPitchs_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //4
    node.getParam("/leftshoulderRoll_controller/motor/id",motorconfig.id);
    node.getParam("/leftshoulderRoll_controller/joint_name",motorconfig.name);motorconfig.name = "leftshoulderRoll";
    node.getParam("/leftshoulderRoll_controller/motor/init",motorconfig.init);
    node.getParam("/leftshoulderRoll_controller/motor/max",motorconfig.max);
    node.getParam("/leftshoulderRoll_controller/motor/min",motorconfig.min);
    node.getParam("/leftshoulderRoll_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //5
    node.getParam("/leftelbowPitch_controller/motor/id",motorconfig.id);
    node.getParam("/leftelbowPitch_controller/joint_name",motorconfig.name);motorconfig.name = "leftelbowPitch";
    node.getParam("/leftelbowPitch_controller/motor/init",motorconfig.init);
    node.getParam("/leftelbowPitch_controller/motor/max",motorconfig.max);
    node.getParam("/leftelbowPitch_controller/motor/min",motorconfig.min);
    node.getParam("/leftelbowPitch_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //6
    node.getParam("/leftelbowRoll_controller/motor/id",motorconfig.id);
    node.getParam("/leftelbowRoll_controller/joint_name",motorconfig.name);motorconfig.name = "leftelbowRoll";
    node.getParam("/leftelbowRoll_controller/motor/init",motorconfig.init);
    node.getParam("/leftelbowRoll_controller/motor/max",motorconfig.max);
    node.getParam("/leftelbowRoll_controller/motor/min",motorconfig.min);
    node.getParam("/leftelbowRoll_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //7
    node.getParam("/leftwristPitch_controller/motor/id",motorconfig.id);
    node.getParam("/leftwristPitch_controller/joint_name",motorconfig.name);motorconfig.name = "leftwristPitch";
    node.getParam("/leftwristPitch_controller/motor/init",motorconfig.init);
    node.getParam("/leftwristPitch_controller/motor/max",motorconfig.max);
    node.getParam("leftwristPitch_controller/motor/min",motorconfig.min);
    node.getParam("leftwristPitch_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //8
    node.getParam("/leftwristRoll_controller/motor/id",motorconfig.id);
    node.getParam("/leftwristRoll_controller/joint_name",motorconfig.name);motorconfig.name = "leftwristRoll";
    node.getParam("/leftwristRoll_controller/motor/init",motorconfig.init);
    node.getParam("/leftwristRoll_controller/motor/max",motorconfig.max);
    node.getParam("/leftwristRoll_controller/motor/min",motorconfig.min);
    node.getParam("/leftwristRoll_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //9
    node.getParam("/leftgripper_controller/motor/id",motorconfig.id);
    node.getParam("/leftgripper_controller/joint_name",motorconfig.name);motorconfig.name = "leftgripper";
    node.getParam("/leftgripper_controller/motor/init",motorconfig.init);
    node.getParam("/leftgripper_controller/motor/max",motorconfig.max);
    node.getParam("/leftgripper_controller/motor/min",motorconfig.min);
    node.getParam("/leftgripper_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //-----------------------------------------------------------------------------
    //Head Motors
    //0
    node.getParam("/headYaw_controller/motor/id",motorconfig.id);
    node.getParam("/headYaw_controller/joint_name",motorconfig.name);motorconfig.name = "headYaw";
    node.getParam("/headYaw_controller/motor/init",motorconfig.init);
    node.getParam("/headYaw_controller/motor/max",motorconfig.max);
    node.getParam("/headYaw_controller/motor/min",motorconfig.min);
    node.getParam("/headYaw_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //1
    node.getParam("/headPitch_controller/motor/id",motorconfig.id);
    node.getParam("/headPitch_controller/joint_name",motorconfig.name);motorconfig.name = "headPitch";
    node.getParam("/headPitch_controller/motor/init",motorconfig.init);
    node.getParam("/headPitch_controller/motor/max",motorconfig.max);
    node.getParam("/headPitch_controller/motor/min",motorconfig.min);
    node.getParam("/headPitch_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //----------------------------------------------------------------------------------
    //Waist Motor
    node.getParam("/waist_controller/motor/id",motorconfig.id);
    node.getParam("/waist_controller/joint_name",motorconfig.name);motorconfig.name = "waist";
    node.getParam("/waist_controller/motor/init",motorconfig.init);
    node.getParam("/waist_controller/motor/max",motorconfig.max);
    node.getParam("/waist_controller/motor/min",motorconfig.min);
    node.getParam("/waist_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //----------------------------------------------------------------------------------
    init_motors();

}

motor_config get_motorconfig(int id)
{
    motor_config empty;
    empty.name = "none";
    empty.init = -1;
    empty.max = -1;
    empty.min = -1;
    empty.id = -1;

    for ( int i = 0 ; i < motorconfig_list.size() ; i++ )
    {
        if ( motorconfig_list.at(i).id == id )
            return motorconfig_list.at(i);
    }

    return empty;
}

motor_config get_motorconfig(string name)
{
    motor_config empty;
    empty.name = "none";
    empty.init = -1;
    empty.max = -1;
    empty.min = -1;
    empty.id = -1;

    for ( int i = 0 ; i < motorconfig_list.size() ; i++ )
    {
        if ( motorconfig_list.at(i).name == name )
            return motorconfig_list.at(i);
    }

    return empty;
}

int getindex_byid(int id)
{
    for ( int i = 0 ; i < motor_list.size() ; i++ )
    {
        if ( motor_list.at(i).id == id)
            return i;
    }

    return -1;
}

void chatterCallbackw(const dynamixel_msgs::MotorStateList::ConstPtr &msg)
{

    //motor connection established
    motor_connected = true;
    ismotorthere_counter = 0;

    for ( int i = 0 ; i < motor_list.size() ; i++)
    {
        motor_list.at(i).status = "not found";
    }
    for ( int i = 0 ; i < msg->motor_states.size() ; i++ )
    {
        int get_id = msg->motor_states[i].id;
        int index = getindex_byid(get_id);
        if ( index == -1 ) continue;

        motor_list.at(index).speed = msg->motor_states[i].speed / 10;
        motor_list.at(index).position = msg->motor_states[i].position;
        motor_list.at(index).load = msg->motor_states[i].load * 1000;
        motor_list.at(index).voltage = msg->motor_states[i].voltage;
        motor_list.at(index).id = msg->motor_states[i].id;
        motor_list.at(index).temp = msg->motor_states[i].temperature;
        motor_list.at(index).status = "ready";
    }

    //============================ output with same Hz


}

void publish_feedbacks()
{

    if ( ismotorthere_counter < 20 )
    {
        ismotorthere_counter++;
    }
    else
    {
        cout<<"not found counter overflow"<<endl;

        for ( int i = 0 ; i < motor_list.size() ; i++)
        {
            motor_list.at(i).status = "not found";
        }

        //motor connection lost
        motor_connected = false;
        system_state = 0;
    }

    sepanta_msgs::upperbodymotorsfeedback feedback_list;

    //TODO1 check for name and index
    for ( int i = 0 ; i < motor_list.size() ; i++)
    {
        sepanta_msgs::motorfeedback feedback;
        feedback.id = motor_list.at(i).id;
        feedback.load = motor_list.at(i).load;
        feedback.speed = motor_list.at(i).speed;
        feedback.position = motor_list.at(i).position;
        feedback.voltage = motor_list.at(i).voltage;
        feedback.temp = motor_list.at(i).temp;
        feedback.name = motor_list.at(i).name;
        feedback.status = motor_list.at(i).status;
        feedback.min = motor_list.at(i).min;
        feedback.max = motor_list.at(i).max;
        feedback.init = motor_list.at(i).init;
        feedback.model = motor_list.at(i).model;
        feedback.p = motor_list.at(i).p;
        feedback.i = motor_list.at(i).i;
        feedback.d = motor_list.at(i).d;

        feedback_list.motorfeedbacks.push_back(feedback);
    }

    chatter_pub_motors.publish(feedback_list);
}

void chatterCallback_right_motors(const sepanta_msgs::upperbodymotors::ConstPtr &msg)
{


    g_Motor_right[0] = msg->shoulder_yawm_position;     //right_shoulder_base1 ?
    g_Motor_right[1] = 0;     //right_shoulder_base2 ?

    g_Motor_right[2] = msg->shoulder_pitchm_position; //right_shoulder_pitchm 113
    g_Motor_right[3] = 0;    //right_shoulder_pitchs 114
    g_Motor_right[4] = msg->shoulder_roll_position; //right_shoulder_roll 115
    g_Motor_right[5] = msg->elbow_pitch_position; //right_elbow_pitch 116

    g_Motor_right[6] = msg->elbow_roll_position;  //right_elbow_roll 117
    g_Motor_right[7] = msg->wrist_pitch_position;  //right_wrist_pitch 118
    g_Motor_right[8] = msg->wrist_roll_position;  //right_wrist_roll 119
    g_Motor_right[9] = msg->gripper_position;  //gripper 120

    //==============================================

    sp_Motor_right[0] = msg->shoulder_yawm_speed;     //right_shoulder_base1 ?
    sp_Motor_right[1] = 0;     //right_shoulder_base2 ?

    sp_Motor_right[2] = msg->shoulder_pitchm_speed; //right_shoulder_pitchm 113
    sp_Motor_right[3] = 0; //right_shoulder_pitchs 114
    sp_Motor_right[4] = msg->shoulder_roll_speed; //right_shoulder_roll 115
    sp_Motor_right[5] = msg->elbow_pitch_speed; //right_elbow_pitch 116

    sp_Motor_right[6] = msg->elbow_roll_speed;  //right_elbow_roll 117
    sp_Motor_right[7] = msg->wrist_pitch_speed;  //right_wrist_pitch 118
    sp_Motor_right[8] = msg->wrist_roll_speed;  //right_wrist_roll 119
    sp_Motor_right[9] = msg->gripper_speed;  //gripper 120
}

void chatterCallback_left_motors(const sepanta_msgs::upperbodymotors::ConstPtr &msg)
{


    g_Motor_left[0] = msg->shoulder_yawm_position;     //left_shoulder_base1 ?
    g_Motor_left[1] = 0;     //left_shoulder_base2 ?

    g_Motor_left[2] = msg->shoulder_pitchm_position; //left_shoulder_pitchm 113
    g_Motor_left[3] = 0;    //left_shoulder_pitchs 114
    g_Motor_left[4] = msg->shoulder_roll_position; //left_shoulder_roll 115
    g_Motor_left[5] = msg->elbow_pitch_position; //left_elbow_pitch 116

    g_Motor_left[6] = msg->elbow_roll_position;  //left_elbow_roll 117
    g_Motor_left[7] = msg->wrist_pitch_position;  //left_wrist_pitch 118
    g_Motor_left[8] = msg->wrist_roll_position;  //left_wrist_roll 119
    g_Motor_left[9] = msg->gripper_position;  //gripper 120

    //==============================================

    sp_Motor_left[0] = msg->shoulder_yawm_speed;     //left_shoulder_base1 ?
    sp_Motor_left[1] = 0;     //left_shoulder_base2 ?

    sp_Motor_left[2] = msg->shoulder_pitchm_speed; //left_shoulder_pitchm 113
    sp_Motor_left[3] = 0; //left_shoulder_pitchs 114
    sp_Motor_left[4] = msg->shoulder_roll_speed; //left_shoulder_roll 115
    sp_Motor_left[5] = msg->elbow_pitch_speed; //left_elbow_pitch 116

    sp_Motor_left[6] = msg->elbow_roll_speed;  //left_elbow_roll 117
    sp_Motor_left[7] = msg->wrist_pitch_speed;  //left_wrist_pitch 118
    sp_Motor_left[8] = msg->wrist_roll_speed;  //left_wrist_roll 119
    sp_Motor_left[9] = msg->gripper_speed;  //gripper 120
}

void chatterCallback_head_motors(const sepanta_msgs::upperbodymotors::ConstPtr &msg)
{
    g_Motor_head[0] = msg->head_yaw_position;     //left_shoulder_base1 ?
    g_Motor_head[1] = msg->head_pitch_position;     //left_shoulder_base2 ?
    g_Motor_head[2] = msg->waist_position;

    sp_Motor_head[0] = msg->head_yaw_speed;     //left_shoulder_base1 ?
    sp_Motor_head[1] = msg->head_pitch_speed;     //left_shoulder_base2 ?
    sp_Motor_head[2] = msg->waist_speed;


}

void set_pid(int index ,int margin ,  int slope ,int punch )
{
    dynamixel_controllers::SetComplianceMargin srv_margin;
    dynamixel_controllers::SetComplianceSlope srv_slope;
    dynamixel_controllers::SetCompliancePunch srv_punch;

    dynamixel_controllers::SetCompliancep srv_p;
    dynamixel_controllers::SetCompliancei srv_i;
    dynamixel_controllers::SetComplianced srv_d;

    string model = motor_list.at(index).model;
    if ( model.at(0) == 'A')
    {
        srv_margin.request.margin = margin;
        srv_slope.request.slope = slope;
        srv_punch.request.punch = punch;
        service_margin[index].call(srv_margin);
        service_slope[index].call(srv_slope);
        service_punch[index].call(srv_punch);
    }
    else
    {
        srv_i.request.margin = margin;
        srv_p.request.slope = slope;
        srv_d.request.punch = punch;
        service_i[index].call(srv_i);
        service_p[index].call(srv_p);
        service_d[index].call(srv_d);
    }

    motor_list.at(index).p = slope;
    motor_list.at(index).i = margin;
    motor_list.at(index).d = punch;

    save_params_pid();
}

void Motor_Update()
{


    //cout<<init_counter<<endl;
    //set pids for first time
     if ( robot_init == true && pid_init == false )
     {
         pid_init = true;
         for (int i = 0; i < motor_list.size() ; i++)
         {
             motor_data item = motor_list.at(i);
             if (  item.id != 102 && item.id != 104 && item.id != 112 && item.id != 114) //slaves were not include
             {
                 if ( item.status == "ready" )
                 {
                     set_pid(i,item.i,item.p,item.d);
                     cout<<"SET PID : ["<<i<<"] "<<item.p<<" "<<item.i<<" "<<item.d<<endl;
                     //boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                 }
             }
         }
     }
    //-----------------------------------------------------
    //right motors update
    for (int i = 0; i < right_motor_count; i++)
    {
        if (  i != 1 && i != 3 ) //slaves were not include
        {
            if ( g_Motortemp_right[i] != g_Motor_right[i] && robot_init == true)
            {
                g_Motortemp_right[i] = g_Motor_right[i];

                std_msgs::Int32 msg;
                msg.data = g_Motor_right[i];

                if ( motor_list.at(i).status != "not found")
                    chatter_pub_motor_right[i].publish(msg);

                //cout<<"send right "<<i<<" "<<g_Motor_right[i]<<endl;

            }
        }
    }

    //left motors update
    for (int i = 0; i < left_motor_count; i++)
    {
        if (  i != 1 && i != 3 ) //slaves were not include
        {
            if ( g_Motortemp_left[i] != g_Motor_left[i] && robot_init == true)
            {
                g_Motortemp_left[i] = g_Motor_left[i];

                std_msgs::Int32 msg;
                msg.data = g_Motor_left[i];

                if ( motor_list.at(i + 10).status != "not found")
                    chatter_pub_motor_left[i].publish(msg);

                //cout<<"send left"<<i<<" "<<g_Motor_left[i]<<endl;


            }
        }
    }

    //head motors update
    for (int i = 0; i < head_motor_count; i++)
    {
        if ( g_Motortemp_head[i] != g_Motor_head[i] && robot_init == true)
        {
            g_Motortemp_head[i] = g_Motor_head[i];

            std_msgs::Int32 msg;
            msg.data = g_Motor_head[i];

            if ( motor_list.at(i + 20).status != "not found")
                chatter_pub_motor_head[i].publish(msg);

            //cout<<"send head"<<i<<" "<<g_Motor_head[i]<<endl;


        }
    }
}

void Motor_Update_speed()
{

    if ( motor_connected && system_state == 0) //state is 0 , wait for motors to connect...
    {
       system_state = 1;
       init_counter = 0;
       robot_init = false;
       pid_init = false;
       reset_temp_values();
    }

   if ( system_state == 0 ) return;

   if ( init_counter < 25)
       init_counter++;
   else
       robot_init = true;

    for (int i = 0; i < right_motor_count; i++)
    {
        if ( i != 1 && i != 3 ) //slaves
        {
            float value = 0;
            if ( sp_Motortemp_right[i] != sp_Motor_right[i] && robot_init == true)
            {
                sp_Motortemp_right[i] = sp_Motor_right[i];

                value = ((float)sp_Motor_right[i]) / 1000;
                dynamixel_controllers::SetSpeed speed;
                speed.request.speed = value;

                if ( motor_list.at(i).status != "not found")
                    service_speed_motor_right[i].call(speed);

                // cout<<"speed right "<<i<<" "<<value<<endl;
            }


        }
    }

    for (int i = 0; i < left_motor_count; i++)
    {
        if ( i != 1 && i != 3 ) //slaves
        {
            float value = 0;
            if ( sp_Motortemp_left[i] != sp_Motor_left[i] && robot_init == true)
            {
                sp_Motortemp_left[i] = sp_Motor_left[i];

                value = ((float)sp_Motor_left[i]) / 1000;
                dynamixel_controllers::SetSpeed speed;
                speed.request.speed = value;

                if ( motor_list.at(i + 10).status != "not found")
                    service_speed_motor_left[i].call(speed);

                //cout<<"speed left "<<i<<" "<<value<<endl;
            }


        }
    }

    for (int i = 0; i < head_motor_count; i++)
    {
        float value = 0;
        if ( sp_Motortemp_head[i] != sp_Motor_head[i] && robot_init == true)
        {
            sp_Motortemp_head[i] = sp_Motor_head[i];

            value = ((float)sp_Motor_head[i]) / 1000;
            dynamixel_controllers::SetSpeed speed;
            speed.request.speed = value;

            if ( motor_list.at(i + 20).status != "not found")
            {
                service_speed_motor_head[i].call(speed);

            }

             //cout<<"speed head "<<i<<" "<<value<<endl;
        }


        
    }
}

void torque_toggle(bool value)
{
    for ( int i = 0 ; i < right_motor_count ; i++ )
    {
        dynamixel_controllers::TorqueEnable t_srv;
        t_srv.request.torque_enable = value;

        if ( motor_list.at(i).status != "not found")
            service_torque_motor_right[i].call(t_srv);
    }

    for ( int i = 0 ; i < left_motor_count ; i++ )
    {
        dynamixel_controllers::TorqueEnable t_srv;
        t_srv.request.torque_enable = value;

        if ( motor_list.at(i + 10).status != "not found")
            service_torque_motor_left[i].call(t_srv);
    }

    for ( int i = 0 ; i < head_motor_count ; i++ )
    {
        dynamixel_controllers::TorqueEnable t_srv;
        t_srv.request.torque_enable = value;

        if ( motor_list.at(i + 20).status != "not found")
            service_torque_motor_head[i].call(t_srv);
    }
}

bool callback_resetmotor(sepanta_msgs::motorreset::Request& request, sepanta_msgs::motorreset::Response& response)
{

    send_log("get [reset] request : " + request.id );

    if ( request.id  == "rightShoulderYaw" ) reset_right_shoulder_yaw();
    if ( request.id  == "rightShoulderPitch" ) reset_right_shoulder_pitch();
    if ( request.id  == "rightShoulderRoll" ) reset_right_shoulder_roll();
    if ( request.id  == "rightElbowPitch" ) reset_right_elbow_pitch();
    if ( request.id  == "rightElbowRoll") reset_right_elbow_roll();
    if ( request.id  == "rightWristPitch" ) reset_right_wrist_pitch();
    if ( request.id  == "rightWristRoll") reset_right_wrist_roll();
    if ( request.id  == "rightGripper" ) reset_right_gripper();
    if ( request.id  == "rightArm") reset_right_arm();

    if ( request.id  == "leftShoulderYaw" ) reset_left_shoulder_yaw();
    if ( request.id  == "leftShoulderPitch" ) reset_left_shoulder_pitch();
    if ( request.id  == "leftShoulderRoll" ) reset_left_shoulder_roll();
    if ( request.id  == "leftElbowPitch" ) reset_left_elbow_pitch();
    if ( request.id  == "leftElbowRoll") reset_left_elbow_roll();
    if ( request.id  == "leftWristPitch" ) reset_left_wrist_pitch();
    if ( request.id  == "leftWristRoll") reset_left_wrist_roll();
    if ( request.id  == "leftGripper" ) reset_left_gripper();
    if ( request.id  == "leftArm") reset_left_arm();

    if ( request.id  == "headYaw" ) reset_head_yaw();
    if ( request.id  == "headPitch" ) reset_head_pitch();
    if ( request.id  == "waist" ) reset_waist();

    return true;
}

bool callback_pidmotor(sepanta_msgs::motorpid::Request& request, sepanta_msgs::motorpid::Response& response)
{
    send_log("get [pid] request : " + request.id );

    uint8_t margin = request.margin;
    uint8_t slope  = request.slope;
    uint8_t punch = request.punch;

    if ( request.id  == "rightshoulderYawm" ) set_pid(0,margin,slope,punch);
    if ( request.id  == "rightshoulderPitchm" ) set_pid(2,margin,slope,punch);
    if ( request.id  == "rightshoulderRoll" ) set_pid(4,margin,slope,punch);
    if ( request.id  == "rightelbowPitch" ) set_pid(5,margin,slope,punch);
    if ( request.id  == "rightelbowRoll") set_pid(6,margin,slope,punch);
    if ( request.id  == "rightwristPitch" ) set_pid(7,margin,slope,punch);
    if ( request.id  == "rightwristRoll") set_pid(8,margin,slope,punch);
    if ( request.id  == "rightgripper" ) set_pid(9,margin,slope,punch);
    if ( request.id  == "leftshoulderYawm" ) set_pid(10,margin,slope,punch);
    if ( request.id  == "leftshoulderPitchm" ) set_pid(12,margin,slope,punch);
    if ( request.id  == "leftshoulderRoll" ) set_pid(14,margin,slope,punch);
    if ( request.id  == "leftelbowPitch" ) set_pid(15,margin,slope,punch);
    if ( request.id  == "leftelbowRoll") set_pid(16,margin,slope,punch);
    if ( request.id  == "leftwristPitch" )set_pid(17,margin,slope,punch);
    if ( request.id  == "leftwristRoll") set_pid(18,margin,slope,punch);
    if ( request.id  == "leftgripper" ) set_pid(19,margin,slope,punch);
    if ( request.id  == "headYaw" ) set_pid(20,margin,slope,punch);
    if ( request.id  == "headPitch" )set_pid(21,margin,slope,punch);
    if ( request.id  == "waist" )set_pid(22,margin,slope,punch);

    return true;
}

bool callback_torquemotor(sepanta_msgs::motortorque::Request& request, sepanta_msgs::motortorque::Response& response)
{
    int a = 0;
    if ( request.status == true) a = 1;

    string v = "False";
    if ( a == 1 )
        v = "True";

    send_log("get [torque] request : " + v );

    torque_toggle(request.status);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "upperbody_core");
    cout << "upperBody core started done..." << endl;

    ros::NodeHandle node_handles[100];
    ros::Subscriber sub_handles[50];
    
    //-----------------------------------------------------------------------------------------

    sub_handles[0] = node_handles[10].subscribe("/motor_states/dx_port", 10, chatterCallbackw);

    sub_handles[1] = node_handles[40].subscribe("upperbodycorein_right_motors", 10, chatterCallback_right_motors); //all right motors
    sub_handles[2] = node_handles[41].subscribe("upperbodycorein_left_motors", 10, chatterCallback_left_motors); //all left motors
    sub_handles[3] = node_handles[41].subscribe("upperbodycorein_head_motors", 10, chatterCallback_head_motors); //all head motors

    sub_handles[4] = node_handles[51].subscribe("upperbodycorein_right_shoulder_yaw", 10, chatterCallback_right_shoulder_yaw);
    sub_handles[5] = node_handles[52].subscribe("upperbodycorein_right_shoulder_pitch", 10, chatterCallback_right_shoulder_pitch);
    sub_handles[6] = node_handles[53].subscribe("upperbodycorein_right_shoulder_roll", 10, chatterCallback_right_shoulder_roll);
    sub_handles[7] = node_handles[54].subscribe("upperbodycorein_right_elbow_pitch", 10, chatterCallback_right_elbow_pitch);
    sub_handles[8] = node_handles[55].subscribe("upperbodycorein_right_elbow_roll", 10, chatterCallback_right_elbow_roll);
    sub_handles[9] = node_handles[56].subscribe("upperbodycorein_right_wrist_pitch", 10, chatterCallback_right_wrist_pitch);
    sub_handles[10] = node_handles[57].subscribe("upperbodycorein_right_wrist_roll", 10, chatterCallback_right_wrist_roll);
    sub_handles[11] = node_handles[58].subscribe("upperbodycorein_right_gripper", 10, chatterCallback_right_gripper);

    sub_handles[12] = node_handles[51].subscribe("upperbodycorein_left_shoulder_yaw", 10, chatterCallback_left_shoulder_yaw);
    sub_handles[13] = node_handles[52].subscribe("upperbodycorein_left_shoulder_pitch", 10, chatterCallback_left_shoulder_pitch);
    sub_handles[14] = node_handles[53].subscribe("upperbodycorein_left_shoulder_roll", 10, chatterCallback_left_shoulder_roll);
    sub_handles[15] = node_handles[54].subscribe("upperbodycorein_left_elbow_pitch", 10, chatterCallback_left_elbow_pitch);
    sub_handles[16] = node_handles[55].subscribe("upperbodycorein_left_elbow_roll", 10, chatterCallback_left_elbow_roll);
    sub_handles[17] = node_handles[56].subscribe("upperbodycorein_left_wrist_pitch", 10, chatterCallback_left_wrist_pitch);
    sub_handles[18] = node_handles[57].subscribe("upperbodycorein_left_wrist_roll", 10, chatterCallback_left_wrist_roll);
    sub_handles[19] = node_handles[58].subscribe("upperbodycorein_left_gripper", 10, chatterCallback_left_gripper);

    sub_handles[20] = node_handles[51].subscribe("upperbodycorein_head_yaw", 10, chatterCallback_head_yaw);
    sub_handles[21] = node_handles[52].subscribe("upperbodycorein_head_pitch", 10, chatterCallback_head_pitch);
    sub_handles[22] = node_handles[53].subscribe("upperbodycorein_waist", 10, chatterCallback_waist);
    //=================================================================================================================

    chatter_pub_motors = node_handles[30].advertise<sepanta_msgs::upperbodymotorsfeedback>("/upperbodycoreout_feedback", 10);

    chatter_pub_motor_right[0] = node_handles[0].advertise<std_msgs::Int32>("/rightshoulderYawm_controller/command", 10);
    chatter_pub_motor_right[1] = node_handles[1].advertise<std_msgs::Int32>("/rightshoulderYaws_controller/command", 10);
    chatter_pub_motor_right[2] = node_handles[2].advertise<std_msgs::Int32>("/rightshoulderPitchm_controller/command", 10);
    chatter_pub_motor_right[3] = node_handles[3].advertise<std_msgs::Int32>("/rightshoulderPitchs_controller/command", 10);
    chatter_pub_motor_right[4] = node_handles[4].advertise<std_msgs::Int32>("/rightshoulderRoll_controller/command", 10);
    chatter_pub_motor_right[5] = node_handles[5].advertise<std_msgs::Int32>("/rightelbowPitch_controller/command", 10);
    chatter_pub_motor_right[6] = node_handles[6].advertise<std_msgs::Int32>("/rightelbowRoll_controller/command", 10);
    chatter_pub_motor_right[7] = node_handles[7].advertise<std_msgs::Int32>("/rightwristPitch_controller/command", 10);
    chatter_pub_motor_right[8] = node_handles[8].advertise<std_msgs::Int32>("/rightwristRoll_controller/command", 10);
    chatter_pub_motor_right[9] = node_handles[9].advertise<std_msgs::Int32>("/rightgripper_controller/command", 10);

    chatter_pub_motor_left[0] = node_handles[0].advertise<std_msgs::Int32>("/leftshoulderYawm_controller/command", 10);
    chatter_pub_motor_left[1] = node_handles[1].advertise<std_msgs::Int32>("/leftshoulderYaws_controller/command", 10);
    chatter_pub_motor_left[2] = node_handles[2].advertise<std_msgs::Int32>("/leftshoulderPitchm_controller/command", 10);
    chatter_pub_motor_left[3] = node_handles[3].advertise<std_msgs::Int32>("/leftshoulderPitchs_controller/command", 10);
    chatter_pub_motor_left[4] = node_handles[4].advertise<std_msgs::Int32>("/leftshoulderRoll_controller/command", 10);
    chatter_pub_motor_left[5] = node_handles[5].advertise<std_msgs::Int32>("/leftelbowPitch_controller/command", 10);
    chatter_pub_motor_left[6] = node_handles[6].advertise<std_msgs::Int32>("/leftelbowRoll_controller/command", 10);
    chatter_pub_motor_left[7] = node_handles[7].advertise<std_msgs::Int32>("/leftwristPitch_controller/command", 10);
    chatter_pub_motor_left[8] = node_handles[8].advertise<std_msgs::Int32>("/leftwristRoll_controller/command", 10);
    chatter_pub_motor_left[9] = node_handles[9].advertise<std_msgs::Int32>("/leftgripper_controller/command", 10);

    chatter_pub_motor_head[0] = node_handles[0].advertise<std_msgs::Int32>("/headYaw_controller/command", 10);
    chatter_pub_motor_head[1] = node_handles[1].advertise<std_msgs::Int32>("/headPitch_controller/command", 10);
    chatter_pub_motor_head[2] = node_handles[1].advertise<std_msgs::Int32>("/waist_controller/command", 10);

    chatter_pub_ack = node_handles[0].advertise<std_msgs::String>("/core_upperbody/ack",10);
    chatter_pub_log = node_handles[1].advertise<std_msgs::String>("/core_upperbody/log",10);
    //========================================================================================================
    //speed service clients

    service_speed_motor_right[0] = node_handles[20].serviceClient<dynamixel_controllers::SetSpeed>("/rightshoulderYawm_controller/set_speed");
    service_speed_motor_right[1] = node_handles[21].serviceClient<dynamixel_controllers::SetSpeed>("/rightshoulderYaws_controller/set_speed");
    service_speed_motor_right[2] = node_handles[22].serviceClient<dynamixel_controllers::SetSpeed>("/rightshoulderPitchm_controller/set_speed");
    service_speed_motor_right[3] = node_handles[23].serviceClient<dynamixel_controllers::SetSpeed>("/rightshoulderPitchs_controller/set_speed");
    service_speed_motor_right[4] = node_handles[24].serviceClient<dynamixel_controllers::SetSpeed>("/rightshoulderRoll_controller/set_speed");
    service_speed_motor_right[5] = node_handles[25].serviceClient<dynamixel_controllers::SetSpeed>("/rightelbowPitch_controller/set_speed");
    service_speed_motor_right[6] = node_handles[26].serviceClient<dynamixel_controllers::SetSpeed>("/rightelbowRoll_controller/set_speed");
    service_speed_motor_right[7] = node_handles[27].serviceClient<dynamixel_controllers::SetSpeed>("/rightwristPitch_controller/set_speed");
    service_speed_motor_right[8] = node_handles[28].serviceClient<dynamixel_controllers::SetSpeed>("/rightwristRoll_controller/set_speed");
    service_speed_motor_right[9] = node_handles[29].serviceClient<dynamixel_controllers::SetSpeed>("/rightgripper_controller/set_speed");

    service_speed_motor_left[0] = node_handles[20].serviceClient<dynamixel_controllers::SetSpeed>("/leftshoulderYawm_controller/set_speed");
    service_speed_motor_left[1] = node_handles[21].serviceClient<dynamixel_controllers::SetSpeed>("/leftshoulderYaws_controller/set_speed");
    service_speed_motor_left[2] = node_handles[22].serviceClient<dynamixel_controllers::SetSpeed>("/leftshoulderPitchm_controller/set_speed");
    service_speed_motor_left[3] = node_handles[23].serviceClient<dynamixel_controllers::SetSpeed>("/leftshoulderPitchs_controller/set_speed");
    service_speed_motor_left[4] = node_handles[24].serviceClient<dynamixel_controllers::SetSpeed>("/leftshoulderRoll_controller/set_speed");
    service_speed_motor_left[5] = node_handles[25].serviceClient<dynamixel_controllers::SetSpeed>("/leftelbowPitch_controller/set_speed");
    service_speed_motor_left[6] = node_handles[26].serviceClient<dynamixel_controllers::SetSpeed>("/leftelbowRoll_controller/set_speed");
    service_speed_motor_left[7] = node_handles[27].serviceClient<dynamixel_controllers::SetSpeed>("/leftwristPitch_controller/set_speed");
    service_speed_motor_left[8] = node_handles[28].serviceClient<dynamixel_controllers::SetSpeed>("/leftwristRoll_controller/set_speed");
    service_speed_motor_left[9] = node_handles[29].serviceClient<dynamixel_controllers::SetSpeed>("/leftgripper_controller/set_speed");

    service_speed_motor_head[0] = node_handles[20].serviceClient<dynamixel_controllers::SetSpeed>("/headYaw_controller/set_speed");
    service_speed_motor_head[1] = node_handles[21].serviceClient<dynamixel_controllers::SetSpeed>("/headPitch_controller/set_speed");
    service_speed_motor_head[2] = node_handles[22].serviceClient<dynamixel_controllers::SetSpeed>("/waist_controller/set_speed");

    service_torque_motor_right[0] = node_handles[20].serviceClient<dynamixel_controllers::TorqueEnable>("/rightshoulderYawm_controller/torque_enable");
    service_torque_motor_right[1] = node_handles[21].serviceClient<dynamixel_controllers::TorqueEnable>("/rightshoulderYaws_controller/torque_enable");
    service_torque_motor_right[2] = node_handles[22].serviceClient<dynamixel_controllers::TorqueEnable>("/rightshoulderPitchm_controller/torque_enable");
    service_torque_motor_right[3] = node_handles[23].serviceClient<dynamixel_controllers::TorqueEnable>("/rightshoulderPitchs_controller/torque_enable");
    service_torque_motor_right[4] = node_handles[24].serviceClient<dynamixel_controllers::TorqueEnable>("/rightshoulderRoll_controller/torque_enable");
    service_torque_motor_right[5] = node_handles[25].serviceClient<dynamixel_controllers::TorqueEnable>("/rightelbowPitch_controller/torque_enable");
    service_torque_motor_right[6] = node_handles[26].serviceClient<dynamixel_controllers::TorqueEnable>("/rightelbowRoll_controller/torque_enable");
    service_torque_motor_right[7] = node_handles[27].serviceClient<dynamixel_controllers::TorqueEnable>("/rightwristPitch_controller/torque_enable");
    service_torque_motor_right[8] = node_handles[28].serviceClient<dynamixel_controllers::TorqueEnable>("/rightwristRoll_controller/torque_enable");
    service_torque_motor_right[9] = node_handles[29].serviceClient<dynamixel_controllers::TorqueEnable>("/rightgripper_controller/torque_enable");

    service_torque_motor_left[0] = node_handles[20].serviceClient<dynamixel_controllers::TorqueEnable>("/leftshoulderYawm_controller/torque_enable");
    service_torque_motor_left[1] = node_handles[21].serviceClient<dynamixel_controllers::TorqueEnable>("/leftshoulderYaws_controller/torque_enable");
    service_torque_motor_left[2] = node_handles[22].serviceClient<dynamixel_controllers::TorqueEnable>("/leftshoulderPitchm_controller/torque_enable");
    service_torque_motor_left[3] = node_handles[23].serviceClient<dynamixel_controllers::TorqueEnable>("/leftshoulderPitchs_controller/torque_enable");
    service_torque_motor_left[4] = node_handles[24].serviceClient<dynamixel_controllers::TorqueEnable>("/leftshoulderRoll_controller/torque_enable");
    service_torque_motor_left[5] = node_handles[25].serviceClient<dynamixel_controllers::TorqueEnable>("/leftelbowPitch_controller/torque_enable");
    service_torque_motor_left[6] = node_handles[26].serviceClient<dynamixel_controllers::TorqueEnable>("/leftelbowRoll_controller/torque_enable");
    service_torque_motor_left[7] = node_handles[27].serviceClient<dynamixel_controllers::TorqueEnable>("/leftwristPitch_controller/torque_enable");
    service_torque_motor_left[8] = node_handles[28].serviceClient<dynamixel_controllers::TorqueEnable>("/leftwristRoll_controller/torque_enable");
    service_torque_motor_left[9] = node_handles[29].serviceClient<dynamixel_controllers::TorqueEnable>("/leftgripper_controller/torque_enable");

    service_torque_motor_head[0] = node_handles[20].serviceClient<dynamixel_controllers::TorqueEnable>("/headYaw_controller/torque_enable");
    service_torque_motor_head[1] = node_handles[21].serviceClient<dynamixel_controllers::TorqueEnable>("/headPitch_controller/torque_enable");
    service_torque_motor_head[2] = node_handles[22].serviceClient<dynamixel_controllers::TorqueEnable>("/waist_controller/torque_enable");

    //========================================================================================================= PID

    service_margin[0] = node_handles[30].serviceClient<dynamixel_controllers::SetComplianceMargin>("/rightshoulderYawm_controller/set_compliance_margin");
    service_margin[1] = node_handles[31].serviceClient<dynamixel_controllers::SetComplianceMargin>("/rightshoulderYaws_controller/set_compliance_margin");
    service_margin[2] = node_handles[32].serviceClient<dynamixel_controllers::SetComplianceMargin>("/rightshoulderPitchm_controller/set_compliance_margin");
    service_margin[3] = node_handles[33].serviceClient<dynamixel_controllers::SetComplianceMargin>("/rightshoulderPitchs_controller/set_compliance_margin");
    service_margin[4] = node_handles[34].serviceClient<dynamixel_controllers::SetComplianceMargin>("/rightshoulderRoll_controller/set_compliance_margin");
    service_margin[5] = node_handles[35].serviceClient<dynamixel_controllers::SetComplianceMargin>("/rightelbowPitch_controller/set_compliance_margin");
    service_margin[6] = node_handles[36].serviceClient<dynamixel_controllers::SetComplianceMargin>("/rightelbowRoll_controller/set_compliance_margin");
    service_margin[7] = node_handles[37].serviceClient<dynamixel_controllers::SetComplianceMargin>("/rightwristPitch_controller/set_compliance_margin");
    service_margin[8] = node_handles[38].serviceClient<dynamixel_controllers::SetComplianceMargin>("/rightwristRoll_controller/set_compliance_margin");
    service_margin[9] = node_handles[39].serviceClient<dynamixel_controllers::SetComplianceMargin>("/rightgripper_controller/set_compliance_margin");
    service_margin[10] = node_handles[40].serviceClient<dynamixel_controllers::SetComplianceMargin>("/leftshoulderYawm_controller/set_compliance_margin");
    service_margin[11] = node_handles[41].serviceClient<dynamixel_controllers::SetComplianceMargin>("/leftshoulderYaws_controller/set_compliance_margin");
    service_margin[12] = node_handles[42].serviceClient<dynamixel_controllers::SetComplianceMargin>("/leftshoulderPitchm_controller/set_compliance_margin");
    service_margin[13] = node_handles[43].serviceClient<dynamixel_controllers::SetComplianceMargin>("/leftshoulderPitchs_controller/set_compliance_margin");
    service_margin[14] = node_handles[44].serviceClient<dynamixel_controllers::SetComplianceMargin>("/leftshoulderRoll_controller/set_compliance_margin");
    service_margin[15] = node_handles[45].serviceClient<dynamixel_controllers::SetComplianceMargin>("/leftelbowPitch_controller/set_compliance_margin");
    service_margin[16] = node_handles[46].serviceClient<dynamixel_controllers::SetComplianceMargin>("/leftelbowRoll_controller/set_compliance_margin");
    service_margin[17] = node_handles[47].serviceClient<dynamixel_controllers::SetComplianceMargin>("/leftwristPitch_controller/set_compliance_margin");
    service_margin[18] = node_handles[48].serviceClient<dynamixel_controllers::SetComplianceMargin>("/leftwristRoll_controller/set_compliance_margin");
    service_margin[19] = node_handles[49].serviceClient<dynamixel_controllers::SetComplianceMargin>("/leftgripper_controller/set_compliance_margin");
    service_margin[20] = node_handles[50].serviceClient<dynamixel_controllers::SetComplianceMargin>("/headYaw_controller/set_compliance_margin");
    service_margin[21] = node_handles[51].serviceClient<dynamixel_controllers::SetComplianceMargin>("/headPitch_controller/set_compliance_margin");
    service_margin[22] = node_handles[52].serviceClient<dynamixel_controllers::SetComplianceMargin>("/waist_controller/set_compliance_margin");

    service_punch[0] = node_handles[30].serviceClient<dynamixel_controllers::SetCompliancePunch>("/rightshoulderYawm_controller/set_compliance_punch");
    service_punch[1] = node_handles[31].serviceClient<dynamixel_controllers::SetCompliancePunch>("/rightshoulderYaws_controller/set_compliance_punch");
    service_punch[2] = node_handles[32].serviceClient<dynamixel_controllers::SetCompliancePunch>("/rightshoulderPitchm_controller/set_compliance_punch");
    service_punch[3] = node_handles[33].serviceClient<dynamixel_controllers::SetCompliancePunch>("/rightshoulderPitchs_controller/set_compliance_punch");
    service_punch[4] = node_handles[34].serviceClient<dynamixel_controllers::SetCompliancePunch>("/rightshoulderRoll_controller/set_compliance_punch");
    service_punch[5] = node_handles[35].serviceClient<dynamixel_controllers::SetCompliancePunch>("/rightelbowPitch_controller/set_compliance_punch");
    service_punch[6] = node_handles[36].serviceClient<dynamixel_controllers::SetCompliancePunch>("/rightelbowRoll_controller/set_compliance_punch");
    service_punch[7] = node_handles[37].serviceClient<dynamixel_controllers::SetCompliancePunch>("/rightwristPitch_controller/set_compliance_punch");
    service_punch[8] = node_handles[38].serviceClient<dynamixel_controllers::SetCompliancePunch>("/rightwristRoll_controller/set_compliance_punch");
    service_punch[9] = node_handles[39].serviceClient<dynamixel_controllers::SetCompliancePunch>("/rightgripper_controller/set_compliance_punch");
    service_punch[10] = node_handles[40].serviceClient<dynamixel_controllers::SetCompliancePunch>("/leftshoulderYawm_controller/set_compliance_punch");
    service_punch[11] = node_handles[41].serviceClient<dynamixel_controllers::SetCompliancePunch>("/leftshoulderYaws_controller/set_compliance_punch");
    service_punch[12] = node_handles[42].serviceClient<dynamixel_controllers::SetCompliancePunch>("/leftshoulderPitchm_controller/set_compliance_punch");
    service_punch[13] = node_handles[43].serviceClient<dynamixel_controllers::SetCompliancePunch>("/leftshoulderPitchs_controller/set_compliance_punch");
    service_punch[14] = node_handles[44].serviceClient<dynamixel_controllers::SetCompliancePunch>("/leftshoulderRoll_controller/set_compliance_punch");
    service_punch[15] = node_handles[45].serviceClient<dynamixel_controllers::SetCompliancePunch>("/leftelbowPitch_controller/set_compliance_punch");
    service_punch[16] = node_handles[46].serviceClient<dynamixel_controllers::SetCompliancePunch>("/leftelbowRoll_controller/set_compliance_punch");
    service_punch[17] = node_handles[47].serviceClient<dynamixel_controllers::SetCompliancePunch>("/leftwristPitch_controller/set_compliance_punch");
    service_punch[18] = node_handles[48].serviceClient<dynamixel_controllers::SetCompliancePunch>("/leftwristRoll_controller/set_compliance_punch");
    service_punch[19] = node_handles[49].serviceClient<dynamixel_controllers::SetCompliancePunch>("/leftgripper_controller/set_compliance_punch");
    service_punch[20] = node_handles[50].serviceClient<dynamixel_controllers::SetCompliancePunch>("/headYaw_controller/set_compliance_punch");
    service_punch[21] = node_handles[51].serviceClient<dynamixel_controllers::SetCompliancePunch>("/headPitch_controller/set_compliance_punch");
    service_punch[22] = node_handles[52].serviceClient<dynamixel_controllers::SetCompliancePunch>("/waist_controller/set_compliance_punch");

    service_slope[0] = node_handles[30].serviceClient<dynamixel_controllers::SetComplianceSlope>("/rightshoulderYawm_controller/set_compliance_slope");
    service_slope[1] = node_handles[31].serviceClient<dynamixel_controllers::SetComplianceSlope>("/rightshoulderYaws_controller/set_compliance_slope");
    service_slope[2] = node_handles[32].serviceClient<dynamixel_controllers::SetComplianceSlope>("/rightshoulderPitchm_controller/set_compliance_slope");
    service_slope[3] = node_handles[33].serviceClient<dynamixel_controllers::SetComplianceSlope>("/rightshoulderPitchs_controller/set_compliance_slope");
    service_slope[4] = node_handles[34].serviceClient<dynamixel_controllers::SetComplianceSlope>("/rightshoulderRoll_controller/set_compliance_slope");
    service_slope[5] = node_handles[35].serviceClient<dynamixel_controllers::SetComplianceSlope>("/rightelbowPitch_controller/set_compliance_slope");
    service_slope[6] = node_handles[36].serviceClient<dynamixel_controllers::SetComplianceSlope>("/rightelbowRoll_controller/set_compliance_slope");
    service_slope[7] = node_handles[37].serviceClient<dynamixel_controllers::SetComplianceSlope>("/rightwristPitch_controller/set_compliance_slope");
    service_slope[8] = node_handles[38].serviceClient<dynamixel_controllers::SetComplianceSlope>("/rightwristRoll_controller/set_compliance_slope");
    service_slope[9] = node_handles[39].serviceClient<dynamixel_controllers::SetComplianceSlope>("/rightgripper_controller/set_compliance_slope");
    service_slope[10] = node_handles[40].serviceClient<dynamixel_controllers::SetComplianceSlope>("/leftshoulderYawm_controller/set_compliance_slope");
    service_slope[11] = node_handles[41].serviceClient<dynamixel_controllers::SetComplianceSlope>("/leftshoulderYaws_controller/set_compliance_slope");
    service_slope[12] = node_handles[42].serviceClient<dynamixel_controllers::SetComplianceSlope>("/leftshoulderPitchm_controller/set_compliance_slope");
    service_slope[13] = node_handles[43].serviceClient<dynamixel_controllers::SetComplianceSlope>("/leftshoulderPitchs_controller/set_compliance_slope");
    service_slope[14] = node_handles[44].serviceClient<dynamixel_controllers::SetComplianceSlope>("/leftshoulderRoll_controller/set_compliance_slope");
    service_slope[15] = node_handles[45].serviceClient<dynamixel_controllers::SetComplianceSlope>("/leftelbowPitch_controller/set_compliance_slope");
    service_slope[16] = node_handles[46].serviceClient<dynamixel_controllers::SetComplianceSlope>("/leftelbowRoll_controller/set_compliance_slope");
    service_slope[17] = node_handles[47].serviceClient<dynamixel_controllers::SetComplianceSlope>("/leftwristPitch_controller/set_compliance_slope");
    service_slope[18] = node_handles[48].serviceClient<dynamixel_controllers::SetComplianceSlope>("/leftwristRoll_controller/set_compliance_slope");
    service_slope[19] = node_handles[49].serviceClient<dynamixel_controllers::SetComplianceSlope>("/leftgripper_controller/set_compliance_slope");
    service_slope[20] = node_handles[50].serviceClient<dynamixel_controllers::SetComplianceSlope>("/headYaw_controller/set_compliance_slope");
    service_slope[21] = node_handles[51].serviceClient<dynamixel_controllers::SetComplianceSlope>("/headPitch_controller/set_compliance_slope");
    service_slope[22] = node_handles[52].serviceClient<dynamixel_controllers::SetComplianceSlope>("/waist_controller/set_compliance_slope");

    //==========================================================================================================

    service_i[0] = node_handles[30].serviceClient<dynamixel_controllers::SetCompliancei>("/rightshoulderYawm_controller/set_compliance_i");
    service_i[1] = node_handles[31].serviceClient<dynamixel_controllers::SetCompliancei>("/rightshoulderYaws_controller/set_compliance_i");
    service_i[2] = node_handles[32].serviceClient<dynamixel_controllers::SetCompliancei>("/rightshoulderPitchm_controller/set_compliance_i");
    service_i[3] = node_handles[33].serviceClient<dynamixel_controllers::SetCompliancei>("/rightshoulderPitchs_controller/set_compliance_i");
    service_i[4] = node_handles[34].serviceClient<dynamixel_controllers::SetCompliancei>("/rightshoulderRoll_controller/set_compliance_i");
    service_i[5] = node_handles[35].serviceClient<dynamixel_controllers::SetCompliancei>("/rightelbowPitch_controller/set_compliance_i");
    service_i[6] = node_handles[36].serviceClient<dynamixel_controllers::SetCompliancei>("/rightelbowRoll_controller/set_compliance_i");
    service_i[7] = node_handles[37].serviceClient<dynamixel_controllers::SetCompliancei>("/rightwristPitch_controller/set_compliance_i");
    service_i[8] = node_handles[38].serviceClient<dynamixel_controllers::SetCompliancei>("/rightwristRoll_controller/set_compliance_i");
    service_i[9] = node_handles[39].serviceClient<dynamixel_controllers::SetCompliancei>("/rightgripper_controller/set_compliance_i");
    service_i[10] = node_handles[40].serviceClient<dynamixel_controllers::SetCompliancei>("/leftshoulderYawm_controller/set_compliance_i");
    service_i[11] = node_handles[41].serviceClient<dynamixel_controllers::SetCompliancei>("/leftshoulderYaws_controller/set_compliance_i");
    service_i[12] = node_handles[42].serviceClient<dynamixel_controllers::SetCompliancei>("/leftshoulderPitchm_controller/set_compliance_i");
    service_i[13] = node_handles[43].serviceClient<dynamixel_controllers::SetCompliancei>("/leftshoulderPitchs_controller/set_compliance_i");
    service_i[14] = node_handles[44].serviceClient<dynamixel_controllers::SetCompliancei>("/leftshoulderRoll_controller/set_compliance_i");
    service_i[15] = node_handles[45].serviceClient<dynamixel_controllers::SetCompliancei>("/leftelbowPitch_controller/set_compliance_i");
    service_i[16] = node_handles[46].serviceClient<dynamixel_controllers::SetCompliancei>("/leftelbowRoll_controller/set_compliance_i");
    service_i[17] = node_handles[47].serviceClient<dynamixel_controllers::SetCompliancei>("/leftwristPitch_controller/set_compliance_i");
    service_i[18] = node_handles[48].serviceClient<dynamixel_controllers::SetCompliancei>("/leftwristRoll_controller/set_compliance_i");
    service_i[19] = node_handles[49].serviceClient<dynamixel_controllers::SetCompliancei>("/leftgripper_controller/set_compliance_i");
    service_i[20] = node_handles[50].serviceClient<dynamixel_controllers::SetCompliancei>("/headYaw_controller/set_compliance_i");
    service_i[21] = node_handles[51].serviceClient<dynamixel_controllers::SetCompliancei>("/headPitch_controller/set_compliance_i");
    service_i[22] = node_handles[52].serviceClient<dynamixel_controllers::SetCompliancei>("/waist_controller/set_compliance_i");

    service_d[0] = node_handles[30].serviceClient<dynamixel_controllers::SetComplianced>("/rightshoulderYawm_controller/set_compliance_d");
    service_d[1] = node_handles[31].serviceClient<dynamixel_controllers::SetComplianced>("/rightshoulderYaws_controller/set_compliance_d");
    service_d[2] = node_handles[32].serviceClient<dynamixel_controllers::SetComplianced>("/rightshoulderPitchm_controller/set_compliance_d");
    service_d[3] = node_handles[33].serviceClient<dynamixel_controllers::SetComplianced>("/rightshoulderPitchs_controller/set_compliance_d");
    service_d[4] = node_handles[34].serviceClient<dynamixel_controllers::SetComplianced>("/rightshoulderRoll_controller/set_compliance_d");
    service_d[5] = node_handles[35].serviceClient<dynamixel_controllers::SetComplianced>("/rightelbowPitch_controller/set_compliance_d");
    service_d[6] = node_handles[36].serviceClient<dynamixel_controllers::SetComplianced>("/rightelbowRoll_controller/set_compliance_d");
    service_d[7] = node_handles[37].serviceClient<dynamixel_controllers::SetComplianced>("/rightwristPitch_controller/set_compliance_d");
    service_d[8] = node_handles[38].serviceClient<dynamixel_controllers::SetComplianced>("/rightwristRoll_controller/set_compliance_d");
    service_d[9] = node_handles[39].serviceClient<dynamixel_controllers::SetComplianced>("/rightgripper_controller/set_compliance_d");
    service_d[10] = node_handles[40].serviceClient<dynamixel_controllers::SetComplianced>("/leftshoulderYawm_controller/set_compliance_d");
    service_d[11] = node_handles[41].serviceClient<dynamixel_controllers::SetComplianced>("/leftshoulderYaws_controller/set_compliance_d");
    service_d[12] = node_handles[42].serviceClient<dynamixel_controllers::SetComplianced>("/leftshoulderPitchm_controller/set_compliance_d");
    service_d[13] = node_handles[43].serviceClient<dynamixel_controllers::SetComplianced>("/leftshoulderPitchs_controller/set_compliance_d");
    service_d[14] = node_handles[44].serviceClient<dynamixel_controllers::SetComplianced>("/leftshoulderRoll_controller/set_compliance_d");
    service_d[15] = node_handles[45].serviceClient<dynamixel_controllers::SetComplianced>("/leftelbowPitch_controller/set_compliance_d");
    service_d[16] = node_handles[46].serviceClient<dynamixel_controllers::SetComplianced>("/leftelbowRoll_controller/set_compliance_d");
    service_d[17] = node_handles[47].serviceClient<dynamixel_controllers::SetComplianced>("/leftwristPitch_controller/set_compliance_d");
    service_d[18] = node_handles[48].serviceClient<dynamixel_controllers::SetComplianced>("/leftwristRoll_controller/set_compliance_d");
    service_d[19] = node_handles[49].serviceClient<dynamixel_controllers::SetComplianced>("/leftgripper_controller/set_compliance_d");
    service_d[20] = node_handles[50].serviceClient<dynamixel_controllers::SetComplianced>("/headYaw_controller/set_compliance_d");
    service_d[21] = node_handles[51].serviceClient<dynamixel_controllers::SetComplianced>("/headPitch_controller/set_compliance_d");
    service_d[22] = node_handles[52].serviceClient<dynamixel_controllers::SetComplianced>("/waist_controller/set_compliance_d");

    service_p[0] = node_handles[30].serviceClient<dynamixel_controllers::SetCompliancep>("/rightshoulderYawm_controller/set_compliance_p");
    service_p[1] = node_handles[31].serviceClient<dynamixel_controllers::SetCompliancep>("/rightshoulderYaws_controller/set_compliance_p");
    service_p[2] = node_handles[32].serviceClient<dynamixel_controllers::SetCompliancep>("/rightshoulderPitchm_controller/set_compliance_p");
    service_p[3] = node_handles[33].serviceClient<dynamixel_controllers::SetCompliancep>("/rightshoulderPitchs_controller/set_compliance_p");
    service_p[4] = node_handles[34].serviceClient<dynamixel_controllers::SetCompliancep>("/rightshoulderRoll_controller/set_compliance_p");
    service_p[5] = node_handles[35].serviceClient<dynamixel_controllers::SetCompliancep>("/rightelbowPitch_controller/set_compliance_p");
    service_p[6] = node_handles[36].serviceClient<dynamixel_controllers::SetCompliancep>("/rightelbowRoll_controller/set_compliance_p");
    service_p[7] = node_handles[37].serviceClient<dynamixel_controllers::SetCompliancep>("/rightwristPitch_controller/set_compliance_p");
    service_p[8] = node_handles[38].serviceClient<dynamixel_controllers::SetCompliancep>("/rightwristRoll_controller/set_compliance_p");
    service_p[9] = node_handles[39].serviceClient<dynamixel_controllers::SetCompliancep>("/rightgripper_controller/set_compliance_p");
    service_p[10] = node_handles[40].serviceClient<dynamixel_controllers::SetCompliancep>("/leftshoulderYawm_controller/set_compliance_p");
    service_p[11] = node_handles[41].serviceClient<dynamixel_controllers::SetCompliancep>("/leftshoulderYaws_controller/set_compliance_p");
    service_p[12] = node_handles[42].serviceClient<dynamixel_controllers::SetCompliancep>("/leftshoulderPitchm_controller/set_compliance_p");
    service_p[13] = node_handles[43].serviceClient<dynamixel_controllers::SetCompliancep>("/leftshoulderPitchs_controller/set_compliance_p");
    service_p[14] = node_handles[44].serviceClient<dynamixel_controllers::SetCompliancep>("/leftshoulderRoll_controller/set_compliance_p");
    service_p[15] = node_handles[45].serviceClient<dynamixel_controllers::SetCompliancep>("/leftelbowPitch_controller/set_compliance_p");
    service_p[16] = node_handles[46].serviceClient<dynamixel_controllers::SetCompliancep>("/leftelbowRoll_controller/set_compliance_p");
    service_p[17] = node_handles[47].serviceClient<dynamixel_controllers::SetCompliancep>("/leftwristPitch_controller/set_compliance_p");
    service_p[18] = node_handles[48].serviceClient<dynamixel_controllers::SetCompliancep>("/leftwristRoll_controller/set_compliance_p");
    service_p[19] = node_handles[49].serviceClient<dynamixel_controllers::SetCompliancep>("/leftgripper_controller/set_compliance_p");
    service_p[20] = node_handles[50].serviceClient<dynamixel_controllers::SetCompliancep>("/headYaw_controller/set_compliance_p");
    service_p[21] = node_handles[51].serviceClient<dynamixel_controllers::SetCompliancep>("/headPitch_controller/set_compliance_p");
    service_p[22] = node_handles[52].serviceClient<dynamixel_controllers::SetCompliancep>("/waist_controller/set_compliance_p");

    //==========================================================================================================

    service_reset_motor = node_handles[70].advertiseService("upperbodycorein_resetmotor", callback_resetmotor);
    service_torque_motor = node_handles[71].advertiseService("upperbodycorein_torquemotor", callback_torquemotor);
    service_pid_motor = node_handles[72].advertiseService("upperbodycorein_pidmotor",callback_pidmotor);

    //===========================================================================================================

    ros::Rate loop_rate(15); //20 Hz


    int dummy[1];
    dummy[11] = 5;

    ros::Rate loop_rate2(0.1); //20 Hz
    loop_rate2.sleep();

    init_config();
   
    cout<<"wait finished"<<endl;

    send_log("upperBody core started done... [V1] [93/07/24]");
    while (ros::ok() && App_exit == false)
    {
        //cout<<"loop"<<endl;
        publish_feedbacks();
        Motor_Update_speed();
        Motor_Update();
        ros::spinOnce();
        loop_rate.sleep();

        send_ack();
    }

    App_exit = true;
    return 0;
}
