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

ros::Publisher chatter_pub_motor_right[right_motor_count];
ros::Publisher chatter_pub_motor_left[left_motor_count];
ros::Publisher chatter_pub_motor_head[head_motor_count];

ros::Publisher chatter_pub_ack;
ros::Publisher chatter_pub_log;

ros::Publisher chatter_pub_motors;
ros::ServiceClient service_speed_motor_right[right_motor_count];
ros::ServiceClient service_speed_motor_left[left_motor_count];
ros::ServiceClient service_speed_motor_head[head_motor_count];

ros::ServiceClient service_torque_motor_right[right_motor_count];
ros::ServiceClient service_torque_motor_left[left_motor_count];
ros::ServiceClient service_torque_motor_head[head_motor_count];

ros::ServiceClient service_margin[total_motor_count];
ros::ServiceClient service_punch[total_motor_count];
ros::ServiceClient service_slope[total_motor_count];

ros::ServiceClient service_i[total_motor_count];
ros::ServiceClient service_d[total_motor_count];
ros::ServiceClient service_p[total_motor_count];

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
    //std_msgs::String msg_log;
    //msg_log.data = msg;
    //chatter_pub_log.publish(msg_log);
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
        g_Motor_left[i] =  motorconfig_list.at(i + right_motor_count).init;
        sp_Motor_left[i] = motorconfig_list.at(i + right_motor_count).speed;
    }

    for ( int i = 0 ; i < head_motor_count ; i++ )
    {
        g_Motor_head[i] =  motorconfig_list.at(i + right_motor_count + left_motor_count).init;
        sp_Motor_head[i] = motorconfig_list.at(i + right_motor_count + left_motor_count).speed;
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

   // load_params_pid();

}

void init_config()
{
    ros::NodeHandle node;
    motor_config motorconfig;
    std::string arg;
    //Right Motors
    //1
    node.getParam("/right1_controller/motor/id",motorconfig.id);
    node.getParam("/right1_controller/joint_name",motorconfig.name);motorconfig.name = "right1";
    node.getParam("/right1_controller/motor/init",motorconfig.init);
    node.getParam("/right1_controller/motor/max",motorconfig.max);
    node.getParam("/right1_controller/motor/min",motorconfig.min);
    node.getParam("/right1_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //5
    node.getParam("/right2_controller/motor/id",motorconfig.id);
    node.getParam("/right2_controller/joint_name",motorconfig.name);motorconfig.name = "right2";
    node.getParam("/right2_controller/motor/init",motorconfig.init);
    node.getParam("/right2_controller/motor/max",motorconfig.max);
    node.getParam("/right2_controller/motor/min",motorconfig.min);
    node.getParam("/right2_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //6
    node.getParam("/right3_controller/motor/id",motorconfig.id);
    node.getParam("/right3_controller/joint_name",motorconfig.name);motorconfig.name = "right3";
    node.getParam("/right3_controller/motor/init",motorconfig.init);
    node.getParam("/right3_controller/motor/max",motorconfig.max);
    node.getParam("/right3_controller/motor/min",motorconfig.min);
    node.getParam("/right3_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //7
    node.getParam("/right4_controller/motor/id",motorconfig.id);
    node.getParam("/right4_controller/joint_name",motorconfig.name);motorconfig.name = "right4";
    node.getParam("/right4_controller/motor/init",motorconfig.init);
    node.getParam("/right4_controller/motor/max",motorconfig.max);
    node.getParam("right4_controller/motor/min",motorconfig.min);
    node.getParam("right4_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //8
    node.getParam("/right5_controller/motor/id",motorconfig.id);
    node.getParam("/right5_controller/joint_name",motorconfig.name);motorconfig.name = "right5";
    node.getParam("/right5_controller/motor/init",motorconfig.init);
    node.getParam("/right5_controller/motor/max",motorconfig.max);
    node.getParam("/right5_controller/motor/min",motorconfig.min);
    node.getParam("/right5_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";

    node.getParam("/right6_controller/motor/id",motorconfig.id);
    node.getParam("/right6_controller/joint_name",motorconfig.name);motorconfig.name = "right6";
    node.getParam("/right6_controller/motor/init",motorconfig.init);
    node.getParam("/right6_controller/motor/max",motorconfig.max);
    node.getParam("/right6_controller/motor/min",motorconfig.min);
    node.getParam("/right6_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";

    //-----------------------------------------------------------------------------
    //Left

    node.getParam("/left1_controller/motor/id",motorconfig.id);
    node.getParam("/left1_controller/joint_name",motorconfig.name);motorconfig.name = "left1";
    node.getParam("/left1_controller/motor/init",motorconfig.init);
    node.getParam("/left1_controller/motor/max",motorconfig.max);
    node.getParam("/left1_controller/motor/min",motorconfig.min);
    node.getParam("/left1_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //5
    node.getParam("/left2_controller/motor/id",motorconfig.id);
    node.getParam("/left2_controller/joint_name",motorconfig.name);motorconfig.name = "left2";
    node.getParam("/left2_controller/motor/init",motorconfig.init);
    node.getParam("/left2_controller/motor/max",motorconfig.max);
    node.getParam("/left2_controller/motor/min",motorconfig.min);
    node.getParam("/left2_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //6
    node.getParam("/left3_controller/motor/id",motorconfig.id);
    node.getParam("/left3_controller/joint_name",motorconfig.name);motorconfig.name = "left3";
    node.getParam("/left3_controller/motor/init",motorconfig.init);
    node.getParam("/left3_controller/motor/max",motorconfig.max);
    node.getParam("/left3_controller/motor/min",motorconfig.min);
    node.getParam("/left3_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //7
    node.getParam("/left4_controller/motor/id",motorconfig.id);
    node.getParam("/left4_controller/joint_name",motorconfig.name);motorconfig.name = "left4";
    node.getParam("/left4_controller/motor/init",motorconfig.init);
    node.getParam("/left4_controller/motor/max",motorconfig.max);
    node.getParam("left4_controller/motor/min",motorconfig.min);
    node.getParam("left4_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";
    //8
    node.getParam("/left5_controller/motor/id",motorconfig.id);
    node.getParam("/left5_controller/joint_name",motorconfig.name);motorconfig.name = "left5";
    node.getParam("/left5_controller/motor/init",motorconfig.init);
    node.getParam("/left5_controller/motor/max",motorconfig.max);
    node.getParam("/left5_controller/motor/min",motorconfig.min);
    node.getParam("/left5_controller/joint_speed",motorconfig.speed);
    arg = "/dynamixel/dx_port/" + boost::lexical_cast<std::string>(motorconfig.id) + "/model_name";
    node.getParam(arg,motorconfig.model);
    if (motorconfig.model == "") motorconfig.model = "none";
    motorconfig.speed *= 10;
    motorconfig_list.push_back(motorconfig);
    motorconfig.name = "";

    node.getParam("/left6_controller/motor/id",motorconfig.id);
    node.getParam("/left6_controller/joint_name",motorconfig.name);motorconfig.name = "left6";
    node.getParam("/left6_controller/motor/init",motorconfig.init);
    node.getParam("/left6_controller/motor/max",motorconfig.max);
    node.getParam("/left6_controller/motor/min",motorconfig.min);
    node.getParam("/left6_controller/joint_speed",motorconfig.speed);
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
    g_Motor_right[0] = msg->position1;  
    g_Motor_right[1] = msg->position2; 
    g_Motor_right[2] = msg->position3;    
    g_Motor_right[3] = msg->position4; 
    g_Motor_right[4] = msg->position5; 
    //==============================================
    sp_Motor_right[0] = msg->speed1;     
    sp_Motor_right[1] = msg->speed2;     
    sp_Motor_right[2] = msg->speed3;
    sp_Motor_right[3] = msg->speed4; 
    sp_Motor_right[4] = msg->speed5;
}

void chatterCallback_left_motors(const sepanta_msgs::upperbodymotors::ConstPtr &msg)
{
    g_Motor_left[0] = msg->position1;  
    g_Motor_left[1] = msg->position2; 
    g_Motor_left[2] = msg->position3;    
    g_Motor_left[3] = msg->position4; 
    g_Motor_left[4] = msg->position5; 
    //==============================================
    sp_Motor_left[0] = msg->speed1;     
    sp_Motor_left[1] = msg->speed2;     
    sp_Motor_left[2] = msg->speed3;
    sp_Motor_left[3] = msg->speed4; 
    sp_Motor_left[4] = msg->speed5;
}

void chatterCallback_head_motors(const sepanta_msgs::upperbodymotors::ConstPtr &msg)
{
    g_Motor_head[0] = msg->head_yaw_position;     //left_shoulder_base1 ?
    g_Motor_head[1] = msg->head_pitch_position;     //left_shoulder_base2 ?
    sp_Motor_head[0] = msg->head_yaw_speed;     //left_shoulder_base1 ?
    sp_Motor_head[1] = msg->head_pitch_speed;     //left_shoulder_base2 ?
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

    //save_params_pid();
}

void Motor_Update()
{
     if ( robot_init == true && pid_init == false )
     {
         pid_init = true;
         for (int i = 0; i < motor_list.size() ; i++)
         {
             motor_data item = motor_list.at(i);
            
                 if ( item.status == "ready" )
                 {
                     set_pid(i,0,16,0);
                     cout<<"SET PID : ["<<i<<"] "<<item.p<<" "<<item.i<<" "<<item.d<<endl;
                     //boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                 }
             
         }
     }
    //-----------------------------------------------------
    //right motors update
    for (int i = 0; i < right_motor_count; i++)
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

    //left motors update
    for (int i = 0; i < left_motor_count; i++)
    {
        
            if ( g_Motortemp_left[i] != g_Motor_left[i] && robot_init == true)
            {
                g_Motortemp_left[i] = g_Motor_left[i];

                std_msgs::Int32 msg;
                msg.data = g_Motor_left[i];

                if ( motor_list.at(i + right_motor_count).status != "not found")
                    chatter_pub_motor_left[i].publish(msg);

                //cout<<"send left"<<i<<" "<<g_Motor_left[i]<<endl;


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

            if ( motor_list.at(i + right_motor_count + left_motor_count).status != "not found")
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

    for (int i = 0; i < left_motor_count; i++)
    {
            float value = 0;
            if ( sp_Motortemp_left[i] != sp_Motor_left[i] && robot_init == true)
            {
                sp_Motortemp_left[i] = sp_Motor_left[i];

                value = ((float)sp_Motor_left[i]) / 1000;
                dynamixel_controllers::SetSpeed speed;
                speed.request.speed = value;

                if ( motor_list.at(i + right_motor_count).status != "not found")
                    service_speed_motor_left[i].call(speed);

                //cout<<"speed left "<<i<<" "<<value<<endl;
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

            if ( motor_list.at(i + right_motor_count + left_motor_count).status != "not found")
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

        if ( motor_list.at(i + right_motor_count).status != "not found")
            service_torque_motor_left[i].call(t_srv);
    }

    for ( int i = 0 ; i < head_motor_count ; i++ )
    {
        dynamixel_controllers::TorqueEnable t_srv;
        t_srv.request.torque_enable = value;

        if ( motor_list.at(i + right_motor_count + left_motor_count).status != "not found")
            service_torque_motor_head[i].call(t_srv);
    }
}

bool callback_resetmotor(sepanta_msgs::motorreset::Request& request, sepanta_msgs::motorreset::Response& response)
{

    send_log("get [reset] request : " + request.id );

    if ( request.id  == "right1" ) reset_right_1();
    if ( request.id  == "right2" ) reset_right_2();
    if ( request.id  == "right3") reset_right_3();
    if ( request.id  == "right4" ) reset_right_4();
    if ( request.id  == "right5") reset_right_5();
    if ( request.id  == "right6") reset_right_gripper();
    if ( request.id  == "rightArm") reset_right_arm();
    if ( request.id  == "left1" ) reset_left_1();
    if ( request.id  == "left2" ) reset_left_2();
    if ( request.id  == "left3") reset_left_3();
    if ( request.id  == "left4" ) reset_left_4();
    if ( request.id  == "left5") reset_left_5();
    if ( request.id  == "left6") reset_left_gripper();
    if ( request.id  == "leftArm") reset_left_arm();
    if ( request.id  == "headYaw" ) reset_head_yaw();
    if ( request.id  == "headPitch" ) reset_head_pitch();
    

    return true;
}

bool callback_pidmotor(sepanta_msgs::motorpid::Request& request, sepanta_msgs::motorpid::Response& response)
{
    send_log("get [pid] request : " + request.id );

    uint8_t margin = request.margin;
    uint8_t slope  = request.slope;
    uint8_t punch = request.punch;

    // if ( request.id  == "right1" ) set_pid(0,margin,slope,punch);
    // if ( request.id  == "right2" ) set_pid(1,margin,slope,punch);
    // if ( request.id  == "right3") set_pid(2,margin,slope,punch);
    // if ( request.id  == "right4" ) set_pid(3,margin,slope,punch);
    // if ( request.id  == "right5") set_pid(4,margin,slope,punch);
    // if ( request.id  == "right6") set_pid(5,margin,slope,punch);
    // if ( request.id  == "left1" ) set_pid(6,margin,slope,punch);
    // if ( request.id  == "left2" ) set_pid(7,margin,slope,punch);
    // if ( request.id  == "left3") set_pid(8,margin,slope,punch);
    // if ( request.id  == "left4" ) set_pid(9,margin,slope,punch);
    // if ( request.id  == "left5") set_pid(10,margin,slope,punch);
    // if ( request.id  == "left6") set_pid(11,margin,slope,punch);
    // if ( request.id  == "headYaw" ) set_pid(12,margin,slope,punch);
    // if ( request.id  == "headPitch" )set_pid(13,margin,slope,punch);
  
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
    sub_handles[1] = node_handles[10].subscribe("upperbodycorein_right_motors", 10, chatterCallback_right_motors); //all right motors
    sub_handles[2] = node_handles[10].subscribe("upperbodycorein_left_motors", 10, chatterCallback_left_motors); //all left motors
    sub_handles[3] = node_handles[10].subscribe("upperbodycorein_head_motors", 10, chatterCallback_head_motors); //all head motors
    sub_handles[4] = node_handles[10].subscribe("upperbodycorein_right_1", 10, chatterCallback_right_1);
    sub_handles[5] = node_handles[10].subscribe("upperbodycorein_right_2", 10, chatterCallback_right_2);
    sub_handles[6] = node_handles[10].subscribe("upperbodycorein_right_3", 10, chatterCallback_right_3);
    sub_handles[7] = node_handles[10].subscribe("upperbodycorein_right_4", 10, chatterCallback_right_4);
    sub_handles[8] = node_handles[10].subscribe("upperbodycorein_right_5", 10, chatterCallback_right_5);
    sub_handles[9] = node_handles[10].subscribe("upperbodycorein_right_6", 10, chatterCallback_right_gripper);
    sub_handles[10] = node_handles[10].subscribe("upperbodycorein_left_1", 10, chatterCallback_left_1);
    sub_handles[11] = node_handles[10].subscribe("upperbodycorein_left_2", 10, chatterCallback_left_2);
    sub_handles[12] = node_handles[10].subscribe("upperbodycorein_left_3", 10, chatterCallback_left_3);
    sub_handles[13] = node_handles[10].subscribe("upperbodycorein_left_4", 10, chatterCallback_left_4);
    sub_handles[14] = node_handles[10].subscribe("upperbodycorein_left_5", 10, chatterCallback_left_5);
    sub_handles[15] = node_handles[10].subscribe("upperbodycorein_left_6", 10, chatterCallback_left_gripper);
    sub_handles[16] = node_handles[10].subscribe("upperbodycorein_head_yaw", 10, chatterCallback_head_yaw);
    sub_handles[17] = node_handles[10].subscribe("upperbodycorein_head_pitch", 10, chatterCallback_head_pitch);
  
    //=================================================================================================================

    chatter_pub_motors = node_handles[5].advertise<sepanta_msgs::upperbodymotorsfeedback>("/upperbodycoreout_feedback", 10);

    chatter_pub_motor_right[0] = node_handles[5].advertise<std_msgs::Int32>("/right1_controller/command", 10);
    chatter_pub_motor_right[1] = node_handles[5].advertise<std_msgs::Int32>("/right2_controller/command", 10);
    chatter_pub_motor_right[2] = node_handles[5].advertise<std_msgs::Int32>("/right3_controller/command", 10);
    chatter_pub_motor_right[3] = node_handles[5].advertise<std_msgs::Int32>("/right4_controller/command", 10);
    chatter_pub_motor_right[4] = node_handles[5].advertise<std_msgs::Int32>("/right5_controller/command", 10);
    chatter_pub_motor_right[5] = node_handles[5].advertise<std_msgs::Int32>("/right6_controller/command", 10);
    chatter_pub_motor_left[0] = node_handles[5].advertise<std_msgs::Int32>("/left1_controller/command", 10);
    chatter_pub_motor_left[1] = node_handles[5].advertise<std_msgs::Int32>("/left2_controller/command", 10);
    chatter_pub_motor_left[2] = node_handles[5].advertise<std_msgs::Int32>("/left3_controller/command", 10);
    chatter_pub_motor_left[3] = node_handles[5].advertise<std_msgs::Int32>("/left4_controller/command", 10);
    chatter_pub_motor_left[4] = node_handles[5].advertise<std_msgs::Int32>("/left5_controller/command", 10);
    chatter_pub_motor_left[5] = node_handles[5].advertise<std_msgs::Int32>("/left6_controller/command", 10);
    chatter_pub_motor_head[0] = node_handles[5].advertise<std_msgs::Int32>("/headYaw_controller/command", 10);
    chatter_pub_motor_head[1] = node_handles[5].advertise<std_msgs::Int32>("/headPitch_controller/command", 10);

    chatter_pub_ack = node_handles[5].advertise<std_msgs::String>("/core_upperbody/ack",10);
    chatter_pub_log = node_handles[5].advertise<std_msgs::String>("/core_upperbody/log",10);
    //========================================================================================================
    //speed service clients

    service_speed_motor_right[0] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/right1_controller/set_speed");
    service_speed_motor_right[1] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/right2_controller/set_speed");
    service_speed_motor_right[2] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/right3_controller/set_speed");
    service_speed_motor_right[3] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/right4_controller/set_speed");
    service_speed_motor_right[4] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/right5_controller/set_speed");
    service_speed_motor_right[5] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/right6_controller/set_speed");
    service_speed_motor_left[0] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/left1_controller/set_speed");
    service_speed_motor_left[1] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/left2_controller/set_speed");
    service_speed_motor_left[2] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/left3_controller/set_speed");
    service_speed_motor_left[3] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/left4_controller/set_speed");
    service_speed_motor_left[4] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/left5_controller/set_speed");
    service_speed_motor_left[5] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/left6_controller/set_speed");
    service_speed_motor_head[0] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/headYaw_controller/set_speed");
    service_speed_motor_head[1] = node_handles[2].serviceClient<dynamixel_controllers::SetSpeed>("/headPitch_controller/set_speed");


    service_torque_motor_right[0] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/right1_controller/torque_enable");
    service_torque_motor_right[1] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/right2_controller/torque_enable");
    service_torque_motor_right[2] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/right3_controller/torque_enable");
    service_torque_motor_right[3] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/right4_controller/torque_enable");
    service_torque_motor_right[4] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/right5_controller/torque_enable");
    service_torque_motor_right[5] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/right6_controller/torque_enable");
    service_torque_motor_left[0] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/left1_controller/torque_enable");
    service_torque_motor_left[1] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/left2_controller/torque_enable");
    service_torque_motor_left[2] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/left3_controller/torque_enable");
    service_torque_motor_left[3] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/left4_controller/torque_enable");
    service_torque_motor_left[4] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/left5_controller/torque_enable");
    service_torque_motor_left[5] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/left6_controller/torque_enable");
    service_torque_motor_head[0] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/headYaw_controller/torque_enable");
    service_torque_motor_head[1] = node_handles[3].serviceClient<dynamixel_controllers::TorqueEnable>("/headPitch_controller/torque_enable");

    //========================================================================================================= PID

    service_margin[0] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/right1_controller/set_compliance_margin");
    service_margin[1] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/right2_controller/set_compliance_margin");
    service_margin[2] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/right3_controller/set_compliance_margin");
    service_margin[3] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/right4_controller/set_compliance_margin");
    service_margin[4] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/right5_controller/set_compliance_margin");
    service_margin[5] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/right6_controller/set_compliance_margin");
    service_margin[6] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/left1_controller/set_compliance_margin");
    service_margin[7] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/left2_controller/set_compliance_margin");
    service_margin[8] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/left3_controller/set_compliance_margin");
    service_margin[9] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/left4_controller/set_compliance_margin");
    service_margin[10] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/left5_controller/set_compliance_margin");
    service_margin[11] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/left6_controller/set_compliance_margin");
    service_margin[12] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/headYaw_controller/set_compliance_margin");
    service_margin[13] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceMargin>("/headPitch_controller/set_compliance_margin");


   
    service_punch[0] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/right1_controller/set_compliance_punch");
    service_punch[1] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/right2_controller/set_compliance_punch");
    service_punch[2] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/right3_controller/set_compliance_punch");
    service_punch[3] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/right4_controller/set_compliance_punch");
    service_punch[4] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/right5_controller/set_compliance_punch");
    service_punch[5] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/right6_controller/set_compliance_punch");
    service_punch[6] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/left1_controller/set_compliance_punch");
    service_punch[7] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/left2_controller/set_compliance_punch");
    service_punch[8] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/left3_controller/set_compliance_punch");
    service_punch[9] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/left4_controller/set_compliance_punch");
    service_punch[10] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/left5_controller/set_compliance_punch");
    service_punch[11] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/left6_controller/set_compliance_punch");
    service_punch[12] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/headYaw_controller/set_compliance_punch");
    service_punch[13] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancePunch>("/headPitch_controller/set_compliance_punch");


    service_slope[0] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/right1_controller/set_compliance_slope");
    service_slope[1] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/right2_controller/set_compliance_slope");
    service_slope[2] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/right3_controller/set_compliance_slope");
    service_slope[3] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/right4_controller/set_compliance_slope");
    service_slope[4] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/right5_controller/set_compliance_slope");
    service_slope[5] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/right6_controller/set_compliance_slope");
    service_slope[6] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/left1_controller/set_compliance_slope");
    service_slope[7] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/left2_controller/set_compliance_slope");
    service_slope[8] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/left3_controller/set_compliance_slope");
    service_slope[9] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/left4_controller/set_compliance_slope");
    service_slope[10] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/left5_controller/set_compliance_slope");
    service_slope[11] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/left6_controller/set_compliance_slope");
    service_slope[12] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/headYaw_controller/set_compliance_slope");
    service_slope[13] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianceSlope>("/headPitch_controller/set_compliance_slope");

   
    //==========================================================================================================

    
    service_p[0] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/right1_controller/set_compliance_p");
    service_p[1] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/right2_controller/set_compliance_p");
    service_p[2] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/right3_controller/set_compliance_p");
    service_p[3] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/right4_controller/set_compliance_p");
    service_p[4] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/right5_controller/set_compliance_p");
    service_p[5] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/right6_controller/set_compliance_p");
    service_p[6] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/left1_controller/set_compliance_p");
    service_p[7] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/left2_controller/set_compliance_p");
    service_p[8] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/left3_controller/set_compliance_p");
    service_p[9] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/left4_controller/set_compliance_p");
    service_p[10] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/left5_controller/set_compliance_p");
    service_p[11] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/left6_controller/set_compliance_p");
    service_p[12] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/headYaw_controller/set_compliance_p");
    service_p[13] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancep>("/headPitch_controller/set_compliance_p");

    service_i[0] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/right1_controller/set_compliance_i");
    service_i[1] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/right2_controller/set_compliance_i");
    service_i[2] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/right3_controller/set_compliance_i");
    service_i[3] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/right4_controller/set_compliance_i");
    service_i[4] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/right5_controller/set_compliance_i");
    service_i[5] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/right6_controller/set_compliance_i");
    service_i[6] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/left1_controller/set_compliance_i");
    service_i[7] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/left2_controller/set_compliance_i");
    service_i[8] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/left3_controller/set_compliance_i");
    service_i[9] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/left4_controller/set_compliance_i");
    service_i[10] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/left5_controller/set_compliance_i");
    service_i[11] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/left6_controller/set_compliance_i");
    service_i[12] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/headYaw_controller/set_compliance_i");
    service_i[13] = node_handles[4].serviceClient<dynamixel_controllers::SetCompliancei>("/headPitch_controller/set_compliance_i");

    service_d[0] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/right1_controller/set_compliance_d");
    service_d[1] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/right2_controller/set_compliance_d");
    service_d[2] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/right3_controller/set_compliance_d");
    service_d[3] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/right4_controller/set_compliance_d");
    service_d[4] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/right5_controller/set_compliance_d");
    service_d[5] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/right6_controller/set_compliance_d");
    service_d[6] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/left1_controller/set_compliance_d");
    service_d[7] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/left2_controller/set_compliance_d");
    service_d[8] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/left3_controller/set_compliance_d");
    service_d[9] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/left4_controller/set_compliance_d");
    service_d[10] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/left5_controller/set_compliance_d");
    service_d[11] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/left6_controller/set_compliance_d");
    service_d[12] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/headYaw_controller/set_compliance_d");
    service_d[13] = node_handles[4].serviceClient<dynamixel_controllers::SetComplianced>("/headPitch_controller/set_compliance_d");

    //==========================================================================================================

    service_reset_motor = node_handles[70].advertiseService("upperbodycorein_resetmotor", callback_resetmotor);
    service_torque_motor = node_handles[71].advertiseService("upperbodycorein_torquemotor", callback_torquemotor);
    service_pid_motor = node_handles[72].advertiseService("upperbodycorein_pidmotor",callback_pidmotor);

    //===========================================================================================================
    cout<<"wait finished 1"<<endl;

    ros::Rate loop_rate(20); //20 Hz
    ros::Rate loop_rate2(0.5); //20 Hz
    loop_rate2.sleep();

    init_config();
   
    cout<<"wait finished 2"<<endl;

    //send_log("upperBody core started done... [V1] [93/07/24]");
    while (ros::ok() && App_exit == false)
    {
        cout<<"loop"<<endl;
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
