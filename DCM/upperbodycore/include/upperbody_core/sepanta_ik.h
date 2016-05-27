
#ifndef _SEPANTA_IK_HPP
#define _SEPANTA_IK_HPP


#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <sepanta_msgs/upperbodymotors.h>
#include <sepanta_msgs/upperbodymotorsfeedback.h>
#include <sepanta_msgs/motorfeedback.h>
#include <sepanta_msgs/motor.h>
#include <sepanta_msgs/motorreset.h>
#include <sepanta_msgs/motorpid.h>
#include <sepanta_msgs/motortorque.h>
#include <sepanta_msgs/grip.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <sepanta_msgs/command.h>
#include <ik/simple_IK.h>
#include <ik/traj_IK.h>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;
using namespace ros;

#define motor_max_speed 800

struct motor_data
{
public:
    int position; //warning this is raw position of motors
    double position_rad_ik;
    int speed;
    int min;
    int max;
    int init;
    ros::Time timestamp;

};

struct motor_range
{
    int max;
    int min;
    int degree;
    string name;
};

inline double Deg2Rad(double deg)
{
    return deg * M_PI / 180;
}

inline double Rad2Deg(double rad)
{
    return rad * 180 / M_PI;
}

class IKSystem
{

public:
IKSystem(ros::NodeHandle n);
ros::Publisher motor_pub[5];
ros::Subscriber sub_handles;
motor_data current_data[5];
std::vector<motor_range> list_motor_configs;
ros::NodeHandle node_handle;

void init_configs();
float convert_motor_to_degree(string model_name,float value);
float convert_degreeMotor_to_degreeIK(int index,float value);
float convert_degreeIK_to_degreeMotor(int index,float value);
int convert_degree_to_motor(string model_name,float value);
void scale_velocity(double * v1,double * v2 , double * v3,int size);
bool convert_all_positions(double * positions,int size);
int convert_radPerSecond_to_speedMotor(double value);
void motors_callback(const sepanta_msgs::upperbodymotorsfeedback::ConstPtr &msg);
void update_arm_motors(int positions[3],int speed[3]);
void update_pen_motor(int position,int speed);
bool SepantaIK(double x0, double y0, double (&q_goal)[3]);
bool open_challange(double q0[3], double x0, double y0, double (&q_goal)[3]);
string go_to_xy(int x,int y,int time);
void init();

};

#endif
