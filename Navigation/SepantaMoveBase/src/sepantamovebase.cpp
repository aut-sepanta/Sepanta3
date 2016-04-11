



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

//=============================================================

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;
using namespace ros;


//MAX SPEED
#define normal_max_linear_speedX  0.4
#define normal_max_linear_speedY  0.33
#define normal_max_angular_speed  0.45
//-
#define goal_max_linear_speedX  0.15
#define goal_max_linear_speedY  0.15
#define goal_max_angular_speed  0.2
//KP
#define normal_kp_linearX 1
#define normal_kp_linearY 0.7
#define norma_kp_angular  1
//-
#define goal_kp_linearX  0.5
#define goal_kp_linearY  0.3
#define goal_kp_angular  0.5
//Ki
#define normal_ki_linearX 0
#define normal_ki_linearY 0
#define normal_ki_angular 0
//-
#define goal_ki_linearX 0
#define goal_ki_linearY 0
#define goal_ki_angular 0
//DESIRE ERRORS
#define normal_desire_errorX 0.1
#define normal_desire_errorY 0.1
#define normal_desire_errorTetha 0.18 // 10 degree
//-
#define goal_desire_errorX 0.02
#define goal_desire_errorY 0.02
#define goal_desire_errorTetha 0.036 // 2 degree

bool App_exit = false;
bool newPath = false;
bool IsGoalValid = false;

double maxLinSpeedX = normal_max_linear_speedX;
double maxLinSpeedY = normal_max_linear_speedY;
double maxTethaSpeed = normal_max_angular_speed;

nav_msgs::Path globalPath;
int globalPathSize;

ros::Publisher mycmd_vel_pub;

double xSpeed=0;
double ySpeed=0;
double tethaSpeed=0;

double desireErrorX = normal_desire_errorX;
double desireErrorY = normal_desire_errorY;
double desireErrorTetha = normal_desire_errorTetha;

double errorX = 0;
double errorY = 0;
double errorTetha = 0;
double errorX_R = 0;
double errorY_R = 0;

double iErrorX = 0;
double iErrorY = 0;
double iErrorTetha = 0;

double LKpX = normal_kp_linearX;
double LKpY = normal_kp_linearY;
double WKp = norma_kp_angular;
double LKiX = normal_ki_linearX;
double LKiY = normal_ki_linearY;
double WKi = normal_ki_angular;

int step = 0;

double position[2] = {0};
double orientation[4] = {0};
double lastPosition[2] = {0};
double lastOrientation[4] = {0};
double tetha = 0;
double lastTetha = 0;

double tempGoalPos[2] = {0};
double tempGoalTetha = 0;

double goalPos[2] = {0};
double goalOri[4] = {0};
double goalTetha = 0;

double maxErrorX = 0, maxErrorY = 0, maxErrorTetha = 0;

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

int sign(double data)
{
    if(data > 0) return 1;
    else if(data < 0) return -1;
    else return 0;
}

int roundData(double data)
{
    if(data>=0)
        return ceil(data);
    else
        return floor(data);
}

double GetToPointsAngle(double x1, double y1, double x2, double y2)
{
    return atan2(y2-y1,x2-x1);
}

void ResetLimits()
{
    desireErrorX = normal_desire_errorX;
    desireErrorY = normal_desire_errorY;
    desireErrorTetha = normal_desire_errorTetha;

    LKpX = normal_kp_linearX;
    LKpY = normal_kp_linearY;
    WKp = norma_kp_angular;

    LKiX = normal_ki_linearX;
    LKiY = normal_ki_linearY;
    WKi = normal_ki_angular;

    maxLinSpeedX = normal_max_linear_speedX;
    maxLinSpeedY = normal_max_linear_speedY;
    maxTethaSpeed = normal_max_angular_speed;
}

void ReduceLimits()
{
    desireErrorX = goal_desire_errorX;
    desireErrorY = goal_desire_errorY;
    desireErrorTetha = goal_desire_errorTetha;

    maxLinSpeedX = goal_max_linear_speedX;
    maxLinSpeedY = goal_max_linear_speedY;
    maxTethaSpeed = goal_max_angular_speed;

    LKpX = goal_kp_linearX;
    LKpY = goal_kp_linearY;
    WKp = goal_kp_angular;

    LKiX = goal_ki_linearX;
    LKiY = goal_ki_linearY;
    WKi = goal_ki_angular;
}

void send_omni(double x,double y ,double w)
{
     geometry_msgs::Twist myTwist;

        myTwist.linear.x = x;
        myTwist.linear.y = -y;
        myTwist.angular.z = -w;

        mycmd_vel_pub.publish(myTwist);
}

int info_counter = 0;
void PathFwr()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    while (!App_exit)
    {
        if ( !IsGoalValid )
    	{
            cout<<"Wait for goal ! ..."<<endl;
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            continue;
    	}

        if(abs(goalPos[0]-position[0])<= desireErrorX && abs(goalPos[1]-position[1])<= desireErrorY && abs(goalTetha-tetha)<= desireErrorTetha)
        {
            IsGoalValid=false;
            cout<<"Goal reached ..."<<endl;

            //return max limitations

            ResetLimits();

            iErrorX = 0;
            iErrorY = 0;
            iErrorTetha = 0;

            send_omni(0,0,0);
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            continue;
        }

        tempGoalPos[0] = globalPath.poses[step].pose.position.x;
        tempGoalPos[1] = globalPath.poses[step].pose.position.y;

        errorX = tempGoalPos[0]-position[0];
        errorY = tempGoalPos[1]-position[1];
        errorTetha = tempGoalTetha-tetha;

        if (errorTetha >= M_PI) errorTetha = errorTetha - 2*M_PI;
        if (errorTetha < -M_PI) errorTetha = 2*M_PI - errorTetha;

        if (errorTetha > 0.833*M_PI) errorTetha = 0.833*M_PI;
        if (errorTetha < -0.833*M_PI) errorTetha = -0.833*M_PI;

        errorX_R = cos(tetha)*errorX+sin(tetha)*errorY;
        errorY_R = -sin(tetha)*errorX+cos(tetha)*errorY;

        iErrorX += errorX_R;
        iErrorY += errorY_R;
        if(abs(errorTetha)<=0.78) iErrorTetha += errorTetha;

        if(abs(errorX_R)>desireErrorX)
            xSpeed = (abs(errorX_R*LKpX+iErrorX*LKiX)<=maxLinSpeedX)?(errorX_R*LKpX+iErrorX*LKiX):sign(errorX_R)*maxLinSpeedX;
        else
        {
            xSpeed = 0;
            iErrorX = 0;
        }

        if(abs(errorY_R)>desireErrorY)
            ySpeed = (abs(errorY_R*LKpY+iErrorY*LKiY)<=maxLinSpeedY)?(errorY_R*LKpY+iErrorY*LKiY):sign(errorY_R)*maxLinSpeedY;
        else
        {
            ySpeed = 0;
            iErrorY = 0;
        }

        if(abs(errorTetha)>desireErrorTetha)
            tethaSpeed = (abs(errorTetha*WKp+iErrorTetha*WKi)<=maxTethaSpeed)?(errorTetha*WKp+iErrorTetha*WKi):sign(errorTetha)*maxTethaSpeed;
        else
        {
            tethaSpeed = 0;
            iErrorTetha = 0;
        }


        send_omni(xSpeed,ySpeed,tethaSpeed);

        info_counter++;
        if ( info_counter>50)
        {
            info_counter= 0;
            cout << xSpeed << "\t" << ySpeed << "\t" << tethaSpeed << "\t" << step << "\t" << errorX << "\t" << errorY << "\t" << errorTetha << endl;
        }
       
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));

        if(abs(errorX_R)<=desireErrorX && abs(errorY_R)<=desireErrorY && abs(errorTetha)<=desireErrorTetha)
        {
            iErrorX = 0;
            iErrorY = 0;
            iErrorTetha = 0;

            if(step+20>=globalPathSize)
            {
                step = globalPathSize-1;

                tempGoalTetha = goalTetha;

                //reduce limita

                 ReduceLimits();
            }
            else
            {
                step +=20;

                tempGoalTetha = GetToPointsAngle(position[0], position[1], globalPath.poses[step].pose.position.x, globalPath.poses[step].pose.position.y);

                if (tempGoalTetha < 0) tempGoalTetha += 2*M_PI;

            }
        }
    }
}

void GetPos(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    lastPosition[0] = position[0];
    lastPosition[1] = position[1];
    lastOrientation[0] = orientation[0];
    lastOrientation[1] = orientation[1];
    lastOrientation[2] = orientation[2];
    lastOrientation[3] = orientation[3];
    lastTetha = tetha;
    position[0] = msg->pose.position.x;
    position[1] = msg->pose.position.y;
    orientation[0] = msg->pose.orientation.x;
    orientation[1] = msg->pose.orientation.y;
    orientation[2] = msg->pose.orientation.z;
    orientation[3] = msg->pose.orientation.w;
    tetha = Quat2Rad(orientation);
}

void GetPath(const nav_msgs::Path::ConstPtr &msg)
{
    globalPath = *msg;
    globalPathSize = globalPath.poses.size();

    step=0;

    IsGoalValid=true;
}

void GetGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr &msg)
{
    goalPos[0] = msg->goal.target_pose.pose.position.x;
    goalPos[1] = msg->goal.target_pose.pose.position.y;
    goalOri[0] = msg->goal.target_pose.pose.orientation.x;
    goalOri[1] = msg->goal.target_pose.pose.orientation.y;
    goalOri[2] = msg->goal.target_pose.pose.orientation.z;
    goalOri[3] = msg->goal.target_pose.pose.orientation.w;
    goalTetha = Quat2Rad(goalOri);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mymovebase");

    ROS_INFO("SepantaMoveBase Version 1.0.3");

    boost::thread _thread_PathFwr(&PathFwr);

    ros::NodeHandle node_handles[10];
    ros::Subscriber sub_handles[5];

    //============================================================================================
    sub_handles[0] = node_handles[0].subscribe("/slam_out_pose", 10, GetPos);
    //============================================================================================
    sub_handles[1] = node_handles[1].subscribe("/move_base/NavfnROS/plan", 10, GetPath);
    //============================================================================================
    sub_handles[2] = node_handles[2].subscribe("/move_base/goal", 10, GetGoal);
    //============================================================================================
    mycmd_vel_pub = node_handles[4].advertise<geometry_msgs::Twist>("sepantamovebase/cmd_vel", 10);
    //============================================================================================

    ros::Rate loop_rate(20);

    while (ros::ok() && App_exit == false)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    _thread_PathFwr.interrupt();
    _thread_PathFwr.join();

    return 0;
}

