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

bool App_exit = false;
bool IsGoalValid = false;

double const maxLinSpeed_const = 0.6;
double const maxTethaSpeed_const = 0.4;
double maxLinSpeed = maxLinSpeed_const;
double maxTethaSpeed = maxTethaSpeed_const;

nav_msgs::Path globalPath;
short globalPathSize;

ros::Publisher chatter_pub[20];
ros::Publisher mycmd_vel_pub;

double xSpeed=0;
double ySpeed=0;
double tethaSpeed=0;

double const desireErrorX_const = 0.1;
double const desireErrorY_const = 0.1;
double const desireErrorTetha_const = 0.18;
double desireErrorX = desireErrorX_const;
double desireErrorY = desireErrorY_const;
double desireErrorTetha = desireErrorTetha_const;

double errorX = 0;
double errorY = 0;
double errorTetha = 0;
double errorX_R=0;
double errorY_R=0;

double LKp = 3;
double WKp = 1;

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

char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

double GetToPointsAngle(double x1, double y1, double x2, double y2)
{
    return atan2(y2-y1,x2-x1);
}


void PreesKeyToExit()
{
    while (!App_exit)
    {
        char ch = getch();

        if(ch=='x')
        {
            App_exit = true;
        }
    }
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

        if(abs(goalPos[0]-position[0])<=0.02 && abs(goalPos[1]-position[1])<=0.02 && abs(goalTetha-tetha)<=0.036)
        {
            IsGoalValid=false;
            cout<<"Goal reached ..."<<endl;
            send_omni(0,0,0);
            maxLinSpeed = maxLinSpeed_const;
            maxTethaSpeed = maxTethaSpeed_const;
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            continue;
        }

        if(step+20 >= globalPathSize)
        {
            tempGoalTetha = goalTetha;
            if(step+10 >= globalPathSize)
            {
                maxLinSpeed = 0.2;
                maxTethaSpeed = 0.1;
            }
        }
        else
            tempGoalTetha = GetToPointsAngle(position[0], position[1], globalPath.poses[step+10].pose.position.x, globalPath.poses[step+10].pose.position.y);

        // cout<<"GOAL :"<<tempGoalPos[0]<<" "<<tempGoalPos[1]<<endl;
        // cout<<"POSITION :"<<position[0]<<" "<<position[1]<<endl;

        if (tempGoalTetha < 0) tempGoalTetha += 2*M_PI;

        errorX = tempGoalPos[0]-position[0];
        errorY = tempGoalPos[1]-position[1];
        errorTetha = tempGoalTetha-tetha;

        if (errorTetha >= M_PI) errorTetha = errorTetha - 2*M_PI;
        if (errorTetha < -M_PI) errorTetha = 2*M_PI - errorTetha;

        if (errorTetha > 0.833*M_PI) errorTetha = 0.833*M_PI;
        if (errorTetha < -0.833*M_PI) errorTetha = -0.833*M_PI;



        errorX_R = cos(tetha)*errorX+sin(tetha)*errorY;
        errorY_R = -sin(tetha)*errorX+cos(tetha)*errorY;

        if(abs(errorX_R)>abs(desireErrorX))
            xSpeed = (abs(errorX_R*LKp)<=abs(maxLinSpeed))?(errorX_R*LKp):sign(errorX_R)*maxLinSpeed;
        else
            xSpeed = 0;

        if(abs(errorY_R)>abs(desireErrorY))
            ySpeed = (abs(errorY_R*LKp)<=abs(maxLinSpeed))?(errorY_R*LKp):sign(errorY_R)*maxLinSpeed;
        else
            ySpeed = 0;

        if(abs(errorTetha)>abs(desireErrorTetha))
            tethaSpeed = (abs(errorTetha*WKp)<=abs(maxTethaSpeed))?(errorTetha*WKp):sign(errorTetha)*maxTethaSpeed;
        else
            tethaSpeed = 0;


        send_omni(xSpeed,ySpeed,tethaSpeed);

        info_counter++;
        if ( info_counter>50)
        {
            info_counter= 0;
             cout << xSpeed << "\t" << ySpeed << "\t" << tethaSpeed << "\t" << step << "\t" << errorX << "\t" << errorY << "\t" << errorTetha << "\t" << endl;

        }
       
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));

        if(abs(errorX)<=desireErrorX && abs(errorY)<=desireErrorY && abs(errorTetha)<=desireErrorTetha)
        {
            if(step+10 > globalPathSize)
                step = globalPathSize-1;
            else
                step +=10;
        }
    }
}

//void GetVelocity(const dynamixel_msgs::MotorStateList::ConstPtr &msg)
//{

//}


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

//    cout << "Courent Position: " << endl;
//    cout << "\tPosition:\n"<< "X: " << position[0] << "\nY: " << position[1] << endl;
//    cout << "\tOrientation:\n"<< "X: " << orientation[0] << "\nY: " << orientation[1] << "\nZ: " << orientation[2] << "\nW: " << orientation[3] << endl;
//    cout << Quat2Rad(orientation) << endl;
}

void GetPath(const nav_msgs::Path::ConstPtr &msg)
{
    globalPath = *msg;
    globalPathSize = globalPath.poses.size();

    step=0;

    IsGoalValid=true;
//   cout << "TempGoal Position: " << endl;
//    cout << "\tPosition:\n"<< "X: " << tempGoalPos[0] << "\nY: " << tempGoalPos[1] << "\nZ: " << tempGoalPos[2] << endl;
//    cout << tempGoalTetha << endl;
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

//    cout << "Goal Position: " << endl;
//    cout << "\tPosition:\n"<< "X: " << goalPos[0] << "\nY: " << goalPos[1] << endl;
//    cout << "\tOrientation:\n"<< "X: " << orientation[0] << "\nY: " << orientation[1] << "\nZ: " << orientation[2] << "\nW: " << orientation[3] << endl;
//    cout << goalTetha << endl;
}


int main(int argc, char **argv)
{
    cout << "MyMoveBase STARTED ..." << endl;

    ros::init(argc, argv, "mymovebase");

    boost::thread _thread_PathFwr(&PathFwr);

    tf::TransformBroadcaster broadcaster;
    ros::NodeHandle node_handles[50];
    ros::Subscriber sub_handles[15];

    //============================================================================================
    sub_handles[0] = node_handles[0].subscribe("/slam_out_pose", 10, GetPos);
    //============================================================================================
    sub_handles[1] = node_handles[1].subscribe("/move_base/NavfnROS/plan", 10, GetPath);
    //============================================================================================
    sub_handles[2] = node_handles[2].subscribe("/move_base/goal", 10, GetGoal);
    //============================================================================================
    mycmd_vel_pub = node_handles[4].advertise<geometry_msgs::Twist>("sepantamovebase/cmd_vel", 10);
    //============================================================================================
//    sub_handles[2] = node_handles[2].subscribe("/AUTROBOTIN_omnidrive", 10, chatterCallback_omnidrive);
    //============================================================================================
//    chatter_pub_motor[0] = node_handles[3].advertise<std_msgs::Int32>("/m1_controller/command", 10);
//    chatter_pub_motor[1] = node_handles[4].advertise<std_msgs::Int32>("/m2_controller/command", 10);
//    chatter_pub_motor[2] = node_handles[5].advertise<std_msgs::Int32>("/m3_controller/command", 10);
    //============================================================================================
//    odom_pub = node_handles[6].advertise<nav_msgs::Odometry>("odom", 50);

    ros::Rate loop_rate(20);

    while (ros::ok() && App_exit == false)
    {
//	broadcaster.sendTransform(
//      	tf::StampedTransform(
//        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
//        ros::Time::now(),"base_link", "base_laser"));
//        OdomPublisher();

        ros::spinOnce();
        loop_rate.sleep();
    }

    _thread_PathFwr.interrupt();
    _thread_PathFwr.join();


    return 0;
}
