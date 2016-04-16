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
#define normal_max_linear_speedX  0.45
#define normal_max_linear_speedY  0.45
#define normal_max_angular_speed  0.5
//-
#define goal_max_linear_speedX  0.15
#define goal_max_linear_speedY  0.15
#define goal_max_angular_speed  0.2
//KP
#define normal_kp_linearX 0.4
#define normal_kp_linearY 0.4
#define norma_kp_angular  0.8
//-
#define goal_kp_linearX  0.3
#define goal_kp_linearY  0.3
#define goal_kp_angular  0.6
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
#define goal_desire_errorX 0.035
#define goal_desire_errorY 0.035
#define goal_desire_errorTetha 0.04 // 2 degree

std::string coutcolor0 = "\033[0;0m";
std::string coutcolor_red = "\033[0;31m";
std::string coutcolor_green = "\033[0;32m";
std::string coutcolor_blue = "\033[0;34m";
std::string coutcolor_magenta = "\033[0;35m";
std::string coutcolor_brown = "\033[0;33m";

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

double LKpX = normal_kp_linearX;
double LKpY = normal_kp_linearY;
double WKp = norma_kp_angular;
double LKiX = normal_ki_linearX;
double LKiY = normal_ki_linearY;
double WKi = normal_ki_angular;

int step = 0;

double position[2] = {0};
double orientation[4] = {0};
double tetha = 0;

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

bool isvirtual = true;
void send_omni(double x,double y ,double w)
{
     geometry_msgs::Twist myTwist;

        if ( isvirtual ) 
        {
            myTwist.linear.x = x;
            myTwist.linear.y = y;
            myTwist.angular.z = w; 
        }
        else
        {
            myTwist.linear.x = x;
            myTwist.linear.y = -y;
            myTwist.angular.z = -w;     
        }
  mycmd_vel_pub.publish(myTwist); 

}

int info_counter = 0;
bool get_goal = false;
int temp_path_size = 0;

void force_stop()
{
    send_omni(0,0,0);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}

int system_state = 0;

//0 => wait for goal
//2 => turn to target
//4 => go on path
//6 => turn to goal
//8 => reached

bool on_the_goal = false;
int step_size  = 40;
int calc_next_point()
{
            bool isgoalnext = false;
            if ( step == globalPathSize-1)
            {
                on_the_goal = true;
                return true;
            }
                
            if(step+step_size >= globalPathSize)
            {
                step = globalPathSize - 1;
                //we are very near to goal so next is the goalstep = globalPathSize-1;
                tempGoalPos[0] = goalPos[0];
                tempGoalPos[1] = goalPos[1];
                tempGoalTetha = goalTetha;
                if (tempGoalTetha < 0) tempGoalTetha += 2*M_PI;

                isgoalnext = true;

                cout<<coutcolor_magenta<<"goal calc"<<coutcolor0<<endl;

               
            }
            else
            {
                //select the point in next 20 step and calc all errors
                step +=step_size;

                tempGoalPos[0] = globalPath.poses[step].pose.position.x;
                tempGoalPos[1] = globalPath.poses[step].pose.position.y;
                tempGoalTetha = GetToPointsAngle(position[0], position[1], globalPath.poses[step].pose.position.x, globalPath.poses[step].pose.position.y);
                if (tempGoalTetha < 0) tempGoalTetha += 2*M_PI;


                 cout<<coutcolor_magenta<<"step calc"<<coutcolor0<<endl;
                 
            }

           

            return isgoalnext;
}

void errors_update()
{
            //calc errorX , errorY , errorTetha 
            errorX = tempGoalPos[0]-position[0];
            errorY = tempGoalPos[1]-position[1];
            errorTetha = tempGoalTetha-tetha;

            if (errorTetha >= M_PI) errorTetha = 2*M_PI - errorTetha;
            if (errorTetha < -M_PI) errorTetha = errorTetha + 2*M_PI;
            //?
            //if (errorTetha > 0.833*M_PI) errorTetha = 0.833*M_PI;
            //if (errorTetha < -0.833*M_PI) errorTetha = -0.833*M_PI;

            errorX_R = cos(tetha)*errorX+sin(tetha)*errorY;
            errorY_R = -sin(tetha)*errorX+cos(tetha)*errorY;
}

void publish_info()
{
        info_counter++;
        if ( info_counter>50)
        {
            info_counter= 0;

            cout << "Speed: " << xSpeed << " - " << ySpeed << " - " << tethaSpeed << endl;
            cout << "Step: " << step << endl;
            cout << "TError: " << errorX << " - " << errorY << " - " << errorTetha << endl;
            cout << "Goal: " << fabs(goalPos[0]-position[0]) << " - " << fabs(goalPos[1]-position[1]) << " - " << fabs(goalTetha-tetha)<< endl; 
        }
}

void controller_update(int x,bool y,bool theta)
{
    if ( x == 1)
    xSpeed = (fabs(errorX_R*LKpX)<=maxLinSpeedX)?(errorX_R*LKpX):sign(errorX_R)*maxLinSpeedX;
    else if ( x == 0)
    xSpeed = 0;
    else if ( x == 2)
    xSpeed = 0.2;

    if ( y )
    ySpeed = (fabs(errorY_R*LKpY)<=maxLinSpeedY)?(errorY_R*LKpY):sign(errorY_R)*maxLinSpeedY;
    else
    ySpeed = 0;

    if ( theta )
    tethaSpeed = (fabs(errorTetha*WKp)<=maxTethaSpeed)?(errorTetha*WKp):sign(errorTetha)*maxTethaSpeed;
    else
    tethaSpeed = 0;

    send_omni(xSpeed,ySpeed,tethaSpeed); 
}

void PathFwr()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    force_stop();
   
    while (!App_exit)
    {

      
        if ( system_state == 0)
        {
            cout<<"Wait for goal ! ... "<<endl;
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

            if ( IsGoalValid && get_goal)
            {
                    IsGoalValid = false;
                    get_goal = false;
                    on_the_goal = false;
                    //get next point
                    //if next is mid point turn to it (State=1)
                    //id next is the goal go for it on path (State=3)
                    bool resutl = calc_next_point();
                    if ( resutl)
                    {
                    	//next is the goal !
                    	cout<<"Next is goal =>3"<<endl;
                    	system_state = 3;
                    }
                    else
                    {
                    	cout<<"Next is step =>1"<<endl;
                        system_state = 1;
                    }
                    
            }
        }
        else
        if ( system_state == 1)
        {
           cout<<"State = 1 -turn to target-"<<endl;
           system_state = 2;
        }
        else
        if ( system_state == 2)
        {
            //turn to goal <loop>
          
            if(fabs(errorTetha)<=desireErrorTetha)
            {
                
                cout<<"DONE ! "<<tetha<<" "<<tempGoalTetha<<" "<<errorTetha<<endl;
                system_state = 3;
                force_stop();
                
            }
            else
            {
                controller_update(0,false,true);
            }

        }
        else
        if ( system_state == 3)
        {
           cout<<"State = 3 -go on path- Step = "<<step<<endl;
           system_state = 4;
        }
        else
        if ( system_state == 4)
        {
           
            if(fabs(errorX_R)<=desireErrorX && fabs(errorY_R)<=desireErrorY && fabs(errorTetha)<=desireErrorTetha)
            {
                bool resutl = calc_next_point();
                if ( resutl )
                {
                    if ( on_the_goal )
                    {
                        system_state = 5;
                    }
                    else
                    {
                        system_state = 3;
                    }
                    
                }
                else
                {
                    cout<<"Temp point reached"<<endl;
                    system_state = 3;
                }
            }
            else
            {
            	if ( step < 40 || (globalPathSize - step) < 40 )
                controller_update(1,true,true); //p
                else
                controller_update(2,false,true); //fixed
            }
        }
        else
        if ( system_state == 5)
        {
            cout<<"State = 5 -turn to goal-"<<endl;
            system_state = 6;
        }
        else
        if ( system_state == 6)
        {
           
            if(fabs(errorTetha)<=desireErrorTetha)
            {
                
                system_state = 7;
                force_stop();
                
            }
            else
            {
                controller_update(0,false,true);
            }
        }
        else
        if ( system_state == 7)
        {
            cout<<"State = 7 -goal reached-"<<endl;
            system_state = 8;
        }
        else
        if ( system_state == 8)
        {
            cout<<"Finished !"<<endl;
            temp_path_size = 0;
            get_goal = false;
            IsGoalValid = false;
            on_the_goal = false;
            force_stop();
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            system_state = 0;
        }

        errors_update();
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    }
}

void GetPos(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    position[0] = msg->pose.position.x;
    position[1] = msg->pose.position.y;
    orientation[0] = msg->pose.orientation.x;
    orientation[1] = msg->pose.orientation.y;
    orientation[2] = msg->pose.orientation.z;
    orientation[3] = msg->pose.orientation.w;
    tetha = Quat2Rad(orientation);
    if (tetha < 0) tetha += 2*M_PI;
}

void GetPath(const nav_msgs::Path::ConstPtr &msg)
{
       

        if ( temp_path_size == 0 && get_goal && msg->poses.size() > 20)
        {
        //this is a new path reset all states

        temp_path_size = msg->poses.size();
        cout<<coutcolor_green<<"get a new PATH from GPLANNER Points : "<< temp_path_size <<coutcolor0<<endl;
        globalPath = *msg;
        globalPathSize = globalPath.poses.size();
        system_state = 0;
        IsGoalValid=true;
        on_the_goal = false;
        step=0;

        }
        //system_state = 0;
        //on_the_goal = false;
        //force_stop();
    
}

void GetGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr &msg)
{
    get_goal = true;
    system_state = 0;
    IsGoalValid=false;
    on_the_goal = false;
    step = 0;
    force_stop();
    temp_path_size = 0;
    globalPathSize = 0;
   

    cout<<coutcolor_green<<"get a new GOAL from USER " <<coutcolor0<<endl;
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

