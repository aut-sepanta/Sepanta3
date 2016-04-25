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
//=============================================================

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


//#define VIRTUALMODE
 int tempLogicState;
 int tempSystemState;


#ifdef VIRTUALMODE
//MAX SPEED
#define normal_max_linear_speedX 0.45
#define normal_max_linear_speedY 0.45
#define normal_max_angular_speed 0.6
//KP
#define normal_kp_linearX 0.4
#define normal_kp_linearY 0.4
#define norma_kp_angular 1.5
#endif

#ifndef VIRTUALMODE //real robot
#define normal_max_linear_speedX 0.45
#define normal_max_linear_speedY 0.45
#define normal_max_angular_speed 0.45
//KP
#define normal_kp_linearX 0.4
#define normal_kp_linearY 0.4
#define norma_kp_angular 1
#endif

//-
#define goal_max_linear_speedX 0.2
#define goal_max_linear_speedY 0.2
#define goal_max_angular_speed 0.2
//-
#define goal_kp_linearX 0.3
#define goal_kp_linearY 0.3
#define goal_kp_angular 0.6
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
//=============================================================

std::string coutcolor0 = "\033[0;0m";
std::string coutcolor_red = "\033[0;31m";
std::string coutcolor_green = "\033[0;32m";
std::string coutcolor_blue = "\033[0;34m";
std::string coutcolor_magenta = "\033[0;35m";
std::string coutcolor_brown = "\033[0;33m";

//=============================================================

bool App_exit = false;
bool IsCmValid = false;
bool IsGoalReached = false;
bool IsRecoveryState = false;
bool IsHectorReset = false;

ros::ServiceClient client_makeplan;
ros::ServiceClient client_resetcostmap;
ros::ServiceClient client_map_save;
ros::ServiceClient client_map_load;

double maxLinSpeedX = normal_max_linear_speedX;
double maxLinSpeedY = normal_max_linear_speedY;
double maxTethaSpeed = normal_max_angular_speed;

ros::Publisher pub_slam_origin;
ros::Publisher pub_slam_reset;

nav_msgs::Path globalPath;
int globalPathSize = 0;
nav_msgs::Path tempPath;
int temp_path_size = 0;

nav_msgs::OccupancyGrid costmap;

ros::Publisher mycmd_vel_pub;
ros::Publisher pub_tts;
ros::Publisher pub_current_goal;
ros::Publisher pub_move;

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
double oldposition[2] = {0};
double orientation[4] = {0};
double tetha = 0;
double oldtetha = 0;

double tempGoalPos[2] = {0};
double tempGoalTetha = 0;

double goalPos[2] = {0};
double goalOri[4] = {0};
double goalTetha = 0;

struct goal_data
{
  public :
    int x; //cm
    int y; //cm
    int yaw; //angle - degree
    int height;
    string id;
};

geometry_msgs::PoseStamped  target_goal_stamped;
goal_data target_goal_data;

float distacne_to_goal = 0;

double maxErrorX = 0, maxErrorY = 0, maxErrorTetha = 0;
std::vector<goal_data> goal_list;


int info_counter = 0;
bool say_enable = true;

int system_state = 0;
int logic_state = 0;
bool on_the_goal = false;
int step_size  = 40;
bool wait_flag = false;
bool idle_flag = false;
bool isrobotmove = false;

inline double Deg2Rad(double deg)
{
    return deg * M_PI / 180;
}

inline double Rad2Deg(double rad)
{
    return rad * 180 / M_PI;
}

void publish_isrobotmove()
{
    std_msgs::Bool _msg;
    _msg.data = isrobotmove;
    pub_move.publish(_msg);
}

void say_message(string data)
{
    if ( say_enable == false ) return;
    std_msgs::String _mes;
    _mes.data = data;
    pub_tts.publish(_mes);
}

void send_omni(double x,double y ,double w)
{
        geometry_msgs::Twist myTwist;



        #ifdef VIRTUALMODE
       
            myTwist.linear.x = x;
            myTwist.linear.y = y;
            myTwist.angular.z = w;
        #else
            myTwist.linear.x = x;
            myTwist.linear.y = -y;
            myTwist.angular.z = -w;
        #endif
       
        mycmd_vel_pub.publish(myTwist); 

}


void force_stop()
{
    send_omni(0,0,0);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}

double GetDistance(double x1, double y1, double x2, double y2)
{
    double x = x2-x1;
    double y = y2-y1;
    return sqrt(x*x + y*y);
}

int GetCurrentStep()
{
    for(int i=0;i<globalPathSize-1;i++)
    {
        if(GetDistance(position[0],position[1],globalPath.poses[i].pose.position.x, globalPath.poses[i].pose.position.y) < GetDistance(position[0],position[1],globalPath.poses[i+1].pose.position.x, globalPath.poses[i+1].pose.position.y))
            return i;
    }
    return globalPathSize-1;
}

void sepantamapengine_savemap()
{
   std_srvs::Empty _s;
   client_map_save.call(_s);
}

void sepantamapengine_loadmap()
{
   std_srvs::Empty _s;
   client_map_load.call(_s);
}

void clean_costmaps()
{
   std_srvs::Empty _s;
   client_resetcostmap.call(_s);
}

//cm cm degree
void update_hector_origin(float x,float y,float yaw)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    pub_slam_origin.publish(msg);
}

void reset_hector_slam()
{
	std_msgs::String _msg;
	_msg.data = "reset";
	pub_slam_reset.publish(_msg);
}


void read_file()
{

        std::string path_points =  ros::package::getPath("managment") + "/maps/points.txt";
        //cout<<path_points<<endl;
        std::string line;
        std::ifstream text;

        goal_list.clear();
        text.open(path_points.c_str(), ios_base::in);

        if (text.is_open())
        {
        	
            getline(text,line);

            while (text.good())
            {
                vector <string> fields;

                boost::split( fields, line, boost::is_any_of( "," ) );

                goal_data gdata;

                 gdata.id = fields[0].c_str();
                 gdata.x = atoi(fields[1].c_str());
                 gdata.y = atoi(fields[2].c_str());
                 gdata.yaw = atoi(fields[3].c_str());
                 gdata.height = atoi(fields[4].c_str());

                goal_list.push_back(gdata);
               
                getline(text,line);

            }
            text.close();

        }
        else
        {
            std::cout << coutcolor_blue << "[Unable to open file]" << coutcolor0 <<std::endl << std::endl;
        }

        std::cout << coutcolor_blue << "read done : " << coutcolor0 << goal_list.size()<<std::endl << std::endl;

}

int find_goal_byname(string name)
{
    for ( int i = 0 ; i < goal_list.size() ; i++ )
    {
        if ( name == goal_list.at(i).id )
        {
           return i;
        }
    }

    return -1;
}


nav_msgs::Path call_make_plan()
{
    nav_msgs::GetPlan srv;

    srv.request.start.header.frame_id = "map";
    srv.request.start.pose.position.x = position[0];
    srv.request.start.pose.position.y = position[1];
    srv.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(tetha);

    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = goalPos[0];
    srv.request.goal.pose.position.y = goalPos[1];
    srv.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(goalTetha);

    srv.request.tolerance = 0.1;
    client_makeplan.call(srv);

    return srv.response.plan;
}

void logic_thread()
{
    nav_msgs::Path result;

    while(ros::ok() && !App_exit)
    {
         boost::this_thread::sleep(boost::posix_time::milliseconds(500));


         if ( logic_state == -1)
         {
            cout<<"Get Error With Hector Status"<<endl;
            say_message("There is a problem with my laser");
           
            reset_hector_slam();
            update_hector_origin(position[0],position[1],tetha);
            boost::this_thread::sleep(boost::posix_time::milliseconds(7000));   
            clean_costmaps();     
            say_message("My laser recovered successfuly");
            boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
            system_state = tempSystemState;
            logic_state = tempLogicState;

         }
         if ( logic_state == 0 )
         {
            IsGoalReached = false;
            if ( idle_flag == false )
            {
                idle_flag = true;
                cout<<"wait for exe , idle"<<endl;
            }
            
         }

         if ( logic_state == 1 )
         {
               idle_flag = false;
               //operation loop
               cout<<coutcolor_green<<"Planning... " <<coutcolor0<<endl;
               result = call_make_plan();
               logic_state = 2;
         }

        if ( logic_state == 2 )
        {
            //check the plan
            cout<<"Check the plan from global planner"<<endl;
           
            if( result.poses.size() == 0 )
            {
                    cout<<coutcolor_red<<"Error in PATH ! "<<coutcolor0<<endl;
                    if(!IsRecoveryState)
                    say_message("Error in path generation");
                    logic_state = 3;   //recovery
                    system_state = -1; //wait
                    force_stop();
            }
            else
            {
                globalPath = result;
                globalPathSize = globalPath.poses.size();
                cout<<coutcolor_green<<"get a new PATH from GPLANNER Points : "<< globalPathSize <<coutcolor0<<endl;
                system_state = 1;
                logic_state = 4; //valid
                force_stop();   

            }
        }

        if ( logic_state == 3 )
        {
        	say_message("Let me think");
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            cout<<coutcolor_red<<" Recovery state " <<coutcolor0<<endl;
            //path error handler 
            //revocery state
           
            if(IsRecoveryState)
            {
            	if(IsHectorReset)
            	{
            		IsRecoveryState = false;
	            	IsHectorReset = false;
	            	say_message("Goal is unreachable");
	            	system_state = 0;
	            	logic_state = 0;
            	}
            	else
            	{
            		IsHectorReset = true;
	            	say_message("reseting hector");
            		reset_hector_slam();
            		update_hector_origin(position[0],position[1],tetha);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                    clean_costmaps();   
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            		logic_state = 1;
	            }

            }
            else
            {
            	IsRecoveryState = true;
            	say_message("reseting cost map");
            	clean_costmaps();
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            	logic_state = 1;
         	}
             
        }

        if ( logic_state == 4 ) //controlling mode
        {
        	IsRecoveryState = false;
        	IsHectorReset = false;
             cout<<coutcolor_magenta<<" logic_state == 4 " <<coutcolor0<<endl;
           if ( IsGoalReached == true)
           {
               cout<<coutcolor_red<<" Goal reached " <<coutcolor0<<endl;
               logic_state = 0;
               continue;
           }
            
            result = call_make_plan();
            if( result.poses.size() != 0 )
             {
                int currentStep = GetCurrentStep();
                for(int i=0;i<result.poses.size()-1 && i+currentStep<globalPathSize-1;i++)
                {
                    if(GetDistance(result.poses[i].pose.position.x, result.poses[i].pose.position.y,globalPath.poses[i+currentStep].pose.position.x, globalPath.poses[i+currentStep].pose.position.y)>0.2)
                    {
                        globalPath = result;
                        globalPathSize = result.poses.size();
                        system_state = 1;
                        cout<<coutcolor_red<<"PATH changed : "<< globalPathSize <<coutcolor0<<endl;
                        break;
                    }
                }

             }
             else
             {
                    cout<<coutcolor_red<<"Error in PATH ! "<<coutcolor0<<endl;
                    logic_state = 2; //invalid
             }
        

        }



    }
}


void exe_slam(goal_data g)
{
    bool valid_point = false;
    //this function get goal and move robot to there
    if ( g.id == "")
    {
       goalPos[0] = (float)(g.x) / 100; //cm => m
       goalPos[1] = (float)(g.y) / 100; //cm => m
       goalTetha = Deg2Rad(g.yaw); //rad => deg

       valid_point = true;
       ROS_INFO("EXECUTE MANUAL POINT");
    }
    else
    {
       int index = find_goal_byname(g.id);
       if ( index != -1)
       {
           goal_data data = (goal_data)goal_list.at(index);
           goalPos[0] = (float)(data.x)/ 100;
           goalPos[1] = (float)(data.y)/ 100;
           goalTetha = Deg2Rad(data.yaw);

           valid_point = true;
           ROS_INFO("EXECUTE LOCAL POINT");
       }
       else
       {
           valid_point = false;
           ROS_ERROR("LOCAL POINT NOT FOUND");
       }
    }

    if ( valid_point && logic_state == 0)
    {
       
        say_message("Got a new goal");
        logic_state = 1;
    }


   
}

void exe_cancle()
{
     force_stop();
     say_message("cancel requested , operation canceled!");
     logic_state = 0;
     system_state = 0;
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








//0 => wait for goal
//2 => turn to target
//4 => go on path
//6 => turn to goal
//8 => reached


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
                //on_the_goal = true;

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

            if (errorTetha >= M_PI) errorTetha =  errorTetha - 2*M_PI;
            if (errorTetha < -M_PI) errorTetha =  errorTetha + 2*M_PI;
            //?
            if (errorTetha > 0.833*M_PI) errorTetha = 0.833*M_PI;
            if (errorTetha < -0.833*M_PI) errorTetha = -0.833*M_PI;

            errorX_R = cos(tetha)*errorX+sin(tetha)*errorY;
            errorY_R = -sin(tetha)*errorX+cos(tetha)*errorY;

            float x1 = position[0];
            float y1 = position[1];
            float x2 = goalPos[0];
            float y2 = goalPos[1];
            float d_1 = (x2 - x1); 
            float d_2 = (y2 - y1);
            distacne_to_goal = d_1 * d_1 + d_2 * d_2;
            distacne_to_goal = sqrt(distacne_to_goal);
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
    xSpeed = 0.3;

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
   
    #ifndef VIRTUALMODE
    say_message("Sepanta Move Base Started");
    #else
    say_message("Sepanta Move Base Started in virtual mode");
    #endif

    while (ros::ok() && !App_exit)
    {
        errors_update();

        if ( system_state == 0 )
        {
            isrobotmove = 0;
        }
        else
        {
            isrobotmove = 1;
        }

        if ( system_state == -1) //wait state
        {

           boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
        if ( system_state == 0)
        {

            if ( wait_flag == false)
            {
               say_message("I am waiting for new goal");
               cout<< coutcolor_green <<"Wait for goal ! ... "<< coutcolor0 <<endl;
               wait_flag = true;
            }
          
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
        else
        if ( system_state == 1)
        {
           force_stop();
           IsGoalReached = false;
           on_the_goal = false;
           ResetLimits();
           step = 0;
           wait_flag = false;
           cout<<"State = 1 -turn to target-"<<endl;

            bool resutl = calc_next_point();
            if ( resutl)
            {
                //next is the goal !
                cout<<"Next is goal =>3"<<endl;
                system_state = 3;
            }
            else
            {
                cout<<"Next is step =>2"<<endl;
                system_state = 2;
            }

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
                	//next is the goal
                	ReduceLimits();

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
                controller_update(1,true,true); //p (X,Y,T)
                else 
                {
                	if ( fabs(errorTetha)<=desireErrorTetha )
                	{
                		controller_update(2,true,true); //fixed (F,Y,T)
                	}
                	else
                	{
                		//if we are going away ! 
                		controller_update(1,true,true); //p  fixed (X,Y,T)
                	}
                }

                	
                
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
                cout<<"DONE ! "<<tetha<<" "<<tempGoalTetha<<" "<<errorTetha<<endl;
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
            say_message("Goal reached");
            IsGoalReached = true;

            on_the_goal = false;
            wait_flag = false;
            force_stop();
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            system_state = 0;
        }

       
        //publish_info();
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    }
}



void GetCostmap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	if(!IsCmValid)
	{
		costmap = *msg;
		IsCmValid = true;
	}
}



bool calc_error(double x1,double y1,double t1,double x2,double y2,double t2,double delta_t)
{
   bool valid = true;
   double v1 = fabs(x2-x1) / delta_t;
   double v2 = fabs(y2-y1) / delta_t;

   double dt = t2-t1;
   if (dt >= M_PI) dt =  dt - 2*M_PI;
   if (dt < -M_PI) dt =  dt + 2*M_PI;

   double v3 = fabs(dt) / delta_t;

   if ( v1 > maxLinSpeedX + 0.1 ) valid = false;
   else if ( v2 > maxLinSpeedY + 0.1 ) valid = false;
   else if ( v3 > maxTethaSpeed + 0.1 ) valid = false;

   // cout<<v1<<" "<<v2<<" "<<v3<<" "<<delta_t<<endl;

   return valid;
}

void hector_problem_detected()
{
    if ( logic_state != -1)
    {
        
            tempLogicState = logic_state;
            logic_state = -1;

            tempSystemState = system_state;
            system_state = -1;

            force_stop();
        }
}

ros::Time old_time;

void GetPos(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    oldposition[0] = msg->pose.position.x;
    oldposition[1] = msg->pose.position.y;
    orientation[0] = msg->pose.orientation.x;
    orientation[1] = msg->pose.orientation.y;
    orientation[2] = msg->pose.orientation.z;
    orientation[3] = msg->pose.orientation.w;
    oldtetha = Quat2Rad(orientation);
    if (oldtetha < 0) oldtetha += 2*M_PI;

    ros::Duration _delta_t = msg->header.stamp - old_time; //delta t in sec
    double delta_t = _delta_t.toSec();

    //cout<<"Temp POSE : "<<oldposition[0]<<" "<<oldposition[1]<<" "<<oldtetha<<endl;
    //cout<<"POSITION"<<oldposition[0]*100<<" | "<<oldposition[1]*100<<" | "<<Rad2Deg(oldtetha)<<endl;
    bool valid = calc_error(oldposition[0],oldposition[1],oldtetha,position[0],position[1],tetha,delta_t);
    if ( true )
    {
        position[0] = oldposition[0];
        position[1] = oldposition[1];
        tetha = oldtetha;
    }
    else
    {
        cout<<"Keeped POSE : "<<position[0]<<" "<<position[1]<<" "<<tetha<<endl;
        if( logic_state != 3) //if we are not in revcovery state and this is not caused by out request or hector reseting
        {
            cout<<coutcolor_red<<"problem with position"<<coutcolor0<<endl;
            //hector_problem_detected();
        }

       
    }

    old_time = msg->header.stamp;
    //=====================================


       
    
    
}




void CheckHectorStatus(const std_msgs::Bool::ConstPtr &msg)
{
   
    if(msg->data == false)
    {
       
        hector_problem_detected();
        
    }
}

bool checkcommand(sepanta_msgs::command::Request  &req,sepanta_msgs::command::Response &res)
{

	ROS_INFO("Service Request....");

    std::string _cmd = req.command;
    std::string _id = req.id;
    int _value1 = req.value1;
    int _value2 = req.value2;
    int _value3 = req.value3;

    if ( _cmd == "reload_points")
    {
    	read_file();
    }

    if ( _cmd == "save_map")
    {
    	sepantamapengine_savemap();
    }

    if ( _cmd == "load_map")
    {
    	sepantamapengine_loadmap();
    }

    if ( _cmd == "exe")
    {

        goal_data g;
        g.id = _id;
        g.x = _value1;
        g.y = _value2;
        g.yaw = _value3;

        exe_slam(g);
    }

    if ( _cmd == "cancle")
    {
        exe_cancle();
    }

    if ( _cmd == "reset_hector")
    {
        reset_hector_slam();
    }

    if ( _cmd == "update_hector_origin")
    {

        update_hector_origin(0,0,0);
    }

    res.result = "done";
    return true;
}



float f = 0.0;
ros::Publisher marker_pub;
ros::Publisher marker_pub2;
ros::Publisher marker_pub3;

void test_vis()
{

    visualization_msgs::Marker points , points2 , points3;

    points.header.frame_id =  "map";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.g = 1.0f;
    points.color.a = 1.0;

    points2.header.frame_id =  "map";
    points2.header.stamp = ros::Time::now();
    points2.action = visualization_msgs::Marker::ADD;
    points2.id = 1;
    points2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    points2.scale.x = 1;
    points2.scale.y = 1;
    points2.scale.z = 0.4;
    points2.color.b = 0.5;
    points2.color.r = 1;
    points2.color.a = 1;

    points3.header.frame_id =  "map";
    points3.header.stamp = ros::Time::now();
    points3.action = visualization_msgs::Marker::ADD;
    points3.id = 2;
    points3.type = visualization_msgs::Marker::ARROW;
    points3.scale.x = 0.5;
    points3.scale.y = 0.05;
    points3.scale.z = 0.05;
    points3.color.b = 0.5;
    points3.color.r = 1;
    points3.color.a = 1;


     // Create the vertices for the points and lines
    for (int i = 0; i < globalPath.poses.size(); i += step_size)
    {
      
      geometry_msgs::Point p;
      p.x = globalPath.poses[i].pose.position.x;
      p.y = globalPath.poses[i].pose.position.y;
      p.z = 0;

      points.points.push_back(p);
      
    }

    for ( int i = 0 ; i < goal_list.size() ; i++)
    {
     
     points2.pose.position.x =  (float)goal_list[i].x / 100;
     points2.pose.position.y =  (float)goal_list[i].y / 100;
     points2.pose.position.z = 0;
     points2.text = goal_list[i].id;
     points2.ns = "text " + goal_list[i].id;
     marker_pub2.publish(points2);


     points3.pose.position.x =  (float)goal_list[i].x / 100;
     points3.pose.position.y =  (float)goal_list[i].y / 100;
     points3.pose.position.z = 0; 
     points3.pose.orientation = tf::createQuaternionMsgFromYaw(Deg2Rad(goal_list[i].yaw));
     points3.ns = "arrow " + goal_list[i].id;

     marker_pub3.publish(points3);
     
    }


    marker_pub.publish(points);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mymovebase");

    ROS_INFO("SepantaMoveBase Version 2.1.1 + Recovery + laser");

    boost::thread _thread_PathFwr(&PathFwr);
    boost::thread _thread_Logic(&logic_thread);

    ros::NodeHandle node_handles[20];
    ros::Subscriber sub_handles[5];

    //============================================================================================
    sub_handles[0] = node_handles[0].subscribe("/slam_out_pose", 10, GetPos);
    //============================================================================================
    sub_handles[1] = node_handles[1].subscribe("/move_base/global_costmap/costmap", 10, GetCostmap);
    //============================================================================================
    sub_handles[2] = node_handles[2].subscribe("/HectorStatus", 10, CheckHectorStatus);
    //============================================================================================
    mycmd_vel_pub = node_handles[3].advertise<geometry_msgs::Twist>("sepantamovebase/cmd_vel", 10);
    pub_slam_origin = node_handles[4].advertise<geometry_msgs::PoseWithCovarianceStamped>("/slam_origin", 1);
    pub_slam_reset = node_handles[5].advertise<std_msgs::String>("syscommand", 1);
    //============================================================================================
    ros::NodeHandle n_service;
    ros::ServiceServer service_command = n_service.advertiseService("sepantamovebase/command", checkcommand);
    //============================================================================================
    pub_tts = node_handles[6].advertise<std_msgs::String>("/texttospeech/message", 10);
    //============================================================================================
    pub_current_goal = node_handles[7].advertise<geometry_msgs::PoseStamped>("current_goal", 0 );
    pub_move = node_handles[7].advertise<std_msgs::Bool>("lowerbodycore/isrobotmove", 10);
    //============================================================================================
    marker_pub =  node_handles[7].advertise<visualization_msgs::Marker>("visualization_marker_steps", 10);
    marker_pub2 =  node_handles[7].advertise<visualization_msgs::Marker>("visualization_marker_goals", 10);
    marker_pub3 =  node_handles[7].advertise<visualization_msgs::Marker>("visualization_marker_goals_arrow", 10);

    client_makeplan = node_handles[9].serviceClient<nav_msgs::GetPlanRequest>("move_base/make_plan");
 	client_resetcostmap = node_handles[10].serviceClient<std_srvs::EmptyRequest>("move_base/clear_costmaps");
 	client_map_save = node_handles[11].serviceClient<std_srvs::EmptyRequest>("sepantamapengenine/save");
    client_map_load = node_handles[12].serviceClient<std_srvs::EmptyRequest>("sepantamapengenine/load");

    ros::Rate loop_rate(20);

    read_file();

    while (ros::ok() && App_exit == false)
    {
        publish_isrobotmove();
    	test_vis();
        ros::spinOnce();
        loop_rate.sleep();
    }

    _thread_PathFwr.interrupt();
    _thread_PathFwr.join();

    _thread_Logic.interrupt();
    _thread_Logic.join();

    return 0;
}