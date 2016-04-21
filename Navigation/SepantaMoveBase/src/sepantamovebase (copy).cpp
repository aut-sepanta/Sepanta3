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


//MAX SPEED
#define normal_max_linear_speedX  0.45
#define normal_max_linear_speedY  0.45
#define normal_max_angular_speed  0.6
//-
#define goal_max_linear_speedX  0.4
#define goal_max_linear_speedY  0.4
#define goal_max_angular_speed  0.5
//KP
#define normal_kp_linearX 0.4
#define normal_kp_linearY 0.4
#define norma_kp_angular  1.5
//-
#define goal_kp_linearX  0.4
#define goal_kp_linearY  0.4
#define goal_kp_angular  0.8
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
#define goal_desire_errorX 0.05
#define goal_desire_errorY 0.05
#define goal_desire_errorTetha 0.09 // 5 degree

//=============================================================

std::string coutcolor0 = "\033[0;0m";
std::string coutcolor_red = "\033[0;31m";
std::string coutcolor_green = "\033[0;32m";
std::string coutcolor_blue = "\033[0;34m";
std::string coutcolor_magenta = "\033[0;35m";
std::string coutcolor_brown = "\033[0;33m";

//=============================================================
bool isrobotmove = false;
bool App_exit = false;
bool newPath = false;
bool IsGoalValid = false;

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
int globalPathSize;

ros::Publisher mycmd_vel_pub;
ros::Publisher pub_tts;
ros::Publisher pub_current_goal;

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

double start_position[2] = {0};
double traveled_score = 0;
double position[2] = {0};
double orientation[4] = {0};
double tetha = 0;

double tempGoalPos[2] = {0};
double tempGoalTetha = 0;

double goalPos[2] = {0};
double goalOri[4] = {0};
double goalTetha = 0;

double current_path_score = 0;

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


float distacne_to_goal = 0;

double maxErrorX = 0, maxErrorY = 0, maxErrorTetha = 0;
std::vector<goal_data> goal_list;

bool isvirtual = true;
int info_counter = 0;
bool get_goal = false;
int temp_path_size = 0;
bool say_enable = true;

int system_state = 0;
int logic_state = 0;
bool on_the_goal = false;
int step_size  = 40;
bool wait_flag = false;




inline double Deg2Rad(double deg)
{
    return deg * M_PI / 180;
}

inline double Rad2Deg(double rad)
{
    return rad * 180 / M_PI;
}

double calc_dist(double x1,double x2,double y1,double y2)
{
    double dist = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
    dist = sqrt(dist);
    return dist;
}
double calc_score(nav_msgs::Path item,int max)
{
    double sum =0;
    double x1,y1,x2,y2;
    x1 = item.poses[0].pose.position.x;
    y1 = item.poses[1].pose.position.y;

    for ( int i = 1 ; i < max ; i++ )
    {
         x2 = item.poses[i].pose.position.x;
         y2 = item.poses[i].pose.position.y;
         double item_dist = calc_dist(x1,x2,y1,y2);
         sum += item_dist;

         x1 = x2;
         y1 = y2;
    }

    return sum;
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

void clean_costmap()
{
   std_srvs::Empty _s;
   client_resetcostmap.call(_s);
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


//logic_state = 0 => wait
//logic_state = 1 => in exe

nav_msgs::Path call_make_plan()
{
 nav_msgs::GetPlan srv;
 srv.request.goal.pose.position.x = goalPos[0];
 srv.request.goal.pose.position.y = goalPos[1];
 srv.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(goalTetha);
 client_makeplan.call(srv);
 return srv.response.plan;
}

void logic_thread()
{
    nav_msgs::Path resut;

    while(ros::ok() && !App_exit)
    {
         boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

         if ( logic_state == 0 )
         {
            cout<<"wait for exe , idle"<<endl;
         }

         if ( logic_state == 1 )
         {
               //operation loop
               cout<<"Get a plan from global planner"<<endl;
               nav_msgs::Path resut = call_make_plan();
               logic_state = 2;
         }

        if ( logic_state == 2 )
        {
            //analise the plan

            
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
        get_goal = true;
        logic_state = 1;
    }


   
}

void exe_cancle()
{
   if ( logic_state != 0)
   {

   }
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


void force_stop()
{
    send_omni(0,0,0);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
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
            //if (errorTetha > 0.833*M_PI) errorTetha = 0.833*M_PI;
            //if (errorTetha < -0.833*M_PI) errorTetha = -0.833*M_PI;

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

void say_message(string data)
{
  if ( say_enable == false ) return;
  std_msgs::String _mes;
  _mes.data = data;
  pub_tts.publish(_mes);
}

void publish_current_goal()
{
  pub_current_goal.publish(target_goal_stamped);
}

 

void PathFwr()
{
	
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    force_stop();
   
    say_message("Sepanta Move Base Started");

    while (ros::ok() && !App_exit)
    {

        if ( system_state != 0)
        	wait_flag = false;
      
        if ( system_state == 0)
        {

            
          
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));

            if ( IsGoalValid && get_goal)
            {
            	 
                     IsGoalValid = false;
                     get_goal = false;
                     ResetLimits();
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
            else
            {
            if ( wait_flag == false)
            {
               say_message("I am waiting for new goal.");
               cout<< coutcolor_green <<"Wait for goal ! ... "<< coutcolor0 <<endl;
               wait_flag = true;
            }
            }
        }
        else
        if ( system_state == 1)
        {
           force_stop();
           calc_next_point();
           errors_update();
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
            temp_path_size = 0;
            get_goal = false;
            IsGoalValid = false;
            on_the_goal = false;
            force_stop();
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            system_state = 0;
        }

        errors_update();
        //publish_info();
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


int say_change_counter = 0;

void GetPath(const nav_msgs::Path::ConstPtr &msg)
{

       
    
}

 // if ( msg->poses.size() < 20 ) return;
        

 //        if ( temp_path_size == 0 )
 //        {
 //        //this is a new path reset all states
 //        temp_path_size = msg->poses.size();
 //        globalPath = *msg;
 //        globalPathSize = globalPath.poses.size();
 //        system_state = 0;
 //        IsGoalValid=true;
 //        on_the_goal = false;
 //        step=0;
 //        current_path_score = calc_score(globalPath,globalPath.poses.size());
 //        cout<<coutcolor_green<<"get a new PATH , GPLANNER Score : "<< current_path_score<< temp_path_size <<coutcolor0<<endl;

        
 //        }
 //        else  if ( temp_path_size != 0 ) //if i am on the path check delta score for new path comming from planner and get it if it is new !
 //        {
 //            double new_score = calc_score(*msg,msg->poses.size());
 //            double t_score = calc_score(globalPath,step);
 //            t_score = t_score - (calc_dist(position[0], tempGoalPos[0],position[1], tempGoalPos[1]));
 //            double old = current_path_score - t_score;
 //            double delta = fabs(new_score-old);

 //            if ( delta >= 0.25)
 //            {

 //                 force_stop(); 

 //                 if ( say_change_counter == 0)
 //                 {say_message("Path Changed"); say_change_counter++;}
 //                 else
 //                 if ( say_change_counter == 1)
 //                 {say_message("i want to change my trajectory"); say_change_counter++;}
 //                 else
 //                 if ( say_change_counter == 2)
 //                 {say_message("i got a new path"); say_change_counter++; }
 //                 else
 //                 if ( say_change_counter == 3)
 //                 {say_message("there is an obstacle on my way");  say_change_counter++;}
 //                 else
 //                 if ( say_change_counter == 4)
 //                 {
 //                    say_change_counter = 0;
 //                 } 

 //                 cout<<coutcolor_red<<"Path Changed !"<<coutcolor0<<endl;
              
 //                    temp_path_size = msg->poses.size();
 //                    globalPath = *msg;
 //                    globalPathSize = globalPath.poses.size();
 //                    IsGoalValid= true;
 //                    on_the_goal = false;
 //                    step=0;
 //                    current_path_score = calc_score(globalPath,globalPath.poses.size());
 //                    cout<<coutcolor_green<<"get a new PATH , GPLANNER Score : "<< current_path_score<< temp_path_size <<coutcolor0<<endl;
 //                    system_state = 1;

 //            }
            

 //        }
 //        //system_state = 0;
 //        //on_the_goal = false;
 //        //force_stop();

void GetGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr &msg)
{
    get_goal = true;
    force_stop();
  
    if ( system_state >= 3  )
    {
        system_state = 1;
    }
   
    cout<<coutcolor_green<<"get a new GOAL from USER " <<coutcolor0<<endl;
    goalPos[0] = msg->goal.target_pose.pose.position.x;
    goalPos[1] = msg->goal.target_pose.pose.position.y;
    goalOri[0] = msg->goal.target_pose.pose.orientation.x;
    goalOri[1] = msg->goal.target_pose.pose.orientation.y;
    goalOri[2] = msg->goal.target_pose.pose.orientation.z;
    goalOri[3] = msg->goal.target_pose.pose.orientation.w;
    goalTetha = Quat2Rad(goalOri);


}

//cm cm degree
void update_hector_origin(int x,int y,int yaw)
{
    float ox = (float)(x) / 100;
    float oy = (float)(y) / 100;
    float oyaw = yaw;
    oyaw = Deg2Rad(oyaw);
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(oyaw);
    msg.pose.pose.position.x = ox;
    msg.pose.pose.position.y = oy;
    pub_slam_origin.publish(msg);
}

void reset_hector_slam()
{
	std_msgs::String _msg;
	_msg.data = "reset";
	pub_slam_reset.publish(_msg);
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
        cout<<"service : "<<_cmd<<endl;
    	read_file();
    }

    if ( _cmd == "save_map")
    {
        cout<<"service : "<<_cmd<<endl;
    	sepantamapengine_savemap();
    }

    if ( _cmd == "load_map")
    {
        cout<<"service : "<<_cmd<<endl;
    	sepantamapengine_loadmap();
    }

    if ( _cmd == "exe")
    {
        cout<<"service : "<<_cmd<<endl;
        goal_data g;
        g.id = _id;
        g.x = _value1;
        g.y = _value2;
        g.yaw = _value3;

        exe_slam(g);
    }

    if ( _cmd == "cancle")
    {
        cout<<"service : "<<_cmd<<endl;
        exe_cancle();
    }

    if ( _cmd == "reset_hector")
    {
        cout<<"service : "<<_cmd<<endl;
        reset_hector_slam();
    }

    if ( _cmd == "update_hector_origin")
    {
        cout<<"service : "<<_cmd<<endl;
        update_hector_origin(0,0,0);
    }

    if ( _cmd == "clean_costmap" )
    {
        cout<<"service : "<<_cmd<<endl;
        clean_costmap();
    }

    res.result = "done";
    return true;
}




float f = 0.0;
ros::Publisher marker_pub;
ros::Publisher marker_pub2;
ros::Publisher marker_pub3;
ros::Publisher pub_isrobotmove;

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
     
     points2.pose.position.x =  goal_list[i].x / 100;
     points2.pose.position.y =  goal_list[i].y / 100;
     points2.pose.position.z = 0;
     points2.text = goal_list[i].id;
     points2.ns = "text " + goal_list[i].id;
     marker_pub2.publish(points2);


     points3.pose.position.x =  goal_list[i].x / 100;
     points3.pose.position.y =  goal_list[i].y / 100;
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

    ROS_INFO("SepantaMoveBase Version 2.0.0");

    boost::thread _thread_PathFwr(&PathFwr);
    boost::thread _thread_Logic(&logic_thread);

    ros::NodeHandle node_handles[20];
    ros::Subscriber sub_handles[5];

 
    //============================================================================================
    sub_handles[0] = node_handles[0].subscribe("/slam_out_pose", 10, GetPos);
    //============================================================================================
    sub_handles[1] = node_handles[1].subscribe("/move_base/NavfnROS/plan", 10, GetPath);
    //============================================================================================
    sub_handles[2] = node_handles[2].subscribe("/move_base/goal", 10, GetGoal);
    //============================================================================================
    mycmd_vel_pub = node_handles[4].advertise<geometry_msgs::Twist>("sepantamovebase/cmd_vel", 10);
    pub_slam_origin = node_handles[8].advertise<geometry_msgs::PoseWithCovarianceStamped>("/slam_origin", 1);
    pub_slam_reset = node_handles[9].advertise<std_msgs::String>("syscommand", 1);
    //============================================================================================
    ros::NodeHandle n_service;
    ros::ServiceServer service_command = n_service.advertiseService("sepantamovebase/command", checkcommand);
    //============================================================================================
    pub_tts = node_handles[4].advertise<std_msgs::String>("/texttospeech/message", 10);
    //============================================================================================
    pub_current_goal = node_handles[5].advertise<geometry_msgs::PoseStamped>("current_goal", 0 );
    //============================================================================================
    pub_isrobotmove = node_handles[8].advertise<std_msgs::Bool>("sepantamovebase/isrobotmove", 10);
    //============================================================================================
    marker_pub =  node_handles[7].advertise<visualization_msgs::Marker>("visualization_marker_steps", 10);
    marker_pub2 =  node_handles[7].advertise<visualization_msgs::Marker>("visualization_marker_goals", 10);
    marker_pub3 =  node_handles[7].advertise<visualization_msgs::Marker>("visualization_marker_goals_arrow", 10);

    client_makeplan = node_handles[8].serviceClient<nav_msgs::GetPlanRequest>("move_base/make_plan");
 	client_resetcostmap = node_handles[9].serviceClient<std_srvs::EmptyRequest>("move_base/clear_costmaps");
 	client_map_save = node_handles[10].serviceClient<std_srvs::EmptyRequest>("sepantamapengenine/save");
    client_map_load = node_handles[11].serviceClient<std_srvs::EmptyRequest>("sepantamapengenine/load");

    ros::Rate loop_rate(20);

    read_file();

    while (ros::ok() && App_exit == false)
    {
    	test_vis();
        ros::spinOnce();
        loop_rate.sleep();

        std_msgs::Bool mes;
        mes.data = isrobotmove;
        pub_isrobotmove.publish(mes);
    }

    force_stop();

    _thread_PathFwr.interrupt();
    _thread_PathFwr.join();

    _thread_Logic.interrupt();
    _thread_Logic.join();

    return 0;
}

