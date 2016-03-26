#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <stdio.h>
#include <stdlib.h>
#include <tbb/atomic.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "athomerobot_msgs/arm.h"
#include "athomerobot_msgs/omnidata.h"
#include "athomerobot_msgs/head.h"
#include "athomerobot_msgs/irsensor.h"
#include "athomerobot_msgs/motortorques.h"

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <actionlib/server/simple_action_server.h>
#include <athomerobot_msgs/sepantaAction.h>

#include "math.h"
using namespace std;
using namespace boost;

nav_msgs::OccupancyGrid static_map;
tbb::atomic<int> Compass;
bool first_init = false;
Eigen::MatrixXf IK_matrix;

float Odometry_raw[3] = {0};

float odom_position_yaw[3] = {0};
float slam_position_yaw[3] = {0};
float odom_speed_xyw[3] = {0};

int speed_Motor[4] = {128,128,128,128};

bool App_exit = false;
int Odometry_speed = 50;

float ratio_X = 15.2;
float ratio_Y = 15.3;
float ratio_W = 1800;

float kp_degree = 3.5;
float ki_degree = 0.03;
float kp_move = 8.5;

float L = 50; //cm
bool isrobotmove = false;
ros::Publisher chatter_pub[20];

float laser_IR[8];
float IR[8];

float pid_i_buffer[20] = {0};

typedef actionlib::SimpleActionServer<athomerobot_msgs::sepantaAction> slam_Server;
slam_Server *globalServer;

bool cancel_request = false;

void omnidrive(int x, int y, int w);

float ConvertQuatToYaw(const geometry_msgs::Quaternion &quat)
{
    float yaw = tf::getYaw(quat);
    return isnan(yaw) ? 0 : yaw;
}

float convert_w_radps(float w) {
    return w / (ratio_W);
}

float convert_vy_mps(float vy) {
    return vy / (ratio_Y*100);
}

float convert_vx_mps(float vx) {
    return vx / (ratio_X*100);
}

float convert_radps_w(float w) {
    return w * (ratio_W);
}

float convert_mps_vy(float mps) {
    return mps * (ratio_Y*100);
}

float convert_mps_vx(float mps) {
    return mps * (ratio_X*100);
}

inline float Deg2Rad(float deg) {
    return deg * M_PI / 180;
}

inline float Rad2Deg(float rad) {
    return rad * 180 / M_PI;
}

void reset_position() 
{ 
   odom_position_yaw[0] = 0;
   odom_position_yaw[1] = 0;
   odom_position_yaw[2] = 0;
}

void chatterCallback_laser(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    int val_count = msg->ranges.size(); //512
    int int_count = msg->intensities.size(); //? //0
  
    float read = 0;

    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "laser";
    scan.angle_min = msg->angle_min;
    scan.angle_max = msg->angle_max;
    scan.angle_increment = msg->angle_increment;
    scan.time_increment = msg->time_increment;
    scan.range_min = msg->range_min;
    scan.range_max = msg->range_max;

    //cout<<scan.range_max<<endl;

    scan.ranges.resize(val_count);
    scan.intensities.resize(int_count);

     for ( int i = 0 ; i < val_count ; i++)
     {
           scan.ranges[i] = msg->ranges[i];
         
           if ( std::isnan(scan.ranges[i]) || isinf(scan.ranges[i]) )
           {
                 scan.ranges[i] = 5;
           }
     }

     for ( int i = 0 ; i < int_count ; i++)
     {
           scan.intensities[i] = msg->intensities[i];
     }

    chatter_pub[2].publish(scan);
}

//void calibration_test()
//{
//    return;
//    cout << "calib start start" << endl;

//    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));


//    //reset_position();

//    omnidrive(0, 50, 0);
//    boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
//    omnidrive(0, 0, 0);
//    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

//    cout << "back" << endl;


//    omnidrive(0, -50, 0);
//    boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
//    omnidrive(0, 0, 0);
//    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

//    cout << "finish" << endl;
//    // cout << position[0] << " , " << position[1] << " , " << position[2] << " , " << delta<< endl;
//}

void Init() 
{

    Eigen::MatrixXf matrix(3,4);
    IK_matrix = matrix;

    IK_matrix(0, 0) = 0.25;
    IK_matrix(0, 1) = 0.25;
    IK_matrix(0, 2) = -0.25;
    IK_matrix(0, 3) = -0.25;

    IK_matrix(1, 0) = -0.25;
    IK_matrix(1, 1) = 0.25;
    IK_matrix(1, 2) = -0.25;
    IK_matrix(1, 3) = 0.25;

    IK_matrix(2, 0) = -0.02;
    IK_matrix(2, 1) = 0.02;
    IK_matrix(2, 2) = 0.02;
    IK_matrix(2, 3) = -0.02;
}

void IK_solver(float delta_time)
{

	float a ;
    float b ;
    float c ;
    float d ;

    a = speed_Motor[2];
    b = speed_Motor[3];
    c = speed_Motor[1];
    d = speed_Motor[0];

    a = (a - 128) * 7.5;
    b = (b - 128) * 7.5;
    c = (c - 128) * 7.5;
    d = (d - 128) * 7.5;

    Odometry_raw[0] = IK_matrix(0, 0) * a + IK_matrix(0, 1) * b + IK_matrix(0, 2) * c + IK_matrix(0, 3) * d;
    Odometry_raw[1] = IK_matrix(1, 0) * a + IK_matrix(1, 1) * b + IK_matrix(1, 2) * c + IK_matrix(1, 3) * d;
    Odometry_raw[2] = IK_matrix(2, 0) * a + IK_matrix(2, 1) * b + IK_matrix(2, 2) * c + IK_matrix(2, 3) * d;

    odom_speed_xyw[0] = Odometry_raw[0];
    odom_speed_xyw[1] = Odometry_raw[1];
    odom_speed_xyw[2] = (-Odometry_raw[2] / 4 ) * 100;

    odom_speed_xyw[0] = convert_vx_mps(odom_speed_xyw[0]);
    odom_speed_xyw[1] = convert_vy_mps(odom_speed_xyw[1]);
    odom_speed_xyw[2] = convert_w_radps(odom_speed_xyw[2]);

    odom_position_yaw[0] += odom_speed_xyw[0] * delta_time; //meter
    odom_position_yaw[1] += odom_speed_xyw[1] * delta_time; //meter
    odom_position_yaw[2] += odom_speed_xyw[2] * delta_time; //radian

   // cout<<odom_position_yaw[0]<<" "<<odom_position_yaw[1]<<" "<<odom_position_yaw[2]<<endl;
}

void Turn_GL(int degree)
{
    ros::Rate r(50);
    if (isrobotmove == 1) return;
    isrobotmove = 1;

    if (degree > 360) degree -= 360;
    if (degree < 0) degree += 360;

    float error = Compass - degree;

    while ((error > 2 || error < -2 )&& App_exit == false)
    {
        error = Compass - degree;

        if (error >= 180) error = error - 360;
        if (error < -180) error = 360 - error;

        if (error > 150) error = 150;
        if (error < -150) error = -150;

        float error_total = error * kp_degree;
        omnidrive(0, 0, error_total);
        if ( cancel_request ) break;
        r.sleep();
    }

    omnidrive(0, 0, 0);
    r.sleep();
    isrobotmove = 0;
}

void Turn_GL_local(int degree)
{
    if (isrobotmove == 1) return;
    int target = (Compass + degree) % 360;
    if (target < 0) target += 360;
    Turn_GL(target);
}

void Move_GLX(float SV, float distance)
{
    ros::Rate r(50);
    if (isrobotmove == 1) return;

    isrobotmove = 1;
    float set_degree = Compass;
    double SW = 0;
    reset_position();

    if ( distance < 0 ) SV = SV * -1;

    distance = abs(distance) / 100;

    if (SV != 0 && distance > 0) {
        while ( abs(odom_position_yaw[0]) < distance && App_exit == false) {
            double error = Compass - set_degree;
            cout<<abs(odom_position_yaw[0])<<" "<<distance<<endl; 

            if (error >= 180) error = error - 360;
            if (error < -180) error = 360 - error;
            SW = error * kp_move ;

            if (SW > 80) SW = 80;
            if (SW < -80) SW = -80;

            //float ss = sin(Deg2Rad(error)) * 300;
            omnidrive(SV, 0, 0);

            if ( cancel_request ) break;
           r.sleep();
        }
    }

    omnidrive(0, 0, 0);
    r.sleep();

    isrobotmove = 0;
}

void Move_GLY(float SV, float distance)
{
    ros::Rate r(50);

    if (isrobotmove == 1) return;

    isrobotmove = 1;
    float set_degree = Compass;
    double SW = 0;
    reset_position();

    if ( distance < 0 ) SV = SV * -1;

    distance = abs(distance) / 100;

    if (SV != 0 && distance > 0) {

        while ( abs(odom_position_yaw[1]) < distance && App_exit == false) {
            double error = Compass - set_degree;
            if (error >= 180) error = error - 360;
            if (error < -180) error = 360 - error;
            SW = error * kp_move;

            if (SW > 80) SW = 80;
            if (SW < -80) SW = -80;

            omnidrive(0, SV, 0);

            if ( cancel_request ) break;

            r.sleep();
        }

    }

    omnidrive(0, 0, 0);
    r.sleep();

    isrobotmove = 0;
}

void chatterCallback_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    //cout<<"get"<<endl;
    geometry_msgs::Pose pose;
    geometry_msgs::Point point;
    geometry_msgs::Quaternion quaternion;
    pose = msg->pose;

    slam_position_yaw[0] = pose.position.x; //meter
    slam_position_yaw[1] = pose.position.y; //meter
    slam_position_yaw[2] = ConvertQuatToYaw(pose.orientation); //radians
}

void chatterCallback_compass(const std_msgs::Int32::ConstPtr &msg)
{
    Compass = msg->data;
}

void chatterCallback_omnispeed(const athomerobot_msgs::omnidata::ConstPtr &msg)
{
    speed_Motor[0] = msg->d0;
    speed_Motor[1] = msg->d1;
    speed_Motor[2] = msg->d2;
    speed_Motor[3] = msg->d3;

   // cout<<speed_Motor[0]<<" "<<speed_Motor[1]<<" "<<speed_Motor[2]<<" "<<speed_Motor[3]<<endl;

}

void chatterCallback_irsensor(const athomerobot_msgs::irsensor::ConstPtr &msg)
{
    IR[0] = msg->d0;
    IR[1] = msg->d1;
    IR[2] = msg->d2;
    IR[3] = msg->d3;
    IR[4] = msg->d4;
    IR[5] = msg->d5;
    IR[6] = msg->d6;
    IR[7] = msg->d7;
}

void chatterCallback_lasersensor(const athomerobot_msgs::irsensor::ConstPtr &msg)
{
    laser_IR[0] = msg->d0;
    laser_IR[1] = msg->d1;
    laser_IR[2] = msg->d2;
    laser_IR[3] = msg->d3;
    laser_IR[4] = msg->d4;
    laser_IR[5] = msg->d5;
    laser_IR[6] = msg->d6;
    laser_IR[7] = msg->d7;
}

void chatterCallback_cmd_vel(const geometry_msgs::Twist &twist_aux)
{
    double vel_x = twist_aux.linear.x;
    double vel_y =  twist_aux.linear.y;
    double vel_th = -1 * twist_aux.angular.z;

    int xx = 0;
    int yy = 0;
    int ww = 0;

    //=============== limits

    cout<<"1"<<vel_x<<" "<<vel_y<<" "<<vel_th<<endl;

    vel_x = vel_x / 2;
    vel_y = vel_y / 2;
    vel_th = vel_th / 2;

    if ( abs(vel_x) <= 10 && vel_y == 0 )
    {
     
      if ( abs(vel_th) > 0.2 ) 
      {
        int s = 1;
        if ( vel_th < 0 )
        {
          s = -1;
        }

        vel_th = s * 0.2;

      }
        

      
    }

   cout<<"1"<<vel_x<<" "<<vel_y<<" "<<vel_th<<endl;

    

    //=============== Convert
    xx = (int)convert_mps_vx(vel_x);
    yy = (int)convert_mps_vy(vel_y);
    ww = (int)convert_radps_w(vel_th);


    //===============
    //send to driver...
    xx = xx / 2;
    yy = yy / 2;
    ww = ww / 2;
    omnidrive(xx,yy,ww);
}

void omnidrive(int x,int y,int w)
{

   athomerobot_msgs::omnidata msg_data;
   msg_data.d0 = x;
   msg_data.d1 = y;
   msg_data.d2 = w;

   chatter_pub[0].publish(msg_data);
}

bool mapCallback(nav_msgs::GetMap::Request &req,nav_msgs::GetMap::Response &res )
{
      // // request is empty; we ignore it

      // // = operator is overloaded to make deep copy (tricky!)
      // res.map = static_map;
      // ROS_INFO("Sending map");
      // return true;
}

void chatterCallback_map(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
   // if ( first_init == false)
   // {
   // first_init = true;
   // static_map.data = msg->data;
   // static_map.header = msg->header;
   // static_map.info = msg->info;
   // ROS_INFO("SEPANTA STATIC MAP INIT DONE");
   // }
}

void PreemptThread()
{
  
   while ( App_exit == false )
   {

        if (globalServer->isPreemptRequested() )
        {
            cancel_request = true;
            ROS_INFO(" cancle requested\n");
            globalServer->setAborted();
            ROS_INFO(" aborted done\n");
            break;
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
   }
}

class myactionserver
{
protected:

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<athomerobot_msgs::sepantaAction> as_;
  std::string action_name_;

  athomerobot_msgs::sepantaFeedback feedback_;
  athomerobot_msgs::sepantaResult result_;

public:

  myactionserver(std::string name) :
    as_(nh_, name, boost::bind(&myactionserver::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    globalServer = &as_;
    //globalServer = &as_;
  }

  ~myactionserver(void)
  {
  }

  void executeCB(const athomerobot_msgs::sepantaGoalConstPtr &interfacegoal)
  {

    boost::thread Preempt_thread(&PreemptThread);
    cancel_request = false;
    
    bool success = true;

    ROS_INFO("EXECUTE NEW ACTION");

    int value = interfacegoal->value;

    if ( interfacegoal->type == "movex")    
    {

           Move_GLX(Odometry_speed,value);
    }
    else if ( interfacegoal->type == "movey")
    {
           Move_GLY(Odometry_speed,value);
    }
    else if ( interfacegoal->type == "tunrgl")
    {
           Turn_GL(value);
    }
    else if ( interfacegoal->type == "turngllocal" )
    {
           Turn_GL_local(value);
    }

    omnidrive(0, 0, 0);

    if(success)
    {
       result_.result= "DONE";
       cout<<"DONE"<<endl;
       as_.setSucceeded(result_);
    }
    else
    {
        result_.result= "MOVEMENT FAILED";
        as_.setSucceeded(result_);
    }

    Preempt_thread.interrupt();
    Preempt_thread.join();
  }


};

int main(int argc, char** argv)
{
  Compass = 0;
  ros::init(argc, argv, "odometry_publisher");
  cout << "SEPANTA ODOMETRY STARTED DONE 17 july" << endl;

  ros::Time::init();
  Init();
  
  ros::Rate ros_rate(30);

  ros::NodeHandle node_handles[15];
  ros::Subscriber sub_handles[15];

  chatter_pub[0] = node_handles[0].advertise<athomerobot_msgs::omnidata>("AUTROBOTIN_omnidrive", 10);
  chatter_pub[1] = node_handles[1].advertise<nav_msgs::Odometry>("odom", 10);
  chatter_pub[2] = node_handles[2].advertise<sensor_msgs::LaserScan>("scan_filter", 50);
  chatter_pub[3] = node_handles[3].advertise<std_msgs::Int32>("AUTROBOTOUT_isrobotmove", 50);
  //=================================================================================================
  sub_handles[0] = node_handles[1].subscribe("AUTROBOTOUT_compass", 10, chatterCallback_compass);
  sub_handles[1] = node_handles[2].subscribe("cmd_vel", 10, chatterCallback_cmd_vel);
  sub_handles[2] = node_handles[3].subscribe("AUTROBOTOUT_irsensor", 10, chatterCallback_irsensor);
  sub_handles[3] = node_handles[4].subscribe("AUTROBOTOUT_lasersensor", 10, chatterCallback_lasersensor);
  sub_handles[4] = node_handles[5].subscribe("AUTROBOTOUT_omnispeed", 10, chatterCallback_omnispeed);
  sub_handles[6] = node_handles[7].subscribe("/slam_out_pose", 10, chatterCallback_pose);
  sub_handles[7] = node_handles[8].subscribe("scan", 10, chatterCallback_laser);
  //sub_handles[8] = node_handles[8].subscribe("AUTROBOTIN_moveglx", 10, chatterCallback_moveglx);
  //sub_handles[9] = node_handles[9].subscribe("AUTROBOTIN_movegly", 10, chatterCallback_movegly);
  //sub_handles[10] = node_handles[10].subscribe("/map", 10, chatterCallback_map);

 //sub_handles[11] = node_handles[11].subscribe("AUTROBOTIN_omnidrive", 10, chatterCallback_omnidrive);

  tf::TransformBroadcaster odom_broadcaster;
  
  //ros::NodeHandle n;
  //ros::ServiceServer service = n.advertiseService("sepanta_static_map", mapCallback);

  myactionserver act("sepanta_action");
  
  //ROS_INFO("SEPANTA ODOMETRY STARTED DONE");

  while(ros::ok())
  {

    ros::spinOnce();
    //======================================================
    IK_solver(0.05);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(slam_position_yaw[2]);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = slam_position_yaw[0];
    odom_trans.transform.translation.y = slam_position_yaw[1];
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);
    
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = slam_position_yaw[0];
    odom.pose.pose.position.y = slam_position_yaw[1];
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;

    //publish the message
    chatter_pub[1].publish(odom);

    //publish isrobotmove
    std_msgs::Int32 mes;
    mes.data = isrobotmove;
    chatter_pub[3].publish(mes);
    //======================================================
    ros_rate.sleep();
  }

  App_exit = true;



  return 0;
} 
