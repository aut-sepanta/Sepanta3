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
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sepanta_msgs/Objects.h>
#include <sepanta_msgs/Object.h>
#include <k2_client/BodyArray.h>
///////////////////////////////////////////////////////////
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/highgui/highgui_c.h"
#include <opencv2/core/core.hpp>
#include "opencv/cv.h"
#include "opencv2/calib3d/calib3d.hpp"
//********************************************** cv_bridge
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sepanta_msgs/Objects.h>
#include <image_transport/image_transport.h>

#include <boost/thread/mutex.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sepanta_msgs/LookForObjectsAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sepanta_msgs/MasterAction.h>
#include <sepanta_msgs/led.h>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;
using namespace ros;

//=============================================================
#define waypoint_1 "roomcenter"
#define waypoint_2 "bedroom"
#define waypoint_3 "shelf"
#define waypoint_4 "kitchen"
//=============================================================

std::string coutcolor0 = "\033[0;0m";
std::string coutcolor_red = "\033[0;31m";
std::string coutcolor_green = "\033[0;32m";
std::string coutcolor_blue = "\033[0;34m";
std::string coutcolor_magenta = "\033[0;35m";
std::string coutcolor_brown = "\033[0;33m";

int Function_state = 0;
bool Function1_result = false;
bool Function2_result = false; //dummy
bool Function3_result = false;
int question_counter = 0;

bool App_exit = false;
ros::Publisher pub_tts;
ros::Publisher pub_spr;
ros::Publisher pub_led;
bool say_enable = true;

int logic_state = 0;
bool isdooropened = false;
bool isspeechready = false;
bool isttsready = true;

string sayMessageId;

string speech_last_command = "";
string temp_speech_last_command = "";
ros::ServiceClient client_navigation;
ros::ServiceClient say_service;

actionlib::SimpleActionClient<sepanta_msgs::MasterAction> * ac_navigation;

void change_led(int r,int g,int b)
{
  sepanta_msgs::led _msg;

  if( r != 0 || g != 0 || b != 0)
  {
     _msg.enable = true;
     _msg.colorR = r;
     _msg.colorG = g;
     _msg.colorB = b;
  }
  else
  {
    _msg.enable = false;
  }

  pub_led.publish(_msg);
}



void print_color(string msg,string color)
{
  if ( color == "red" )
  {
    std::cout<<coutcolor_red<<msg<<coutcolor0<<std::endl;
  }
  if ( color == "green" )
  {
    std::cout<<coutcolor_green<<msg<<coutcolor0<<std::endl;
  }
  if ( color == "blue" )
  {
    std::cout<<coutcolor_blue<<msg<<coutcolor0<<std::endl;
  }
  if ( color == "brown" )
  {
    std::cout<<coutcolor_brown<<msg<<coutcolor0<<std::endl;
  }
  if ( color == "coutcolor_magenta" )
  {
    std::cout<<coutcolor_magenta<<msg<<coutcolor0<<std::endl;
  }
}

void sepanta_wait(int ms)
{
  boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

string navigation_go_to(string location)
{
  sepanta_msgs::MasterGoal goal;
  goal.action = "exe";
  goal.id = location;

  ac_navigation->sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac_navigation->waitForResult(ros::Duration(1000));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac_navigation->getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    sepanta_msgs::MasterResult::ConstPtr _res = ac_navigation->getResult();
    ROS_INFO("Action result : %s",_res->result.c_str());

    return _res->result.c_str();

  }
  else
  {
     ac_navigation->cancelGoal();
     ROS_INFO("Action did not finish before the time out.");

      return "time_out";
  }

}

void navigation_cancel()
{
   ac_navigation->cancelGoal();
}

void say_message(string data)
{
    if ( say_enable == false ) return;
  	isttsready = false;
    sepanta_msgs::command _msg;
   _msg.request.command = data;
    say_service.call(_msg);
    sayMessageId = _msg.response.result;
    while(!isttsready)
    {
    	sepanta_wait(1000);
    }
}

void chatterCallback_kinect2_body(const k2_client::BodyArray::ConstPtr &msg)
{
   print_color("Get body","green");

   for ( int i = 0 ; i < msg->bodies.size() ; i++)
   {
       if ( msg->bodies.at(i).isTracked)
       {
          stringstream convert; // stringstream used for the conversion
          convert << i << " " <<   msg->bodies.at(i).jointPositions.at(0).position.x << " " <<   msg->bodies.at(i).jointPositions.at(0).position.y << " " <<   msg->bodies.at(i).jointPositions.at(0).position.z ;
          string _index = convert.str();

         // msg->bodies.at(i).jointPositions.at(0).position.x;
          print_color(_index,"red");
       }
   }
}

void chatterCallback_door(const std_msgs::Bool::ConstPtr &msg)
{
   isdooropened = msg->data;
}

void chatterCallback_speech(const std_msgs::String::ConstPtr &msg)
{
   cout<<coutcolor_brown<<"Speech Get : "<<msg->data<<coutcolor0<<endl;
   if ( temp_speech_last_command != msg->data )
   {
      temp_speech_last_command = msg->data;
      speech_last_command = temp_speech_last_command;
      isspeechready = true;
   }  
}

void chatterCallback_ttsfb(const std_msgs::String::ConstPtr &msg)
{
	if(!isttsready && msg->data==sayMessageId)
	{
		isttsready = true;
	}
}

void send_feedback_to_speech(string cmd)
{
   std_msgs::String _msg;
   _msg.data = cmd;
   pub_spr.publish(_msg);
}



void Function_1()
{
   // if ( Function_state == 0 )
   // {
   //    say_message("I am going to kitchen and i should bring a coca for you!");
   //    Function_state = 1;
   // }
   // else if ( Function_state == 1)
   // {
   //    navigation_go_to("kitchen");
   //    Function_state = 2;
   // }
   // else if ( Function_state == 2)
   // {
   //      cout<<coutcolor_blue<<"Function [1] STATE [2] : wait for navigation to kitchen"<<coutcolor0<<endl;
	  //   	Function_state = 3;
   // }
   // else if ( Function_state == 3)
   // {
   //     Function1_result = false;
   //     cout<<coutcolor_blue<<"Function [1] STATE [3] : Send Start for object"<<coutcolor0<<endl;
   //     Function_state = 4;
   // }
   // else if ( Function_state == 4)
   // {
   // 	   cout<<coutcolor_blue<<"Function [1] STATE [4] : wait for object recognition"<<coutcolor0<<endl;
   // 	   Function_state = 5;
   // }
   // else if ( Function_state == 5)
   // {
   //    Function1_result = process_object("coca");
   //    if ( Function1_result )
   //    {
   //      say_message("well done ! , i found the coca !");
   //      say_message("i wonder, if i had my arms , could i pick up the coca? Money does not guarantee hapiness. But no money, no arms! ");
   //    }
   //    else
   //    {
   //      say_message("I cant find the coca.");
   //      say_message("it does not matter. I could not grab it even if I found it. Money does not guarantee hapiness. But no money, no arms! ");
   //    }

   //    Function_state = 6;
   // }
   // else if ( Function_state == 6)
   // {
   //    say_message("i am goint to room center to report my perception");
   //    navigation_go_to("roomcenter");
   //    Function_state = 7;
   // }
   // else if ( Function_state == 7)
   // {
   // 	   cout<<coutcolor_blue<<"Function [1] STATE [7] : wait for navigation to room center"<<coutcolor0<<endl;
	  //    Function_state = 8;
   // }
   // else if ( Function_state == 8 )
   // {
   	   
   //    if ( Function1_result )
   //    {
   //         say_message("well done ! , i found the coca !");
   //         say_message("But if you expect me to bring it for you, prepare a set of arms for me!");
   //    }
   //    else
   //    {
   //         say_message("I cant find the coca.");
   //         say_message("it does not matter. I could not grab it even if I found it.");
   //    }
   // 	   //========================================
   // 	   say_message("operation Done");

   // 	   Function_state = 0;
   // 	   logic_state = 3;
   // }
}

void Function_2()
{
   // cout<<"Header :"<<Function_state<<endl;
   // if ( Function_state == 0 )
   // {
   //   question_counter = 0;
   // 	 say_message("It is my pleasure to answer your questions!");
   // 	 Function_state = 1;
   // }
   // else if ( Function_state == 1)
   // {
   // 	  say_message("ask");
   //    send_feedback_to_speech("qstart");
   //    Function_state = 2;
   // }
   // else if ( Function_state == 2)
   // {
   //    if ( isspeechready )
   //    { 
   //        isspeechready = false;
   //        if ( speech_last_command == "qready")
   //        {
   //          //say_message("ask");
   //          Function_state = 3;
   //        }
   //        else
   //        {

   //           send_feedback_to_speech("qstart");
   //        }
   //    }
   // }
   // else if ( Function_state == 3)
   // {
   //          //Ready 
   //          if ( isspeechready )
   //          {
   //             isspeechready = false;
   //             if ( speech_last_command != "0")
   //             {
   //               say_message(speech_last_command);
   //               if ( question_counter < 4)
   //               {
   //               question_counter++;
   //               Function_state = 1;
   //               }
   //               else
   //               {
   //                  say_message("Finished");
   //                  Function_state = 4;
   //               }
   //             }
   //             else
   //             {
   //                 Function_state = 1;
   //             }
   //          }
   //          else
   //          {
   //                cout<<coutcolor_blue<<"Function [2] STATE [3] : wait for user question"<<coutcolor0<<endl;
   //          }
   // }
   // else if ( Function_state == 4)
   // {
   //     Function_state = 0;
   //     logic_state = 3;
   // }
}

void Function_3()
{
     // if ( Function_state == 0 )
     // {
     //   string cmd = "I am going to find the " + desire_object_name + " for you";
     //   say_message(cmd);
     //   Function_state = 1;
     // }
     // else if ( Function_state == 1)
     // {
     //   navigation_go_to("bedroom");
     //   Function_state = 2;
     // }
     // else if ( Function_state == 2)
     // {
     //      cout<<coutcolor_blue<<"Function [3] STATE [2] : wait for navigation to bedroom"<<coutcolor0<<endl;
  	  //   	Function_state = 3;
     // }
     // else if ( Function_state == 3)
     // {
     //     Function3_result = false;
     // 	   Function_state = 4;
     // }
     // else if ( Function_state == 4)
     // {
     //     cout<<coutcolor_blue<<"Function [3] STATE [4] : wait for object recognition"<<coutcolor0<<endl;
     // 	   Function_state = 5;
     // }
     // else if ( Function_state == 5)
     // {
     //    bool result = process_object(desire_object_name);

     //    if ( result )
     //    {
     //      //we find it
     //      string cmd = "I Find the " + desire_object_name + " for you";
     //      say_message(cmd);

     //      cmd = "i wonder, if i had my arms could i pick up the  " +  desire_object_name + " ? Oh money ! money is the problem";
     //      say_message(cmd);
          
              
     //      say_message("i am going to the room center");
     //      navigation_go_to("roomcenter");
         
     //      Function_state = 10;

     //    }
     //    else 
     //    {
     //      //we cant find it
     //      string cmd = "I Could not find the " + desire_object_name + " in bedroom ";
     //      say_message(cmd);

     //      say_message("it is better to check the shelf , maybe i will find it there ");
     //      navigation_go_to("shelf");
        
     //      Function_state = 6;
     //    }
     // }
     // else if ( Function_state == 6)
     // {
     //      cout<<coutcolor_blue<<"Function [3] STATE [6] : wait for navigation to shelf"<<coutcolor0<<endl;
  	  //   	Function_state = 7;    
     // }
     // else if ( Function_state == 7)
     // {
     //    Function3_result = false;
     // 	  Function_state = 8;
     // }
     // else if ( Function_state == 8)
     // {
     //     cout<<coutcolor_blue<<"Function [3] STATE [8] : wait for object recognition"<<coutcolor0<<endl;
     // 	   Function_state = 9;
     // }
     // else if ( Function_state == 9)
     // {
     //   bool result = process_object(desire_object_name);
   	 //   string cmd = "";
     //   if ( result )
     //    {
     //      //we find it
     //       cmd = "I Find the " + desire_object_name + " for you";
     //       say_message(cmd);
     //       cmd = "I have the  " +  desire_object_name + ". But wait! How on earth can I pick it up when I have no arms!?";
     //       say_message(cmd);
     //    }
     //    else 
     //    {
     //      //we cant find it
     //     cmd = "I Could not find the " + desire_object_name + " on the shelf ";
     //     say_message(cmd);
     //     cmd = "i want to have 2 arms like my other friends and humans ! Oh money ! money is the problem";
     //     say_message(cmd);
         
     //    }

     //      say_message("i am going to the room center");
     //      navigation_go_to("roomcenter");
     //      Function_state = 10;
     //   }
     //   else if ( Function_state == 10)
     //   {
     //   	    cout<<coutcolor_blue<<"Function [3] STATE [10] : wait for navigation to room center"<<coutcolor0<<endl;
    	//     	Function_state = 11;
    	    
     //   }
     //   else if ( Function_state == 11 )
     //   {
     //   	   //report object recognition status
     //   	   //========================================
     //   	   say_message("operation Done");
     //   	   Function_state = 0;
     //   	   logic_state = 3;
     //   }
} 

void process_speech_command(string message)
{
   isspeechready = false;
   speech_last_command = "";

   if ( logic_state == 4 )
   {
	   	if ( message == "ready")
	   	{
		   	logic_state = 5;
		   	cout<<coutcolor_green<<"Speech is ready !"<<coutcolor0<<endl;
	    }
	    else if ( message == "#error#" )
	    {
	    	logic_state = 3;
	    	cout<<coutcolor_red<<"Speech has error :"<< message <<coutcolor0<<endl;
	    }
   }
   else if ( logic_state == 5 )
   {
      //6 kitchen
      //7 questions
      //8 object
      //=================================================
       // cout<<coutcolor_green<<"SPEECH GET PROCESS : "<<message<<coutcolor0<<endl;
   	   // if ( message == "1" ){logic_state = 6;}else
   	   // if ( message == "2" ){logic_state = 7;}else
   	   // if ( message == "3" ){logic_state = 8; desire_object_name = "soda";}else
   	   // if ( message == "4" ){logic_state = 8; desire_object_name = "coffee";}else
   	   // if ( message == "5" ){logic_state = 9;}
   	 
   	   // else
   	   // {
       // cout<<coutcolor_red<<"Invalid Speech Command for logic state 5 : "<< message <<coutcolor0<<endl;
       // logic_state = 3;
   	   // }
   }
}

void logic_thread()
{
    sepanta_wait(1000);
    say_message("Sepanta Stage 1. Navigation test started");


    
    while(ros::ok() && !App_exit && false)
    {
         sepanta_wait(500);

         if ( logic_state == 0 )
         {
           print_color("idle","blue");
           change_led(0,0,0);
           isspeechready = false;
           logic_state = 1;
         }
         else
         if ( logic_state == 1 )
         {
            if ( isdooropened == false )
            {
                print_color("wait for door","brown");
                change_led(0,0,250);
            }
            else
            {
                print_color("the door is opened","green");
                say_message("The door is opened!");
                change_led(0,250,0);
                navigation_go_to(waypoint_1);
                logic_state = 2; 
            }
         }
         else
         if ( logic_state == 2 )
         {
            cout<<coutcolor_green<<"STATE [2] : wait for navigation"<<coutcolor0<<endl;
            say_message("Hello , I am Sepanta 3 the next generation of A U T Service Robot!");
            logic_state = 3;
         }
         else if ( logic_state == 3)
         {
          say_message("I am listening to your order");
         	cout<<coutcolor_green<<"STATE [3] : send start to speech"<<coutcolor0<<endl;
          logic_state = 4;
         	send_feedback_to_speech("start");
         }
         else if ( logic_state == 4)
         {
         	if ( isspeechready ) process_speech_command(speech_last_command);
          else 
          {
            cout<<coutcolor_green<<"STATE [4] : send start to speech Again !"<<coutcolor0<<endl;
            send_feedback_to_speech("start");
          }
         }
         else if ( logic_state == 5)
         {
         	cout<<coutcolor_green<<"STATE [5] : wait for command !"<<coutcolor0<<endl;
         	if ( isspeechready ) process_speech_command(speech_last_command);
         }
         else if ( logic_state == 6 )
         {
            cout<<coutcolor_green<<"STATE [6] : in operation"<<coutcolor0<<endl;
            Function_1();
         }
         else if ( logic_state == 7 )
         {
            cout<<coutcolor_green<<"STATE [7] : in operation"<<coutcolor0<<endl;
            Function_2();
         }
         else if ( logic_state == 8 )
         {
            cout<<coutcolor_green<<"STATE [8] : in operation"<<coutcolor0<<endl;
            Function_3();
         }
          else if ( logic_state == 9 )
         {
            cout<<coutcolor_green<<"STATE [9] : going to charger"<<coutcolor0<<endl;
            say_message("Ok! ,  My Battery is low . i am going to charge my selft");
            navigation_go_to("charger");
            logic_state = 10;
         }
         else if ( logic_state == 10 )
          {
            //cout<<coutcolor_green<<"STATE [10] : finshed :)"<<coutcolor0<<endl;
            cout<<coutcolor_blue<<"STATE [10] : wait for navigation to charger"<<coutcolor0<<endl;
            logic_state = 11;
          }
          else if ( logic_state == 11 )
          {
            cout<<coutcolor_blue<<"STATE [11] : say goodbye"<<coutcolor0<<endl;
            say_message("well done ! , hope to see you again ! . goodbye.");
            break;
          }

    }

     print_color("scenario logic terminated","red");
}

void chatterCallback_depth(const sensor_msgs::ImageConstPtr& input_image)
{

    
    float sum = 0;
    int count = 0;

    for ( int j = 260 ; j < 380 ; j++)
    {
      for ( int k = 180 ; k < 300 ; k++)
      {
           int x = input_image->data[k*640 + j];
           //cout<<x<<" ";
           if ( x != 0 && x != 255)
           {
           count++;
           sum += x;
           }
      }
    }

    //cout<<count<<endl;

    if ( count > 0)
    sum = sum / count;

    cout<<sum<<endl;
    //cv_bridge::CvImagePtr cv_ptr;
    //cv_ptr = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::TYPE_32FC1);

    //geometry_msgs::Point position;
    //cv::Scalar color;

    //cv::imshow("Objects Visualizer", cv_ptr->image);
    //cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scenario1");
    ros::Time::init();

    print_color("stage 1 , navigation test started","green");
  
    ros::NodeHandle node_handle;
    ros::Subscriber sub_handles[5];   
    //=========================================================================================
    pub_tts = node_handle.advertise<std_msgs::String>("/texttospeech/message", 10);
    sub_handles[0] = node_handle.subscribe("lowerbodycore/isdooropened", 10, chatterCallback_door);
    sub_handles[1] = node_handle.subscribe("/speechRec/cmd_spch", 10, chatterCallback_speech);
    sub_handles[2] = node_handle.subscribe("/texttospeech/queue", 10, chatterCallback_ttsfb);
    sub_handles[3] = node_handle.subscribe("kinect2/bodyArray",10,chatterCallback_kinect2_body);
    sub_handles[4] = node_handle.subscribe("/camera/depth/image",10,chatterCallback_depth);
    pub_spr = node_handle.advertise<std_msgs::String>("/speechRec/feedback_spch", 10);
    client_navigation = node_handle.serviceClient<sepanta_msgs::command>("sepantamovebase/command");
    say_service = node_handle.serviceClient<sepanta_msgs::command>("texttospeech/say");
    pub_led = node_handle.advertise<sepanta_msgs::led>("/lowerbodycore/led", 10);

    //==========================================================================================
    ros::Rate loop_rate(20);
    //action #1 SepantaNavigation
    ac_navigation = new actionlib::SimpleActionClient<sepanta_msgs::MasterAction>("SepantaMoveBaseAction", true);
    ROS_INFO("Waiting for action server to start [SepantaNavigation].");
    ac_navigation->waitForServer(); 
    ROS_INFO("SepantaNavigation Action Server Started OK");

    boost::thread _thread_Logic(&logic_thread);
   
    while (ros::ok() && App_exit == false)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    _thread_Logic.interrupt();
    _thread_Logic.join();
    
    App_exit = true;
    return 0;
}

