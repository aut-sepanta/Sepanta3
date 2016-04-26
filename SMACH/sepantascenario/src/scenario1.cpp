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

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;
using namespace ros;

//=============================================================

struct object_data
{
  public:
    string name;
    geometry_msgs::Pose _3dlocation;
    geometry_msgs::Point _2dlocation;

};

std::vector<object_data> object_list;
std::string coutcolor0 = "\033[0;0m";
std::string coutcolor_red = "\033[0;31m";
std::string coutcolor_green = "\033[0;32m";
std::string coutcolor_blue = "\033[0;34m";
std::string coutcolor_magenta = "\033[0;35m";
std::string coutcolor_brown = "\033[0;33m";
bool App_exit = false;
ros::Publisher pub_tts;
ros::Publisher pub_spr;
bool say_enable = true;

int logic_state = 0;
bool isdooropened = false;
bool isrobotmove = false;
bool isspeechready = false;
bool isobjectready = false;

string speech_last_command = "";
string temp_speech_last_command = "";
ros::ServiceClient client_navigation;
ros::ServiceClient client_object_on;
ros::ServiceClient client_object_off;

string desire_object_name = "";

void object_recognition_start()
{
   isobjectready = false;
    std_srvs::Empty _srv;
    client_object_on.call(_srv);
   

}

void object_recognition_stop()
{
    std_srvs::Empty _srv;
    client_object_off.call(_srv);
    
}



void chatterCallback_object(const sepanta_msgs::Objects::ConstPtr &msg)
{
  
   if ( isobjectready == false)
   {
    
     string out_report = "";
     object_list.clear();
     for ( int i = 0 ; i < msg->objects.size() ; i++)
      {
         object_data d;
         d.name = msg->objects.at(i).label;
         d._3dlocation = msg->objects.at(i).pose;
         d._2dlocation = msg->objects.at(i).center_2d;
         object_list.push_back(d);
         if ( i != msg->objects.size() - 1)
         out_report += d.name + " | ";
         else
        out_report += d.name;

      }

      cout<<coutcolor_magenta<<"GET objects :"<<" "<<out_report<<coutcolor0<<endl;
      isobjectready = true;
      cout<<coutcolor_brown<<"isobjectready : "<<isobjectready<<out_report<<coutcolor0<<endl;
       object_recognition_stop();
      
  }
 
}

void navigation_go_to(string location)
{
   if ( isrobotmove ) return;


   sepanta_msgs::command _msg;
   _msg.request.id = location;
   _msg.request.command = "exe";
   client_navigation.call(_msg);

     boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
}

void navigation_cancel()
{
   if ( isrobotmove == false ) return;
   sepanta_msgs::command _msg;
   _msg.request.command = "cancel";
   client_navigation.call(_msg);
}

void say_message(string data)
{
    if ( say_enable == false ) return;
    std_msgs::String _mes;
    _mes.data = data;
    pub_tts.publish(_mes);
}


void chatterCallback_door(const std_msgs::Bool::ConstPtr &msg)
{
   isdooropened = msg->data;
}

void chatterCallback_move(const std_msgs::Bool::ConstPtr &msg)
{
   isrobotmove = msg->data;
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

void send_feedback_to_speech(string cmd)
{
   std_msgs::String _msg;
   _msg.data = cmd;
   pub_spr.publish(_msg);
}




bool process_object(string name)
{
	//comming soon !
  for ( int i = 0 ; i < object_list.size() ; i++)
  {
     if ( name == object_list.at(i).name)
     {
      return true;
     }
  }

  return false;
}

int Function_state = 0;
bool Function1_result = false;
bool Function2_result = false; //dummy
bool Function3_result = false;
//Function
//1 Go to kitchen and do object recognition for something
void Function_1()
{
   if ( Function_state == 0 )
   {

      say_message("I am going to kitchen and i should bring a coca for you!");
      Function_state = 1;
   }
   else if ( Function_state == 1)
   {
      navigation_go_to("kitchen");
      Function_state = 2;
   }
   else if ( Function_state == 2)
   {
        cout<<coutcolor_blue<<"Function [1] STATE [2] : wait for navigation to kitchen"<<coutcolor0<<endl;

	    if ( isrobotmove == false)
	    {
	    	Function_state = 3;
	    }
   }
   else if ( Function_state == 3)
   {
   	    //call for object recognition
      Function1_result = false;
       object_recognition_start();
        cout<<coutcolor_blue<<"Function [1] STATE [3] : Send Start for object"<<coutcolor0<<endl;
       Function_state = 4;
   }
   else if ( Function_state == 4)
   {
   	   cout<<coutcolor_blue<<"Function [1] STATE [4] : wait for object recognition"<<coutcolor0<<endl;

   	   if ( isobjectready )
   	   {
   	   	  Function_state = 5;
   	   }
   }
   else if ( Function_state == 5)
   {
       Function1_result = process_object("coca");
      if ( Function1_result )
      {
             say_message("well done ! , i found the coca !");
             boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

             say_message("i wonder, if i had my arms could i pick up the coca? Oh money ! money is the problem");
             boost::this_thread::sleep(boost::posix_time::milliseconds(10000));

      }
      else
      {
             say_message("I cant find the coca.");
              boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

             say_message("i want to have 2 arms like my other friends and humans !");
             boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
            
      }


      Function_state = 6;
   }
   else if ( Function_state == 6)
   {
      say_message("i am goint to room center to report my perception");
      navigation_go_to("roomcenter");
      Function_state = 7;
   }
   else if ( Function_state == 7)
   {
   	   cout<<coutcolor_blue<<"Function [1] STATE [7] : wait for navigation to room center"<<coutcolor0<<endl;

	    if ( isrobotmove == false)
	    {
	    	Function_state = 8;
	    }
   }
   else if ( Function_state == 8 )
   {
   	   //report object recognition status
   if ( Function1_result )
      {
             say_message("well done ! , i found the coca !");
             boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

             say_message("i wonder, if i had my arms could i pick up the coca? Oh money ! money is the problem");
             boost::this_thread::sleep(boost::posix_time::milliseconds(10000));

      }
      else
      {
             say_message("I cant find the coca.");
              boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

             say_message("i want to have 2 arms like my other friends and humans !");
             boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
            
      }
   	   //========================================
   	   say_message("operation Done");

   	   Function_state = 0;
   	   logic_state = 3;
   }
}

int question_counter = 0;
//3 Do Speech Test (50 Questions)
void Function_2()
{
   cout<<"Header :"<<Function_state<<endl;
   if ( Function_state == 0 )
   {
     question_counter = 0;
   	 say_message("It is my pleasure to answare your questions!");
   	 Function_state = 1;
   }
   else if ( Function_state == 1)
   {
      send_feedback_to_speech("qstart");
      Function_state = 2;
   }
   else if ( Function_state == 2)
   {
      if ( isspeechready )
      { 
          isspeechready = false;
          if ( speech_last_command == "qready")
          {
            //say_message("ask");
            Function_state = 3;
          }
          else
          {

             send_feedback_to_speech("qstart");
          }
      }
   }
   else if ( Function_state == 3)
   {
            //Ready 
            if ( isspeechready )
            {
               isspeechready = false;
               if ( speech_last_command != "0")
               {
                 say_message(speech_last_command);
                 boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
                 if ( question_counter < 4)
                 {

                 say_message("Next");
                
                 question_counter++;
                 Function_state = 1;
                 }
                 else
                 {
                    say_message("Finished");
                    Function_state = 4;
                 }
               }
               else
               {
                   Function_state = 1;
               }
            }
            else
            {
                  cout<<coutcolor_blue<<"Function [2] STATE [3] : wait for user question"<<coutcolor0<<endl;
            }
   }
   else if ( Function_state == 4)
   {
       Function_state = 0;
       logic_state = 3;
   }
}

//3 Find a Object
void Function_3()
{
   if ( Function_state == 0 )
   {
   

      string cmd = "I am going to find the " + desire_object_name + " for you";
        say_message(cmd);
          boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

     Function_state = 1;
   }
   else if ( Function_state == 1)
   {
     navigation_go_to("bedroom");
     Function_state = 2;
   }
   else if ( Function_state == 2)
   {
        cout<<coutcolor_blue<<"Function [3] STATE [2] : wait for navigation to bedroom"<<coutcolor0<<endl;

	    if ( isrobotmove == false)
	    {
	    	Function_state = 3;
	    }
   }
   else if ( Function_state == 3)
   {
     Function3_result = false;
   	   object_recognition_start();
   	   Function_state = 4;
   }
   else if ( Function_state == 4)
   {
       cout<<coutcolor_blue<<"Function [3] STATE [4] : wait for object recognition"<<coutcolor0<<endl;

   	   if ( isobjectready )
   	   {
   	   	  Function_state = 5;
   	   }
   }
   else if ( Function_state == 5)
   {
      bool result = process_object(desire_object_name);

      if ( result )
      {
        //we find it
        string cmd = "I Find the " + desire_object_name + " for you";
        say_message(cmd);
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
          say_message("i am going to the room center");
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
         navigation_go_to("roomcenter");
       
          Function_state = 10;

      }
      else 
      {
        //we cant find it
        string cmd = "I Could not find the " + desire_object_name + " in bedroom ";
        say_message(cmd);
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
         say_message("it is better to check the shelf , maybe i will find it there ");
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
        navigation_go_to("shelf");
      
        Function_state = 6;
      }
   }
   else if ( Function_state == 6)
   {
        cout<<coutcolor_blue<<"Function [3] STATE [6] : wait for navigation to shelf"<<coutcolor0<<endl;

	    if ( isrobotmove == false)
	    {
	    	Function_state = 7;
	    }
   }
   else if ( Function_state == 7)
   {
      Function3_result = false;

   	   object_recognition_start();
   	   Function_state = 8;
   }
   else if ( Function_state == 8)
   {
       cout<<coutcolor_blue<<"Function [3] STATE [8] : wait for object recognition"<<coutcolor0<<endl;

   	   if ( isobjectready )
   	   {
   	   	  Function_state = 9;
   	   }
   }
   else if ( Function_state == 9)
   {
      bool result = process_object(desire_object_name);
 string cmd = "";
     if ( result )
      {
        //we find it
         cmd = "I Find the " + desire_object_name + " for you";
       
       
      }
      else 
      {
        //we cant find it
       cmd = "I Could not find the " + desire_object_name + " on the shelf ";
       
        
       
      }

        say_message(cmd);
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
        say_message("i am going to the room center");
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
        navigation_go_to("roomcenter");
      
        Function_state = 10;
   }
  else if ( Function_state == 10)
   {
   	   cout<<coutcolor_blue<<"Function [3] STATE [10] : wait for navigation to room center"<<coutcolor0<<endl;

	    if ( isrobotmove == false)
	    {
	    	Function_state = 11;
	    }
   }
   else if ( Function_state == 11 )
   {
   	   //report object recognition status


   	   //========================================
   	   say_message("operation Done");
   	   Function_state = 0;
   	   logic_state = 3;
   }


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


       cout<<coutcolor_green<<"SPEECH GET PROCESS : "<<message<<coutcolor0<<endl;
   	   if ( message == "1" ){logic_state = 6;}else
   	   if ( message == "2" ){logic_state = 7;}else
   	   if ( message == "3" ){logic_state = 8; desire_object_name = "coffee";}else
   	   if ( message == "4" ){logic_state = 8; desire_object_name = "spray";}else
   	   if ( message == "5" ){logic_state = 8; desire_object_name = "coca";}else
       if ( message == "6" ){logic_state = 9;}
   	 
   	   else
   	   {
       cout<<coutcolor_red<<"Invalid Speech Command for logic state 5 : "<< message <<coutcolor0<<endl;
       logic_state = 3;
   	   }
   }
}

void logic_thread()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    say_message("sepanta demo scenario started");

    while(ros::ok() && !App_exit)
    {
         boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

         if ( logic_state == 0 )
         {
           cout<<coutcolor_green<<"idle"<<coutcolor0<<endl;
           isspeechready = false;
           isobjectready = false;
           logic_state = 1;
         }
         else
         if ( logic_state == 1 )
         {
            //init state
            if ( isdooropened == false )
            {
                cout<<coutcolor_green<<"wait for door"<<coutcolor0<<endl;
            }
            else
            {
                cout<<coutcolor_green<<"The door is opened"<<coutcolor0<<endl;
                say_message("The door is opened!");
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                navigation_go_to("roomcenter");
                logic_state = 2; //wait for navigation
              
            }
           
         }
         else
         if ( logic_state == 2 )
         {
            //wait for navigation
            cout<<coutcolor_green<<"STATE [2] : wait for navigation"<<coutcolor0<<endl;

            if ( isrobotmove == false)
            {
            	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                say_message("Hello , I am Sepanta 3 the next generation of Amir kaa bir University of Technology Service Robot!");
            	boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
            	logic_state = 3;
            }
         }
         else if ( logic_state == 3)
         {
            say_message("I am listening to your order");
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
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
            boost::this_thread::sleep(boost::posix_time::milliseconds(7000));

            navigation_go_to("charger");

            logic_state = 10;
         }
         else if ( logic_state == 10 )
          {
               //cout<<coutcolor_green<<"STATE [10] : finshed :)"<<coutcolor0<<endl;
                cout<<coutcolor_blue<<"STATE [10] : wait for navigation to charger"<<coutcolor0<<endl;

                if ( isrobotmove == false)
                {
                  logic_state = 11;
                }

            
          }
          else if ( logic_state == 11 )
          {
              cout<<coutcolor_blue<<"STATE [11] : say goodbye"<<coutcolor0<<endl;
            say_message("well done ! , hope to see you again ! . goodbye.");
            boost::this_thread::sleep(boost::posix_time::milliseconds(7000));
            break;
          }

    }

     cout<<coutcolor_green<<"Terminated :)"<<coutcolor0<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scenario1");
    ros::Time::init();

    ROS_INFO("Sepanta Scenario Started v 1.0");
    
    boost::thread _thread_Logic(&logic_thread);

    ros::NodeHandle node_handles[10];
    ros::Subscriber sub_handles[10];

    pub_tts = node_handles[0].advertise<std_msgs::String>("/texttospeech/message", 10);
    sub_handles[0] = node_handles[1].subscribe("lowerbodycore/isdooropened", 10, chatterCallback_door);
    sub_handles[1] = node_handles[2].subscribe("/speechRec/cmd_spch", 10, chatterCallback_speech);
    pub_spr = node_handles[3].advertise<std_msgs::String>("/speechRec/feedback_spch", 10);
    client_navigation = node_handles[4].serviceClient<sepanta_msgs::command>("sepantamovebase/command");
    client_object_on  = node_handles[4].serviceClient<std_srvs::Empty>("object_recognition/turn_on");
    client_object_off = node_handles[4].serviceClient<std_srvs::Empty>("object_recognition/turn_off");
    sub_handles[2] = node_handles[5].subscribe("lowerbodycore/isrobotmove",10,chatterCallback_move);
    sub_handles[3] = node_handles[6].subscribe("object_recognition/objects",10,chatterCallback_object);

    ros::Rate loop_rate(20);

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

