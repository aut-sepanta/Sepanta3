#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <athomerobot_msgs/slamactionAction.h>

#include "athomerobot_msgs/maptools.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include <iostream>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <pcl/common/time.h>
#include <fstream>
#include "std_msgs/Bool.h"

/////////////TODO////////////////////
// #include <athomerobot_msgs/gesture_detectAction.h>
// #include <athomerobot_msgs/face_detectAction.h>
#include <athomerobot_msgs/grip_partyAction.h>

#include <athomerobot_msgs/sepantaAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/package.h>
#include "athomerobot_msgs/arm.h"
#include "athomerobot_msgs/head.h"

//#include "emergency/position.h"
using namespace std;

#define FD_TIME 5 //TODO
#define GD_TIME 50 //TODO 
#define GF_TIME 10000 //TODO 

typedef actionlib::SimpleActionClient<athomerobot_msgs::slamactionAction> SLAMClient;
typedef actionlib::SimpleActionClient<athomerobot_msgs::grip_partyAction> GFClient;//TODO
typedef athomerobot_msgs::head head_msg;
typedef std_msgs::Int32 int_msg;

void goWithSlam(string where);
void logicThread();
void speak(string sentence);
void speechLogicCallback(const std_msgs::String &msg);

SLAMClient *globalSLAM;
// GFClient *globalGF;//TODO Bayad avaz she tebghe object
int id_global;
head_msg my_head_msg;
int_msg msg_z;


ros::Publisher speakPublisher;
ros::Publisher greenPublisher;


ros::ServiceClient client;

int state = -1;
int personCounter = 0;
int deliveredObject = 0;
int personnumforfinding = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "egpsr");
     ros::NodeHandle speechNodeHandle;
    ros::NodeHandle advertiseNodeHandle;
    ros::NodeHandle LogicNodeHandleToTask;
    ros::Subscriber speechSubscriber = speechNodeHandle.subscribe("AUTROBOTIN_speech", 1, speechLogicCallback);
    speakPublisher = advertiseNodeHandle.advertise<std_msgs::String>("/AUTROBOTOUT_speak", 10);/*for example AUTROBOT_from_find_me*/

    greenPublisher = advertiseNodeHandle.advertise<std_msgs::Bool>("AUTROBOTIN_greenlight", 10);


    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));


    state = 0;
    boost::thread logic_thread(&logicThread);

    ros::spin();

    logic_thread.join();
    logic_thread.interrupt();


}
int Counter=0;
void logicThread()
{
   
    while (ros::ok)
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        if (state == 0)
        {
            speak("hi");
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("centeroflivingroom");
            state = 1;
        }
        else if (state == 1)     // go to center room
        {
            speak("question");
            // boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
            state = -1;////////////state 2 hazv shood
        }
        else if (state == 2)
        {
           speak("sorry i can not grasp this i will come back");
           boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            goWithSlam("centeroflivingroom");
            Counter++;
             if ( Counter < 3)

                state = 1; /// state 2 hazv shood
            else
               // state = 6;
            state = 3;

        }
   
        else if (state == 3)
        {
             ROS_INFO("exit");
            goWithSlam("exit");//wait for gesture detection
            state = 4;
           

        }else if(state==4){
             ROS_INFO("finish");
            break;
        }

    }

}


void goWithSlam(string where)
{
    //edwin's code :D
    int R_TIME = 100000;
    SLAMClient slamAction("slam_action", true);
    globalSLAM = &slamAction ;
    ROS_INFO("wait for slam server");
    slamAction.waitForServer();
    ROS_INFO("connected to slam server");
    ROS_INFO("Going %s with slam...", where.c_str());
    athomerobot_msgs::slamactionGoal interfacegoal;
    interfacegoal.x = 0;
    interfacegoal.y = 0;
    interfacegoal.yaw = 0;
    interfacegoal.ID = where;

    //slamAction->sendGoal(interfacegoal);
    ROS_INFO("goal sent to slam... waiting for reach there.");

    slamAction.sendGoal(interfacegoal);
    bool finished_before_timeout = slamAction.waitForResult(ros::Duration(R_TIME));
    actionlib::SimpleClientGoalState state = slamAction.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Slam finished\n");
    }
    else
    {
        ROS_INFO("Finished Slam with current State: %s\n",
                 slamAction.getState().toString().c_str());
    }
}

void speak(string sentence)
{
    std_msgs::String msg;
    msg.data = sentence;
    speakPublisher.publish(msg);
}

void speechLogicCallback(const std_msgs::String &msg) {
    // std::cout << msg.data << std::endl;

 if ((msg.data == "speech_done") ) {
    
        ROS_INFO("speech_done");
      
    }
    else if (msg.data == "sponge"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("hallwaycenter");
            state=2;

    }else if (msg.data == "shampoo"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("centerhallway");
             state=2;
    }else if (msg.data == "brush"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("centerhallway");
             state=2;
    }else if (msg.data == "detergent"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("centerhallway");
                 state=2;
    }else if (msg.data == "toothpaste"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("centerhallway");
                 state=2;
    }else if (msg.data == "orangejuice"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("bar");
                 state=2;
    }else if (msg.data == "water"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("bar");
                     state=2;
    }else if (msg.data == "milk"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("bar");
                 state=2;
    }else if (msg.data == "energydrink"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("");
             state=2;
    }else if (msg.data == "chocolatemilk"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("bar");
                 state=2;
    }else if (msg.data == "grapejuice"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("bar");
                 state=2;
    }else if (msg.data == "beer"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("bar");
                 state=2;
    }else if (msg.data == "cola"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("bar");
                 state=2;
    }else if (msg.data == "strawberrycookies"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("centeroflivingroom");
             state=2;
    }else if (msg.data == "chocolatecookies"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("centeroflivingroom");
                 state=2;
    }else if (msg.data == "chips"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("centeroflivingroom");
                 state=2;
    }else if (msg.data == "bread"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("centeroflivingroom");
             state=2;
    }else if (msg.data == "crackers"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("centeroflivingroom");
             state=2;
    }else if (msg.data == "biscuits"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("centeroflivingroom");
             state=2;
    }else if (msg.data == "pringles"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            ROS_INFO("state=0");
            goWithSlam("centeroflivingroom");
             state=2;
    }else if (msg.data == "noodles"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
          //  ROS_INFO("state=0");
            goWithSlam("centerkitchen");
             state=2;
    }else if (msg.data == "beans"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
           // ROS_INFO("state=0");
            goWithSlam("centerkitchen");
             state=2;
    }else if (msg.data == "coffee"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
          //  ROS_INFO("state=0");
            goWithSlam("centerkitchen");
             state=2;
    }else if (msg.data == "babyfood"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            
            goWithSlam("centerkitchen");
             state=2;
    }else if (msg.data == "yeast"){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            
            goWithSlam("centerkitchen");
             state=2;
    }

}

