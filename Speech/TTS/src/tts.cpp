#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <stdlib.h>
#include <sepanta_msgs/command.h>
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <ctime>
#include <unistd.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <boost/thread.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;


string temp_message = "";
bool busy = false;
ros::ServiceServer service;
ros::Publisher pub_tts;
ros::Publisher pub_spr;
int global_id = 100;
bool isttsready = true;
int current_play_id = 0;
int tts_state = 0;

struct tts_data
{
    public:
        string message;
        int id;
};

std::vector<tts_data> main_queue;

void say(string message)
{
	if ( busy ) return; 
	busy = true;
	string commad = "";
	system((string("rosrun sound_play say.py \"")+message+string("\"")).c_str());
	busy = false;
}

void say_natural(string message)
{
    
    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    std_msgs::String _mes;
    _mes.data = message;
    pub_tts.publish(_mes);
}


void chatterCallback_ttsfb(const std_msgs::String::ConstPtr &msg)
{ 
    if(tts_state == 1 && msg->data=="stop")
    {
       tts_state = 0;
       std_msgs::String _msg;

       _msg.data =  boost::lexical_cast<string>(current_play_id);
       pub_spr.publish(_msg);
    }
}

bool checkcommand(sepanta_msgs::command::Request  &req,sepanta_msgs::command::Response &res)
{
    ROS_INFO("Say Service Request....");

       tts_data d;
       d.id = global_id;
      
       std::string _cmd = req.command;
       d.message = _cmd;
       main_queue.push_back(d);


       res.result = boost::lexical_cast<string>(global_id);

       global_id++;
       if ( global_id > 100000)
        global_id = 100;

    return true;
}


void _logic()
{
    cout<<"logic"<<endl;
    while(ros::ok())
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    
        if ( tts_state == 0)
        {
            cout<<"state 0"<<endl;
             if ( main_queue.size() != 0 )
             {
                 cout<<"run1"<<endl;
                tts_data d = main_queue.at(0);
                current_play_id = d.id;
                 cout<<"run2"<<endl;
                main_queue.erase(main_queue.begin());
                tts_state = 1;
                cout<<"before call"<<endl;
                say_natural(d.message);
             }
        }
        else
        {
            cout<<"state 1"<<endl;
        }


    }
}
   

int main(int argc, char **argv)
{
    ros::init(argc, argv, "texttospeech");

    ros::NodeHandle node_handles[5];
    ros::Subscriber sub_handles[5];

    boost::thread _thread_Logic(&_logic);
    //===========================================================================================
    pub_spr = node_handles[0].advertise<std_msgs::String>("/texttospeech/queue", 10);  //for clients
    pub_tts = node_handles[0].advertise<std_msgs::String>("/texttospeech/message", 10); //to minipc
    sub_handles[0] = node_handles[0].subscribe("/texttospeech/feedback", 10, chatterCallback_ttsfb); //from minipc
    service = node_handles[0].advertiseService("/texttospeech/say",checkcommand); //from clients
    

    //============================================================================================

    ros::Rate loop_rate(20);

    while (ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

     _thread_Logic.interrupt();
    _thread_Logic.join();

    return 0;
}
