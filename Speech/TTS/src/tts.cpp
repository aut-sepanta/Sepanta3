#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <stdlib.h>

using namespace std;

string temp_message = "";
bool busy = false;

void say(string message)
{
	if ( busy ) return; 
	busy = true;
	string commad = "";
	system((string("rosrun sound_play say.py \"")+message+string("\"")).c_str());
	busy = false;
}

void chatterCallback_speech(const std_msgs::String::ConstPtr &msg)
{
	if ( temp_message == msg->data ) return;
	if ( msg->data == "" ) return;

    say(msg->data);
    temp_message = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "texttospeech");

    ros::NodeHandle node_handles[5];
    ros::Subscriber sub_handles[5];

    //===========================================================================================
    sub_handles[0] = node_handles[0].subscribe("/texttospeech/message", 10, chatterCallback_speech);
    //============================================================================================

    ros::Rate loop_rate(20);

    while (ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
