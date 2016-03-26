#include "ros/ros.h"
#include <iostream>
#include <string>
#include <sepanta_msgs/sound.h>
#include <boost/thread.hpp>
#include "soundclient.h"
#include <boost/thread.hpp>
#include <std_msgs/String.h>
using namespace std;
std_msgs::String msg_ack;
ros::Publisher AckPublisher;
ros::Publisher LogPublisher;

bool sound_request(sepanta_msgs::sound::Request  &req,sepanta_msgs::sound::Response &res);
bool play_sound(string path);

soundclient *soundClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sound_client");
    std::cout<<"sound client started"<<std::endl;

    ros::Time::init();
    ros::Rate loop_rate(20);
    
    ros::NodeHandle nh_[4];
    AckPublisher = nh_[2].advertise<std_msgs::String>("core_sound/ack", 1);
    LogPublisher = nh_[3].advertise<std_msgs::String>("core_sound/log", 1);
    ros::ServiceServer service_play_sound = nh_[0].advertiseService("pgitic_sound", sound_request);
    soundClient = new soundclient();

 	std_msgs::String log_msg;   
    log_msg.data = "core_sound started v_1 30_7_93";
	LogPublisher.publish(log_msg);
	ROS_INFO("core_sound started v_1 30_7_93");

    ros::Rate loop_rate2(1); //20 Hz
    loop_rate2.sleep();
    log_msg.data = "Sound core started done. v_1 30_7_93";
    LogPublisher.publish(log_msg);

    while (ros::ok())
    {
        msg_ack.data = "Ok";
        AckPublisher.publish(msg_ack);
        ros::spinOnce();
        loop_rate.sleep();
    }
        soundClient->stop();
    return 0;
}
bool sound_request(sepanta_msgs::sound::Request  &req,sepanta_msgs::sound::Response &res)
{
	std_msgs::String log_msg;
	res.result = true;
	if(req.command=="play")
	{
		string log_string;
		log_string = "core_play ";
		log_string.append(req.path);
		log_msg.data = log_string;
		LogPublisher.publish(log_msg);
		ROS_INFO(" core_play ");
        cout<<req.path<<endl;
        soundClient->play(req.path);
	}
	else if(req.command=="pause")
	{
		log_msg.data=" core_pause ";
		LogPublisher.publish(log_msg);
		ROS_INFO(" core_pause ");
        soundClient->pause();
	}
	else if(req.command=="resume")
	{
		log_msg.data=" core_resume";
		LogPublisher.publish(log_msg);
		ROS_INFO(" core_resume ");
        soundClient->resume();
	}
	else if(req.command=="stop")
	{		
		log_msg.data=" core_stop ";
		LogPublisher.publish(log_msg);
		ROS_INFO(" core_stop ");
        soundClient->stop();
	}
    else if(req.command=="set_default_path")
    {
    	string log_string;
		log_string = "core_set_default_path ";
		log_string.append(req.path);
		log_msg.data = log_string;
		LogPublisher.publish(log_msg);
		ROS_INFO(" core_set_default_path ");
        soundClient->set_default_path(req.path);
    }
	return res.result;
}
