#include "client_sound.h"


client_sound::client_sound()
{
	service_sound = nh_[0].serviceClient<sepanta_msgs::sound>("pgitic_sound");	
}
client_sound::~client_sound()
{
	sepanta_msgs::sound srv_sound;
    srv_sound.request.command = "stop";
    bool r = service_sound.call(srv_sound);
    if ( r == false)
    {
		ROS_INFO("there is a problem check if the core program is running?");
    }
}
void client_sound::play(std::string path)
{
	    sepanta_msgs::sound srv_sound;
        srv_sound.request.command = "play";
        srv_sound.request.path = path;
        bool r = service_sound.call(srv_sound);
        if ( r == false)
        {
        	ROS_INFO("there is a problem check if the core program is running?");
		}
}
void client_sound::stop()
{
	sepanta_msgs::sound srv_sound;
    srv_sound.request.command = "stop";
    bool r = service_sound.call(srv_sound);
    if ( r == false)
    {
		ROS_INFO("there is a problem check if the core program is running?");
    }
}
void client_sound::resume()
{
    sepanta_msgs::sound srv_sound;
    srv_sound.request.command = "resume";
    bool r = service_sound.call(srv_sound);
    if ( r == false)
    {
		ROS_INFO("there is a problem check if the core program is running?");
    }
}
void client_sound::pause()
{
	sepanta_msgs::sound srv_sound;
    srv_sound.request.command = "pause";
    bool r = service_sound.call(srv_sound);
    if ( r == false)
    {
     	ROS_INFO("there is a problem check if the core program is running?");    }

	}
void client_sound::setDefaultPath(std::string path)
{
	sepanta_msgs::sound srv_sound;
    srv_sound.request.command = "set_default_path";
	srv_sound.request.path = path;
    bool r = service_sound.call(srv_sound);
    if ( r == false)
    {
		ROS_INFO("there is a problem check if the core program is running?");
    }
}

