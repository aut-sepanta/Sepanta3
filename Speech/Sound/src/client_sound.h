#include <ros/ros.h>
#include <sepanta_msgs/sound.h>

class client_sound
{
	private:
	     ros::ServiceClient service_sound;
	     ros::NodeHandle nh_[1];
	public:
	    client_sound();
	    ~client_sound();
	    void play(std::string path);
	    void stop();
	    void resume();
	    void pause();
	    void setDefaultPath(std::string path);
};
