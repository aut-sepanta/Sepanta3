#include <ros/ros.h>
#include <sepanta_msgs/sound.h>
#include <std_msgs/String.h>
#include <boost/filesystem.hpp>

class soundclient
{

	private:
		ros::NodeHandle nh_;
		ros::Publisher LogPublisher;
		std::string default_path;
	 	std_msgs::String sound_log;

		void play_sound(std::string path);
	public:
	    soundclient();
	    ~soundclient();
	    void play(std::string path);
	    void stop();
	    void resume();
	    void pause();
	    void set_default_path(std::string path);
};
