#include "soundclient.h"
using namespace std;

soundclient::soundclient()
{
	LogPublisher = nh_.advertise<std_msgs::String>("core_sound/log",1);
    default_path = getenv("HOME");
    default_path = default_path+"/Desktop";

}
soundclient::~soundclient()
{
    system("xmms2 stop");
}
void soundclient::play_sound(std::string path)
{
    bool response=true;
    std::string str,str2;
    str="xmms2 add ";
    str2=path;
    std::size_t found = path.find(" ");
    if (found != std::string::npos)
    {
        response = false;
        std_msgs::String sound_log;
        sound_log.data = "error :file path shouldn't have space";
        LogPublisher.publish(sound_log);
        std::cout<<"error: file path shouldnt have space"<<std::endl;
    }
    std::size_t foundP = path.find("(");
    if (found != std::string::npos)
    {
        response = false;
        std_msgs::String sound_log;
        sound_log.data = "error :file path shouldn't have ( )";
        LogPublisher.publish(sound_log);
        std::cout<<"error: file path shouldnt have ( )"<<std::endl;
    }
    std::size_t foundSlash = path.find("/");
    if (foundSlash != std::string::npos)
    {
        str.append(str2);
    }
    else
    {
        str.append(default_path);
        str.append("/");
        str.append(str2);
    }
    system("xmms2 clear");
    // const boost::filesystem::path &base_dir = str;
    // if(!boost::filesystem::exists (base_dir))
    // {
    // 	std_msgs::String sound_log;
    //     sound_log.data = "File doesn't exist";
    //     LogPublisher.publish(sound_log);
    //     cout<<"file doesn't exist"<<endl;
    // }
    // else
    // {
    	system(str.c_str());
    	system("xmms2 play");
    	string log_string;
    	log_string = "play";
    	log_string.append(str.c_str());
           sound_log.data = "play ";
        LogPublisher.publish(sound_log);
	    cout<<"play "<<str<<endl;

    // }
}
void soundclient::play(std::string path)
{
    system("xmms2 stop");
    play_sound(path);
}
void soundclient::stop()
{
	sound_log.data = "stop";
    LogPublisher.publish(sound_log);
    cout<<"stop"<<endl;
    system("xmms2 stop");
    system("xmms2 clear");
}
void soundclient::resume()
{
	sound_log.data = "resume";
    LogPublisher.publish(sound_log);
    cout<<"resume"<<endl;
    system("xmms2 play");
}
void soundclient::pause()
{
	sound_log.data = "pause";
    LogPublisher.publish(sound_log);
    cout<<"pause"<<endl;
    system("xmms2 pause");
}
void soundclient::set_default_path(std::string path)
{
	sound_log.data = "set default path";
    LogPublisher.publish(sound_log);
    cout<<"set default path"<<endl;
    default_path = path;
}
