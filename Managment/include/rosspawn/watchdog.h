#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <map>
#include <string>
#include <utility>
#include <fstream>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>


#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <dirent.h>
#include <signal.h>
#include <unistd.h>
#include <regex.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <ros/package.h>

#include <iostream>
#include <string>
#include <stdlib.h>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"
#include <dynamixel_msgs/MotorStateList.h>
#include <visualization_msgs/Marker.h>

class watchdog
{

public:
    ros::Subscriber sub_kinect;
    ros::Subscriber sub_ack;
    boost::thread *global_wd;
    bool signal;
    bool ack;
    int timeout;
    std::string callback_name;
    int callback_mode;
    int wd_counter;
    int wd_max;
    bool appexit;
    std::string name;
    std::string ack_mode;

    watchdog(int vtimeout,std::string callname,std::string node_name,std::string mode)
    {
        ack_mode = mode;
        wd_max = 5;
        name = node_name;
        callback_name = callname;
        callback_mode = 0;
        ack = false;
        signal = false;
        timeout = vtimeout;
        appexit = false;
        init();
    }
    
    ~watchdog()
    {
       appexit = true;
    }

    void wd();
    void callbackRawImage_kinectrgb(const sensor_msgs::Image::ConstPtr& msg);
    void callbackRaw_ack(const std_msgs::String::ConstPtr& msg);
    void callbackRaw_motor(const dynamixel_msgs::MotorStateList::ConstPtr &msg);
    void callbackRaw_marker(const visualization_msgs::Marker::ConstPtr& msg);
    void init();
    void kill();
    
};
