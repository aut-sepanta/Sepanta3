
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

#include <sepanta_msgs/sound.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

using namespace std;

class client_tcp 
{

private:
    bool tcp_init;
    ros::NodeHandle node_handles[100];
  
    ros::Subscriber sub_tcp;
    ros::Subscriber sub_tcp_ack;
    ros::ServiceClient service_tcp;

    void init();


public:


bool isconnected;
bool iscommandthere;

void callback_tcp(const std_msgs::String::ConstPtr &msg);
void callback_tcpack(const std_msgs::String::ConstPtr &msg);

void send_tcp(string msg);
string get_command();

string command;

client_tcp()
{
    init();
}

~client_tcp()
{

}

};
