#include "upperbodyclient.h"
#include <upperbodycore_msgs/objectsposition.h>
#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <math.h>

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

bool appExit = false;

using namespace std;

ros::Publisher chatter_pub_ack;
ros::Publisher chatter_pub_log;



void send_ack()
{
    std_msgs::String msg_ack;
    msg_ack.data = "ok";
    chatter_pub_ack.publish(msg_ack);
}

void send_log(std::string msg)
{
    std_msgs::String msg_log;
    msg_log.data = msg;
    chatter_pub_log.publish(msg_log);
}

int main(int argc, char **argv)
{
   
    ros::init(argc, argv, "dummy");
    std::cout<<"upperbody client started"<<std::endl;

    ros::Time::init();
    ros::Rate loop_rate(20);

    ros::NodeHandle n,n1,n2;

    chatter_pub_ack = n.advertise<std_msgs::String>("/motor/ack",10);
    chatter_pub_log = n.advertise<std_msgs::String>("/motor/log",10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        
        send_ack();
    }

    appExit = true;
  

    return 0;
}

