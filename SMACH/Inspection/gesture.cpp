
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <athomerobot/tcpconnector.h>
#include <athomerobot/tcpconnector.hpp>
#include <athomerobot/tcpacceptor.hpp>
#include <athomerobot/tcpstream.hpp>

#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <errno.h>

#include <ros/ros.h>
#include <stdio.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <std_msgs/String.h>

#include "athomerobot_msgs/joint.h"
#include "athomerobot_msgs/user.h"
#include "athomerobot_msgs/users.h"

using namespace std;
bool App_exit = false;
ros::Publisher chatter_pub[20];

int last_rightx = 0;
int counter[6] = {0};
int counter_reset[6] = {0};
int sleep_wait = 0;

ros::Publisher handStopPublisher;



void chatterCallback_skeleton(const athomerobot_msgs::usersConstPtr &msg)
{
    int count = msg->count;

    if  ( sleep_wait > 10 )
    {
        for ( int i = 0 ; i < count ; i++ )
        {
            athomerobot_msgs::user item = msg->users.at(i);
            int right_handx = item.RightHand.x;
            int right_handy = item.RightHand.y;
            int left_handy = item.LeftHand.y;
            int torsoy = item.Torso.y;
            int headx = item.Head.x;
            int heady = item.Head.y;
            int headz = item.Head.z;

            if ( counter[i] > 4)
            {
                        sleep_wait = 0;
                        counter[i] = 0;
                        counter_reset[i] = 0;
                        cout << "bye bye detected for : " << i << "   X : " << item.Torso.x << " Z : " << item.Torso.z << endl;
            }

            if ( counter_reset[i] > 4)
            {
                        counter[i] = 0;
                        counter_reset[i] = 0;
            }

            //+_+_+_+_+_+_+_+_+_+_+_+_+_+_shagahyegh+_+_+_+_+_+_+_+_+_+
            if ((heady < right_handy) && (item.RightHand.mode == 2)&&(headz < 200))
            {
                    std_msgs::String gesturemsg;
                    gesturemsg.data = "stop";        
                    handStopPublisher.publish(gesturemsg);
                    ROS_INFO("stop");
            }
            else if ((heady < left_handy) && (item.LeftHand.mode == 2) && (headz < 200))
            {
                    std_msgs::String gesturemsg;
                    gesturemsg.data = "start";        
                    handStopPublisher.publish(gesturemsg);
                    ROS_INFO("start");
            } 
            
            //+_+_+_+_+_+_+_+_+_+_+_+_+_+_shagahyegh+_+_+_+_+_+_+_+_+_+

            if ( headx < right_handx && torsoy < right_handy && item.RightHand.mode == 2)
            {
                 int velocity = (int)((last_rightx - right_handx) / 0.13);
                 last_rightx = right_handx;

                 //cout<<"v "<< i <<velocity<<endl;
                if ( abs(velocity) > 60 )
                {
                    counter[i]++; 
                }

                if ( abs(velocity) < 20 )
                {
                    counter_reset[i]++; 
                }

            }
            else
            {
                counter[i] = 0;
                counter_reset[i] = 0;
            }
        }

    }
    else
    {
        sleep_wait++;
        //cout << "wait..." <<endl;

        for ( int i = 0 ; i < count ; i++ )
        {
          counter[i] = 0;
          counter_reset[i] = 0;
        }
    }    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tcp_server");
    ros::Time::init();

    cout << "SEPANTA GESTURE  STARTED DONE (93/04/02)" << endl;

    ros::Rate ros_rate(20);

    ros::NodeHandle node_handles[15];
    ros::Subscriber sub_handles[15];
    ros::NodeHandle advertiseNodeHandle;

    sub_handles[0] = node_handles[2].subscribe("AUTROBOTOUT_skeleton", 10, chatterCallback_skeleton);

    handStopPublisher = advertiseNodeHandle.advertise<std_msgs::String>("basicfunction_from_gesture", 10);/*for example AUTROBOT_from_find_me*/

    while (ros::ok())
    {
        ros::spinOnce();
        ros_rate.sleep();
    }

    App_exit = true;

    return 0;
}

