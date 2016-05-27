
//ROS
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>

#include <stdio.h>
#include <stdlib.h>
#include <tcpacceptor.h>
#include <tcpacceptor.hpp>
#include <tcpstream.hpp>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <errno.h>

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <dirent.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/algorithm/string.hpp>


TCPStream* stream = NULL;
TCPAcceptor* acceptor = NULL;

bool mutex = false;
bool tcp_can = false;

void process_commandx(string input)
{
    return;
    if ( mutex  ) return;
    mutex = true;

     vector<string> strs2;
     boost::split(strs2,input,boost::is_any_of(","));

     if ( strs2.size() > 1)
     {
         if ( strs2.at(0) == "CAM1" ||  strs2.at(0) == "CAM2" ||  strs2.at(0) == "CAM3" )
         {
             string data = strs2.at(1);
             cout<<"GET CAM :" <<data<<endl;
         }
     }


    mutex = false;
}

void tcpsendX(string message)
{
    message =  message;
    if ( stream != NULL && tcp_can)
    {
        stream->send(message.c_str(),message.size());
        cout<<"TCP SEND DONE"<<endl;
    }
    else
    {
        cout<<"TCP SEND Failed"<<endl;
    }
}


int tcpserver_mainX()
{
    cout<<"SIM SERVER SERVER STARTED DONE"<<endl;
    //listener
    acceptor = new TCPAcceptor(4001);

    if (acceptor->start() == 0) {


        while (1) {
            stream = acceptor->accept();

            if (stream != NULL) {
                ssize_t len;
                char line[100];

                cout<<"Unity Sim Connected"<<endl;
                tcp_can = true;  
                int header = 0;
                string valid_data = "";

                //read
                while ((len = stream->receive(line, sizeof(line))) > 0) {
                    line[len] = 0;
                    

                    // %data$
                    for ( int i = 0 ; i < len ; i++)
                    {
                        if ( line[i] == '%' && header == 0)
                        {
                            header++;
                        }
                        else
                            if ( header == 1)
                            {
                                if ( line[i] != '$')
                                    valid_data += line[i];
                                else
                                {
                                    string temp = valid_data;
                                    process_commandx(temp);
                                    valid_data = "";
                                    header = 0;
                                }
                            }
                    }
                }
                tcp_can = false;
                delete stream;
                cout<<"Unity Sim Disconnected"<<endl;
            }
        }
    }

}

void get_loop()
{

}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "simserver");
    ros::Time::init();
    cout<<"Sim server started done V 1.0.0"<<endl;

    ros::Rate loop_rate(20);

    boost::thread _thread_logic1(&tcpserver_mainX);
    boost::thread _thread_logic2(&get_loop);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


}
