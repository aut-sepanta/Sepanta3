
//ROS
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>

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
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/highgui/highgui_c.h"
#include <opencv2/core/core.hpp>
#include "opencv/cv.h"
#include "opencv2/calib3d/calib3d.hpp"
//********************************************** cv_bridge
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <sepanta_msgs/motor.h>

ros::Publisher chatter_pub[20];

TCPStream* stream = NULL;
TCPAcceptor* acceptor1 = NULL;

TCPStream* streamc = NULL;
TCPAcceptor* acceptorc = NULL;


bool mutex = false;
bool tcp_can = false;
bool tcp_canc = false;

float ratio_X = 15.2;
float ratio_Y = 15.3;
float ratio_W = 1800;

cv_bridge::CvImagePtr cv_ptr;

typedef unsigned char BYTE;

static const std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";


static inline bool is_base64(BYTE c) {
    return (isalnum(c) || (c == '+') || (c == '/'));
}


std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len) {
  std::string ret;
  int i = 0;
  int j = 0;
  unsigned char char_array_3[3];
  unsigned char char_array_4[4];

  while (in_len--) {
    char_array_3[i++] = *(bytes_to_encode++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for(i = 0; (i <4) ; i++)
        ret += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i)
  {
    for(j = i; j < 3; j++)
      char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] = char_array_3[2] & 0x3f;

    for (j = 0; (j < i + 1); j++)
      ret += base64_chars[char_array_4[j]];

    while((i++ < 3))
      ret += '=';

  }

  return ret;

}

std::vector<BYTE> base64_decode(std::string const& encoded_string) {
    int in_len = encoded_string.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    BYTE char_array_4[4], char_array_3[3];
    std::vector<BYTE> ret;

    while (in_len-- && ( encoded_string[in_] != '=') && is_base64(encoded_string[in_])) {
        char_array_4[i++] = encoded_string[in_]; in_++;
        if (i ==4) {
            for (i = 0; i <4; i++)
                char_array_4[i] = base64_chars.find(char_array_4[i]);

            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

            for (i = 0; (i < 3); i++)
                ret.push_back(char_array_3[i]);
            i = 0;
        }
    }

    if (i) {
        for (j = i; j <4; j++)
            char_array_4[j] = 0;

        for (j = 0; j <4; j++)
            char_array_4[j] = base64_chars.find(char_array_4[j]);

        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

        for (j = 0; (j < i - 1); j++) ret.push_back(char_array_3[j]);
    }

    return ret;
}


ros::Publisher _pub_pan;
ros::Publisher _pub_tilt;


void update_motors(int a,int b)
{
	sepanta_msgs::motor _msg;
	_msg.position = a;
	_msg.speed = 150;

	_pub_pan.publish(_msg);


	sepanta_msgs::motor _msg2;
	_msg2.position = b;
	_msg2.speed = 150;

	_pub_tilt.publish(_msg2);


}
void process_commandx(string input)
{ 
	
    vector<string> strs2;
    boost::split(strs2,input,boost::is_any_of("|"));

    //std::cout<<strs2.at(0)<<std::endl;
    //std::cout<<strs2.at(1)<<std::endl;

    std::string _input = strs2.at(1).substr(1,strs2.at(1).size()-2);


    vector<string> strs3;
    boost::split(strs3,_input,boost::is_any_of(","));
 

    int a = atoi(strs3[0].c_str());
    int b = atoi(strs3[1].c_str());


    

    int pan_mid = 500;
    if ( b > 180 ) b = -1 * (360 - b);

    int final_pan = pan_mid - b;

    //====================================

    int tilt_mid = 3000;
    int tilt_min = 2600;
    int tilt_max = 3400;


    if ( a > 180 ) a = -1 * (360 - a);
    a = a * 4;

    int final_tilt = tilt_mid + a;

    std::cout<<final_pan<<" "<<final_tilt<<std::endl;


    update_motors(final_pan,final_tilt);

}

bool x = false;
void tcpsendX(string message)
{
   // if ( stream != NULL && tcp_can)
   // {
    //    stream->send(message.c_str(),message.size());
     //   cout<<"TCP SEND DONE"<<endl;
   // }
    //else
   // {
    //    cout<<"TCP SEND Failed"<<endl;
    //}
}

int cx = 0;
void tcpsendC(char message[],int len)
{
    //cx++;
   // if ( cx < 5 ) return;
    
    //cx = 0;
    
    if ( streamc != NULL && tcp_canc)
    {
    	char head[4];
    	//uint32_t size = htonl(len);
        uint32_t size = len;

    	streamc->send((char*)&size, sizeof(size));
        streamc->send(message,len);
        //x  = true;
        //std::cout<<message<<std::endl;
       // cout<<"TCP SEND DONE CAM"<<endl;
    }
    else
    {
        cout<<"TCP SEND Failed CAM"<<endl;
    }
}

int tcpserver_mainX()
{
    cout<<"Main Server Starated : 4001"<<endl;
    //listener
    acceptor1 = new TCPAcceptor(4001);

    if (acceptor1->start() == 0) {


        while (1) {
            stream = acceptor1->accept();

            if (stream != NULL) {
                ssize_t len;
                char line[100];

                cout<<"Unity Sim main Connected"<<endl;
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

                                    //cout<<"GET :"<<temp<<endl;
                                    process_commandx(temp);
                                    valid_data = "";
                                    header = 0;
                                }
                            }
                    }
                }
                tcp_can = false;
                delete stream;
                cout<<"Unity Sim main Disconnected"<<endl;
            }
        }
    }

}

int tcpserver_main_cam()
{
    //TCPStream* stream = NULL;
    //TCPAcceptor* acceptor = NULL;
    cout<<"CAM SERVER STARTED : 4010"<<endl;
    //listener
    acceptorc = new TCPAcceptor(4010);

    if (acceptorc->start() == 0) {

        while (1) {
            streamc = acceptorc->accept();

            if (streamc != NULL) {
                ssize_t len;
                char line[100];

                cout<<"CAM CONNECTED"<<endl;
                tcp_canc = true;
                int header = 0;
                string valid_data = "";

                //read
                while ((len = streamc->receive(line, sizeof(line))) > 0) {
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
                                    //process_cam(temp);
                                    valid_data = "";
                                    header = 0;
                                }
                            }
                    }
                }
                tcp_canc = false;
                delete streamc;
                cout<<"CAM DISCONNECTED"<<endl;
            }
        }
    }

}

void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg) 
{
	if (msg->width != 0 )
	{
		try 
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

                        std::vector<uchar> buf;


		                cv::imencode(".jpg", cv_ptr->image, buf);
           
                        //uchar *enc_msg = new uchar[buf.size()];
                        //for(int i=0; i < buf.size(); i++) enc_msg[i] = buf[i];
                        //string encoded = base64_encode(enc_msg, buf.size());
                        
                        tcpsendC((char*)&(buf[0]),buf.size());
                        //std::cout<<encoded<<std::endl;

		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

}



int main (int argc, char** argv)
{
    ros::init(argc, argv, "simserver");
    ros::Time::init();
    cout<<"ROS-UNITY INTERFACE"<<endl;

    ros::Rate loop_rate(20);

    ros::NodeHandle node_handles[50];
    ros::Subscriber sub_handles[15];

    //=======================================
    //advertise
    chatter_pub[0] = node_handles[0].advertise<sensor_msgs::Image>("simserver/uav1/cam", 10);
    sub_handles[0] = node_handles[1].subscribe("/camera/rgb/image_color", 1, rosImageCallBack);

    _pub_pan = node_handles[10].advertise<sepanta_msgs::motor>("upperbodycorein_head_yaw", 1);
    _pub_tilt = node_handles[10].advertise<sepanta_msgs::motor>("upperbodycorein_head_pitch",1);
  
    boost::thread _thread_logic1(&tcpserver_main_cam);
    boost::thread _thread_logic22(&tcpserver_mainX);
   
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

}
