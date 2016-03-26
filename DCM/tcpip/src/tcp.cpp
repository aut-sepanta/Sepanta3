#include <ros/ros.h>
#include <stdio.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

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

#include <sepanta_msgs/speechkill.h>
#include <sepanta_msgs/sound.h>

//V3
//#define USB_MODE
using namespace std;
string global_tcp;

ros::Publisher chatter_pub[20];

TCPStream* stream_tcp = NULL;
TCPAcceptor* acceptor_tcp = NULL;

bool App_exit = false;

string out_put = "";
ros::ServiceServer __srv_stop;

ros::Publisher chatter_pub_ack;
ros::Publisher chatter_pub_log;

ros::ServiceServer service_tcp ;

bool isconnected = false;

std::vector<std::string> command_list;

void send_ack()
{
    std_msgs::String msg_ack;
    if ( isconnected )
    msg_ack.data = "ok";
    else
    msg_ack.data = "error";

    chatter_pub_ack.publish(msg_ack);
}

void send_log(string msg)
{
    std_msgs::String msg_log;
    msg_log.data = msg;
    chatter_pub_log.publish(msg_log);
}

void kill_server()
{
   acceptor_tcp->~TCPAcceptor();
   delete acceptor_tcp;
}

bool callkill_server(sepanta_msgs::speechkill::Request &req,sepanta_msgs::speechkill::Response &resp)
{
   kill_server();
   return true;
}

void process_command(string input,int mode)
{
     if ( mode == 1 )
     {
        cout<<"tcp : "<<input<<endl;
        send_log("tcp : " + input);
        global_tcp = input;

        std_msgs::String msg;
        msg.data = global_tcp;
        chatter_pub[1].publish(msg);
     }
}

void tcp_write_tcp(std::string msg,string mode)
{
    int len = msg.length();
    string data =  msg.c_str() ;
    if ( stream_tcp != NULL )
    {
        stream_tcp->send(data.c_str(), len);
        cout<<"write : "<<data<<endl;
        send_log("write : " + data);
    }
}

void sendd()
{
    while(ros::ok())
    {
      // tcp_write_tcp("12345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890","1");
      boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    }
}

void serial_write(string  msg,string mode)
{
  if ( isconnected == false ) return;

  #ifdef USB_MODE

    if ( main_serial == NULL ) return ;
    if ( main_serial->isOpen() == false ) return;

    string in_put = mode + "," + msg + "\n";

    cout<<"write : "<<in_put<<endl;
    main_serial->write(in_put);

  #else

    if ( mode == "1")
        tcp_write_tcp(msg,mode);


  #endif
}

std::vector<char> tcp_data;

void tcp_read_tcp() //9898
{
    stream_tcp = NULL;
    acceptor_tcp = NULL;
    acceptor_tcp = new TCPAcceptor(9898);

    if (acceptor_tcp->start() == 0) {
        while (App_exit == false) {
            cout<<"wait for tcp..."<<endl;
            stream_tcp = acceptor_tcp->accept();
            cout<<"tcp connected..."<<endl;
            send_log("tcp connected");
            if (stream_tcp != NULL) {
                ssize_t len;
                char line[1000];

                int header = 0;
                string valid_data = "";
                isconnected = true;
                while ((len = stream_tcp->receive(line, sizeof(line))) > 0 && App_exit ==false)
                {
                    //cout<<line<<endl;

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
                                            // cout<<valid_data<<endl;
                                            //=================================
                                            string temp = valid_data;
                                            temp = temp.substr(1);

                                            if ( valid_data[0] == '1' )
                                                process_command(temp,1);
                                            //==================================
                                            valid_data = "";
                                            header = 0;
                                        }
                                    }
                    }
                }
                delete stream_tcp;
                cout<<"tcp disconnected..."<<endl;
                send_log("tcp disconnected");
                isconnected = false;
            }
        }
    }
}

void chatterCallback_tcp(const std_msgs::String::ConstPtr &msg)
{
    string in_put = msg->data;
    send_log("SEND : " + in_put);
    serial_write(in_put,"1");
}

bool tcp_request(sepanta_msgs::sound::Request  &req,sepanta_msgs::sound::Response &res)
{
    res.result = true;

    send_log("SEND : " + req.command);
    serial_write(req.command,"1");

    return res.result;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "windows_communication_tcp");
    ros::Time::init();

    cout << "WINDOWS COMMUNICATION TCP STARTED" << endl;

    boost::thread _threadsp(&tcp_read_tcp);
    boost::thread _threadsend(&sendd);

    ros::Rate ros_rate(20);

    ros::NodeHandle node_handles[15];
    ros::Subscriber sub_handles[15];

    chatter_pub[1] = node_handles[1].advertise<std_msgs::String>("pgitic_tcp_out", 10); //send to speakers
    //==========================================================================================
    sub_handles[0] = node_handles[4].subscribe("pgitic_tcp_in",10,chatterCallback_tcp);
    
    ros::NodeHandle nn;
    chatter_pub_ack = nn.advertise<std_msgs::String>("/core_tcp/ack",10);
    chatter_pub_log = nn.advertise<std_msgs::String>("/core_tcp/log",10);

    service_tcp = nn.advertiseService("pgitic_service_tcp", tcp_request);

    ros::Rate loop_rate2(1); //20 Hz
    loop_rate2.sleep();

    send_log("tcp core started done...");

    while(ros::ok())
    {
        ros::spinOnce();
        ros_rate.sleep();
        send_ack();
    }

    App_exit = true;

  #ifdef USB_MODE
    _thread.interrupt();
    _thread.join();
  #else
    _threadsp.interrupt();
    _threadsp.join();
    _threadsp.~thread();


    _threadsend.interrupt();
    _threadsend.join();
    _threadsend.~thread();

    acceptor_tcp->~TCPAcceptor();
    
    delete acceptor_tcp;
    delete stream_tcp;
  #endif;

    return 0;
}
