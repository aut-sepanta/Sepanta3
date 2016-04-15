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
#include <sepanta_msgs/arm.h>
#include <sepanta_msgs/omnidata.h>
#include <sepanta_msgs/head.h>
#include <sepanta_msgs/irsensor.h>
#include <sepanta_msgs/motortorques.h>
#include <sepanta_msgs/speechkill.h>
#include <sepanta_msgs/sound.h>

int speedx = 150;
int speedy = 150;
int speedt = -150;
//V3
//#define USB_MODE
using namespace std;
string global_tcp;

ros::Publisher chatter_pub[20];

TCPStream* stream_tcp1 = NULL;
TCPAcceptor* acceptor_tcp1 = NULL;

TCPStream* stream_tcp2 = NULL;
TCPAcceptor* acceptor_tcp2 = NULL;

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
   acceptor_tcp1->~TCPAcceptor();
   delete acceptor_tcp1;

   acceptor_tcp2->~TCPAcceptor();
   delete acceptor_tcp2;
}

bool callkill_server(sepanta_msgs::speechkill::Request &req,sepanta_msgs::speechkill::Response &resp)
{
   kill_server();
   return true;
}

void set_omni(int x,int y,int w)
{
 sepanta_msgs::omnidata msg;
 msg.d0 = x;
 msg.d1 = y;
 msg.d2 = w;
 chatter_pub[2].publish(msg);
}

void robot_forward()
{
set_omni(speedx,0,0);
}

void robot_backward()
{
   set_omni(-speedx,0,0);
}

void robot_left()
{
   set_omni(0,-speedy,0);
}

void robot_right()
{
    set_omni(0,speedy,0);
}

void robot_turn_left()
{
   set_omni(0,0,-speedt);
}

void robot_turn_right()
{
   set_omni(0,0,speedt);
}

void robot_stop()
{
   set_omni(0,0,0);
}
bool es = false;


void reset_costmap()
{
cout<<"get reset costmap "<<endl;
}

void reset_hector()
{
cout<<"get reset hector "<<endl;
}

void offset_hector(string x,string y)
{
cout<<"get offset_hector "<<endl;
}

void move_x(string value)
{
cout<<"get move x"<<endl;
}

void move_y(string value)
{
cout<<"get move y "<<endl;
}

void turngl(string value)
{
cout<<"get turngl "<<endl;
}

void turnlocal(string value)
{
   cout<<"get turnlocal "<<endl;
}

void move_cancle()
{
   cout<<"get move cancle "<<endl;
}

void process_command(string input)
{
        cout<<"tcp : "<<input<<endl;
        send_log("tcp : " + input);
        global_tcp = input;

        std_msgs::String msg;
        msg.data = global_tcp;
        chatter_pub[1].publish(msg);

        //==============================================================
       std::vector<std::string> plugin_list;
       boost::algorithm::split(plugin_list,input, boost::is_any_of(","));

       if ( plugin_list[0] == "COMMAND")
       {
          if ( plugin_list[1] == "forward") robot_forward();
          if ( plugin_list[1] == "backward") robot_backward();
          if ( plugin_list[1] == "stop") robot_stop();
          if ( plugin_list[1] == "left") robot_left();
          if ( plugin_list[1] == "right") robot_right();
          if ( plugin_list[1] == "turnleft") robot_turn_left();
          if ( plugin_list[1] == "turnright") robot_turn_right();
          if ( plugin_list[1] == "es0") es = false;
          if ( plugin_list[1] == "es1") es = true;

          if ( plugin_list[1] == "reset_costmap")  reset_costmap();
          if ( plugin_list[1] == "reset_hector")  reset_hector();
          if ( plugin_list[1] == "offset_hector")  offset_hector(plugin_list[2],plugin_list[3]);

          if ( plugin_list[1] == "move_x")  move_x(plugin_list[2]);
          if ( plugin_list[1] == "move_y")  move_y(plugin_list[2]);
          if ( plugin_list[1] == "move_turnlocal")  turnlocal(plugin_list[2]);
          if ( plugin_list[1] == "move_turnto")  turngl(plugin_list[2]);
          if ( plugin_list[1] == "cancle")  move_cancle();
          

       }

}

void send_es()
{
        std_msgs::String msg;

        if ( es == false )
        	msg.data = "false";
        else
        	msg.data = "true";

        chatter_pub[3].publish(msg);
}

void tcp_write_tcp(std::string msg,string mode)
{
    int len = msg.length();
    string data =  msg.c_str() ;
    if ( stream_tcp1 != NULL )
    {
        stream_tcp1->send(data.c_str(), len);
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

void tcp_read_tcp1() //3000
{
    stream_tcp1 = NULL;
    acceptor_tcp1 = NULL;
    acceptor_tcp1 = new TCPAcceptor(3000);

    if (acceptor_tcp1->start() == 0) {
        while (App_exit == false) {
            cout<<"wait for tcp..."<<endl;
            stream_tcp1 = acceptor_tcp1->accept();
            cout<<"tcp connected..."<<endl;
            send_log("tcp connected");
            if (stream_tcp1 != NULL) {
                ssize_t len;
                char line[1000];

                int header = 0;
                string valid_data = "";
                isconnected = true;
                while ((len = stream_tcp1->receive(line, sizeof(line))) > 0 && App_exit ==false)
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
                                            process_command(temp);
                                            //==================================
                                            valid_data = "";
                                            header = 0;
                                        }
                                    }
                    }
                }
                delete stream_tcp1;
                cout<<"tcp disconnected..."<<endl;
                send_log("tcp disconnected");
                isconnected = false;
            }
        }
    }
}
void tcp_read_tcp2() //3100
{
    stream_tcp2 = NULL;
    acceptor_tcp2 = NULL;
    acceptor_tcp2 = new TCPAcceptor(3100);

    if (acceptor_tcp2->start() == 0) {
        while (App_exit == false) {
            cout<<"wait for tcp..."<<endl;
            stream_tcp2 = acceptor_tcp2->accept();
            cout<<"tcp connected..."<<endl;
            send_log("tcp connected");
            if (stream_tcp2 != NULL) {
                ssize_t len;
                char line[1000];

                int header = 0;
                string valid_data = "";
                isconnected = true;
                while ((len = stream_tcp2->receive(line, sizeof(line))) > 0 && App_exit ==false)
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
                                            process_command(temp);
                                            //==================================
                                            valid_data = "";
                                            header = 0;
                                        }
                                    }
                    }
                }
                delete stream_tcp2;
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

    boost::thread _threadsp1(&tcp_read_tcp1);
    boost::thread _threadsp2(&tcp_read_tcp2);
    boost::thread _threadsend(&sendd);

    ros::Rate ros_rate(20);

    ros::NodeHandle node_handles[15];
    ros::Subscriber sub_handles[15];

    chatter_pub[1] = node_handles[1].advertise<std_msgs::String>("tcpip/out", 1); 
    chatter_pub[2] = node_handles[2].advertise<sepanta_msgs::omnidata>("lowerbodycore/omnidrive", 1);
    chatter_pub[3] = node_handles[3].advertise<std_msgs::String>("tcpip/es", 1);
    //==========================================================================================
    sub_handles[0] = node_handles[4].subscribe("tcpip/in",10,chatterCallback_tcp);
    
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
        send_es();
        send_ack();
    }

    App_exit = true;

  #ifdef USB_MODE
    _thread.interrupt();
    _thread.join();
  #else

    _threadsp1.interrupt();
    _threadsp1.join();
    _threadsp1.~thread();

    _threadsp2.interrupt();
    _threadsp2.join();
    _threadsp2.~thread();

    _threadsend.interrupt();
    _threadsend.join();
    _threadsend.~thread();

    acceptor_tcp1->~TCPAcceptor();
    acceptor_tcp2->~TCPAcceptor();
    
    delete acceptor_tcp1;
    delete stream_tcp1;

    delete acceptor_tcp2;
    delete stream_tcp2;

  #endif;

    return 0;
}
