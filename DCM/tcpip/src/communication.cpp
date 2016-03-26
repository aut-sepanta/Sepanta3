#include <ros/ros.h>
#include <stdio.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "serial/serial.h"

#include "athomerobot_msgs/joint.h"
#include "athomerobot_msgs/user.h"
#include "athomerobot_msgs/users.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include "athomerobot_msgs/windows.h"

#include <stdio.h>
#include <stdlib.h>
#include <athomerobot/tcpacceptor.h>
#include <athomerobot/tcpacceptor.hpp>
#include <athomerobot/tcpstream.hpp>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <errno.h>

//V3
//#define USB_MODE

using namespace std;
string global_speech;
int global_z;

ros::Publisher chatter_pub[20];
serial::Serial *main_serial;

TCPStream* stream_speech = NULL;
TCPAcceptor* acceptor_speech = NULL;

TCPStream* stream_kinect = NULL;
TCPAcceptor* acceptor_kinect = NULL;

bool App_exit = false;

string out_put = "";

struct joint
{
public :
    int x;
    int y;
    int z;
    int img_x;
    int img_y;
    int mode;
};

struct kinect_user
{
    joint Head;
    joint Torso;
    joint RightHand;
    joint LeftHand;
    joint LeftFoot;
    joint RightFoot;
    int id;
};

std::vector<kinect_user> user_list;

int gg_z;
int gg_x;
int gg_tracked;


void send_skeleton()
{
    athomerobot_msgs::users msg;
    msg.count = user_list.size();
    msg.x = gg_x;
    msg.z = gg_z;
    msg.tracked = gg_tracked;
    
    //cout<<user_list.size()<<endl;

    if ( msg.count != 0 )
    {
        for ( int i = 0 ; i < msg.count ; i++ )
        {
            kinect_user item_user = user_list.at(i);

            athomerobot_msgs::user msg_user;

            msg_user.id = item_user.id;
            msg_user.Head.x = item_user.Head.x;
            msg_user.Head.y = item_user.Head.y;
            msg_user.Head.z = item_user.Head.z;
            msg_user.Head.img_x = item_user.Head.img_x;
            msg_user.Head.img_y = item_user.Head.img_y;
            msg_user.Head.mode = item_user.Head.mode;

            msg_user.LeftHand.x = item_user.LeftHand.x;
            msg_user.LeftHand.y = item_user.LeftHand.y;
            msg_user.LeftHand.z = item_user.LeftHand.z;
            msg_user.LeftHand.img_x = item_user.LeftHand.img_x;
            msg_user.LeftHand.img_y = item_user.LeftHand.img_y;
            msg_user.LeftHand.mode = item_user.LeftHand.mode;

            msg_user.RightHand.x = item_user.RightHand.x;
            msg_user.RightHand.y = item_user.RightHand.y;
            msg_user.RightHand.z = item_user.RightHand.z;
            msg_user.RightHand.img_x = item_user.RightHand.img_x;
            msg_user.RightHand.img_y = item_user.RightHand.img_y;
            msg_user.RightHand.mode = item_user.RightHand.mode;

            msg_user.Torso.x = item_user.Torso.x;
            msg_user.Torso.y = item_user.Torso.y;
            msg_user.Torso.z = item_user.Torso.z;
            msg_user.Torso.img_x = item_user.Torso.img_x;
            msg_user.Torso.img_y = item_user.Torso.img_y;
            msg_user.Torso.mode = item_user.Torso.mode;

            msg_user.RightFoot.x = item_user.RightFoot.x;
            msg_user.RightFoot.y = item_user.RightFoot.y;
            msg_user.RightFoot.z = item_user.RightFoot.z;
            msg_user.RightFoot.img_x = item_user.RightFoot.img_x;
            msg_user.RightFoot.img_y = item_user.RightFoot.img_y;
            msg_user.RightFoot.mode = item_user.RightFoot.mode;

            msg_user.LeftFoot.x = item_user.LeftFoot.x;
            msg_user.LeftFoot.y = item_user.LeftFoot.y;
            msg_user.LeftFoot.z = item_user.LeftFoot.z;
            msg_user.LeftFoot.img_x = item_user.LeftFoot.img_x;
            msg_user.LeftFoot.img_y = item_user.LeftFoot.img_y;
            msg_user.LeftFoot.mode = item_user.LeftFoot.mode;

            msg.users.push_back(msg_user);
        }
    }

    chatter_pub[0].publish(msg);

}


void process_command(string input,int mode)
{
     // if ( mode == 1 )
     // {
         cout<<"Speech : "<<input<<endl;
         global_speech = input;

        std_msgs::String msg;
        msg.data = global_speech;
        chatter_pub[1].publish(msg);
    // }

       // if ( mode == 2 )
       // {
       //     vector <string> fields;
       //     boost::split( fields, input, boost::is_any_of( "," ) );
           

       //     int count  = boost::lexical_cast<int>( (string)fields[0] );

       //     //cout<<fields.size()<<endl;
       //     user_list.clear();

       //     if ( count > 0 )
       //     {
       //     //cout<<input<<endl;

       //     for ( int i = 0 ; i < count ; i++)
       //     {

       //     kinect_user new_user;
       //     new_user.id  = boost::lexical_cast<int>( (string)fields[1 + i*37] );

       //     new_user.Head.img_x = boost::lexical_cast<int>( (string)fields[2 + i*37] );
       //     new_user.Head.img_y = boost::lexical_cast<int>( (string)fields[3+ i*37] );
       //     new_user.Head.x = boost::lexical_cast<int>( (string)fields[4 + i*37] );
       //     new_user.Head.y = boost::lexical_cast<int>( (string)fields[5 + i*37] );
       //     new_user.Head.z = boost::lexical_cast<int>( (string)fields[6+ i*37] );
       //     new_user.Head.mode = boost::lexical_cast<int>( (string)fields[7+ i*37] );

       //     new_user.LeftHand.img_x = boost::lexical_cast<int>( (string)fields[8+ i*37] );
       //     new_user.LeftHand.img_y = boost::lexical_cast<int>( (string)fields[9+ i*37] );
       //     new_user.LeftHand.x = boost::lexical_cast<int>( (string)fields[10+ i*37] );
       //     new_user.LeftHand.y = boost::lexical_cast<int>( (string)fields[11+ i*37] );
       //     new_user.LeftHand.z = boost::lexical_cast<int>( (string)fields[12+ i*37] );
       //     new_user.LeftHand.mode = boost::lexical_cast<int>( (string)fields[13+ i*37] );

       //     new_user.RightHand.img_x = boost::lexical_cast<int>( (string)fields[14+ i*37] );
       //     new_user.RightHand.img_y = boost::lexical_cast<int>( (string)fields[15+ i*37] );
       //     new_user.RightHand.x = boost::lexical_cast<int>( (string)fields[16+ i*37] );
       //     new_user.RightHand.y = boost::lexical_cast<int>( (string)fields[17+ i*37] );
       //     new_user.RightHand.z = boost::lexical_cast<int>( (string)fields[18+ i*37] );
       //     new_user.RightHand.mode = boost::lexical_cast<int>( (string)fields[19+ i*37] );

       //     new_user.Torso.img_x = boost::lexical_cast<int>( (string)fields[20+ i*37] );
       //     new_user.Torso.img_y = boost::lexical_cast<int>( (string)fields[21+ i*37] );
       //     new_user.Torso.x = boost::lexical_cast<int>( (string)fields[22+ i*37] );
       //     new_user.Torso.y = boost::lexical_cast<int>( (string)fields[23+ i*37] );
       //     new_user.Torso.z = boost::lexical_cast<int>( (string)fields[24+ i*37] );
       //     new_user.Torso.mode = boost::lexical_cast<int>( (string)fields[25+ i*37] );

       //     new_user.LeftFoot.img_x = boost::lexical_cast<int>( (string)fields[26+ i*37] );
       //     new_user.LeftFoot.img_y = boost::lexical_cast<int>( (string)fields[27+ i*37] );
       //     new_user.LeftFoot.x = boost::lexical_cast<int>( (string)fields[28+ i*37] );
       //     new_user.LeftFoot.y = boost::lexical_cast<int>( (string)fields[29+ i*37] );
       //     new_user.LeftFoot.z = boost::lexical_cast<int>( (string)fields[30+ i*37] );
       //     new_user.LeftFoot.mode = boost::lexical_cast<int>( (string)fields[31+ i*37] );

       //     new_user.RightFoot.img_x = boost::lexical_cast<int>( (string)fields[32+ i*37] );
       //     new_user.RightFoot.img_y = boost::lexical_cast<int>( (string)fields[33+ i*37] );
       //     new_user.RightFoot.x = boost::lexical_cast<int>( (string)fields[34+ i*37] );
       //     new_user.RightFoot.y = boost::lexical_cast<int>( (string)fields[35+ i*37] );
       //     new_user.RightFoot.z = boost::lexical_cast<int>( (string)fields[36+ i*37] );
       //     new_user.RightFoot.mode = boost::lexical_cast<int>( (string)fields[37+ i*37] );

       //     user_list.push_back(new_user);
       //     }

        
       //     //cout<<count<<endl;
       //     }

       //   int p = fields.size();
       //   //cout<<p<<endl;
       //    gg_tracked = boost::lexical_cast<int>( (string)fields[p-3] );
       //    gg_x = boost::lexical_cast<int>( (string)fields[p-2] );
       //    gg_z = boost::lexical_cast<int>( (string)fields[p-1] );
       //    cout<<gg_z<<" "<<gg_x<<" "<<gg_tracked<<endl;
            
//           cout<<"USERS : "<<count<<endl;


    // send_skeleton();
    // }
    // if ( mode == 3 )
    //  {
    // cout<<input<<endl;
    //vector <string> fields;
    //boost::split( fields, input, boost::is_any_of( "," ) );
    //global_z = boost::lexical_cast<int>( (string)fields[1] );
    //cout<<global_z<<endl;

    //std_msgs::Int32 msg;
    //msg.data = global_z;
    // chatter_pub[2].publish(msg);
    //   }
    //

     //cout<<input<<endl;

}



void tcp_write_speech(std::string msg,string mode)
{
    int len = msg.length();
    string data =  msg.c_str() ;
    if ( stream_speech != NULL )
    {
        stream_speech->send(data.c_str(), len);
        cout<<"write : "<<data<<endl;
    }
}

void tcp_write_skel(std::string msg,string mode)
{
    int len = msg.length();
    string data =  msg.c_str() ;

    if ( stream_kinect != NULL )
    {
        stream_kinect->send(data.c_str(), len);
        cout<<"write : "<<data<<endl;
    }
}

void sendd()
{
    while(true)
    {
      // tcp_write_speech("12345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890","1");
      // boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
}

void serial_write(string  msg,string mode)
{
#ifdef USB_MODE

    if ( main_serial == NULL ) return ;
    if ( main_serial->isOpen() == false ) return;

    string in_put = mode + "," + msg + "\n";

    cout<<"write : "<<in_put<<endl;
    main_serial->write(in_put);

#else

    if ( mode == "1")
        tcp_write_speech(msg,mode);

    if ( mode == "2" || mode == "3")
        tcp_write_skel(msg,mode);

#endif
}

std::vector<char> speech_data;

void tcp_read_speech() //3000
{
    stream_speech = NULL;
    acceptor_speech = NULL;
    acceptor_speech = new TCPAcceptor(3000);

    if (acceptor_speech->start() == 0) {
        while (App_exit == false) {
            stream_speech = acceptor_speech->accept();
            cout<<"speech connected..."<<endl;
            if (stream_speech != NULL) {
                ssize_t len;
                char line[1000];

                int header = 0;
                string valid_data = "";

                while ((len = stream_speech->receive(line, sizeof(line))) > 0 && App_exit ==false)
                {
                    //cout<<line<<endl;

                    for ( int i = 0 ; i < len ; i++)
                    {
                        // if ( line[i] == '<' && header == 0)
                        // {
                        //     header++;
                        // }
                        // else
                        //     if ( line[i] == '%' && header == 1)
                        //     {
                        //         header++;
                        //     }
                        //     else
                        //         if ( line[i]== '>' && header == 2)
                        //         {
                        //             header++;
                        //         }
                        //         else
                        //             if ( header == 3)
                        //             {

                                        if ( line[i] != '$')
                                            valid_data += line[i];
                                        else
                                        {
                                            // cout<<valid_data<<endl;
                                            //=================================
                                            string temp = valid_data;
                                            //temp = temp.substr(1);

                                            //if ( valid_data[0] == '1' )
                                                process_command(temp,1);
                                            //==================================
                                            valid_data = "";
                                            header = 0;
                                        }
                                  //  }
                    }
                }
                delete stream_speech;
            }
        }
    }
}

void tcp_read_skel() //4000
{

    // stream_kinect = NULL;
    // acceptor_kinect = NULL;
    // acceptor_kinect = new TCPAcceptor(4000);

    // if (acceptor_kinect->start() == 0) {
    //     while (App_exit == false) {
    //         stream_kinect = acceptor_kinect->accept();
    //         cout<<"skeleton connected..."<<endl;
    //         if (stream_kinect != NULL) {
    //             ssize_t len;
    //             char line[1000];

    //             int header = 0;
    //             string valid_data = "";

    //             while ((len = stream_kinect->receive(line, sizeof(line))) > 0 && App_exit ==false)
    //             {
    //                 //cout<<line<<endl;

    //                 for ( int i = 0 ; i < len ; i++)
    //                 {
    //                     if ( line[i] == '<' && header == 0)
    //                     {
    //                         header++;
    //                     }
    //                     else
    //                         if ( line[i] == '%' && header == 1)
    //                         {
    //                             header++;
    //                         }
    //                         else
    //                             if ( line[i]== '>' && header == 2)
    //                             {
    //                                 header++;
    //                             }
    //                             else
    //                                 if ( header == 3)
    //                                 {

    //                                     if ( line[i] != '$')
    //                                         valid_data += line[i];
    //                                     else
    //                                     {
    //                                         // cout<<valid_data<<endl;
    //                                         //=================================
    //                                         string temp = valid_data;
    //                                         temp = temp.substr(1);

    //                                         if ( valid_data[0] == '2' )
    //                                             process_command(temp,2);
    //                                         if ( valid_data[0] == '3' )
    //                                             process_command(temp,3);
    //                                         //==================================
    //                                         valid_data = "";
    //                                         header = 0;
    //                                     }
    //                                 }
    //                 }
    //             }
    //             delete stream_kinect;
    //         }
    //     }
    // }
}

void serial_read()
{

    while(App_exit == false)
    {

        try
        {
            try
            {


                serial::Serial my_serial("/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0", 115200, serial::Timeout::simpleTimeout(1000));
                main_serial = &my_serial;

                cout << "Is the serial port open?";
                if (my_serial.isOpen()) {
                    cout << " Yes." << endl;
                } else
                    cout << " No." << endl;

                uint8_t buffer[1000];
                int m = 0;
                uint8_t read;
                int mode = 0;

                while(App_exit == false)
                {
                    my_serial.read(&read,1);
                    if ( read == (char)'a')
                    {
                        my_serial.read(&read,1);
                        if ( read == (char)'b')
                        {
                            my_serial.read(&read,1);
                            if ( read == (char)'c')
                            {
                                my_serial.read(&read,1);
                                if ( read == (char)'1')
                                {
                                    //speech
                                    mode = 1;
                                }
                                else if ( read == (char)'2')
                                {
                                    //skel
                                    mode = 2;
                                }
                                else if ( read == (char)'3')
                                {
                                    //followme
                                    mode = 3;
                                }
                                else if ( read == (char)'e')
                                {
                                    //empty
                                    mode = 0;

                                }
                                cout<<"head ok"<<endl;

                                //                                if ( mode != 0 )
                                //                                {
                                //                                    bzero(buffer,1000);
                                //                                    int m = 0;
                                //                                    out_put = "";
                                //                                    while ( buffer[m] != '$' && App_exit == false)
                                //                                    {
                                //                                        my_serial.read(&buffer[m] ,1);
                                //                                        if ( buffer[m] == '$') break;
                                //                                        out_put += buffer[m];
                                //                                        m += 1;
                                //                                    }
                                //                                    //cout<<"mode"<<mode<<endl;
                                //                                    process_command(out_put,mode);
                                //                                }
                                //                                else
                                //                                {
                                //                                    //cout<<"empty"<<endl;
                                //                                }
                            }
                        }
                    }



                }
            }//try
            catch (serial::SerialException e)
            {
                cout<<"Read error ! :(..."<<endl;
            }

        }//try
        catch (serial::IOException e)
        {
            cout<<"port not opened ! :(..."<<endl;
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    }//while
}//main


void chatterCallback_speech(const std_msgs::String::ConstPtr &msg)
{
    string in_put = msg->data;
    //cout<<"get"<<in_put<<endl;
    serial_write(in_put,"1");
}

void chatterCallback_followme(const std_msgs::String::ConstPtr &msg)
{
    string in_put = msg->data;
    serial_write(in_put,"3");
}

bool checkwindows(athomerobot_msgs::windows::Request  &req,athomerobot_msgs::windows::Response &res)
{
    std::string str = req.command;

    ROS_INFO("Windows Service Request.... : ");
    	cout<<str<<endl;

   
    serial_write(str,"1");
   
        //serial_write("off","2");
    
    res.result = "done";
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "windows_communication");
    ros::Time::init();

    cout << "SEPANTA WINDOWS COMMUNICATION STARTED DONE 17 july" << endl;

#ifdef USB_MODE
    //boost::thread _thread(&serial_read);
#else
    boost::thread _threadsp(&tcp_read_speech);
    boost::thread _threadsk(&tcp_read_skel);
    boost::thread _threadsend(&sendd);
#endif

    ros::Rate ros_rate(20);

    ros::NodeHandle node_handles[15];
    ros::Subscriber sub_handles[15];

    chatter_pub[0] = node_handles[0].advertise<athomerobot_msgs::users>("AUTROBOTOUT_skeleton", 10);
    chatter_pub[1] = node_handles[1].advertise<std_msgs::String>("AUTROBOTIN_speech", 10); //send to speakers
    chatter_pub[2] = node_handles[3].advertise<std_msgs::Int32>("FOLLOWME_zres", 10); //send to speakers
    //==========================================================================================
    sub_handles[0] = node_handles[4].subscribe("AUTROBOTOUT_speak",10,chatterCallback_speech);
    sub_handles[1] = node_handles[5].subscribe("FOLLOWME_rectreq",10,chatterCallback_followme);

    ros::NodeHandle n_service;
    ros::ServiceServer service_windows = n_service.advertiseService("AUTROBOTINSRV_windows", checkwindows);

    while(ros::ok())
    {
        ros::spinOnce();
        ros_rate.sleep();


      //serial_write("HELLO FROM SEPANTA ","1");

        //cout<<"loop"<<endl;
    }

    App_exit = true;

#ifdef USB_MODE
    _thread.interrupt();
    _thread.join();
#else
    _threadsp.interrupt();
    _threadsp.join();

    _threadsk.interrupt();
    _threadsk.join();

    _threadsend.interrupt();
    _threadsend.join();

    acceptor_speech->~TCPAcceptor();
    acceptor_kinect->~TCPAcceptor();

    delete acceptor_kinect;
    delete acceptor_speech;
    delete stream_kinect;
    delete stream_speech;
#endif;

    return 0;
}
