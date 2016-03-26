#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/Int32.h"
#include <boost/thread.hpp>
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <tbb/atomic.h>
#include <boost/lexical_cast.hpp>

#include "serial/serial.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>

#include "athomerobot_msgs/arm.h"
#include "athomerobot_msgs/omnidata.h"
#include "athomerobot_msgs/head.h"
#include "athomerobot_msgs/irsensor.h"
#include "athomerobot_msgs/motortorques.h"

#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/JointState.h>
#include <dynamixel_controllers/SetComplianceSlope.h>
#include <dynamixel_controllers/SetCompliancePunch.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#define ENABLE_SERIAL



using namespace std;

std_msgs::Int32 ali; 
ros::Publisher publish_;
ros::Publisher motor_publish_;
bool App_exit=false;

void test();
void serial_logic();
bool search(string name_ , string name2);
void visionLogicCallback(const std_msgs::Int32 &msg);

//========================================
string phonenum="+989356724964";
string obj1="Homekey";
string obj2="Cellphone";
string Massage="I got your massage , i will search it and answer to you!";
string answer ;
int VISIONF=3;
//=========================================

int main(int argc, char **argv) {




    /// initialize
    ros::init(argc, argv, "Demo");
    ros::NodeHandle n_Publish;
    ros::NodeHandle n_Publish2;
    ros::NodeHandle visionNodeHandle;
    // publisher
    publish_ = n_Publish.advertise<std_msgs::Int32>("DEMOOUT", 1);
    motor_publish_ = n_Publish2.advertise<std_msgs::Int32>("/Zmotor_controller/command", 10);
    ros::Subscriber visionLogicSubscriber = visionNodeHandle.subscribe("DEMOIN", 1, visionLogicCallback);

     boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    boost::thread Preempt_thread(&serial_logic);



    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    Preempt_thread.interrupt();
    Preempt_thread.join();
    App_exit=true;


    //======================================== kills

    return 0;
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
}
void test(){
    cout << "Demo start..." << endl;
    for(int i=0;i<20;i++){
        ali.data=20;
        publish_.publish(ali);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    }
}


void serial_logic()
{
    int i;
    string temp;
    string str[10];
#ifdef ENABLE_SERIAL
    cout << "serial reader CM9 started..." << endl;
    while (App_exit == false && ros::ok())
    {

        try
        {
            try
            {

                serial::Serial my_serial("/dev/serial/by-path/pci-0000:00:1d.0-usb-0:1.2:1.0-port0", 9600, serial::Timeout::simpleTimeout(1000));

                cout << "Is the serial port open? ";
                if (my_serial.isOpen())
                {

                    cout << " Yes." << endl;
                }
                else
                    cout << " No." << endl;

                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

                uint8_t result_write[8];


                uint8_t serial_state = 0;



                while (App_exit == false && ros::ok())
                {
                    if(serial_state==0){
                        cout<<"=========================================="<<endl;
                        cout<<"delet last SMS"<<endl;
                        for(i=1;i<16;i++){


                            my_serial.write("AT+CMGD=" + boost::lexical_cast<string>(i));
                            result_write[0] = 0x0d;
                            my_serial.write(result_write, 1);
                            temp= my_serial.readline();
                            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                            temp= my_serial.readline();
                            serial_state =1;
                            cout<<i ;
                            str[0]="A";
                            cout<<"=========================================="<<endl;
                        }
                        cout<<"watting for sms "<<endl;
                    }
                    else if ( serial_state == 1 )
                    {
                        str[0]=my_serial.readline();
                        cout<<"str m"<<str[0];
                        if(search(str[0],"+CMTI:")){cout<<"state2"<<endl;serial_state = 2;str[0]="A";}
                    }
                    else if ( serial_state == 2 ) //read sms
                    {
                        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                        while(my_serial.available() !=0)my_serial.readline();
                        cout << "Command Read" << endl;
                        my_serial.write("AT+CMGR=1");
                        result_write[0] = 0x0d;
                        my_serial.write(result_write, 1);


                        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                        str[0]=my_serial.readline();
                        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                        str[1]=my_serial.readline();
                        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                        str[2]=my_serial.readline();
                        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                        str[3]=my_serial.readline();
                        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                        str[4]=my_serial.readline();

                        str[6]=str[0]+str[1]+str[2]+str[3]+str[4];
                        while(my_serial.available() !=0)
                            str[5]=my_serial.readline();
                            cout <<str[1]<<" str"<<endl;


                            
                        if(search(str[1],phonenum)){
                            cout<<"=========================================="<<endl;
                            cout<<str[2]<<endl;
                            if(search(str[2],obj1)){
                                cout<<obj1<<"ferastadam"<<endl;
                                ali.data=1;
                                publish_.publish(ali);
                                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                            }else if(search(str[2],obj2)){
                                                                cout<<obj2<<"ferastadam"<<endl;

                                ali.data=2;
                                publish_.publish(ali);
                                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                            }

                            cout<<"=========================================="<<endl;
                        }
                        serial_state=3;

                    }else if ( serial_state == 3 ) //write sms
                    {
                        cout<<"start sending sms"<<endl;
                        my_serial.write("AT+CMGS=");
                        result_write[0] = 0x22; my_serial.write(result_write, 1);
                        my_serial.write(phonenum);
                        result_write[0] = 0x22;my_serial.write(result_write, 1);
                        result_write[0] = 0x0d;my_serial.write(result_write, 1);
                        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                         my_serial.write(Massage);
                         result_write[0] = 0x1A;my_serial.write(result_write, 1);
                        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
                        while(my_serial.available() !=0)cout<<my_serial.readline();
                        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
                        while(my_serial.available() !=0)cout<<my_serial.readline();
                       serial_state=4;
                       cout<<"=========================================="<<endl;
                      } else if ( serial_state == 4 ) //write sms
                     {
                        while(VISIONF==3 && ros::ok());

                        if(VISIONF==1){
                            answer="your "+obj1+" is in home" ;
                        }else if(VISIONF==2){
                            answer="your "+obj2+" is in home" ;
                        }else if(VISIONF==4){
                            answer="Sorry,I can not found it";
                        }
                        cout<<"start sending answer"<<endl;
                        my_serial.write("AT+CMGS=");
                        result_write[0] = 0x22; my_serial.write(result_write, 1);
                        my_serial.write(phonenum);
                        result_write[0] = 0x22;my_serial.write(result_write, 1);
                        result_write[0] = 0x0d;my_serial.write(result_write, 1);
                        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                         my_serial.write(answer);
                         result_write[0] = 0x1A;my_serial.write(result_write, 1);
                        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
                        while(my_serial.available() !=0)cout<<my_serial.readline();
                        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
                        while(my_serial.available() !=0)cout<<my_serial.readline();
                       serial_state=5;
                       VISIONF=3;



                    }else
                    {
                        serial_state=0;
                    }








                    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                }

            }//try
            catch (serial::SerialException e)
            {
                cout << "Read error ! :(..." << endl;
            }

        }//try
        catch (serial::IOException e)
        {
            cout << "port not opened ! :(..." << endl;
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    }//while
#endif
}
bool flag_choose;
bool search(string name_ , string name2)
{
    if(name_!= ""){
        cout<<name_<<endl;
        cout<<name2<<endl;
        for (int i =  name_.find(name2, 0); i != string::npos; i = name_.find(name2, i)) {

            flag_choose = true;
            i++;


        }
        if (flag_choose)
        {
            cout<<"true"<<endl;
            flag_choose=false;
            return true;
        }
        else{
            cout<<"false"<<endl;
            return false;
        }
    }
    return false;
}

void visionLogicCallback(const std_msgs::Int32 &msg)
{
    cout<<"callback"<<endl;

        VISIONF=msg.data;


}

