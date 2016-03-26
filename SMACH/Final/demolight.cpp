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

int light=0;

void test();
void serial_logic();
bool search(string name_ , string name2);
void visionLogicCallback(const std_msgs::Int32 &msg);


//========================================
string phonenum="+989124265573";
string obj1="Homekey";
string obj2="Cellphone";
string Massage="I got your massage , i will search it and answer to you!";
string answer ;
//=========================================

int main(int argc, char **argv) {




    /// initialize
    ros::init(argc, argv, "Demolight");
    ros::NodeHandle visionNodeHandle;



   // ros::Rate loop ros::Subscriber motionLogicSubscriber = LogicNodeHandleToTask.subscribe("AUTROBOT_from_motion_to_logic", 1, motionLogicCallback);
    ros::Subscriber visionLogicSubscriber = visionNodeHandle.subscribe("DEMOLIGHT", 1, visionLogicCallback);
//AUTROBOT_from_vision_to_demolight

  boost::thread Preempt_thread(&serial_logic);



    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    App_exit=true;
    Preempt_thread.interrupt();
    Preempt_thread.join();


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
    int lightf=0;

#ifdef ENABLE_SERIAL
    cout << "serial reader CM9 started..." << endl;
    while (App_exit == false && ros::ok())
    {

        try
        {
            try
            {

                serial::Serial my_serial("/dev/serial/by-path/pci-0000:00:1d.0-usb-0:1.1:1.0-port0", 9600, serial::Timeout::simpleTimeout(1000));

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

                    if(serial_state==0 ){
                        if(light==0)
                        {
                            my_serial.write("#C111EC");
                            my_serial.write("#C111EC");
                            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                            cout<<"=========================================="<<endl;
                            cout<<"light off"<<endl;
                            cout<<"=========================================="<<endl;
                            lightf=0;
                        }else if (light==1){
                            my_serial.write("#C000EC");
                            my_serial.write("#C000EC");
                            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                            cout<<"=========================================="<<endl;
                            cout<<"light on"<<endl;
                            cout<<"=========================================="<<endl;
                            lightf=1;                        }


                        }

                  }

                    boost::this_thread::sleep(boost::posix_time::milliseconds(10));


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
    if (msg.data==1)
    {
        light=1;

    }else{
        light=0;
    }
}
