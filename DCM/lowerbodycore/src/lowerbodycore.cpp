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
#include <signal.h>
#include "serial/serial.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include "sepanta_msgs/arm.h"
#include "sepanta_msgs/omnidata.h"
#include "sepanta_msgs/head.h"
#include "sepanta_msgs/irsensor.h"
#include "sepanta_msgs/led.h"
#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/JointState.h>
#include <dynamixel_controllers/SetComplianceSlope.h>
#include <dynamixel_controllers/SetCompliancePunch.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <iostream>

#define ENABLE_SERIAL
#define FILTER_QUEU 10
//=============================================================

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;

void Omnidrive(float vx, float vy, float w);

bool robot_init = false;
bool App_exit = false;
int control_mode = 0;

float L = 50; //cm
int mobileplatform_motors_write[4] = {128, 128, 128, 128};
int mobileplatform_motors_read[4] = {128, 128, 128, 128};
tbb::atomic<int> keypad_status_raw;
tbb::atomic<int> keypad_status;
tbb::atomic<int> last_keypad_value;
int serial_state = 1;

float laser_IR[10];
int IR[5] = {0};
int EMS_STOP = 0;
int EMS_STOP2 = 0;
bool isdooropened = true;
bool isledenable = false;
int balarm = 0;
char led_color[3] = {0};


ros::Publisher chatter_pub[20];
ros::Publisher chatter_pub_motor[20];
ros::Publisher pub_ack;

tbb::atomic<int> Compass;

int key_pad_reset = 0;
int robot_max_speedx = 400;
int robot_max_speedy = 350;
int robot_max_speedw = 400;

//Status
int control_mode_old = 0;
int voltage_down = 0;
int voltage_up = 0;
bool green_light = false;
bool red_light = false;
bool btn_start = false;
bool btn_stop = false;
int serial_read_hz = 0;
int serial_rw_count = 0;
const int baudrate = 115200;
bool isserialopen = false;
char desire_z = 100;

float omnidrive_x = 0;
float omnidrive_y = 0;
float omnidrive_w = 0;

int w1_current = 128;
int w2_current = 128;
int w3_current = 128;
int w4_current = 128;

const string port_name  = "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0";

void chatterCallback_led(const sepanta_msgs::led::ConstPtr &msg)
{
   isledenable = msg->enable;
   led_color[0] = msg->colorR;
   led_color[1] = msg->colorG;
   led_color[2] = msg->colorB;
}

void chatterCallback_z(const std_msgs::Int32::ConstPtr &msg)
{
   desire_z = msg->data;
}

void chatterCallback_alarm(const std_msgs::Int32::ConstPtr &msg)
{
   balarm = msg->data;
}

void chatterCallback_greenlight(const std_msgs::Bool::ConstPtr &msg)
{
    cout<<msg->data<<endl;
    green_light = msg->data;
}

void chatterCallback_redlight(const std_msgs::Bool::ConstPtr &msg)
{
    cout<<msg->data<<endl;
    red_light = msg->data;
}

void chatterCallback_laser(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    int val_count = msg->ranges.size(); //1080

    //min 10 cm  (0.1 meter)
    //max 500 cm (5 meters)
    
    std::cout<<val_count<<std::endl;

    float read = 0;
    int valid_count = 0;

    for ( int i = 0 ; i < 10 ; i++)
    {
        read = 0;
        valid_count = 0;
        laser_IR[9 - i] = 0;

        for ( int j = 0 ; j < 108 ; j++)
        {
            read = msg->ranges[i * 108 + j];

            if ( !std::isnan(read) && !isinf(read) )
            {
                laser_IR[9 - i] += read;
                valid_count++;
            }
        }

        if ( valid_count > 0)
            laser_IR[9 - i] = (laser_IR[9 - i] / valid_count) * 100;
        else
            laser_IR[9 - i] = 500;

        laser_IR[9 - i] = (int)laser_IR[9 - i];

    }

    if ( laser_IR[3] < 50 || laser_IR[4] < 50 || laser_IR[5] < 50 )
    {
    	isdooropened = false;
    }
    else
    {
    	isdooropened = true;
    }

}

void chatterCallback_omnidrive(const sepanta_msgs::omnidata::ConstPtr &msg)
{
    cout<<"omni"<<endl;
    omnidrive_x = -msg->d0;
    omnidrive_y = -msg->d1;
    omnidrive_w = -msg->d2;
}

void chatterCallback_tcpes(const std_msgs::String::ConstPtr &msg)
{
   if ( msg->data == "false")
   {
       EMS_STOP2 = 0;
   }
   else
   {
       EMS_STOP2 = 1;
   }
}

void logic()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    ROS_INFO("Sepanta III DCM Started. Version : 2016-March-19");
    while (App_exit == false)
    {
        //HZ ?
        serial_rw_count = 0;
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        serial_read_hz =  serial_rw_count;
        ROS_INFO("USB Serial Hz %d",serial_read_hz);
    }
}

void smooth_drive()
{
     Omnidrive(0,0,0);
     boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
     while (App_exit == false)
     {
        
         Omnidrive(omnidrive_x,omnidrive_y,omnidrive_w);
         
         boost::this_thread::sleep(boost::posix_time::milliseconds(25)); //40 hz
     }
}



void Omnidrive(float vx, float vy, float vw)
{
   
    vw = -vw / 100; 
    //this is for scale only for better intraction of number
    //this vw is scaled to match scale factor for vx,vy values
    //Speed Limits
    if ( vx > robot_max_speedx ) vx = robot_max_speedx;
    if ( vy > robot_max_speedy ) vy = robot_max_speedy;
    if ( vw > robot_max_speedw ) vw = robot_max_speedw;

    if ( vx < -robot_max_speedx ) vx = -robot_max_speedx;
    if ( vy < -robot_max_speedy ) vy = -robot_max_speedy;
    if ( vw < -robot_max_speedw ) vw = -robot_max_speedw;

    //FK
    float w1 = (vx - vy - vw * (L)) / 7.5;
    float w2 = (vx + vy + vw * (L)) / 7.5;
    float w3 = -(vx - vy + vw * (L)) / 7.5;
    float w4 = -(vx + vy - vw * (L)) / 7.5;

    w1 += 128;
    w2 += 128;
    w3 += 128;
    w4 += 128;

    if (w1 > 254) w1 = 254;
    if (w1 < 1) w1 = 1;
    if (w2 > 254) w2 = 254;
    if (w2 < 1) w2 = 1;
    if (w3 > 254) w3 = 254;
    if (w3 < 1) w3 = 1;
    if (w4 > 254) w4 = 254;
    if (w4 < 1) w4 = 1;

    if ( EMS_STOP == 1 || EMS_STOP2 == 1)
    {
        ROS_WARN("Check Emergency Stop Button ( 1 or 2 ) , OmniDrive(0,0,0)");
        w4 = 128;
        w3 = 128;
        w2 = 128;
        w1 = 128;
    }

     if ( w1_current != w1)
     {
         if ( w1 > w1_current)
             w1_current++;
         else
             w1_current--;
     }
     if ( w2_current != w2)
     {
         if ( w2 > w2_current)
             w2_current++;
         else
             w2_current--;
     }
     if ( w3_current != w3)
     {
         if ( w3 > w3_current)
             w3_current++;
         else
             w3_current--;
     }
     if ( w4_current != w4)
     {
         if ( w4 > w4_current)
             w4_current++;
         else
             w4_current--;
     }

     mobileplatform_motors_write[0] = w1_current;
     mobileplatform_motors_write[1] = w2_current;
     mobileplatform_motors_write[2] = w3_current;
     mobileplatform_motors_write[3] = w4_current;

}

void Update()
{
    //Publisg Omni Speed
    sepanta_msgs::omnidata omni_info;
    omni_info.d0 = mobileplatform_motors_read[0];
    omni_info.d1 = mobileplatform_motors_read[1];
    omni_info.d2 = mobileplatform_motors_read[2];
    omni_info.d3 = mobileplatform_motors_read[3];
    chatter_pub[4].publish(omni_info);

    //Publisg IR Sensors
    // sepanta_msgs::irsensor sensor_info;
    // sensor_info.d0 = IR[0];
    // sensor_info.d1 = IR[1];
    // sensor_info.d2 = IR[2];
    // sensor_info.d3 = IR[3];
    // sensor_info.d4 = IR[4];
    // chatter_pub[6].publish(sensor_info);

    //Publish Laser Sensors
    sepanta_msgs::irsensor sensor_info_laser;
    sensor_info_laser.d0 = (int)laser_IR[0];
    sensor_info_laser.d1 = (int)laser_IR[1];
    sensor_info_laser.d2 = (int)laser_IR[2];
    sensor_info_laser.d3 = (int)laser_IR[3];
    sensor_info_laser.d4 = (int)laser_IR[4];
    sensor_info_laser.d5 = (int)laser_IR[5];
    sensor_info_laser.d6 = (int)laser_IR[6];
    sensor_info_laser.d7 = (int)laser_IR[7];
    sensor_info_laser.d8 = (int)laser_IR[8];
    sensor_info_laser.d9 = (int)laser_IR[9];

    chatter_pub[7].publish(sensor_info_laser);

    //Publisg Keypad
    // std_msgs::Int32 key_msg;
    // key_msg.data = keypad_status;
    // chatter_pub[8].publish(key_msg); //keypad

    //Publish EMS_Stop
    std_msgs::Int32 ems_msg;
    ems_msg.data = EMS_STOP;
    chatter_pub[9].publish(ems_msg); //ems stop

    //Publisg Btn_Start
    std_msgs::Bool btn_msg;
    btn_msg.data = btn_start;
    chatter_pub[11].publish(btn_msg);// btn_start

    //Publisg Mode
    // std_msgs::Int32 mode_msg;
    // mode_msg.data = control_mode;
    // chatter_pub[12].publish(mode_msg); //Mode

    //Publish Voltage Down
    // std_msgs::Int32 v1_msg;
    // v1_msg.data = voltage_down;
    // chatter_pub[13].publish(v1_msg); //voltage down

    //Publish Voltage Up
    // std_msgs::Int32 v2_msg;
    // v2_msg.data = voltage_up;
    // chatter_pub[14].publish(v2_msg); //voltage up

    std_msgs::Bool bool_msg;
    bool_msg.data = isdooropened;
    chatter_pub[15].publish(bool_msg); //door status
}


void serial_logic()
{

#ifdef ENABLE_SERIAL

    uint8_t result_write[30];
    uint8_t result_read[30];

    while (App_exit == false)
    {
        try
        {
            try
            {
                serial::Serial my_serial(port_name, baudrate, serial::Timeout::simpleTimeout(500));

                //Config Serial
                my_serial.close();
                my_serial.setBaudrate(baudrate);
                serial::parity_t val1 = serial::parity_none;
                my_serial.setParity(val1);
                serial::stopbits_t val2 = serial::stopbits_one;
                my_serial.setStopbits(val2);
                serial::bytesize_t val3 = serial::eightbits;
                my_serial.setBytesize(val3);
                my_serial.open();
             
                //=======================================================

                if (my_serial.isOpen())
                {
                    ROS_INFO("USB Serial Port OK? : YES");
                    //result_write[0] = 0;
                    //my_serial.write(result_write, 1);
                }
                else
                {
                    ROS_ERROR("USB Serial Port Not Found");
                    continue;
                }

                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

                int queuercmpsx[FILTER_QUEU];
                int queuercmpsy[FILTER_QUEU];
                int queuerir1[FILTER_QUEU];
                int queuerir2[FILTER_QUEU];
                int queuerir3[FILTER_QUEU];
                int queuerir4[FILTER_QUEU];
                int queuerir5[FILTER_QUEU];
                int queuerv1[FILTER_QUEU];
                int queuerv2[FILTER_QUEU];

               
                while (App_exit == false)
                {

                    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
                    serial_rw_count++;
                    /////////////////////////////////////////////////////
                    //Write Packet

                    result_write[0] = 255;
                    result_write[1] = 190;
                    result_write[2] = 255;
                    result_write[3] = 100;
                    result_write[4] = 40;
                    result_write[5] = (uint8_t)mobileplatform_motors_write[0];
                    result_write[6] = (uint8_t)mobileplatform_motors_write[1];
                    result_write[7] = (uint8_t)mobileplatform_motors_write[2];
                    result_write[8] = (uint8_t)mobileplatform_motors_write[3];

                    //cout<<(int)result_write[5]<<" "<<(int)result_write[6]<<" "<<(int)result_write[7]<<" "<<(int)result_write[8]<<endl;

                    char green = 25;
                    char red = 25;
                    char led_enable = 25;
                    char alarm_enable = 0;

                    if ( green_light ) green = 75;
                    if ( red_light   ) red = 75;
                    if  ( isledenable ) led_enable = 75;
                   

                    result_write[9] = green;
                    result_write[10] = red;
                    result_write[11] = led_enable;

                    result_write[12] = led_color[0];
                    result_write[13] = led_color[1];
                    result_write[14] = led_color[2];
                    result_write[15] = (char)balarm;
                    result_write[16] = desire_z;
                    result_write[17] = 0;
                    result_write[18] = 0;

                    uint16_t sum = 0;

                    for ( int i = 0 ; i < 12 ; i++)
                    {
                    	sum += result_write[i + 5];
                    }

                    result_write[17] = (uint8_t)sum;
                    result_write[18] = (uint8_t)(sum >> 8);

                    my_serial.write(result_write, 19);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                    my_serial.flush();

                    //ROS_INFO("USB Serial Write");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(15));
                    
                    continue; 
                   
                    uint8_t read;

                    while (App_exit == false)
                    {
                        my_serial.read(&read, 1);

                        if ( read == 255 )
                        {
                            my_serial.read(&read, 1);

                            if ( read == 190 )
                            {
                                my_serial.read(&read, 1);

                                if ( read == 255)
                                {
                                    //cout<<"READ HEADER OK"<<endl;
                                    //Read Packet
                                    my_serial.read(result_read,8);
                                    mobileplatform_motors_read[0] = result_read[0];
                                    mobileplatform_motors_read[1] = result_read[1];
                                    mobileplatform_motors_read[2] = result_read[2];
                                    mobileplatform_motors_read[3] = result_read[3];

                                    int teta = ( result_read[5] << 8 ) + ( result_read[4] ) ; //Compass (2 byte)
                             
                                    char ems = result_read[6];
                                    char bstart = result_read[7];

                                    if ( bstart < 50 )
                                        btn_start = false;
                                    else
                                        btn_start = true;

                                    if ( ems < 50 )
                                        EMS_STOP = false;
                                    else
                                        EMS_STOP = true;
                                   
                                    break;
                                }
                            }
                        }
                    }
                }

            }//try
            catch (serial::SerialException e)
            {
                ROS_ERROR("USB Serial Read Error");
            }

        }//try
        catch (serial::IOException e)
        {
            ROS_ERROR("USB Serial Port Error");
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    }//while
#endif
}

int main(int argc, char **argv)
{

    keypad_status_raw = 0;
    keypad_status = 0;
    last_keypad_value = 0;

    boost::thread _thread_logic(&logic);
    boost::thread _thread_serial(&serial_logic);
    boost::thread _thread_drive(&smooth_drive);

    ros::init(argc, argv, "lowerbodycore");

    ros::NodeHandle node_handles[50];
    ros::Subscriber sub_handles[15];

    //===========================================================================================

    chatter_pub[4] = node_handles[0].advertise<sepanta_msgs::omnidata>("lowerbodycore/omnispeed", 10);
    chatter_pub[7] = node_handles[2].advertise<sepanta_msgs::irsensor>("lowerbodycore/lasersensors", 10);
    chatter_pub[9] = node_handles[4].advertise<std_msgs::Int32>("lowerbodycore/bntems", 10);
    chatter_pub[11] = node_handles[5].advertise<std_msgs::Bool>("lowerbodycore/btnstart", 10);
    chatter_pub[15] = node_handles[14].advertise<std_msgs::Bool>("lowerbodycore/isdooropened", 10);

    sub_handles[1] = node_handles[15].subscribe("lowerbodycore/omnidrive", 10, chatterCallback_omnidrive);
    sub_handles[2] = node_handles[16].subscribe("scan", 10, chatterCallback_laser);
    sub_handles[3] = node_handles[17].subscribe("lowerbodycore/greenlight", 10, chatterCallback_greenlight);
    sub_handles[4] = node_handles[18].subscribe("lowerbodycore/redlight", 10, chatterCallback_redlight);
    sub_handles[5] = node_handles[19].subscribe("lowerbodycore/led", 10, chatterCallback_led);
    sub_handles[6] = node_handles[20].subscribe("lowerbodycore/alarm", 10, chatterCallback_alarm);
    sub_handles[7] = node_handles[21].subscribe("lowerbodycore/desireZ", 10, chatterCallback_z);
    sub_handles[8] = node_handles[22].subscribe("tcpip/es", 10, chatterCallback_tcpes);

    pub_ack = node_handles[14].advertise<std_msgs::String>("lowerbodycore/ack",10);
    //============================================================================================

    ros::Rate loop_rate(20);

    while (ros::ok() && App_exit == false)
    {
        //update publish loop
        Update();
        ros::spinOnce();
        loop_rate.sleep();

        //===================================
        std_msgs::String _msg;
        _msg.data = "ok";
        pub_ack.publish(_msg);
    }

    App_exit = true;

    _thread_drive.interrupt();
    _thread_drive.join();

    _thread_logic.interrupt();
    _thread_logic.join();

    _thread_serial.interrupt();
    _thread_serial.join();

    return 0;
}
