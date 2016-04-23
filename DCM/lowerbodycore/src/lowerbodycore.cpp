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

ros::Publisher chatter_pub[20];
ros::Publisher chatter_pub_motor[20];
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
const int baudrate = 1000000;
bool isserialopen = false;

const string port_name  = "/dev/serial/by-id/usb-ROBOTIS_CO._LTD._ROBOTIS_Virtual_COM_Port-if00";

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

float omnidrive_x = 0;
float omnidrive_y = 0;
float omnidrive_w = 0;

void chatterCallback_omnidrive(const sepanta_msgs::omnidata::ConstPtr &msg)
{
    cout<<"omni"<<endl;
    omnidrive_x = -msg->d0;
    omnidrive_y = msg->d1;
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

float w1_current = 128;
float w2_current = 128;
float w3_current = 128;
float w4_current = 128;

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
    sepanta_msgs::irsensor sensor_info;
    sensor_info.d0 = IR[0];
    sensor_info.d1 = IR[1];
    sensor_info.d2 = IR[2];
    sensor_info.d3 = IR[3];
    sensor_info.d4 = IR[4];
    chatter_pub[6].publish(sensor_info);

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
    std_msgs::Int32 key_msg;
    key_msg.data = keypad_status;
    chatter_pub[8].publish(key_msg); //keypad

    //Publish EMS_Stop
    std_msgs::Int32 ems_msg;
    ems_msg.data = EMS_STOP;
    chatter_pub[9].publish(ems_msg); //ems stop

    //Publisg Btn_Start
    std_msgs::Bool btn_msg;
    btn_msg.data = btn_start;
    chatter_pub[11].publish(btn_msg);// btn_start

    //Publisg Mode
    std_msgs::Int32 mode_msg;
    mode_msg.data = control_mode;
    chatter_pub[12].publish(mode_msg); //Mode

    //Publish Voltage Down
    std_msgs::Int32 v1_msg;
    v1_msg.data = voltage_down;
    chatter_pub[13].publish(v1_msg); //voltage down

    //Publish Voltage Up
    std_msgs::Int32 v2_msg;
    v2_msg.data = voltage_up;
    chatter_pub[14].publish(v2_msg); //voltage up

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
                    result_write[0] = 0;
                    my_serial.write(result_write, 1);
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

                    boost::this_thread::sleep(boost::posix_time::milliseconds(25));
                    serial_rw_count++;
                    /////////////////////////////////////////////////////
                    //Write Packet

                    result_write[0] = 255;
                    result_write[1] = 190;
                    result_write[2] = 255;
                    result_write[3] = 100;
                    result_write[4] = 40;
                    result_write[5] = mobileplatform_motors_write[0];
                    result_write[6] = mobileplatform_motors_write[1];
                    result_write[7] = mobileplatform_motors_write[2];
                    result_write[8] = mobileplatform_motors_write[3];

                    char green = 25;
                    char red = 25;
                    if ( green_light ) green = 75;
                    if ( red_light   ) red = 75;

                    result_write[9] = 75;
                    result_write[10] = 75;

                    my_serial.write(result_write, 11);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                    my_serial.flush();

                    //ROS_INFO("USB Serial Write");

                    boost::this_thread::sleep(boost::posix_time::milliseconds(15));

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
                                    //Read Packet
                                    my_serial.read(result_read,21);
                                    mobileplatform_motors_read[0] = result_read[0];
                                    mobileplatform_motors_read[1] = result_read[1];
                                    mobileplatform_motors_read[2] = result_read[2];
                                    mobileplatform_motors_read[3] = result_read[3];

                                    if (  result_read[4] != 0)
                                    {
                                        keypad_status = result_read[4];
                                    }

                                    int x = ( result_read[6] << 8 ) + ( result_read[5] ) ; //Compass (2 byte)
                                    int y = ( result_read[8] << 8 ) + ( result_read[7] ) ; //Compass (2 byte)

                                    control_mode = result_read[9];


                                    for ( int i = FILTER_QUEU  - 2 ; i >= 0 ; i--)
                                    {
                                        queuercmpsx[i + 1] = queuercmpsx[i];
                                        queuercmpsy[i + 1] = queuercmpsy[i];
                                        queuerir1[i+1] =  queuerir1[i];
                                        queuerir2[i+1] =  queuerir2[i];
                                        queuerir3[i+1] =  queuerir3[i];
                                        queuerir4[i+1] =  queuerir4[i];
                                        queuerir5[i+1] =  queuerir5[i];
                                        queuerv1[i+1] = queuerv1[i];
                                        queuerv2[i+1] = queuerv2[i];

                                    }

                                    queuercmpsx[0] = x;
                                    queuercmpsy[0] = y;
                                    queuerir1[0] = (int)result_read[12];
                                    queuerir2[0] = (int)result_read[13];
                                    queuerir3[0] = (int)result_read[14];
                                    queuerir4[0] = (int)result_read[15];
                                    queuerir5[0] = (int)result_read[16];
                                    queuerv1[0] = ( result_read[18] << 8 ) + ( result_read[17] );
                                    queuerv2[0] = ( result_read[20] << 8 ) + ( result_read[19] );

                                    float sum1 = 0;
                                    float sum12 = 0;

                                    float sum2 = 0;
                                    float sum3 = 0;
                                    float sum4 = 0;
                                    float sum5 = 0;
                                    float sum6 = 0;
                                    float sum7 = 0;
                                    float sum8 = 0;

                                    for ( int i = 0 ; i < FILTER_QUEU ; i++ )
                                    {
                                        sum1 += queuercmpsx[i];
                                        sum12 += queuercmpsy[i];
                                        sum2 += queuerir1[i];
                                        sum3 += queuerir2[i];
                                        sum4 += queuerir3[i];
                                        sum5 += queuerir4[i];
                                        sum6 += queuerir5[i];
                                        sum7 += queuerv1[i];
                                        sum8 += queuerv2[i];
                                    }


                                    x = (int)(sum1 / FILTER_QUEU);
                                    y = (int)(sum12 / FILTER_QUEU);

                                    if ( x > 1000 )
                                        x = -1 * ( 65536 - x );

                                    if ( y > 1000 )
                                        y = -1 * ( 65536 - y );

                                    Compass = atan2(x,y) * 57.29;
                                    if (Compass < 0) Compass = Compass + 360;


                                    char ems = result_read[10];
                                    char bstart = result_read[11];

                                    if ( bstart < 50 )
                                        btn_start = false;
                                    else
                                        btn_start = true;

                                    if ( ems < 50 )
                                        EMS_STOP = 0;
                                    else
                                        EMS_STOP = 1;
                                    
                                    //cout<<"EMS : "<<EMS_STOP<<endl;

                                    

                                    IR[0] = (int)(sum2 / FILTER_QUEU);
                                    IR[1] = (int)(sum3 / FILTER_QUEU);
                                    IR[2] = (int)(sum4 / FILTER_QUEU);
                                    IR[3] = (int)(sum5 / FILTER_QUEU);
                                    IR[4] = (int)(sum6 / FILTER_QUEU);


                                    voltage_up = (int)(sum7 / 10);
                                    voltage_down = (int)(sum8 / 10);


                                    if ( control_mode_old != control_mode )
                                    {
                                        if ( control_mode == 1 )
                                        {
                                            red_light = true;
                                            control_mode_old = 1;
                                        }
                                        else
                                        {
                                            red_light = false;
                                            control_mode_old = 2;
                                        }
                                    }


                                    //ROS_INFO("USB Serial Read");
                                   
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
    chatter_pub[6] = node_handles[1].advertise<sepanta_msgs::irsensor>("lowerbodycore/irsensors", 10);
    chatter_pub[7] = node_handles[2].advertise<sepanta_msgs::irsensor>("lowerbodycore/lasersensors", 10);
    chatter_pub[8] = node_handles[3].advertise<std_msgs::Int32>("lowerbodycore/joistick", 10);
    chatter_pub[9] = node_handles[4].advertise<std_msgs::Int32>("lowerbodycore/bntems", 10);
    chatter_pub[11] = node_handles[5].advertise<std_msgs::Bool>("lowerbodycore/btnstart", 10);
    chatter_pub[12] = node_handles[6].advertise<std_msgs::Int32>("lowerbodycore/controlmode", 10); //1 down //2 up
    chatter_pub[13] = node_handles[7].advertise<std_msgs::Int32>("lowerbodycore/voltagedown", 10);
    chatter_pub[14] = node_handles[8].advertise<std_msgs::Int32>("lowerbodycore/voltageup", 10);
    chatter_pub[15] = node_handles[14].advertise<std_msgs::Bool>("lowerbodycore/isdooropened", 10);
    sub_handles[1] = node_handles[9].subscribe("lowerbodycore/omnidrive", 10, chatterCallback_omnidrive);
    sub_handles[2] = node_handles[10].subscribe("scan", 10, chatterCallback_laser);
    sub_handles[9] = node_handles[11].subscribe("lowerbodycore/greenlight", 10, chatterCallback_greenlight);
    sub_handles[10] = node_handles[12].subscribe("lowerbodycore/redlight", 10, chatterCallback_redlight);
    sub_handles[11] = node_handles[13].subscribe("tcpip/es", 10, chatterCallback_tcpes);
    //============================================================================================

    ros::Rate loop_rate(20);

    while (ros::ok() && App_exit == false)
    {
        //update publish loop
        Update();
        ros::spinOnce();
        loop_rate.sleep();
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
