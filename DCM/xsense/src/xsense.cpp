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

#include "serial/serial.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;
bool App_exit = false;
int Compass = 0;

ros::Publisher chatter_pub[20];

float IEEE_754_to_float(const uint8_t raw[4]) {
        int sign = (raw[0] >> 7) ? -1 : 1;

        int8_t exponent = (raw[0] << 1) + (raw[1] >> 7) - 126;

        uint32_t fraction_bits = ((raw[1] & 0x7F) << 16) + (raw[2] << 8) + raw[3];

        float fraction = 0.5f;
        for (uint8_t ii = 0; ii < 24; ++ii)
                fraction += ldexpf((fraction_bits >> (23 - ii)) & 1, -(ii + 1));

        float significand = sign * fraction;

        return ldexpf(significand, exponent);
}

void serial_logic()
{
    cout << "serial reader Xsense started... 17 july" << endl;
    while (App_exit == false)
    {

        try
        {

            try
            {

                serial::Serial my_serial("/dev/serial/by-id/usb-Xsens_USB-serial_Converter_XSUWTK6Q-if00-port0", 115200 , serial::Timeout::simpleTimeout(1000));

                cout << "Is the serial port open? ";
                if (my_serial.isOpen())
                {
                    cout << " Yes." << endl;
                }
                else
                    cout << " No." << endl;

                    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

                    uint8_t read = 0;
                    uint8_t result_read[30] = {0};
                    uint8_t raw[4] = {0};

                        while (App_exit == false)
                        {
                            my_serial.read(&read, 1);

                            if ( read == 250 )
                            {
                                my_serial.read(&read, 1);

                                if ( read == 255 )
                                {
                                    my_serial.read(&read, 1);

                                    if ( read == 50 )
                                    {
                                        my_serial.read(&read, 1);

                                        if ( read == 12 )
                                        {
                                            ROS_INFO("Header ok...");

                                            my_serial.read(result_read,12);

                                            raw[0] = result_read[8];
                                            raw[1] = result_read[9];
                                            raw[2] = result_read[10];
                                            raw[3] = result_read[11];

                                           float x = IEEE_754_to_float(raw);
                                           Compass = (int)x;

                                           if ( Compass >= 360 ) Compass = Compass % 360;
                                           if ( Compass < 0 ) Compass += 360;

                                           cout<<x<<endl;
                                           //================================
                                           my_serial.read(&read, 1);
                                        }
                                    }
                                }


                            }

                            boost::this_thread::sleep(boost::posix_time::milliseconds(20));

                       }//while

            }//try
            catch (serial::SerialException e)
            {
                cout << "Read Error [Xsense]" << endl;
            }

        }//try
        catch (serial::IOException e)
        {
            cout << "Port Not Opened [Xsense]" << endl;
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    }//while

}

void update_data()
{
    std_msgs::Int32 c_msg;
    c_msg.data = Compass;
    chatter_pub[10].publish(c_msg); //Compass
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xsense");
    ros::Time::init();

    boost::thread _thread_logic(&serial_logic);

    ros::Rate ros_rate(20);

    ros::NodeHandle node_handles[15];
    ros::Subscriber sub_handles[15];
    
    chatter_pub[10] = node_handles[0].advertise<std_msgs::Int32>("AUTROBOTOUT_compass", 10);

    while (ros::ok())
    {
        ros::spinOnce();
        ros_rate.sleep();
        update_data();
    }

    App_exit = true;

    _thread_logic.interrupt();
    _thread_logic.join();


    return 0;
}

