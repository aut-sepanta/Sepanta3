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
//===================================================================================================
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

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;

string temp_message = "";
bool busy = false;

void say(string message)
{
	if ( busy ) return; 
	busy = true;
	system((string("rosrun sound_play say.py \"")+message+string("\"")).c_str());
	busy = false;
}

void chatterCallback_speech(const std_msgs::String::ConstPtr &msg)
{
	if ( temp_message == msg->data ) return;
	if ( msg->data == "" ) return;

    say(msg->data);
    temp_message = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "texttospeech");

    ros::NodeHandle node_handles[5];
    ros::Subscriber sub_handles[5];
    //===========================================================================================

    sub_handles[0] = node_handles[0].subscribe("/texttospeech/message", 10, chatterCallback_speech);
    
    //============================================================================================

    ros::Rate loop_rate(20);

    while (ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
