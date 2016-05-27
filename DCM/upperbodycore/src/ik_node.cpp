#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <sepanta_msgs/upperbodymotors.h>
#include <sepanta_msgs/upperbodymotorsfeedback.h>
#include <sepanta_msgs/motorfeedback.h>
#include <sepanta_msgs/motor.h>
#include <sepanta_msgs/motorreset.h>
#include <sepanta_msgs/motorpid.h>
#include <sepanta_msgs/motortorque.h>
#include <sepanta_msgs/grip.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <sepanta_msgs/command.h>
#include <ik/simple_IK.h>
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;
using namespace ros;

#include <pcl/common/centroid.h>

#include <ros/ros.h>

#include <sepanta_msgs/MasterAction.h>

#include <actionlib/server/simple_action_server.h>

#include <upperbody_core/sepanta_ik.h>

IKSystem *_iksystem;

class IKAction
{

public:

    actionlib::SimpleActionServer<sepanta_msgs::MasterAction> action_server_;
    std::string action_name_;
    sepanta_msgs::MasterFeedback feedback_;
    sepanta_msgs::MasterResult result_;
  
    IKAction(ros::NodeHandle nh, std::string name) :
    action_server_(nh, name,boost::bind(&IKAction::executeCB, this, _1), false) ,
    action_name_(name)
    {

    action_server_.start();
    }


void executeCB(const sepanta_msgs::MasterGoalConstPtr &goal) 
{
    if (!action_server_.isActive()) return;

     cout<<"GO TO ACTION FOR X Y TIME : "<<goal->iParam1<<" "<<goal->iParam2<<" "<<goal->iParam3<<endl;
     string result = _iksystem->go_to_xy(goal->iParam1,goal->iParam2,goal->iParam3);
     cout<<"GO TO ACTION FINISHED WITH : "<<result<<endl;

     result_.result = result;
     action_server_.setSucceeded(result_);
}

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "upperbody_ik");
    ROS_INFO("upperbody ik_node started");
    ros::Time::init();

    ros::NodeHandle n;
    ros::NodeHandle n1;
    _iksystem = new IKSystem(n1);
    IKAction *_ikaction = new  IKAction(n,"gotoxyaction");

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
