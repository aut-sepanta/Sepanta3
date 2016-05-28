#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sepanta_msgs/LookForObjectsAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_LookForObjects");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<sepanta_msgs::LookForObjectsAction> ac("look_for_objects", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  sepanta_msgs::LookForObjectsGoal goal;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());

    sepanta_msgs::LookForObjectsResult::ConstPtr _res = ac.getResult();


  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}