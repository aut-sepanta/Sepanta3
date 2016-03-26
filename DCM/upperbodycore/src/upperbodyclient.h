
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

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

class upperbodyclient 
{

private:
    ros::NodeHandle node_handles[100];
    ros::Subscriber sub_handles[50];
    ros::Subscriber sub_motors;

    ros::Publisher  chatter_pub_motor_right[8];
    ros::Publisher  chatter_pub_motor_left[8];
    ros::Publisher  chatter_pub_motor_head[3];

    ros::Publisher chatter_pub_allmotors_right;
    ros::Publisher chatter_pub_allmotors_left;
    ros::Publisher chatter_pub_allmotors_head;

    ros::ServiceClient service_reset_motor;
    sepanta_msgs::motorreset reset;

    ros::ServiceClient service_pid_motor;
    sepanta_msgs::motorpid pid;

    ros::ServiceClient service_torquemotor;
    sepanta_msgs::motortorque torque;

    ros::ServiceClient service_grip;
    ros::ServiceClient service_amg;
    ros::Subscriber sub_gripwait;

    bool motor_init;
    bool grip_wait;
    void init();

public:

    struct motor_data
    {
      public :
        int speed;
        int position;
        float load;
        int voltage;
        int temp;
        int id;
        std::string name;
        std::string status;
        int max;
        int min;
        int init;
        std::string model;
        int p;
        int i;
        int d;

    };

std::vector<motor_data> allMotors;

void setMotor_rightShoulderYaw(int position,int speed);
void setMotor_rightShoulderPitch(int position,int speed);
void setMotor_rightShoulderRoll(int position,int speed);
void setMotor_rightElbowPitch(int position,int speed);
void setMotor_rightElbowRoll(int position,int speed);
void setMotor_rightWristPitch(int position,int speed);
void setMotor_rightWristRoll(int position,int speed);
void setMotor_rightGripper(int position,int speed);

void resetMotor_rightShoulderYaw();
void resetMotor_rightShoulderPitch();
void resetMotor_rightShoulderRoll();
void resetMotor_rightElbowPitch();
void resetMotor_rightElbowRoll();
void resetMotor_rightWristPitch();
void resetMotor_rightWristRoll();
void resetMotor_rightGripper();
void resetMotor_rightArm();

void setMotor_leftShoulderYaw(int position,int speed);
void setMotor_leftShoulderPitch(int position,int speed);
void setMotor_leftShoulderRoll(int position,int speed);
void setMotor_leftElbowPitch(int position,int speed);
void setMotor_leftElbowRoll(int position,int speed);
void setMotor_leftWristPitch(int position,int speed);
void setMotor_leftWristRoll(int position,int speed);
void setMotor_leftGripper(int position,int speed);

void resetMotor_leftShoulderYaw();
void resetMotor_leftShoulderPitch();
void resetMotor_leftShoulderRoll();
void resetMotor_leftElbowPitch();
void resetMotor_leftElbowRoll();
void resetMotor_leftWristPitch();
void resetMotor_leftWristRoll();
void resetMotor_leftGripper();
void resetMotor_leftArm();

void setMotor_headYaw(int position,int speed);
void setMotor_headPitch(int position,int speed);
void setMotor_waist(int position,int speed);

void resetMotor_headYaw();
void resetMotor_headPitch();
void resetMotor_waist();

void setMotors_rightArm(int positions[8],int speeds[8]);
void setMotors_leftArm(int positions[8],int speeds[8]);
void setMotors_head(int positions[2],int speeds[2]);



void callback_motors(const sepanta_msgs::upperbodymotorsfeedback::ConstPtr &msg);
void chatterCallback_gripbusy(const std_msgs::String::ConstPtr &msg);

//======================================================================================

motor_data getMotor_rightShoulderYaw();
motor_data getMotor_rightShoulderPitch();
motor_data getMotor_rightShoulderRoll();
motor_data getMotor_rightElbowPitch();
motor_data getMotor_rightElbowRoll();
motor_data getMotor_rightWristPitch();
motor_data getMotor_rightWristRoll();
motor_data getMotor_rightGripper();

motor_data getMotor_leftShoulderYaw();
motor_data getMotor_leftShoulderPitch();
motor_data getMotor_leftShoulderRoll();
motor_data getMotor_leftElbowPitch();
motor_data getMotor_leftElbowRoll();
motor_data getMotor_leftWristPitch();
motor_data getMotor_leftWristRoll();
motor_data getMotor_leftGripper();

motor_data getMotor_headYaw();
motor_data getMotor_headPitch();
motor_data getMotor_waist();

void setMotorPid_rightShoulderYaw(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_rightShoulderPitch(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_rightShoulderRoll(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_rightElbowPitch(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_rightElbowRoll(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_rightWristPitch(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_rightWristRoll(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_rightGripper(uint8_t i,uint8_t p,uint8_t d);

void setMotorPid_leftShoulderYaw(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_leftShoulderPitch(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_leftShoulderRoll(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_leftElbowPitch(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_leftElbowRoll(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_leftWristPitch(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_leftWristRoll(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_leftGripper(uint8_t i,uint8_t p,uint8_t d);

void setMotorPid_headYaw(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_headPitch(uint8_t i,uint8_t p,uint8_t d);
void setMotorPid_waist(uint8_t i,uint8_t p,uint8_t d);

void torqueToggle_allMotors(bool value);

void right_side();
void right_down();
void right_up();
void right_front();

void left_side();
void left_down();
void left_up();
void left_front();

std::string grip_left(float X,float Y,float Z,float alpha,float beta,float gama);
std::string grip_right(float X,float Y,float Z,float alpha,float beta,float gama);
bool getgripwait();
void setamg(std::string command);

//====================================================================================

upperbodyclient()
{
    motor_init = false;
    init();
}

~upperbodyclient()
{

}

};
