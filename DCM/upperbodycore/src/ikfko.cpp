#include "upperbodyclient.h"
#include <upperbodycore_msgs/objectsposition.h>
#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <math.h>

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

bool appExit = false;
upperbodyclient *gClinet;
ros::Subscriber sub_position_objects;
bool grip_busy = false;
bool grip_cancle = false;

std::string coutcolor0 = "\033[0;0m";
std::string coutcolor_red = "\033[0;31m";
std::string coutcolor_green = "\033[0;32m";
std::string coutcolor_blue = "\033[0;34m";
std::string coutcolor_magenta = "\033[0;35m";
std::string coutcolor_brown = "\033[0;33m";

using namespace std;

struct object_data
{
public :

    std::string name;
    float x;
    float y;
    float z;
};

std::vector<object_data> object_list;

float pi=3.141592;

Eigen::MatrixXd RotTrans(char type,double x){
    Eigen::MatrixXd t(4,4);
    if (type=='X'){
        t<<1 ,0 ,0 ,0,
                0, cos(x), -sin(x) ,0,
                0, sin(x), cos(x) ,0,
                0, 0, 0 ,1;
    }
    if (type=='Y'){
        t<<cos(x), 0, sin(x), 0,
                0,1,0,0,
                -sin(x) ,0, cos(x) ,0,
                0 ,0, 0, 1;
    }
    if (type=='Z'){
        t<<cos(x) ,-sin(x) ,0, 0,
                sin(x) ,cos(x) ,0, 0,
                0 ,0 ,1, 0,
                0 ,0,0, 1;
    }
    return t;
}
Eigen::MatrixXd RotTrans(char type,double x[]){
    Eigen::MatrixXd t(4,4);
    if (type=='P'){
        t<<1, 0, 0 ,x[0],
                0, 1, 0, x[1],
                0, 0 ,1 ,x[2],
                0 ,0, 0 ,1;
    }
    return t;
}

double Qsx0=0;
double Qsy0=0;
double Qsz0=0;
double Qey0=0;
double Qwz0=0;
double Qwy0=0;
double Qwx0=0;
//============================
ros::ServiceServer service_grip;
ros::ServiceServer service_amg;
//===========================
ros::Publisher pub_grip_busy;

struct grip_position
{
public:
    double x;
    double y;
    double z;
    double a;
    double b;
    double g;
};

grip_position end_position;

ros::Publisher chatter_pub_ack;
ros::Publisher chatter_pub_log;

bool start_grip = false;
std::string right_left = "right";

void send_ack()
{
    std_msgs::String msg_ack;
    msg_ack.data = "ok";
    chatter_pub_ack.publish(msg_ack);
}

void send_log(std::string msg)
{
    std_msgs::String msg_log;
    msg_log.data = msg;
    chatter_pub_log.publish(msg_log);
}

//============================ AMG

int cycle(float v,float d)
{
    double Time=2*(d/100)/(v/360);
    std::cout<<"Time="<<Time<<std::endl;
    double dmax=45;
    if (d>dmax){
        d=dmax;
    }

    double a=d/dmax;
    //refrence realtime position
    upperbodyclient::motor_data PRH1 = gClinet->getMotor_rightShoulderYaw();
    upperbodyclient::motor_data PRH2 = gClinet->getMotor_rightShoulderPitch();
    upperbodyclient::motor_data PRH3 = gClinet->getMotor_rightShoulderRoll();
    upperbodyclient::motor_data PRH4 = gClinet->getMotor_rightElbowPitch();
    upperbodyclient::motor_data PRH5 = gClinet->getMotor_rightElbowRoll();
    upperbodyclient::motor_data PRH6 = gClinet->getMotor_rightWristPitch();
    upperbodyclient::motor_data PRH7 = gClinet->getMotor_rightWristRoll();

    upperbodyclient::motor_data PLH1 = gClinet->getMotor_leftShoulderYaw();
    upperbodyclient::motor_data PLH2 = gClinet->getMotor_leftShoulderPitch();
    upperbodyclient::motor_data PLH3 = gClinet->getMotor_leftShoulderRoll();
    upperbodyclient::motor_data PLH4 = gClinet->getMotor_leftElbowPitch();
    upperbodyclient::motor_data PLH5 = gClinet->getMotor_leftElbowRoll();
    upperbodyclient::motor_data PLH6 = gClinet->getMotor_leftWristPitch();
    upperbodyclient::motor_data PLH7 = gClinet->getMotor_leftWristRoll();

    //refrens global start
    double RH1=1991;
    double RH2=3400;
    double RH3=1540;
    double RH4=2560;
    double RH5=2544;
    double RH6=1107;
    double RH7=2832;

    //refrens global start
    double LH1=1414;
    double LH2=937;
    double LH3=2432;
    double LH4=1596;
    double LH5=1944;
    double LH6=1291;
    double LH7=2011;

    //global ta jolo
    double DLHj1=226;
    double DLHj2=0;
    double DLHj3=0;
    double DLHj4=92;
    double DLHj5=0;
    double DLHj6=0;
    double DLHj7=0;

    //global ta jolo
    double DRHj1=226;
    double DRHj2=0;
    double DRHj3=0;
    double DRHj4=92;
    double DRHj5=0;
    double DRHj6=0;
    double DRHj7=0;

    //global ta aghab
    double DLHa1=-314;
    double DLHa2=0;
    double DLHa3=0;
    double DLHa4=-137;
    double DLHa5=0;
    double DLHa6=0;
    double DLHa7=0;

    //global ta jolo
    double DRHa1=-314;
    double DRHa2=0;
    double DRHa3=0;
    double DRHa4=-137;
    double DRHa5=0;
    double DRHa6=0;
    double DRHa7=0;




    double AsXr;
    double BsXr;
    double AsYr;
    double BsYr;
    double AsZr;
    double BsZr;
    double AeYr;
    double BeYr;
    double AwZr;
    double BwZr;
    double AwYr;
    double BwYr;
    double AwXr;
    double BwXr;

    double AsXl;
    double BsXl;
    double AsYl;
    double BsYl;
    double AsZl;
    double BsZl;
    double AeYl;
    double BeYl;
    double AwZl;
    double BwZl;
    double AwYl;
    double BwYl;
    double AwXl;
    double BwXl;


    if ( gClinet->allMotors.size() != 0 )
    {
        ///left hand
        AsYl = (float)-180/2048;


        AsXl = (float)180/2048;
        BsXl = 937;

        AsZl = (float)-180/2048;
        BsZl = 2432;

        AeYl = (float)-180/2048;
        BeYl = 1356;

        AwZl = (float)-180/2048;
        BwZl = 1944;

        AwYl = (float)-180/2048;
        BwYl = 1291;

        AwXl = (float)-180/2048;
        BwXl = 2011;


        ////
        ///right hand
        AsYr = (float)-180/2048;
        BsYr = 1947;

        AsXr = (float)180/2048;
        BsXr = 3400;

        AsZr = (float)-180/2048;
        BsZr = 1540;

        AeYr = (float)-180/2048;
        BeYr = 2320;

        AwZr = (float)-180/2048;
        BwZr = 2544;

        AwYr = (float)-180/2048;
        BwYr = 1107;

        AwXr = (float)-180/2048;
        BwXr = 2832;



    }
    else
        std::cout<<"nomotors"<<std::endl;





    std::cout<<"1"<<std::endl;



    // Right hand //mire aghab

    double Qs_Ysend=RH1+a*DRHa1;
    int speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    double Qs_Xsend=RH2+a*DRHa2;
    int speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    double Qs_Zsend=RH3+a*DRHa3;
    int speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    double Qe_Ysend=RH4+a*DRHa4;
    int speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    double Qw_Zsend=RH5+a*DRHa5;
    int speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    double Qw_Ysend=RH6+a*DRHa6;
    int speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    double Qw_Xsend=RH7+a*DRHa7;
    int speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);


    // Left hand //mire jolo

    Qs_Ysend=LH1+a*DLHj1;
    speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2+a*DLHj2;
    speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3+a*DLHj3;
    speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4+a*DLHj4;
    speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5+a*DLHj5;
    speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6+a*DLHj6;
    speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7+a*DLHj7;
    speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000/2));

    std::cout<<"3"<<std::endl;

    //Right hand //mire jolo
    Qs_Ysend=RH1+a*DRHj1;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=RH2+a*DRHj2;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=RH3+a*DRHj3;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=RH4+a*DRHj4;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=RH5+a*DRHj5;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=RH6+a*DRHj6;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=RH7+a*DRHj7;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    //Left hand //mire aghab
    Qs_Ysend=LH1+a*DLHa1;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2+a*DLHa2;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3+a*DLHa3;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4+a*DLHa4;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5+a*DLHa5;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6+a*DLHa6;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7+a*DLHa7;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);

    std::cout<<"motor sent commands are="<<std::endl<<Qs_Ysend<<std::endl<<Qs_Xsend<<std::endl<<Qs_Zsend<<std::endl<<Qe_Ysend<<std::endl<<Qw_Zsend<<std::endl<<Qw_Ysend<<std::endl<<Qw_Xsend<<std::endl<<std::endl ;
    boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000/2));

}

int cycle_left(float v,float d)
{
    double Time=2*(d/100)/(v/360);
    std::cout<<"Time="<<Time<<std::endl;
    double dmax=45;
    if (d>dmax){
        d=dmax;
    }

    double a=d/dmax;
    //refrence realtime position
    upperbodyclient::motor_data PRH1 = gClinet->getMotor_rightShoulderYaw();
    upperbodyclient::motor_data PRH2 = gClinet->getMotor_rightShoulderPitch();
    upperbodyclient::motor_data PRH3 = gClinet->getMotor_rightShoulderRoll();
    upperbodyclient::motor_data PRH4 = gClinet->getMotor_rightElbowPitch();
    upperbodyclient::motor_data PRH5 = gClinet->getMotor_rightElbowRoll();
    upperbodyclient::motor_data PRH6 = gClinet->getMotor_rightWristPitch();
    upperbodyclient::motor_data PRH7 = gClinet->getMotor_rightWristRoll();

    upperbodyclient::motor_data PLH1 = gClinet->getMotor_leftShoulderYaw();
    upperbodyclient::motor_data PLH2 = gClinet->getMotor_leftShoulderPitch();
    upperbodyclient::motor_data PLH3 = gClinet->getMotor_leftShoulderRoll();
    upperbodyclient::motor_data PLH4 = gClinet->getMotor_leftElbowPitch();
    upperbodyclient::motor_data PLH5 = gClinet->getMotor_leftElbowRoll();
    upperbodyclient::motor_data PLH6 = gClinet->getMotor_leftWristPitch();
    upperbodyclient::motor_data PLH7 = gClinet->getMotor_leftWristRoll();

    //refrens global start
    double RH1=1991;
    double RH2=3400;
    double RH3=1540;
    double RH4=2560;
    double RH5=2544;
    double RH6=1107;
    double RH7=2832;

    //refrens global start
    double LH1=1414;
    double LH2=937;
    double LH3=2432;
    double LH4=1596;
    double LH5=1944;
    double LH6=1291;
    double LH7=2011;

    //global ta jolo
    double DLHj1=226;
    double DLHj2=0;
    double DLHj3=0;
    double DLHj4=92;
    double DLHj5=0;
    double DLHj6=0;
    double DLHj7=0;

    //global ta jolo
    double DRHj1=226;
    double DRHj2=0;
    double DRHj3=0;
    double DRHj4=92;
    double DRHj5=0;
    double DRHj6=0;
    double DRHj7=0;

    //global ta aghab
    double DLHa1=-314;
    double DLHa2=0;
    double DLHa3=0;
    double DLHa4=-137;
    double DLHa5=0;
    double DLHa6=0;
    double DLHa7=0;

    //global ta jolo
    double DRHa1=-314;
    double DRHa2=0;
    double DRHa3=0;
    double DRHa4=-137;
    double DRHa5=0;
    double DRHa6=0;
    double DRHa7=0;




    double AsXr;
    double BsXr;
    double AsYr;
    double BsYr;
    double AsZr;
    double BsZr;
    double AeYr;
    double BeYr;
    double AwZr;
    double BwZr;
    double AwYr;
    double BwYr;
    double AwXr;
    double BwXr;

    double AsXl;
    double BsXl;
    double AsYl;
    double BsYl;
    double AsZl;
    double BsZl;
    double AeYl;
    double BeYl;
    double AwZl;
    double BwZl;
    double AwYl;
    double BwYl;
    double AwXl;
    double BwXl;


    if ( gClinet->allMotors.size() != 0 )
    {
        ///left hand
        AsYl = (float)-180/2048;


        AsXl = (float)180/2048;
        BsXl = 937;

        AsZl = (float)-180/2048;
        BsZl = 2432;

        AeYl = (float)-180/2048;
        BeYl = 1356;

        AwZl = (float)-180/2048;
        BwZl = 1944;

        AwYl = (float)-180/2048;
        BwYl = 1291;

        AwXl = (float)-180/2048;
        BwXl = 2011;


        ////
        ///right hand
        AsYr = (float)-180/2048;
        BsYr = 1947;

        AsXr = (float)180/2048;
        BsXr = 3400;

        AsZr = (float)-180/2048;
        BsZr = 1540;

        AeYr = (float)-180/2048;
        BeYr = 2320;

        AwZr = (float)-180/2048;
        BwZr = 2544;

        AwYr = (float)-180/2048;
        BwYr = 1107;

        AwXr = (float)-180/2048;
        BwXr = 2832;



    }
    else
        std::cout<<"nomotors"<<std::endl;





    std::cout<<"1"<<std::endl;



    // Right hand //mire jolo

    double Qs_Ysend=RH1+a*DRHj1;
    int speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    double Qs_Xsend=RH2+a*DRHj2;
    int speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    double Qs_Zsend=RH3+a*DRHj3;
    int speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    double Qe_Ysend=RH4+a*DRHj4;
    int speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    double Qw_Zsend=RH5+a*DRHj5;
    int speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    double Qw_Ysend=RH6+a*DRHj6;
    int speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    double Qw_Xsend=RH7+a*DRHj7;
    int speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);


    // Left hand //mire aghab

    Qs_Ysend=LH1+a*DLHa1;
    speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2+a*DLHa2;
    speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3+a*DLHa3;
    speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4+a*DLHa4;
    speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5+a*DLHa5;
    speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6+a*DLHa6;
    speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7+a*DLHa7;
    speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000/2));

    std::cout<<"3"<<std::endl;

    //Right hand //mire aghab
    Qs_Ysend=RH1+a*DRHa1;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=RH2+a*DRHa2;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=RH3+a*DRHa3;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=RH4+a*DRHa4;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=RH5+a*DRHa5;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=RH6+a*DRHa6;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=RH7+a*DRHa7;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    //Left hand //mire jolo
    Qs_Ysend=LH1+a*DLHj1;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2+a*DLHj2;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3+a*DLHj3;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4+a*DLHj4;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5+a*DLHj5;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6+a*DLHj6;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7+a*DLHj7;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000/2));




    std::cout<<"motor sent commands are="<<std::endl<<Qs_Ysend<<std::endl<<Qs_Xsend<<std::endl<<Qs_Zsend<<std::endl<<Qe_Ysend<<std::endl<<Qw_Zsend<<std::endl<<Qw_Ysend<<std::endl<<Qw_Xsend<<std::endl<<std::endl ;

}

int start(int d)
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    double dmax=45;
    if (d>dmax)
    {
        d=dmax;
    }


    double a=d/dmax;
    double Time=6;

    //refrens global start
    double RH1=1991;
    double RH2=3400;
    double RH3=1540;
    double RH4=2560;
    double RH5=2544;
    double RH6=1107;
    double RH7=2832;

    //refrens global start
    double LH1=1414;
    double LH2=937;
    double LH3=2432;
    double LH4=1596;
    double LH5=1944;
    double LH6=1291;
    double LH7=2011;


    //global ta jolo
    double DRH1=226;
    double DRH2=0;
    double DRH3=0;
    double DRH4=92;
    double DRH5=0;
    double DRH6=0;
    double DRH7=0;

    //global ta aghab
    double DLH1=-314;
    double DLH2=0;
    double DLH3=0;
    double DLH4=-137;
    double DLH5=0;
    double DLH6=0;
    double DLH7=0;


    double AsXr;
    double BsXr;
    double AsYr;
    double BsYr;
    double AsZr;
    double BsZr;
    double AeYr;
    double BeYr;
    double AwZr;
    double BwZr;
    double AwYr;
    double BwYr;
    double AwXr;
    double BwXr;

    double AsXl;
    double BsXl;
    double AsYl;
    double BsYl;
    double AsZl;
    double BsZl;
    double AeYl;
    double BeYl;
    double AwZl;
    double BwZl;
    double AwYl;
    double BwYl;
    double AwXl;
    double BwXl;


    // if (left) {
    if ( gClinet->allMotors.size() != 0 )
    {
        ///left hand
        AsYl = (float)-180/2048;


        AsXl = (float)180/2048;
        BsXl = 937;

        AsZl = (float)-180/2048;
        BsZl = 2432;

        AeYl = (float)-180/2048;
        BeYl = 1356;

        AwZl = (float)-180/2048;
        BwZl = 1944;

        AwYl = (float)-180/2048;
        BwYl = 1291;

        AwXl = (float)-180/2048;
        BwXl = 2011;


        ////
        ///right hand
        AsYr = (float)-180/2048;
        BsYr = 1947;

        AsXr = (float)180/2048;
        BsXr = 3400;

        AsZr = (float)-180/2048;
        BsZr = 1540;

        AeYr = (float)-180/2048;
        BeYr = 2320;

        AwZr = (float)-180/2048;
        BwZr = 2544;

        AwYr = (float)-180/2048;
        BwYr = 1107;

        AwXr = (float)-180/2048;
        BwXr = 2832;



    }
    else
        std::cout<<"nomotors"<<std::endl;


    //Right hand //mire jolo



    double  Qs_Ysend=RH1+a*DRH1;
    int speeds_Y=((fabs(Qs_Ysend-RH1)*fabs(AsYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    double  Qs_Xsend=RH2+a*DRH2;
    int speeds_X=((fabs(Qs_Xsend-RH2)*fabs(AsXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    double  Qs_Zsend=RH3+a*DRH3;
    int speeds_Z=((fabs(Qs_Zsend-RH3)*fabs(AsZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    double  Qe_Ysend=RH4+a*DRH4;
    int speede_Y=((fabs(Qe_Ysend-RH4)*fabs(AeYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    double   Qw_Zsend=RH5+a*DRH5;
    int speedw_Z=((fabs(Qw_Zsend-RH5)*fabs(AwZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    double   Qw_Ysend=RH6+a*DRH6;
    int speedw_Y=((fabs(Qw_Ysend-RH6)*fabs(AwYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    double  Qw_Xsend=RH7+a*DRH7;
    int speedw_X=((fabs(Qw_Xsend-RH7)*fabs(AwXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    //Left hand //mire aghab

    Qs_Ysend=LH1+a*DLH1;
    speeds_Y=((fabs(Qs_Ysend-LH1)*fabs(AsYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2+a*DLH2;
    speeds_X=((fabs(Qs_Xsend-LH2)*fabs(AsXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3+a*DLH3;
    speeds_Z=((fabs(Qs_Zsend-LH3)*fabs(AsZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4+a*DLH4;
    speede_Y=((fabs(Qe_Ysend-LH4)*fabs(AeYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5+a*DLH5;
    speedw_Z=((fabs(Qw_Zsend-LH5)*fabs(AwZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6+a*DLH6;
    speedw_Y=((fabs(Qw_Ysend-LH6)*fabs(AwYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7+a*DLH7;
    speedw_X=((fabs(Qw_Xsend-LH7)*fabs(AwXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);



    boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000));


    std::cout<<"motor sent commands are="<<std::endl<<Qs_Ysend<<std::endl<<Qs_Xsend<<std::endl<<Qs_Zsend<<std::endl<<Qe_Ysend<<std::endl<<Qw_Zsend<<std::endl<<Qw_Ysend<<std::endl<<Qw_Xsend<<std::endl<<std::endl ;

}

int end()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    double Time=3.5;

    //refrence realtime position
    upperbodyclient::motor_data RH1 = gClinet->getMotor_rightShoulderYaw();
    upperbodyclient::motor_data RH2 = gClinet->getMotor_rightShoulderPitch();
    upperbodyclient::motor_data RH3 = gClinet->getMotor_rightShoulderRoll();
    upperbodyclient::motor_data RH4 = gClinet->getMotor_rightElbowPitch();
    upperbodyclient::motor_data RH5 = gClinet->getMotor_rightElbowRoll();
    upperbodyclient::motor_data RH6 = gClinet->getMotor_rightWristPitch();
    upperbodyclient::motor_data RH7 = gClinet->getMotor_rightWristRoll();

    upperbodyclient::motor_data LH1 = gClinet->getMotor_leftShoulderYaw();
    upperbodyclient::motor_data LH2 = gClinet->getMotor_leftShoulderPitch();
    upperbodyclient::motor_data LH3 = gClinet->getMotor_leftShoulderRoll();
    upperbodyclient::motor_data LH4 = gClinet->getMotor_leftElbowPitch();
    upperbodyclient::motor_data LH5 = gClinet->getMotor_leftElbowRoll();
    upperbodyclient::motor_data LH6 = gClinet->getMotor_leftWristPitch();
    upperbodyclient::motor_data LH7 = gClinet->getMotor_leftWristRoll();

    //refrence global
    double gr1=1991;
    double gr2=3400;
    double gr3=1540;
    double gr4=2560;
    double gr5=2544;
    double gr6=1107;
    double gr7=2832;

    double gl1=1414;
    double gl2=937;
    double gl3=2432;
    double gl4=1596;
    double gl5=1944;
    double gl6=1291;
    double gl7=2011;



    double AsXr;
    double BsXr;
    double AsYr;
    double BsYr;
    double AsZr;
    double BsZr;
    double AeYr;
    double BeYr;
    double AwZr;
    double BwZr;
    double AwYr;
    double BwYr;
    double AwXr;
    double BwXr;

    double AsXl;
    double BsXl;
    double AsYl;
    double BsYl;
    double AsZl;
    double BsZl;
    double AeYl;
    double BeYl;
    double AwZl;
    double BwZl;
    double AwYl;
    double BwYl;
    double AwXl;
    double BwXl;



    // if (left) {
    if ( gClinet->allMotors.size() != 0 )
    {
        ///left hand
        AsYl = (float)-180/2048;


        AsXl = (float)180/2048;
        BsXl = 937;

        AsZl = (float)-180/2048;
        BsZl = 2432;

        AeYl = (float)-180/2048;
        BeYl = 1356;

        AwZl = (float)-180/2048;
        BwZl = 1944;

        AwYl = (float)-180/2048;
        BwYl = 1291;

        AwXl = (float)-180/2048;
        BwXl = 2011;


        ////
        ///right hand
        AsYr = (float)-180/2048;
        BsYr = 1947;

        AsXr = (float)180/2048;
        BsXr = 3400;

        AsZr = (float)-180/2048;
        BsZr = 1540;

        AeYr = (float)-180/2048;
        BeYr = 2320;

        AwZr = (float)-180/2048;
        BwZr = 2544;

        AwYr = (float)-180/2048;
        BwYr = 1107;

        AwXr = (float)-180/2048;
        BwXr = 2832;



    }
    else
        std::cout<<"nomotors"<<std::endl;





    std::cout<<"1"<<std::endl;



    // Right hand //mire global


    double Qs_Ysend=gr1;
    int speeds_Y=((fabs(Qs_Ysend-RH1.position)*fabs(AsYr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    double Qs_Xsend=gr2;
    int speeds_X=((fabs(Qs_Xsend-RH2.position)*fabs(AsXr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    double Qs_Zsend=gr3;
    int speeds_Z=((fabs(Qs_Zsend-RH3.position)*fabs(AsZr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    double Qe_Ysend=gr4;
    int speede_Y=((fabs(Qe_Ysend-RH4.position)*fabs(AeYr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    double Qw_Zsend=gr5;
    int speedw_Z=((fabs(Qw_Zsend-RH5.position)*fabs(AwZr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    double Qw_Ysend=gr6;
    int speedw_Y=((fabs(Qw_Ysend-RH6.position)*fabs(AwYr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    double Qw_Xsend=gr7;
    int speedw_X=((fabs(Qw_Xsend-RH7.position)*fabs(AwXr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);


    // Left hand //mire global

    Qs_Ysend=gl1;
    speeds_Y=((fabs(Qs_Ysend-LH1.position)*fabs(AsYl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=gl2;
    speeds_X=((fabs(Qs_Xsend-LH2.position)*fabs(AsXl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=gl3;
    speeds_Z=((fabs(Qs_Zsend-LH3.position)*fabs(AsZl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=gl4;
    speede_Y=((fabs(Qe_Ysend-LH4.position)*fabs(AeYl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=gl5;
    speedw_Z=((fabs(Qw_Zsend-LH5.position)*fabs(AwZl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=gl6;
    speedw_Y=((fabs(Qw_Ysend-LH6.position)*fabs(AwYl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=gl7;
    speedw_X=((fabs(Qw_Xsend-LH7.position)*fabs(AwXl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(3500));

    std::cout<<"3"<<std::endl;
}

int Global_start()
{

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    double Time=2;

    //refrence realtime position
    upperbodyclient::motor_data RH1 = gClinet->getMotor_rightShoulderYaw();
    upperbodyclient::motor_data RH2 = gClinet->getMotor_rightShoulderPitch();
    upperbodyclient::motor_data RH3 = gClinet->getMotor_rightShoulderRoll();
    upperbodyclient::motor_data RH4 = gClinet->getMotor_rightElbowPitch();
    upperbodyclient::motor_data RH5 = gClinet->getMotor_rightElbowRoll();
    upperbodyclient::motor_data RH6 = gClinet->getMotor_rightWristPitch();
    upperbodyclient::motor_data RH7 = gClinet->getMotor_rightWristRoll();

    upperbodyclient::motor_data LH1 = gClinet->getMotor_leftShoulderYaw();
    upperbodyclient::motor_data LH2 = gClinet->getMotor_leftShoulderPitch();
    upperbodyclient::motor_data LH3 = gClinet->getMotor_leftShoulderRoll();
    upperbodyclient::motor_data LH4 = gClinet->getMotor_leftElbowPitch();
    upperbodyclient::motor_data LH5 = gClinet->getMotor_leftElbowRoll();
    upperbodyclient::motor_data LH6 = gClinet->getMotor_leftWristPitch();
    upperbodyclient::motor_data LH7 = gClinet->getMotor_leftWristRoll();


    //refrence global
    double gr1=1991;
    double gr2=3400;
    double gr3=1540;
    double gr4=2560;
    double gr5=2544;
    double gr6=1107;
    double gr7=2832;

    double gl1=1414;
    double gl2=937;
    double gl3=2432;
    double gl4=1596;
    double gl5=1944;
    double gl6=1291;
    double gl7=2011;



    double AsXr;
    double BsXr;
    double AsYr;
    double BsYr;
    double AsZr;
    double BsZr;
    double AeYr;
    double BeYr;
    double AwZr;
    double BwZr;
    double AwYr;
    double BwYr;
    double AwXr;
    double BwXr;

    double AsXl;
    double BsXl;
    double AsYl;
    double BsYl;
    double AsZl;
    double BsZl;
    double AeYl;
    double BeYl;
    double AwZl;
    double BwZl;
    double AwYl;
    double BwYl;
    double AwXl;
    double BwXl;



    // if (left) {
    if ( gClinet->allMotors.size() != 0 )
    {
        ///left hand
        AsYl = (float)-180/2048;


        AsXl = (float)180/2048;
        BsXl = 937;

        AsZl = (float)-180/2048;
        BsZl = 2432;

        AeYl = (float)-180/2048;
        BeYl = 1356;

        AwZl = (float)-180/2048;
        BwZl = 1944;

        AwYl = (float)-180/2048;
        BwYl = 1291;

        AwXl = (float)-180/2048;
        BwXl = 2011;


        ////
        ///right hand
        AsYr = (float)-180/2048;
        BsYr = 1947;

        AsXr = (float)180/2048;
        BsXr = 3400;

        AsZr = (float)-180/2048;
        BsZr = 1540;

        AeYr = (float)-180/2048;
        BeYr = 2320;

        AwZr = (float)-180/2048;
        BwZr = 2544;

        AwYr = (float)-180/2048;
        BwYr = 1107;

        AwXr = (float)-180/2048;
        BwXr = 2832;



    }
    else
        std::cout<<"nomotors"<<std::endl;


    //Right hand

    double  Qs_Ysend=gr1;
    int speeds_Y=((fabs(Qs_Ysend-RH1.position)*fabs(AsYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    double  Qs_Xsend=gr2;
    int speeds_X=((fabs(Qs_Xsend-RH2.position)*fabs(AsXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    double  Qs_Zsend=gr3;
    int speeds_Z=((fabs(Qs_Zsend-RH3.position)*fabs(AsZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    double  Qe_Ysend=gr4;
    int speede_Y=((fabs(Qe_Ysend-RH4.position)*fabs(AeYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    double   Qw_Zsend=gr5;
    int speedw_Z=((fabs(Qw_Zsend-RH5.position)*fabs(AwZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    double   Qw_Ysend=gr6;
    int speedw_Y=((fabs(Qw_Ysend-RH6.position)*fabs(AwYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    double  Qw_Xsend=gr7;
    int speedw_X=((fabs(Qw_Xsend-RH7.position)*fabs(AwXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    //Left hand

    Qs_Ysend=gl1;
    speeds_Y=((fabs(Qs_Ysend-LH1.position)*fabs(AsYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=gl2;
    speeds_X=((fabs(Qs_Xsend-LH2.position)*fabs(AsXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=gl3;
    speeds_Z=((fabs(Qs_Zsend-LH3.position)*fabs(AsZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=gl4;
    speede_Y=((fabs(Qe_Ysend-LH4.position)*fabs(AeYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=gl5;
    speedw_Z=((fabs(Qw_Zsend-LH5.position)*fabs(AwZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=gl6;
    speedw_Y=((fabs(Qw_Ysend-LH6.position)*fabs(AwYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=gl7;
    speedw_X=((fabs(Qw_Xsend-LH7.position)*fabs(AwXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);


    std::cout<<"motor sent commands are="<<std::endl<<Qs_Ysend<<std::endl<<Qs_Xsend<<std::endl<<Qs_Zsend<<std::endl<<Qe_Ysend<<std::endl<<Qw_Zsend<<std::endl<<Qw_Ysend<<std::endl<<Qw_Xsend<<std::endl<<std::endl ;
    boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000));

}

int Global_end()
{
    gClinet->setMotor_rightShoulderYaw(gClinet->getMotor_rightShoulderYaw().init,100);
    gClinet->setMotor_rightShoulderPitch(gClinet->getMotor_rightShoulderPitch().init,100);
    gClinet->setMotor_rightShoulderRoll(gClinet->getMotor_rightShoulderRoll().init,100);
    gClinet->setMotor_rightElbowPitch(gClinet->getMotor_rightElbowPitch().init,100);
    gClinet->setMotor_rightElbowRoll(gClinet->getMotor_rightElbowRoll().init,100);
    gClinet->setMotor_rightWristPitch(gClinet->getMotor_rightWristPitch().init,100);
    gClinet->setMotor_rightWristRoll(gClinet->getMotor_rightWristRoll().init,100);

    gClinet->setMotor_leftShoulderYaw(gClinet->getMotor_leftShoulderYaw().init,100);
    gClinet->setMotor_leftShoulderPitch(gClinet->getMotor_leftShoulderPitch().init,100);
    gClinet->setMotor_leftShoulderRoll(gClinet->getMotor_leftShoulderRoll().init,100);
    gClinet->setMotor_leftElbowPitch(gClinet->getMotor_leftElbowPitch().init,100);
    gClinet->setMotor_leftElbowRoll(gClinet->getMotor_leftElbowRoll().init,100);
    gClinet->setMotor_leftWristPitch(gClinet->getMotor_leftWristPitch().init,100);
    gClinet->setMotor_leftWristRoll(gClinet->getMotor_leftWristRoll().init,100);

}

int start_left(int d)
{
    double dmax=45;
    if (d>dmax)
    {
        d=dmax;
    }
    double a=d/dmax;
    double Time=6;
    //refrens global start
    double RH1=1991;
    double RH2=3400;
    double RH3=1540;
    double RH4=2560;
    double RH5=2544;
    double RH6=1107;
    double RH7=2832;

    //refrens global start
    double LH1=1414;
    double LH2=937;
    double LH3=2432;
    double LH4=1596;
    double LH5=1944;
    double LH6=1291;
    double LH7=2011;

    //global ta jolo
    double DLH1=226;
    double DLH2=0;
    double DLH3=0;
    double DLH4=92;
    double DLH5=0;
    double DLH6=0;
    double DLH7=0;


    //global ta aghab
    double DRH1=-314;
    double DRH2=0;
    double DRH3=0;
    double DRH4=-137;
    double DRH5=0;
    double DRH6=0;
    double DRH7=0;


    double AsXr;
    double BsXr;
    double AsYr;
    double BsYr;
    double AsZr;
    double BsZr;
    double AeYr;
    double BeYr;
    double AwZr;
    double BwZr;
    double AwYr;
    double BwYr;
    double AwXr;
    double BwXr;

    double AsXl;
    double BsXl;
    double AsYl;
    double BsYl;
    double AsZl;
    double BsZl;
    double AeYl;
    double BeYl;
    double AwZl;
    double BwZl;
    double AwYl;
    double BwYl;
    double AwXl;
    double BwXl;


    // if (left) {
    if ( gClinet->allMotors.size() != 0 )
    {
        ///left hand
        AsYl = (float)-180/2048;


        AsXl = (float)180/2048;
        BsXl = 937;

        AsZl = (float)-180/2048;
        BsZl = 2432;

        AeYl = (float)-180/2048;
        BeYl = 1356;

        AwZl = (float)-180/2048;
        BwZl = 1944;

        AwYl = (float)-180/2048;
        BwYl = 1291;

        AwXl = (float)-180/2048;
        BwXl = 2011;


        ////
        ///right hand
        AsYr = (float)-180/2048;
        BsYr = 1947;

        AsXr = (float)180/2048;
        BsXr = 3400;

        AsZr = (float)-180/2048;
        BsZr = 1540;

        AeYr = (float)-180/2048;
        BeYr = 2320;

        AwZr = (float)-180/2048;
        BwZr = 2544;

        AwYr = (float)-180/2048;
        BwYr = 1107;

        AwXr = (float)-180/2048;
        BwXr = 2832;



    }
    else
        std::cout<<"nomotors"<<std::endl;


    //Right hand //mire jolo



    double  Qs_Ysend=RH1+a*DRH1;
    int speeds_Y=((fabs(Qs_Ysend-RH1)*fabs(AsYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    double  Qs_Xsend=RH2+a*DRH2;
    int speeds_X=((fabs(Qs_Xsend-RH2)*fabs(AsXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    double  Qs_Zsend=RH3+a*DRH3;
    int speeds_Z=((fabs(Qs_Zsend-RH3)*fabs(AsZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    double  Qe_Ysend=RH4+a*DRH4;
    int speede_Y=((fabs(Qe_Ysend-RH4)*fabs(AeYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    double   Qw_Zsend=RH5+a*DRH5;
    int speedw_Z=((fabs(Qw_Zsend-RH5)*fabs(AwZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    double   Qw_Ysend=RH6+a*DRH6;
    int speedw_Y=((fabs(Qw_Ysend-RH6)*fabs(AwYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    double  Qw_Xsend=RH7+a*DRH7;
    int speedw_X=((fabs(Qw_Xsend-RH7)*fabs(AwXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    //Left hand //mire aghab

    Qs_Ysend=LH1+a*DLH1;
    speeds_Y=((fabs(Qs_Ysend-LH1)*fabs(AsYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2+a*DLH2;
    speeds_X=((fabs(Qs_Xsend-LH2)*fabs(AsXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3+a*DLH3;
    speeds_Z=((fabs(Qs_Zsend-LH3)*fabs(AsZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4+a*DLH4;
    speede_Y=((fabs(Qe_Ysend-LH4)*fabs(AeYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5+a*DLH5;
    speedw_Z=((fabs(Qw_Zsend-LH5)*fabs(AwZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6+a*DLH6;
    speedw_Y=((fabs(Qw_Ysend-LH6)*fabs(AwYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7+a*DLH7;
    speedw_X=((fabs(Qw_Xsend-LH7)*fabs(AwXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);


    std::cout<<"motor sent commands are="<<std::endl<<Qs_Ysend<<std::endl<<Qs_Xsend<<std::endl<<Qs_Zsend<<std::endl<<Qe_Ysend<<std::endl<<Qw_Zsend<<std::endl<<Qw_Ysend<<std::endl<<Qw_Xsend<<std::endl<<std::endl ;
    boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000));

}

int stair_down()
{
    float d = 35;
    float v = 22;
    double Time=2*(d/100)/(v/360);
    std::cout<<"Time="<<Time<<std::endl;
    double dmax=45;
    if (d>dmax){
        d=dmax;
    }
    double a=d/dmax;
    //refrens global start
    double RH1=1991;
    double RH2=3400;
    double RH3=1540;
    double RH4=2560;
    double RH5=2544;
    double RH6=1107;
    double RH7=2832;

    //refrens global start
    double LH1=1424;
    double LH2=905;
    double LH3=2432;
    double LH4=1596;
    double LH5=1944;
    double LH6=1291;
    double LH7=2011;

    //global ta jolo
    double DLHj1=226;
    double DLHj2=0;
    double DLHj3=0;
    double DLHj4=92;
    double DLHj5=0;
    double DLHj6=0;
    double DLHj7=0;

    //global ta jolo
    double DRHj1=226;
    double DRHj2=0;
    double DRHj3=0;
    double DRHj4=92;
    double DRHj5=0;
    double DRHj6=0;
    double DRHj7=0;

    //global ta aghab
    double DLHa1=-314;
    double DLHa2=0;
    double DLHa3=0;
    double DLHa4=-137;
    double DLHa5=0;
    double DLHa6=0;
    double DLHa7=0;

    //global ta jolo
    double DRHa1=-314;
    double DRHa2=0;
    double DRHa3=0;
    double DRHa4=-137;
    double DRHa5=0;
    double DRHa6=0;
    double DRHa7=0;

    double AsXr;
    double BsXr;
    double AsYr;
    double BsYr;
    double AsZr;
    double BsZr;
    double AeYr;
    double BeYr;
    double AwZr;
    double BwZr;
    double AwYr;
    double BwYr;
    double AwXr;
    double BwXr;

    double AsXl;
    double BsXl;
    double AsYl;
    double BsYl;
    double AsZl;
    double BsZl;
    double AeYl;
    double BeYl;
    double AwZl;
    double BwZl;
    double AwYl;
    double BwYl;
    double AwXl;
    double BwXl;


    if ( gClinet->allMotors.size() != 0 )
    {
        ///left hand
        AsYl = (float)-180/2048;


        AsXl = (float)180/2048;
        BsXl = 905;

        AsZl = (float)-180/2048;
        BsZl = 2432;

        AeYl = (float)-180/2048;
        BeYl = 1356;

        AwZl = (float)-180/2048;
        BwZl = 1944;

        AwYl = (float)-180/2048;
        BwYl = 1291;

        AwXl = (float)-180/2048;
        BwXl = 2011;


        ////
        ///right hand
        AsYr = (float)-180/2048;
        BsYr = 1947;

        AsXr = (float)180/2048;
        BsXr = 3400;

        AsZr = (float)-180/2048;
        BsZr = 1540;

        AeYr = (float)-180/2048;
        BeYr = 2320;

        AwZr = (float)-180/2048;
        BwZr = 2544;

        AwYr = (float)-180/2048;
        BwYr = 1107;

        AwXr = (float)-180/2048;
        BwXr = 2832;



    }
    else
        std::cout<<"nomotors"<<std::endl;


    std::cout<<"1"<<std::endl;
    double i;

    for (i=0;i<2;i++){

        //refrence realtime position
        upperbodyclient::motor_data PRH1 = gClinet->getMotor_rightShoulderYaw();
        upperbodyclient::motor_data PRH2 = gClinet->getMotor_rightShoulderPitch();
        upperbodyclient::motor_data PRH3 = gClinet->getMotor_rightShoulderRoll();
        upperbodyclient::motor_data PRH4 = gClinet->getMotor_rightElbowPitch();
        upperbodyclient::motor_data PRH5 = gClinet->getMotor_rightElbowRoll();
        upperbodyclient::motor_data PRH6 = gClinet->getMotor_rightWristPitch();
        upperbodyclient::motor_data PRH7 = gClinet->getMotor_rightWristRoll();

        upperbodyclient::motor_data PLH1 = gClinet->getMotor_leftShoulderYaw();
        upperbodyclient::motor_data PLH2 = gClinet->getMotor_leftShoulderPitch();
        upperbodyclient::motor_data PLH3 = gClinet->getMotor_leftShoulderRoll();
        upperbodyclient::motor_data PLH4 = gClinet->getMotor_leftElbowPitch();
        upperbodyclient::motor_data PLH5 = gClinet->getMotor_leftElbowRoll();
        upperbodyclient::motor_data PLH6 = gClinet->getMotor_leftWristPitch();
        upperbodyclient::motor_data PLH7 = gClinet->getMotor_leftWristRoll();


        // Right hand //mire aghab

        double Qs_Ysend=RH1+a*DRHa1;
        int speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

        double Qs_Xsend=RH2+a*DRHa2;
        int speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

        double Qs_Zsend=RH3+a*DRHa3;
        int speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

        double Qe_Ysend=RH4+a*DRHa4;
        int speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

        double Qw_Zsend=RH5+a*DRHa5;
        int speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

        double Qw_Ysend=RH6+a*DRHa6;
        int speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

        double Qw_Xsend=RH7+a*DRHa7;
        int speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);


        // Left hand //mire jolo

        Qs_Ysend=LH1+a*DLHj1;
        speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

        Qs_Xsend=LH2+a*DLHj2;
        speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

        Qs_Zsend=LH3+a*DLHj3;
        speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

        Qe_Ysend=LH4+a*DLHj4;
        speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

        Qw_Zsend=LH5+a*DLHj5;
        speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

        Qw_Ysend=LH6+a*DLHj6;
        speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

        Qw_Xsend=LH7+a*DLHj7;
        speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);

        boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000/2));

        std::cout<<"3"<<std::endl;

        //Right hand //mire jolo
        Qs_Ysend=RH1+a*DRHj1;
        gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

        Qs_Xsend=RH2+a*DRHj2;
        gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

        Qs_Zsend=RH3+a*DRHj3;
        gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

        Qe_Ysend=RH4+a*DRHj4;
        gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

        Qw_Zsend=RH5+a*DRHj5;
        gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

        Qw_Ysend=RH6+a*DRHj6;
        gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

        Qw_Xsend=RH7+a*DRHj7;
        gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

        //Left hand //mire aghab
        Qs_Ysend=LH1+a*DLHa1;
        gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

        Qs_Xsend=LH2+a*DLHa2;
        gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

        Qs_Zsend=LH3+a*DLHa3;
        gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

        Qe_Ysend=LH4+a*DLHa4;
        gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

        Qw_Zsend=LH5+a*DLHa5;
        gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

        Qw_Ysend=LH6+a*DLHa6;
        gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

        Qw_Xsend=LH7+a*DLHa7;
        gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);
        std::cout<<"end of cycle"<<std::endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000/2));
    }


    // Right hand //mire aghab
    //refrence realtime position
    upperbodyclient::motor_data PRH1 = gClinet->getMotor_rightShoulderYaw();
    upperbodyclient::motor_data PRH2 = gClinet->getMotor_rightShoulderPitch();
    upperbodyclient::motor_data PRH3 = gClinet->getMotor_rightShoulderRoll();
    upperbodyclient::motor_data PRH4 = gClinet->getMotor_rightElbowPitch();
    upperbodyclient::motor_data PRH5 = gClinet->getMotor_rightElbowRoll();
    upperbodyclient::motor_data PRH6 = gClinet->getMotor_rightWristPitch();
    upperbodyclient::motor_data PRH7 = gClinet->getMotor_rightWristRoll();

    upperbodyclient::motor_data PLH1 = gClinet->getMotor_leftShoulderYaw();
    upperbodyclient::motor_data PLH2 = gClinet->getMotor_leftShoulderPitch();
    upperbodyclient::motor_data PLH3 = gClinet->getMotor_leftShoulderRoll();
    upperbodyclient::motor_data PLH4 = gClinet->getMotor_leftElbowPitch();
    upperbodyclient::motor_data PLH5 = gClinet->getMotor_leftElbowRoll();
    upperbodyclient::motor_data PLH6 = gClinet->getMotor_leftWristPitch();
    upperbodyclient::motor_data PLH7 = gClinet->getMotor_leftWristRoll();



    double Qs_Ysend=RH1+a*DRHa1;
    int speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    double Qs_Xsend=RH2+a*DRHa2;
    int speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    double Qs_Zsend=RH3+a*DRHa3;
    int speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    double Qe_Ysend=RH4+a*DRHa4;
    int speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    double Qw_Zsend=RH5+a*DRHa5;
    int speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    double Qw_Ysend=RH6+a*DRHa6;
    int speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    double Qw_Xsend=RH7+a*DRHa7;
    int speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time/2))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);


    // Left hand //mire jolo

    Qs_Ysend=LH1+a*DLHj1;
    speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2+a*DLHj2;
    speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3+a*DLHj3;
    speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4+a*DLHj4;
    speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5+a*DLHj5;
    speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6+a*DLHj6;
    speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7+a*DLHj7;
    speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time/2))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000/2));


}

int sitting()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    double Time=4;
    double Time2=4;
    //refrens init start
    double RH1=1947;
    double RH2=3400;
    double RH3=1540;
    double RH4=2320;
    double RH5=2544;
    double RH6=1107;
    double RH7=2832;

    //refrens init start
    double LH1=1380;
    double LH2=905;
    double LH3=2432;
    double LH4=1356;
    double LH5=1944;
    double LH6=1291;
    double LH7=2011;

    //init ta baste
    double DLH1=277;
    double DLH2=0;
    double DLH3=0;
    double DLH4=1244;
    double DLH5=0;
    double DLH6=0;
    double DLH7=0;

    //init ta baste
    double DRH1=277;
    double DRH2=0;
    double DRH3=0;
    double DRH4=1244;
    double DRH5=0;
    double DRH6=0;
    double DRH7=0;


    double WP=900;
    double WL=380;
    double WR=1800;

    double AsXr;
    double BsXr;
    double AsYr;
    double BsYr;
    double AsZr;
    double BsZr;
    double AeYr;
    double BeYr;
    double AwZr;
    double BwZr;
    double AwYr;
    double BwYr;
    double AwXr;
    double BwXr;

    double AsXl;
    double BsXl;
    double AsYl;
    double BsYl;
    double AsZl;
    double BsZl;
    double AeYl;
    double BeYl;
    double AwZl;
    double BwZl;
    double AwYl;
    double BwYl;
    double AwXl;
    double BwXl;


    // if (left) {
    if ( gClinet->allMotors.size() != 0 )
    {
        ///left hand
        AsYl = (float)-180/2048;


        AsXl = (float)180/2048;
        BsXl = 905;

        AsZl = (float)-180/2048;
        BsZl = 2432;

        AeYl = (float)-180/2048;
        BeYl = 1356;

        AwZl = (float)-180/2048;
        BwZl = 1944;

        AwYl = (float)-180/2048;
        BwYl = 1291;

        AwXl = (float)-180/2048;
        BwXl = 2011;


        ////
        ///right hand
        AsYr = (float)-180/2048;
        BsYr = 1947;

        AsXr = (float)180/2048;
        BsXr = 3400;

        AsZr = (float)-180/2048;
        BsZr = 1540;

        AeYr = (float)-180/2048;
        BeYr = 2320;

        AwZr = (float)-180/2048;
        BwZr = 2544;

        AwYr = (float)-180/2048;
        BwYr = 1107;

        AwXr = (float)-180/2048;
        BwXr = 2832;



    }
    else
        std::cout<<"nomotors"<<std::endl;

    //Right hand //jam mishe

    double  Qs_Ysend=RH1+DRH1;
    int speeds_Y=((fabs(Qs_Ysend-RH1)*fabs(AsYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    double  Qs_Xsend=RH2+DRH2;
    int speeds_X=((fabs(Qs_Xsend-RH2)*fabs(AsXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    double  Qs_Zsend=RH3+DRH3;
    int speeds_Z=((fabs(Qs_Zsend-RH3)*fabs(AsZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    double  Qe_Ysend=RH4+DRH4;
    int speede_Y=((fabs(Qe_Ysend-RH4)*fabs(AeYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    double   Qw_Zsend=RH5+DRH5;
    int speedw_Z=((fabs(Qw_Zsend-RH5)*fabs(AwZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    double   Qw_Ysend=RH6+DRH6;
    int speedw_Y=((fabs(Qw_Ysend-RH6)*fabs(AwYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    double  Qw_Xsend=RH7+DRH7;
    int speedw_X=((fabs(Qw_Xsend-RH7)*fabs(AwXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    //Left hand //jam mishe

    Qs_Ysend=LH1+DLH1;
    speeds_Y=((fabs(Qs_Ysend-LH1)*fabs(AsYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2+DLH2;
    speeds_X=((fabs(Qs_Xsend-LH2)*fabs(AsXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3+DLH3;
    speeds_Z=((fabs(Qs_Zsend-LH3)*fabs(AsZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4+DLH4;
    speede_Y=((fabs(Qe_Ysend-LH4)*fabs(AeYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5+DLH5;
    speedw_Z=((fabs(Qw_Zsend-LH5)*fabs(AwZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6+DLH6;
    speedw_Y=((fabs(Qw_Ysend-LH6)*fabs(AwYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7+DLH7;
    speedw_X=((fabs(Qw_Xsend-LH7)*fabs(AwXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
    double QWaist_send=WL;
    speedw_X=((fabs(QWaist_send-WP)*fabs(AwXl)/6)/(1))/0.01;

    cout<<QWaist_send<<","<<speedw_X<<endl;

    gClinet->setMotor_waist(QWaist_send,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    QWaist_send=WR;
    speedw_X=((fabs(QWaist_send-WL)*fabs(AwXl)/6)/(2))/0.01;
    gClinet->setMotor_waist(QWaist_send,speedw_X);
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    QWaist_send=WP;
    speedw_X=((fabs(QWaist_send-WR)*fabs(AwXl)/6)/(1))/0.01;
    gClinet->setMotor_waist(QWaist_send,speedw_X);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    //refrence realtime position
    upperbodyclient::motor_data PRH1 = gClinet->getMotor_rightShoulderYaw();
    upperbodyclient::motor_data PRH2 = gClinet->getMotor_rightShoulderPitch();
    upperbodyclient::motor_data PRH3 = gClinet->getMotor_rightShoulderRoll();
    upperbodyclient::motor_data PRH4 = gClinet->getMotor_rightElbowPitch();
    upperbodyclient::motor_data PRH5 = gClinet->getMotor_rightElbowRoll();
    upperbodyclient::motor_data PRH6 = gClinet->getMotor_rightWristPitch();
    upperbodyclient::motor_data PRH7 = gClinet->getMotor_rightWristRoll();

    upperbodyclient::motor_data PLH1 = gClinet->getMotor_leftShoulderYaw();
    upperbodyclient::motor_data PLH2 = gClinet->getMotor_leftShoulderPitch();
    upperbodyclient::motor_data PLH3 = gClinet->getMotor_leftShoulderRoll();
    upperbodyclient::motor_data PLH4 = gClinet->getMotor_leftElbowPitch();
    upperbodyclient::motor_data PLH5 = gClinet->getMotor_leftElbowRoll();
    upperbodyclient::motor_data PLH6 = gClinet->getMotor_leftWristPitch();
    upperbodyclient::motor_data PLH7 = gClinet->getMotor_leftWristRoll();



    Qs_Ysend=LH1;
    speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2;
    speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3;
    speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4;
    speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5;
    speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6;
    speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7;
    speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);


    Qs_Ysend=RH1;
    speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=RH2;
    speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=RH3;
    speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=RH4;
    speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=RH5;
    speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=RH6;
    speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=RH7;
    speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    std::cout<<"motor sent commands are="<<std::endl<<Qs_Ysend<<std::endl<<Qs_Xsend<<std::endl<<Qs_Zsend<<std::endl<<Qe_Ysend<<std::endl<<Qw_Zsend<<std::endl<<Qw_Ysend<<std::endl<<Qw_Xsend<<std::endl<<std::endl ;
}

int compound()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    double Time=2;
    double Time2=2.2;
    double Time3=1.8;
    double Time4=2;
    //refrens init start
    double RH1=1947;
    double RH2=3400;
    double RH3=1540;
    double RH4=2320;
    double RH5=2544;
    double RH6=1107;
    double RH7=2832;

    //refrens init start
    double LH1=1380;
    double LH2=905;
    double LH3=2432;
    double LH4=1356;
    double LH5=1944;
    double LH6=1291;
    double LH7=2011;


    //step1-init ta baste
    double FLH1=277;
    double FLH2=0;
    double FLH3=0;
    double FLH4=1244;
    double FLH5=0;
    double FLH6=0;
    double FLH7=0;

    //step1-init ta baste
    double FRH1=277;
    double FRH2=0;
    double FRH3=0;
    double FRH4=1244;
    double FRH5=0;
    double FRH6=0;
    double FRH7=0;

    //step2-init ta baz
    double SLH1=490;
    double SLH2=545;
    double SLH3=-407;
    double SLH4=1159;
    double SLH5=0;
    double SLH6=0;
    double SLH7=0;

    //step2-init ta baz
    double SRH1=490;
    double SRH2=-545;
    double SRH3=-407;
    double SRH4=1159;
    double SRH5=0;
    double SRH6=0;
    double SRH7=0;

    double WP=900;
    double WL=380;
    double WR=1800;

    double AsXr;
    double BsXr;
    double AsYr;
    double BsYr;
    double AsZr;
    double BsZr;
    double AeYr;
    double BeYr;
    double AwZr;
    double BwZr;
    double AwYr;
    double BwYr;
    double AwXr;
    double BwXr;

    double AsXl;
    double BsXl;
    double AsYl;
    double BsYl;
    double AsZl;
    double BsZl;
    double AeYl;
    double BeYl;
    double AwZl;
    double BwZl;
    double AwYl;
    double BwYl;
    double AwXl;
    double BwXl;


    // if (left) {
    if ( gClinet->allMotors.size() != 0 )
    {
        ///left hand
        AsYl = (float)-180/2048;


        AsXl = (float)180/2048;
        BsXl = 905;

        AsZl = (float)-180/2048;
        BsZl = 2432;

        AeYl = (float)-180/2048;
        BeYl = 1356;

        AwZl = (float)-180/2048;
        BwZl = 1944;

        AwYl = (float)-180/2048;
        BwYl = 1291;

        AwXl = (float)-180/2048;
        BwXl = 2011;


        ////
        ///right hand
        AsYr = (float)-180/2048;
        BsYr = 1947;

        AsXr = (float)180/2048;
        BsXr = 3400;

        AsZr = (float)-180/2048;
        BsZr = 1540;

        AeYr = (float)-180/2048;
        BeYr = 2320;

        AwZr = (float)-180/2048;
        BwZr = 2544;

        AwYr = (float)-180/2048;
        BwYr = 1107;

        AwXr = (float)-180/2048;
        BwXr = 2832;



    }
    else
        std::cout<<"nomotors"<<std::endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    //First Step

    //Right hand

    double  Qs_Ysend=RH1+FRH1;
    int speeds_Y=((fabs(Qs_Ysend-RH1)*fabs(AsYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    double  Qs_Xsend=RH2+FRH2;
    int speeds_X=((fabs(Qs_Xsend-RH2)*fabs(AsXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    double  Qs_Zsend=RH3+FRH3;
    int speeds_Z=((fabs(Qs_Zsend-RH3)*fabs(AsZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    double  Qe_Ysend=RH4+FRH4;
    int speede_Y=((fabs(Qe_Ysend-RH4)*fabs(AeYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    double   Qw_Zsend=RH5+FRH5;
    int speedw_Z=((fabs(Qw_Zsend-RH5)*fabs(AwZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    double   Qw_Ysend=RH6+FRH6;
    int speedw_Y=((fabs(Qw_Ysend-RH6)*fabs(AwYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    double  Qw_Xsend=RH7+FRH7;
    int speedw_X=((fabs(Qw_Xsend-RH7)*fabs(AwXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    //Left hand

    Qs_Ysend=LH1+FLH1;
    speeds_Y=((fabs(Qs_Ysend-LH1)*fabs(AsYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2+FLH2;
    speeds_X=((fabs(Qs_Xsend-LH2)*fabs(AsXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3+FLH3;
    speeds_Z=((fabs(Qs_Zsend-LH3)*fabs(AsZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4+FLH4;
    speede_Y=((fabs(Qe_Ysend-LH4)*fabs(AeYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5+FLH5;
    speedw_Z=((fabs(Qw_Zsend-LH5)*fabs(AwZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6+FLH6;
    speedw_Y=((fabs(Qw_Ysend-LH6)*fabs(AwYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7+FLH7;
    speedw_X=((fabs(Qw_Xsend-LH7)*fabs(AwXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000));

    boost::this_thread::sleep(boost::posix_time::milliseconds(1700));

    upperbodyclient::motor_data PRH1 = gClinet->getMotor_rightShoulderYaw();
    upperbodyclient::motor_data PRH2 = gClinet->getMotor_rightShoulderPitch();
    upperbodyclient::motor_data PRH3 = gClinet->getMotor_rightShoulderRoll();
    upperbodyclient::motor_data PRH4 = gClinet->getMotor_rightElbowPitch();
    upperbodyclient::motor_data PRH5 = gClinet->getMotor_rightElbowRoll();
    upperbodyclient::motor_data PRH6 = gClinet->getMotor_rightWristPitch();
    upperbodyclient::motor_data PRH7 = gClinet->getMotor_rightWristRoll();

    upperbodyclient::motor_data PLH1 = gClinet->getMotor_leftShoulderYaw();
    upperbodyclient::motor_data PLH2 = gClinet->getMotor_leftShoulderPitch();
    upperbodyclient::motor_data PLH3 = gClinet->getMotor_leftShoulderRoll();
    upperbodyclient::motor_data PLH4 = gClinet->getMotor_leftElbowPitch();
    upperbodyclient::motor_data PLH5 = gClinet->getMotor_leftElbowRoll();
    upperbodyclient::motor_data PLH6 = gClinet->getMotor_leftWristPitch();
    upperbodyclient::motor_data PLH7 = gClinet->getMotor_leftWristRoll();

    //// Second Step

    Qs_Ysend=LH1+SLH1;
    speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2+SLH2;
    speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3+SLH3;
    speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4+SLH4;
    speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5+SLH5;
    speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6+SLH6;
    speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7+SLH7;
    speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);


    Qs_Ysend=RH1+SRH1;
    speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=RH2+SRH2;
    speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=RH3+SRH3;
    speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=RH4+SRH4;
    speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=RH5+SRH5;
    speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=RH6+SRH6;
    speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=RH7+SRH7;
    speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(Time2*1000));
    boost::this_thread::sleep(boost::posix_time::milliseconds(3200));

    PRH1 = gClinet->getMotor_rightShoulderYaw();
    PRH2 = gClinet->getMotor_rightShoulderPitch();
    PRH3 = gClinet->getMotor_rightShoulderRoll();
    PRH4 = gClinet->getMotor_rightElbowPitch();
    PRH5 = gClinet->getMotor_rightElbowRoll();
    PRH6 = gClinet->getMotor_rightWristPitch();
    PRH7 = gClinet->getMotor_rightWristRoll();

    PLH1 = gClinet->getMotor_leftShoulderYaw();
    PLH2 = gClinet->getMotor_leftShoulderPitch();
    PLH3 = gClinet->getMotor_leftShoulderRoll();
    PLH4 = gClinet->getMotor_leftElbowPitch();
    PLH5 = gClinet->getMotor_leftElbowRoll();
    PLH6 = gClinet->getMotor_leftWristPitch();
    PLH7 = gClinet->getMotor_leftWristRoll();



    //// Third Step

    double QWaist_send=WL;
   int speedW=((fabs(QWaist_send-WP)*fabs(AwXl)/6)/(Time3))/0.01;
    gClinet->setMotor_waist(QWaist_send,speedW);




    boost::this_thread::sleep(boost::posix_time::milliseconds(Time3*1000));

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));


    PRH1 = gClinet->getMotor_rightShoulderYaw();
    PRH2 = gClinet->getMotor_rightShoulderPitch();
    PRH3 = gClinet->getMotor_rightShoulderRoll();
    PRH4 = gClinet->getMotor_rightElbowPitch();
    PRH5 = gClinet->getMotor_rightElbowRoll();
    PRH6 = gClinet->getMotor_rightWristPitch();
    PRH7 = gClinet->getMotor_rightWristRoll();

    PLH1 = gClinet->getMotor_leftShoulderYaw();
    PLH2 = gClinet->getMotor_leftShoulderPitch();
    PLH3 = gClinet->getMotor_leftShoulderRoll();
    PLH4 = gClinet->getMotor_leftElbowPitch();
    PLH5 = gClinet->getMotor_leftElbowRoll();
    PLH6 = gClinet->getMotor_leftWristPitch();
    PLH7 = gClinet->getMotor_leftWristRoll();


    //// Forth Step




    //Right hand

     Qs_Ysend=RH1+FRH1;
     speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

     Qs_Xsend=RH2+FRH2;
     speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

     Qs_Zsend=RH3+FRH3;
     speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

     Qe_Ysend=RH4+FRH4;
     speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

      Qw_Zsend=RH5+FRH5;
     speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

      Qw_Ysend=RH6+FRH6;
     speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

     Qw_Xsend=RH7+FRH7;
     speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    //Left hand

    Qs_Ysend=LH1+FLH1;
    speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2+FLH2;
    speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3+FLH3;
    speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4+FLH4;
    speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5+FLH5;
    speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6+FLH6;
    speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7+FLH7;
    speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);
    QWaist_send=WP;
    speedW=((fabs( QWaist_send-WL)*fabs(AwXl)/6)/(Time4))/0.01;
    gClinet->setMotor_waist(QWaist_send,speedW);

   boost::this_thread::sleep(boost::posix_time::milliseconds(Time4*1000));
////
    Qs_Ysend=LH1;
    speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2;
    speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3;
    speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4;
    speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5;
    speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6;
    speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7;
    speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);


    Qs_Ysend=RH1;
    speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=RH2;
    speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=RH3;
    speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=RH4;
    speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=RH5;
    speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=RH6;
    speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=RH7;
    speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);


    boost::this_thread::sleep(boost::posix_time::milliseconds(Time4*1000));
}

int foot_forward_body_backward()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));



    double Time=5;


    //refrens init start
    double RH1=1947;
    double RH2=3400;
    double RH3=1540;
    double RH4=2320;
    double RH5=2544;
    double RH6=1107;
    double RH7=2832;

    //refrens init start
    double LH1=1380;
    double LH2=905;
    double LH3=2432;
    double LH4=1356;
    double LH5=1944;
    double LH6=1291;
    double LH7=2011;

    //step1-init ta baste
    double FLH1=277;
    double FLH2=0;
    double FLH3=0;
    double FLH4=1244;
    double FLH5=0;
    double FLH6=0;
    double FLH7=0;

    //step1-init ta baste
    double FRH1=277;
    double FRH2=0;
    double FRH3=0;
    double FRH4=1244;
    double FRH5=0;
    double FRH6=0;
    double FRH7=0;



    double AsXr;
    double BsXr;
    double AsYr;
    double BsYr;
    double AsZr;
    double BsZr;
    double AeYr;
    double BeYr;
    double AwZr;
    double BwZr;
    double AwYr;
    double BwYr;
    double AwXr;
    double BwXr;

    double AsXl;
    double BsXl;
    double AsYl;
    double BsYl;
    double AsZl;
    double BsZl;
    double AeYl;
    double BeYl;
    double AwZl;
    double BwZl;
    double AwYl;
    double BwYl;
    double AwXl;
    double BwXl;


    if ( gClinet->allMotors.size() != 0 )
    {
        ///left hand
        AsYl = (float)-180/2048;


        AsXl = (float)180/2048;
        BsXl = 905;

        AsZl = (float)-180/2048;
        BsZl = 2432;

        AeYl = (float)-180/2048;
        BeYl = 1356;

        AwZl = (float)-180/2048;
        BwZl = 1944;

        AwYl = (float)-180/2048;
        BwYl = 1291;

        AwXl = (float)-180/2048;
        BwXl = 2011;


        ////
        ///right hand
        AsYr = (float)-180/2048;
        BsYr = 1947;

        AsXr = (float)180/2048;
        BsXr = 3400;

        AsZr = (float)-180/2048;
        BsZr = 1540;

        AeYr = (float)-180/2048;
        BeYr = 2320;

        AwZr = (float)-180/2048;
        BwZr = 2544;

        AwYr = (float)-180/2048;
        BwYr = 1107;

        AwXr = (float)-180/2048;
        BwXr = 2832;



    }
    else
        std::cout<<"nomotors"<<std::endl;


    double Qs_Ysend=LH1+FLH1;
    int speeds_Y=((fabs(Qs_Ysend-LH1)*fabs(AsYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    double Qs_Xsend=LH2+FLH2;
    int speeds_X=((fabs(Qs_Xsend-LH2)*fabs(AsXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    double Qs_Zsend=LH3+FLH3;
    int speeds_Z=((fabs(Qs_Zsend-LH3)*fabs(AsZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    double Qe_Ysend=LH4+FLH4;
    int speede_Y=((fabs(Qe_Ysend-LH4)*fabs(AeYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    double  Qw_Zsend=LH5+FLH5;
    int speedw_Z=((fabs(Qw_Zsend-LH5)*fabs(AwZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    double Qw_Ysend=LH6+FLH6;
    int speedw_Y=((fabs(Qw_Ysend-LH6)*fabs(AwYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    double  Qw_Xsend=LH7+FLH7;
    int speedw_X=((fabs(Qw_Xsend-LH7)*fabs(AwXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);
    //right hand

    Qs_Ysend=RH1+FRH1;
    speeds_Y=((fabs(Qs_Ysend-RH1)*fabs(AsYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=RH2+FRH2;
    speeds_X=((fabs(Qs_Xsend-RH2)*fabs(AsXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=RH3+FRH3;
    speeds_Z=((fabs(Qs_Zsend-RH3)*fabs(AsZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=RH4+FRH4;
    speede_Y=((fabs(Qe_Ysend-RH4)*fabs(AeYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=RH5+FRH5;
    speedw_Z=((fabs(Qw_Zsend-RH5)*fabs(AwZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=RH6+FRH6;
    speedw_Y=((fabs(Qw_Ysend-RH6)*fabs(AwYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=RH7+FRH7;
    speedw_X=((fabs(Qw_Xsend-RH7)*fabs(AwXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000));

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    upperbodyclient::motor_data PRH1 = gClinet->getMotor_rightShoulderYaw();
    upperbodyclient::motor_data PRH2 = gClinet->getMotor_rightShoulderPitch();
    upperbodyclient::motor_data PRH3 = gClinet->getMotor_rightShoulderRoll();
    upperbodyclient::motor_data PRH4 = gClinet->getMotor_rightElbowPitch();
    upperbodyclient::motor_data PRH5 = gClinet->getMotor_rightElbowRoll();
    upperbodyclient::motor_data PRH6 = gClinet->getMotor_rightWristPitch();
    upperbodyclient::motor_data PRH7 = gClinet->getMotor_rightWristRoll();

    upperbodyclient::motor_data PLH1 = gClinet->getMotor_leftShoulderYaw();
    upperbodyclient::motor_data PLH2 = gClinet->getMotor_leftShoulderPitch();
    upperbodyclient::motor_data PLH3 = gClinet->getMotor_leftShoulderRoll();
    upperbodyclient::motor_data PLH4 = gClinet->getMotor_leftElbowPitch();
    upperbodyclient::motor_data PLH5 = gClinet->getMotor_leftElbowRoll();
    upperbodyclient::motor_data PLH6 = gClinet->getMotor_leftWristPitch();
    upperbodyclient::motor_data PLH7 = gClinet->getMotor_leftWristRoll();

    Qs_Ysend=LH1;
    speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2;
    speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3;
    speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4;
    speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5;
    speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6;
    speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7;
    speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);


    Qs_Ysend=RH1;
    speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=RH2;
    speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=RH3;
    speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=RH4;
    speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=RH5;
    speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=RH6;
    speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=RH7;
    speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000));
}

int shooting()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1300));

    double Time=2;
    double Time2=4;
    double Time3=1;
    double Time4=2;
    double Time5=2;
    //refrens init start
    double IRH1=1947;
    double IRH2=3400;
    double IRH3=1540;
    double IRH4=2320;
    double IRH5=2544;
    double IRH6=1107;
    double IRH7=2832;

    //refrens init start
    double ILH1=1380;
    double ILH2=905;
    double ILH3=2432;
    double ILH4=1356;
    double ILH5=1944;
    double ILH6=1291;
    double ILH7=2011;


    //refrens global start
    double RH1=1991;
    double RH2=3400;
    double RH3=1540;
    double RH4=2560;
    double RH5=2544;
    double RH6=1107;
    double RH7=2832;

    //refrens global start
    double LH1=1424;
    double LH2=905;
    double LH3=2432;
    double LH4=1596;
    double LH5=1944;
    double LH6=1291;
    double LH7=2011;

    // step2
    double SL1=226;
    double SL2=0;
    double SL3=0;
    double SL4=92;
    double SL5=0;
    double SL6=0;
    double SL7=0;

    //step2
    double SR1=-314;
    double SR2=0;
    double SR3=0;
    double SR4=-137;
    double SR5=0;
    double SR6=0;
    double SR7=0;

    // step3
    double TL1=-314;
    double TL2=0;
    double TL3=0;
    double TL4=-137;
    double TL5=0;
    double TL6=0;
    double TL7=0;

    //step3
    double TR1=226;
    double TR2=0;
    double TR3=0;
    double TR4=92;
    double TR5=0;
    double TR6=0;
    double TR7=0;


    double AsXr;
    double BsXr;
    double AsYr;
    double BsYr;
    double AsZr;
    double BsZr;
    double AeYr;
    double BeYr;
    double AwZr;
    double BwZr;
    double AwYr;
    double BwYr;
    double AwXr;
    double BwXr;

    double AsXl;
    double BsXl;
    double AsYl;
    double BsYl;
    double AsZl;
    double BsZl;
    double AeYl;
    double BeYl;
    double AwZl;
    double BwZl;
    double AwYl;
    double BwYl;
    double AwXl;
    double BwXl;


    if ( gClinet->allMotors.size() != 0 )
    {
        ///left hand
        AsYl = (float)-180/2048;


        AsXl = (float)180/2048;
        BsXl = 905;

        AsZl = (float)-180/2048;
        BsZl = 2432;

        AeYl = (float)-180/2048;
        BeYl = 1356;

        AwZl = (float)-180/2048;
        BwZl = 1944;

        AwYl = (float)-180/2048;
        BwYl = 1291;

        AwXl = (float)-180/2048;
        BwXl = 2011;


        ////
        ///right hand
        AsYr = (float)-180/2048;
        BsYr = 1947;

        AsXr = (float)180/2048;
        BsXr = 3400;

        AsZr = (float)-180/2048;
        BsZr = 1540;

        AeYr = (float)-180/2048;
        BeYr = 2320;

        AwZr = (float)-180/2048;
        BwZr = 2544;

        AwYr = (float)-180/2048;
        BwYr = 1107;

        AwXr = (float)-180/2048;
        BwXr = 2832;



    }
    else
        std::cout<<"nomotors"<<std::endl;

    ///first step-global start
    //Right hand

    double  Qs_Ysend=RH1;
    int speeds_Y=((fabs(Qs_Ysend-IRH1)*fabs(AsYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    double  Qs_Xsend=RH2;
    int speeds_X=((fabs(Qs_Xsend-IRH2)*fabs(AsXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    double  Qs_Zsend=RH3;
    int speeds_Z=((fabs(Qs_Zsend-IRH3)*fabs(AsZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    double  Qe_Ysend=RH4;
    int speede_Y=((fabs(Qe_Ysend-IRH4)*fabs(AeYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    double   Qw_Zsend=RH5;
    int speedw_Z=((fabs(Qw_Zsend-IRH5)*fabs(AwZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    double   Qw_Ysend=RH6;
    int speedw_Y=((fabs(Qw_Ysend-IRH6)*fabs(AwYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    double  Qw_Xsend=RH7;
    int speedw_X=((fabs(Qw_Xsend-IRH7)*fabs(AwXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    //Left hand

    Qs_Ysend=LH1;
    speeds_Y=((fabs(Qs_Ysend-ILH1)*fabs(AsYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2;
    speeds_X=((fabs(Qs_Xsend-ILH2)*fabs(AsXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3;
    speeds_Z=((fabs(Qs_Zsend-ILH3)*fabs(AsZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4;
    speede_Y=((fabs(Qe_Ysend-ILH4)*fabs(AeYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5;
    speedw_Z=((fabs(Qw_Zsend-ILH5)*fabs(AwZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6;
    speedw_Y=((fabs(Qw_Ysend-ILH6)*fabs(AwYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7;
    speedw_X=((fabs(Qw_Xsend-ILH7)*fabs(AwXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);
    boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000));
    boost::this_thread::sleep(boost::posix_time::milliseconds(10000));

    ///second step-hands1

    upperbodyclient::motor_data PRH1 = gClinet->getMotor_rightShoulderYaw();
    upperbodyclient::motor_data PRH2 = gClinet->getMotor_rightShoulderPitch();
    upperbodyclient::motor_data PRH3 = gClinet->getMotor_rightShoulderRoll();
    upperbodyclient::motor_data PRH4 = gClinet->getMotor_rightElbowPitch();
    upperbodyclient::motor_data PRH5 = gClinet->getMotor_rightElbowRoll();
    upperbodyclient::motor_data PRH6 = gClinet->getMotor_rightWristPitch();
    upperbodyclient::motor_data PRH7 = gClinet->getMotor_rightWristRoll();

    upperbodyclient::motor_data PLH1 = gClinet->getMotor_leftShoulderYaw();
    upperbodyclient::motor_data PLH2 = gClinet->getMotor_leftShoulderPitch();
    upperbodyclient::motor_data PLH3 = gClinet->getMotor_leftShoulderRoll();
    upperbodyclient::motor_data PLH4 = gClinet->getMotor_leftElbowPitch();
    upperbodyclient::motor_data PLH5 = gClinet->getMotor_leftElbowRoll();
    upperbodyclient::motor_data PLH6 = gClinet->getMotor_leftWristPitch();
    upperbodyclient::motor_data PLH7 = gClinet->getMotor_leftWristRoll();


    Qs_Ysend=LH1+SL1;
    speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2+SL2;
    speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3+SL3;
    speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4+SL4;
    speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5+SL5;
    speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6+SL6;
    speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7+SL7;
    speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time2))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);


    Qs_Ysend=RH1+SR1;
    speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=RH2+SR2;
    speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=RH3+SR3;
    speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=RH4+SR4;
    speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=RH5+SR5;
    speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=RH6+SR6;
    speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=RH7+SR7;
    speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time2))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);


    boost::this_thread::sleep(boost::posix_time::milliseconds(Time2*1000));
    boost::this_thread::sleep(boost::posix_time::milliseconds(400));



    PRH1 = gClinet->getMotor_rightShoulderYaw();
    PRH2 = gClinet->getMotor_rightShoulderPitch();
    PRH3 = gClinet->getMotor_rightShoulderRoll();
    PRH4 = gClinet->getMotor_rightElbowPitch();
    PRH5 = gClinet->getMotor_rightElbowRoll();
    PRH6 = gClinet->getMotor_rightWristPitch();
    PRH7 = gClinet->getMotor_rightWristRoll();

    PLH1 = gClinet->getMotor_leftShoulderYaw();
    PLH2 = gClinet->getMotor_leftShoulderPitch();
    PLH3 = gClinet->getMotor_leftShoulderRoll();
    PLH4 = gClinet->getMotor_leftElbowPitch();
    PLH5 = gClinet->getMotor_leftElbowRoll();
    PLH6 = gClinet->getMotor_leftWristPitch();
    PLH7 = gClinet->getMotor_leftWristRoll();



    Qs_Ysend=LH1+TL1;
    speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time3))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2+TL2;
    speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time3))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3+TL3;
    speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time3))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4+TL4;
    speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time3))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5+TL5;
    speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time3))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6+TL6;
    speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time3))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7+TL7;
    speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time3))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);


    Qs_Ysend=RH1+TR1;
    speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time3))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=RH2+TR2;
    speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time3))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=RH3+TR3;
    speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time3))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=RH4+TR4;
    speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time3))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=RH5+TR5;
    speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time3))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=RH6+TR6;
    speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time3))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=RH7+TR7;
    speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time3))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(Time3*1000));
    boost::this_thread::sleep(boost::posix_time::milliseconds(1600));

    PRH1 = gClinet->getMotor_rightShoulderYaw();
    PRH2 = gClinet->getMotor_rightShoulderPitch();
    PRH3 = gClinet->getMotor_rightShoulderRoll();
    PRH4 = gClinet->getMotor_rightElbowPitch();
    PRH5 = gClinet->getMotor_rightElbowRoll();
    PRH6 = gClinet->getMotor_rightWristPitch();
    PRH7 = gClinet->getMotor_rightWristRoll();

    PLH1 = gClinet->getMotor_leftShoulderYaw();
    PLH2 = gClinet->getMotor_leftShoulderPitch();
    PLH3 = gClinet->getMotor_leftShoulderRoll();
    PLH4 = gClinet->getMotor_leftElbowPitch();
    PLH5 = gClinet->getMotor_leftElbowRoll();
    PLH6 = gClinet->getMotor_leftWristPitch();
    PLH7 = gClinet->getMotor_leftWristRoll();


    Qs_Ysend=LH1;
    speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=LH2;
    speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=LH3;
    speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=LH4;
    speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=LH5;
    speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=LH6;
    speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=LH7;
    speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time4))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);


    Qs_Ysend=RH1;
    speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=RH2;
    speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=RH3;
    speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=RH4;
    speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=RH5;
    speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=RH6;
    speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=RH7;
    speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time4))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(Time4*1000));




    PRH1 = gClinet->getMotor_rightShoulderYaw();
    PRH2 = gClinet->getMotor_rightShoulderPitch();
    PRH3 = gClinet->getMotor_rightShoulderRoll();
    PRH4 = gClinet->getMotor_rightElbowPitch();
    PRH5 = gClinet->getMotor_rightElbowRoll();
    PRH6 = gClinet->getMotor_rightWristPitch();
    PRH7 = gClinet->getMotor_rightWristRoll();

    PLH1 = gClinet->getMotor_leftShoulderYaw();
    PLH2 = gClinet->getMotor_leftShoulderPitch();
    PLH3 = gClinet->getMotor_leftShoulderRoll();
    PLH4 = gClinet->getMotor_leftElbowPitch();
    PLH5 = gClinet->getMotor_leftElbowRoll();
    PLH6 = gClinet->getMotor_leftWristPitch();
    PLH7 = gClinet->getMotor_leftWristRoll();


    Qs_Ysend=ILH1;
    speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time5))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=ILH2;
    speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time5))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=ILH3;
    speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time5))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=ILH4;
    speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time5))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=ILH5;
    speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time5))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=ILH6;
    speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time5))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=ILH7;
    speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time5))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);


    Qs_Ysend=IRH1;
    speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time5))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=IRH2;
    speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time5))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=IRH3;
    speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time5))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=IRH4;
    speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time5))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=IRH5;
    speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time5))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=IRH6;
    speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time5))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=IRH7;
    speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time5))/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);
    boost::this_thread::sleep(boost::posix_time::milliseconds(Time5*1000));
}

int online()
{
    double Time=8.4;
    std::cout<<"Time="<<Time<<std::endl;
    double a=0.66;
    //refrence realtime position
    upperbodyclient::motor_data PRH1 = gClinet->getMotor_rightShoulderYaw();
    upperbodyclient::motor_data PRH2 = gClinet->getMotor_rightShoulderPitch();
    upperbodyclient::motor_data PRH3 = gClinet->getMotor_rightShoulderRoll();
    upperbodyclient::motor_data PRH4 = gClinet->getMotor_rightElbowPitch();
    upperbodyclient::motor_data PRH5 = gClinet->getMotor_rightElbowRoll();
    upperbodyclient::motor_data PRH6 = gClinet->getMotor_rightWristPitch();
    upperbodyclient::motor_data PRH7 = gClinet->getMotor_rightWristRoll();

    upperbodyclient::motor_data PLH1 = gClinet->getMotor_leftShoulderYaw();
    upperbodyclient::motor_data PLH2 = gClinet->getMotor_leftShoulderPitch();
    upperbodyclient::motor_data PLH3 = gClinet->getMotor_leftShoulderRoll();
    upperbodyclient::motor_data PLH4 = gClinet->getMotor_leftElbowPitch();
    upperbodyclient::motor_data PLH5 = gClinet->getMotor_leftElbowRoll();
    upperbodyclient::motor_data PLH6 = gClinet->getMotor_leftWristPitch();
    upperbodyclient::motor_data PLH7 = gClinet->getMotor_leftWristRoll();

    //refrens global start
    double RH1=1991;
    double RH2=3400;
    double RH3=1540;
    double RH4=2560;
    double RH5=2544;
    double RH6=1107;
    double RH7=2832;

    //refrens global start
    double LH1=1424;
    double LH2=905;
    double LH3=2432;
    double LH4=1596;
    double LH5=1944;
    double LH6=1291;
    double LH7=2011;

    //global ta jolo
    double DLHj1=226;
    double DLHj2=0;
    double DLHj3=0;
    double DLHj4=92;
    double DLHj5=0;
    double DLHj6=0;
    double DLHj7=0;

    //global ta jolo
    double DRHj1=226;
    double DRHj2=0;
    double DRHj3=0;
    double DRHj4=92;
    double DRHj5=0;
    double DRHj6=0;
    double DRHj7=0;

    //global ta aghab
    double DLHa1=-314;
    double DLHa2=0;
    double DLHa3=0;
    double DLHa4=-137;
    double DLHa5=0;
    double DLHa6=0;
    double DLHa7=0;

    //global ta jolo
    double DRHa1=-314;
    double DRHa2=0;
    double DRHa3=0;
    double DRHa4=-137;
    double DRHa5=0;
    double DRHa6=0;
    double DRHa7=0;


    double AsXr;
    double BsXr;
    double AsYr;
    double BsYr;
    double AsZr;
    double BsZr;
    double AeYr;
    double BeYr;
    double AwZr;
    double BwZr;
    double AwYr;
    double BwYr;
    double AwXr;
    double BwXr;

    double AsXl;
    double BsXl;
    double AsYl;
    double BsYl;
    double AsZl;
    double BsZl;
    double AeYl;
    double BeYl;
    double AwZl;
    double BwZl;
    double AwYl;
    double BwYl;
    double AwXl;
    double BwXl;


    if ( gClinet->allMotors.size() != 0 )
    {
        ///left hand
        AsYl = (float)-180/2048;


        AsXl = (float)180/2048;
        BsXl = 905;

        AsZl = (float)-180/2048;
        BsZl = 2432;

        AeYl = (float)-180/2048;
        BeYl = 1356;

        AwZl = (float)-180/2048;
        BwZl = 1944;

        AwYl = (float)-180/2048;
        BwYl = 1291;

        AwXl = (float)-180/2048;
        BwXl = 2011;


        ////
        ///right hand
        AsYr = (float)-180/2048;
        BsYr = 1947;

        AsXr = (float)180/2048;
        BsXr = 3400;

        AsZr = (float)-180/2048;
        BsZr = 1540;

        AeYr = (float)-180/2048;
        BeYr = 2320;

        AwZr = (float)-180/2048;
        BwZr = 2544;

        AwYr = (float)-180/2048;
        BwYr = 1107;

        AwXr = (float)-180/2048;
        BwXr = 2832;



    }
    else
        std::cout<<"nomotors"<<std::endl;


    std::cout<<"1"<<std::endl;
    double i;

    for (i=0;i<4;i++)
    {

        // Right hand //mire aghab

        double Qs_Ysend=RH1+a*DRHa1;
        int speeds_Y=((fabs(Qs_Ysend-PRH1.position)*fabs(AsYr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

        double Qs_Xsend=RH2+a*DRHa2;
        int speeds_X=((fabs(Qs_Xsend-PRH2.position)*fabs(AsXr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

        double Qs_Zsend=RH3+a*DRHa3;
        int speeds_Z=((fabs(Qs_Zsend-PRH3.position)*fabs(AsZr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

        double Qe_Ysend=RH4+a*DRHa4;
        int speede_Y=((fabs(Qe_Ysend-PRH4.position)*fabs(AeYr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

        double Qw_Zsend=RH5+a*DRHa5;
        int speedw_Z=((fabs(Qw_Zsend-PRH5.position)*fabs(AwZr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

        double Qw_Ysend=RH6+a*DRHa6;
        int speedw_Y=((fabs(Qw_Ysend-PRH6.position)*fabs(AwYr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

        double Qw_Xsend=RH7+a*DRHa7;
        int speedw_X=((fabs(Qw_Xsend-PRH7.position)*fabs(AwXr)/6)/(Time/2))/0.01;
        gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);


        // Left hand //mire jolo

        Qs_Ysend=LH1+a*DLHj1;
        speeds_Y=((fabs(Qs_Ysend-PLH1.position)*fabs(AsYl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

        Qs_Xsend=LH2+a*DLHj2;
        speeds_X=((fabs(Qs_Xsend-PLH2.position)*fabs(AsXl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

        Qs_Zsend=LH3+a*DLHj3;
        speeds_Z=((fabs(Qs_Zsend-PLH3.position)*fabs(AsZl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

        Qe_Ysend=LH4+a*DLHj4;
        speede_Y=((fabs(Qe_Ysend-PLH4.position)*fabs(AeYl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

        Qw_Zsend=LH5+a*DLHj5;
        speedw_Z=((fabs(Qw_Zsend-PLH5.position)*fabs(AwZl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

        Qw_Ysend=LH6+a*DLHj6;
        speedw_Y=((fabs(Qw_Ysend-PLH6.position)*fabs(AwYl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

        Qw_Xsend=LH7+a*DLHj7;
        speedw_X=((fabs(Qw_Xsend-PLH7.position)*fabs(AwXl)/6)/(Time/2))/0.01;
        gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);

        boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000/2));

        std::cout<<"3"<<std::endl;

        //Right hand //mire jolo
        Qs_Ysend=RH1+a*DRHj1;
        gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

        Qs_Xsend=RH2+a*DRHj2;
        gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

        Qs_Zsend=RH3+a*DRHj3;
        gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

        Qe_Ysend=RH4+a*DRHj4;
        gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

        Qw_Zsend=RH5+a*DRHj5;
        gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

        Qw_Ysend=RH6+a*DRHj6;
        gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

        Qw_Xsend=RH7+a*DRHj7;
        gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

        //Left hand //mire aghab
        Qs_Ysend=LH1+a*DLHa1;
        gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

        Qs_Xsend=LH2+a*DLHa2;
        gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

        Qs_Zsend=LH3+a*DLHa3;
        gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

        Qe_Ysend=LH4+a*DLHa4;
        gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

        Qw_Zsend=LH5+a*DLHa5;
        gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

        Qw_Ysend=LH6+a*DLHa6;
        gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

        Qw_Xsend=LH7+a*DLHa7;
        gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);

        boost::this_thread::sleep(boost::posix_time::milliseconds(Time*1000/2));

    }
}

//=========================== AMG
//=========================== GRIP IK FK [RIGHT|LEFT]
//94/02/05

int grip_right(float x,float y,float z,float a,float b,float g)
{
    // boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
    // boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    double AsX;
    double BsX;
    double AsY;
    double BsY;
    double AsZ;
    double BsZ;
    double AeY;
    double BeY;
    double AwZ;
    double BwZ;
    double AwY;
    double BwY;
    double AwX;
    double BwX;

    if ( gClinet->allMotors.size() != 0 )
    {

        AsY = (float)-180/2048;
        BsY = 1947;
        upperbodyclient::motor_data MsY = gClinet->getMotor_rightShoulderYaw();
        //Qsy0 =0;//AsY*(MsY.position-BsY)*pi/180;

        AsX = (float)180/2048;
        BsX = 3400;
        upperbodyclient::motor_data MsX = gClinet->getMotor_rightShoulderPitch();
        //Qsx0 =0;//AsX*(MsX.position-BsX)*pi/180;

        AsZ = (float)-180/2048;
        BsZ = 1540;
        upperbodyclient::motor_data MsZ = gClinet->getMotor_rightShoulderRoll();
        //Qsz0 =0;//AsZ*(MsZ.position-BsZ)*pi/180;

        AeY = (float)-180/2048;
        BeY = 2320;
        upperbodyclient::motor_data MeY = gClinet->getMotor_rightElbowPitch();
        //Qey0 =0;//AeY*(MeY.position-BeY)*pi/180;

        AwZ = (float)-180/2048;
        BwZ = 2544;
        upperbodyclient::motor_data MwZ = gClinet->getMotor_rightElbowRoll();
        //Qwz0 = 0;//AeZ*(MwZ.position-BwZ)*pi/180;

        AwY = (float)-180/2048;
        BwY = 1107;
        upperbodyclient::motor_data MwY = gClinet->getMotor_rightWristPitch();
        //Qwy0 = 0;//AwY*(MwY.position-BwY)*pi/180;

        AwX = (float)-180/2048;
        BwX = 2832;
        upperbodyclient::motor_data MwX = gClinet->getMotor_rightWristRoll();
        //Qwx0 = 0;//AwZ*(MwX.position-BwX)*pi/180;

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        //   std::cout<<"Qs_Y="<<Qsy0<<"   Qs_X="<<Qsx0<<"   Qs_Z="<<Qsz0<<"   Qe_Y="<<Qey0 <<"   Qe_Z="<<Qwx0<<"   Qw_y="<<Qwy0<<"   Qw_Z="<<Qwz0<<std::endl;


    }
    else
        std::cout<<"nomotors"<<std::endl;


    double a1=10*pi/180; // (rad)
    double L1=0.24; // (m)
    double L2=0.3; // (m)
    double L3=0.2; // (m)
    double L4=0.07; // (m)
    double L5=0.12; // (m)
    //double Lee=0.05;
    //double Tc=2; // (s)
    //3118
    double Qs_Xrad;
    double Qs_Yrad;
    double Qs_Zrad;
    double Qe_Yrad;
    double Qw_Xrad;
    double Qw_Yrad;
    double Qw_Zrad;

    //
    double QsxL=-180*pi/180;
    double QsxH=5*pi/180;
    double QsyL=-180*pi/180;
    double QsyH=40*pi/180;
    double QszL=-180*pi/180;
    double QszH=180*pi/180;
    double QeyL=-180*pi/180;
    double QeyH=180*pi/180;
    double QwzL=-180*pi/180;
    double QwzH=180*pi/180;
    double QwyL=-180*pi/180;
    double QwyH=180*pi/180;
    double QwxL=-180*pi/180;
    double QwxH=22.5*pi/180;

    //
    //double X=0.60;
    //double Y=-0.30;
    //double Z=-0.15;
    //double A=0*pi/180;
    //double B=-100*pi/180;
    //double G=0*pi/180;

    double X=x;
    double Y=y;
    double Z=z;
    double A=a*pi/180;
    double B=b*pi/180;
    double G=g*pi/180;

    /////reseting the refrence

    upperbodyclient::motor_data pos1 = gClinet->getMotor_rightShoulderYaw();
    double PR1=pos1.position;

    upperbodyclient::motor_data pos2 = gClinet->getMotor_rightShoulderPitch();
    double PR2=pos2.position;

    upperbodyclient::motor_data pos3 = gClinet->getMotor_rightShoulderRoll();
    double PR3=pos3.position;

    upperbodyclient::motor_data pos4 = gClinet->getMotor_rightElbowPitch();
    double PR4=pos4.position;

    upperbodyclient::motor_data pos5 = gClinet->getMotor_rightElbowRoll();
    double PR5=pos5.position;

    upperbodyclient::motor_data pos6 = gClinet->getMotor_rightWristPitch();
    double PR6=pos6.position;

    upperbodyclient::motor_data pos7 = gClinet->getMotor_rightWristRoll();
    double PR7=pos7.position;

    std::cout<<"refrence positions are="<<PR1<<std::endl<<PR2<<std::endl<< PR3<<std::endl<< PR4<<std::endl<<PR5<<std::endl<<PR6<<std::endl<<PR7<<std::endl <<std::endl ;


    //
    int Error;
    int Flag=0;

    for (double Qwx=QwxL;Qwx<=QwxH;Qwx+=1*pi/180){

        // IK
        Eigen::MatrixXd P1;
        Eigen::MatrixXd Pee;
        Eigen::MatrixXd P;

        double w[3]= {0,-L1/cos(a1),0};
        P1=RotTrans('X',-a1)*RotTrans('P', w); // Shoulder Transfer Matrix with Respect to Base

        double v[3]= {X,Y,Z};
        Pee=RotTrans('P',v)*RotTrans('X',A)*RotTrans('Y',B)*RotTrans('Z',G); // Endeffector Transfer Matrix with Respect to Base
        P=Pee.inverse()*P1;

        double X1=P(0,3);
        double  Y1=P(1,3);
        double  Z1=P(2,3);
        double  Qey=-acos((pow(X1,2)+pow(Y1-L4*sin(Qwx),2)+pow(Z1-L5-L4*cos(Qwx),2)-pow(L2,2)-pow(L3,2))/(2*L2*L3));
        double  t1=(Y1*cos(Qwx)-Z1*sin(Qwx)+L5*sin(Qwx))/L2; // t1=sin(Qey)*sin(Qwz)
        double  Qwz=asin(t1/sin(Qey));
        //
        double C1=-L3-L2*cos(Qey);
        double C2=-L2*cos(Qwz)*sin(Qey);
        double Qwy=asin(-X1/pow((pow(C1,2)+pow(C2,2)),0.5))-atan(C2/C1);

        Eigen::MatrixXd P2;
        double g1[3]={0,0,-L2};
        double g2[3]={0,0,-L3};
        double g3[3]={0,0,-L4};
        double g4[3]={0,0,-L5};
        P2=RotTrans('P',g1)*RotTrans('Y',Qey)*RotTrans('P',g2)*RotTrans('Z',Qwz)*RotTrans('Y',Qwy)*RotTrans('P',g3)*RotTrans('X',Qwx)*RotTrans('P',g4); // Endeffector Transfer Matrix with Respect to Shoulder
        Eigen::MatrixXd f2=RotTrans('Y',Qwy);

        Eigen::MatrixXd Px;
        Px=P1.inverse()*Pee*P2.inverse();

        double Qsx=asin(-Px(1,2));
        double Qsy=atan(Px(0,2)/Px(2,2));
        double Qsz=atan(Px(1,0)/Px(1,1));

        Eigen::MatrixXd P3;
        double h1[3]={0,-L1/cos(a1),0};
        double h2[3]={0,0,-L2};
        double h3[3]={0,0,-L3};
        double h4[3]={0,0,-L4};
        double h5[3]={0,0,-L5};
        P3=RotTrans('X',-a1)*RotTrans('P',h1)*RotTrans('Y',Qsy)*RotTrans('X',Qsx)*RotTrans('Z',Qsz)*RotTrans('P',h2)*RotTrans('Y',Qey)*RotTrans('P',h3)*RotTrans('Z',Qwz)*RotTrans('Y',Qwy)*RotTrans('P',h4)*RotTrans('X',Qwx)*RotTrans('P',h5);
        double Xee=P3(0,3);
        double Yee=P3(1,3);
        double Zee=P3(2,3);
        //Y

        double aa=pow(pow((Xee-X),2)+pow((Yee-Y),2)+pow((Zee-Z),2),0.5); //norm

        if (aa>1e-6)
        {
            Error=1;
            //     disp('IK has Error!');
        }
        else if (std::isnan(Qey)||std::isnan(Qwz)||std::isnan(Qwy)||std::isnan(Qsx)||std::isnan(Qsy)||std::isnan(Qsz)||std::isinf(Qey)||std::isinf(Qwz)||std::isinf(Qwy)||std::isinf(Qsx)||std::isinf(Qsy)||std::isinf(Qsz))
        {
            Error=2;
        }
        //     disp('Not Reachable!')
        else if ((QsxL>Qsx)||(Qsx>QsxH)||(QsyL>Qsy)||(Qsy>QsyH)||(QszL>Qsz)||(Qsz>QszH)||(QeyL>Qey)||(Qey>QeyH)||(QwxL>Qwx)||(Qwx>QwxH)||(QwyL>Qwy)||(Qwy>QwyH)||(QwzL>Qwz)||(Qwz>QwzH))
        {
            Error=3;
        }
        //     disp('Joint Limit!')
        else
        {
            Error=0;
        }

        if (Error==0){
            if (Flag==0){

                Qs_Xrad=Qsx;
                Qs_Yrad=Qsy;
                Qs_Zrad=Qsz;
                Qe_Yrad=Qey;
                Qw_Zrad=Qwz;
                Qw_Yrad=Qwy;
                Qw_Xrad=Qwx;


                Flag=1;
            }
            else{

                double AngleMovement=fabs(Qsx0-Qsx)+fabs(Qsy0-Qsy)+fabs(Qsz0-Qsz)+fabs(Qey0-Qey)+fabs(Qwx0-Qwx)+fabs(Qwy0-Qwy)+fabs(Qwz0-Qwz);
                double AngleMovement1=fabs(Qsx0-Qs_Xrad)+fabs(Qsy0-Qs_Yrad)+fabs(Qsz0-Qs_Zrad)+fabs(Qey0-Qe_Yrad)+fabs(Qwx0-Qw_Xrad)+fabs(Qwy0-Qw_Yrad)+fabs(Qwz0-Qw_Zrad);

                if (AngleMovement<AngleMovement1){
                    Qs_Xrad=Qsx;Qs_Yrad=Qsy;Qs_Zrad=Qsz;Qe_Yrad=Qey;Qw_Xrad=Qwx;Qw_Yrad=Qwy;Qw_Zrad=Qwz;

                }
            }
        }
    }

    if (Flag==0)
    {
        std::cout << "There is not any answer."<<std::endl;
        return 0;
    }
    std::cout<<"desired angles are="<<Qs_Yrad<<std::endl<<Qs_Xrad<<std::endl<< Qs_Zrad<<std::endl<< Qe_Yrad<<std::endl<< Qw_Zrad<<std::endl<< Qw_Yrad<<std::endl<<Qw_Xrad<<std::endl <<std::endl ;


    double Time=2;

    double Qs_Ysend=((Qs_Yrad*180/pi)/AsY)+BsY;
    int speeds_Y=((fabs(Qs_Ysend-PR1)*fabs(AsY)/6)/Time)/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    double Qs_Xsend=((Qs_Xrad*180/pi)/AsX)+BsX;
    int speeds_X=((fabs(Qs_Xsend-PR2)*fabs(AsX)/6)/Time)/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    double Qs_Zsend=((Qs_Zrad*180/pi)/AsZ)+BsZ;
    int speeds_Z=((fabs(Qs_Zsend-PR3)*fabs(AsZ)/6)/Time)/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    double Qe_Ysend=((Qe_Yrad*180/pi)/AeY)+BeY;
    int speede_Y=((fabs(Qe_Ysend-PR4)*fabs(AeY)/6)/Time)/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    double Qw_Zsend=((Qw_Zrad*180/pi)/AwZ)+BwZ;
    int speedw_Z=((fabs(Qw_Zsend-PR5)*fabs(AwZ)/6)/Time)/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    double Qw_Ysend=((Qw_Yrad*180/pi)/AwY)+BwY;
    int speedw_Y=((fabs(Qw_Ysend-PR6)*fabs(AwY)/6)/Time)/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    double Qw_Xsend=((Qw_Xrad*180/pi)/AwX)+BwX;
    int speedw_X=((fabs(Qw_Xsend-PR7)*fabs(AwX)/6)/Time)/0.01;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

    std::cout<<"desired velocities are="<<speeds_Y<<std::endl<<speeds_X<<std::endl<<speeds_Z<<std::endl<< speede_Y<<std::endl<<speedw_Z<<std::endl<<speedw_Y<<std::endl<<speedw_X<<std::endl <<std::endl ;



    std::cout<<"motor sent commands are="<<std::endl<<Qs_Ysend<<std::endl<<Qs_Xsend<<std::endl<<Qs_Zsend<<std::endl<<Qe_Ysend<<std::endl<<Qw_Zsend<<std::endl<<Qw_Ysend<<std::endl<<Qw_Xsend<<std::endl<<std::endl ;

    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));


    upperbodyclient::motor_data MssY = gClinet->getMotor_rightShoulderYaw();
    Qsy0 =AsY*(MssY.position-BsY)*pi/180;
    double bb=MssY.position;

    upperbodyclient::motor_data MssX = gClinet->getMotor_rightShoulderPitch();
    Qsx0 =AsX*(MssX.position-BsX)*pi/180;
    double aa=MssX.position;


    upperbodyclient::motor_data MssZ = gClinet->getMotor_rightShoulderRoll();
    Qsz0 =AsZ*(MssZ.position-BsZ)*pi/180;
    double cc=MssZ.position;

    upperbodyclient::motor_data MeeY = gClinet->getMotor_rightElbowPitch();
    Qey0 =AeY*(MeeY.position-BeY)*pi/180;
    double dd=MeeY.position;

    upperbodyclient::motor_data MwwZ = gClinet->getMotor_rightElbowRoll();
    Qwz0 = AwZ*(MwwZ.position-BwZ)*pi/180;
    double ee=MwwZ.position;

    upperbodyclient::motor_data MwwY = gClinet->getMotor_rightWristPitch();
    Qwy0 = AwY*(MwwY.position-BwY)*pi/180;
    double ff=MwwY.position;

    upperbodyclient::motor_data MwwX = gClinet->getMotor_rightWristRoll();
    Qwx0 = AwX*(MwwX.position-BwX)*pi/180;
    double gg=MwwX.position;

    std::cout<<"motor read positions are="<<std::endl<<bb<<std::endl<<aa<<std::endl<<cc<<std::endl<<dd<<std::endl<<ee<<std::endl<<ff<<std::endl<<gg<<std::endl <<std::endl;


    Eigen::MatrixXd P4;
    double k1[3]={0,-L1/cos(a1),0};
    double k2[3]={0,0,-L2};
    double k3[3]={0,0,-L3};
    double k4[3]={0,0,-L4};
    double k5[3]={0,0,-L5};
    P4=RotTrans('X',-a1)*RotTrans('P',k1)*RotTrans('Y',Qsy0)*RotTrans('X',Qsx0)*RotTrans('Z',Qsz0)*RotTrans('P',k2)*RotTrans('Y',Qey0)*RotTrans('P',k3)*RotTrans('Z',Qwz0)*RotTrans('Y',Qwy0)*RotTrans('P',k4)*RotTrans('X',Qwx0)*RotTrans('P',k5);
    double Xfk=P4(0,3);
    double Yfk=P4(1,3);
    double Zfk=P4(2,3);
    std::cout<<std::endl<<Xfk<<std::endl<<Yfk<<std::endl<<Zfk<<std::endl;

    double fasele=pow(pow(Xfk-X,2)+pow(Yfk-Y,2)+pow(Zfk-Z,2),0.5);
    std::cout<<std::endl<<"fasele is ="<<fasele<<std::endl;
    if (fasele>0.03)
        std::cout<<std::endl<<"not in domain"<<std::endl;

    //boost::this_thread::sleep(boost::posix_time::milliseconds(2500));

    ////checking the motors position

    upperbodyclient::motor_data CP1 = gClinet->getMotor_rightShoulderYaw();
    double cp1=MssY.position;

    upperbodyclient::motor_data CP2 = gClinet->getMotor_rightShoulderPitch();
    double cp2=MssX.position;


    upperbodyclient::motor_data CP3 = gClinet->getMotor_rightShoulderRoll();
    double cp3=MssZ.position;

    upperbodyclient::motor_data CP4 = gClinet->getMotor_rightElbowPitch();
    double cp4=MeeY.position;

    upperbodyclient::motor_data CP5 = gClinet->getMotor_rightElbowRoll();
    double cp5=MwwZ.position;

    upperbodyclient::motor_data CP6 = gClinet->getMotor_rightWristPitch();
    double cp6=MwwY.position;

    upperbodyclient::motor_data CP7 = gClinet->getMotor_rightWristRoll();
    double cp7=MwwX.position;

    if (cp1>3000 || cp1<1040){
        std::cout<<std::endl<<"angle cp1 out of domain"<<std::endl;
    }
    if (cp2>3570 || cp2<2030){
        std::cout<<std::endl<<"angle cp2 out of domain"<<std::endl;
    }
    if (cp3>2000 || cp3<70){
        std::cout<<std::endl<<"angle cp3 out of domain"<<std::endl;
    }
    if (cp4>2870 || cp4<1730){
        std::cout<<std::endl<<"angle cp4 out of domain"<<std::endl;
    }
    if (cp5>2820 || cp5<300){
        std::cout<<std::endl<<"angle cp5 out of domain"<<std::endl;
    }
    if (cp6>3370 || cp6<1430){
        std::cout<<std::endl<<"angle cp6 out of domain"<<std::endl;
    }
    if (cp7>2750 || cp7<1530){
        std::cout<<std::endl<<"angle cp7 out of domain"<<std::endl;
    }

    if (Flag==0)
    {
        //std::cout << "There is not any answer. [Error!] "<<std::endl;

        if ( Error == 1)
            std::cout << "There is not any answer. IK has Error!"<<std::endl;
        if ( Error == 2)
            std::cout << "There is not any answer. Not Reachable!"<<std::endl;
        if ( Error == 3)
            std::cout << "There is not any answer. Joint Limit!"<<std::endl;

        return Error;
    }
    else
    {
        return 0;
    }

}

int grip_process_right(double xend,double yend,double zend)
{

    double al;
    double be;
    double ga;

    //double Xkinect=0.14;
    //double Ykinect=0.16;
    //double Zkinect=0.66;

    double Qgrip_send=1885;
    double speed_grip=150;

    gClinet->setMotor_rightGripper(Qgrip_send,speed_grip);

    //double Xend=0.5;
    //double Yend=-0.35;
    //double Zend=-0.2;

    //for kinect axes
    double Yend=-xend;
    double Zend=-yend;
    double Xend=zend;

    double KinOffX=0.16;
    double KinOffY=0.05;
    //TODO change this if we dont have any move forward
    double RobotGo=0.6; //if Robot moves forward

    double teta=0;// rotation of torso
    //double RobotUp=0.09;

    double Zoffset=0.10;
    double Yoffset=-0.06;
    double L1=0.24;

    double Xk=L1*sin(teta*pi/180)*cos(10*pi/180);
    double Yk=L1*(1-cos(teta*pi/180))*cos(10*pi/180);

    std::cout<<std::endl<<"Xk is = "<<Xk<<std::endl;
    std::cout<<std::endl<<"Yk is = "<<Yk<<std::endl;

    double Xg=Xend+KinOffX-RobotGo;
    double Yg=Yend+Yoffset+KinOffY;
    double Z=Zend+Zoffset;  //-RobotUp;



    if (Yend<-0.1 && Yend>=-0.2)
        ga=30;
    else if(Yend<-0.20 && Yend>=-0.3)
        ga=10;
    else if (Yend<-0.3)
        ga=0;

    if (Zend>-0.15)
        be=-135;
    else if (Zend<-0.3)
        be=-90;
    else
        be=-105;


    al = 0;

    double alpha=al;
    double betha=be;
    double gamma=ga-teta;

    double X=Xg*cos(teta*pi/180)+Yg*sin(teta*pi/180)-Xk;
    double Y=-Xg*sin(teta*pi/180)+Yg*cos(teta*pi/180)-Yk;


    std::cout<<std::endl<<"X is = "<<X<<std::endl;
    std::cout<<std::endl<<"Y is = "<<Y<<std::endl;
    std::cout<<std::endl<<"Z is = "<<Z<<std::endl;

    double A=(atan2(cos(gamma*pi/180)*sin(alpha*pi/180)-cos(alpha*pi/180)*sin(betha*pi/180)*sin(gamma*pi/180),cos(alpha*pi/180)*cos(betha*pi/180)))*180/pi;
    double B=(asin(sin(alpha*pi/180)*sin(gamma*pi/180)+cos(alpha*pi/180)*cos(gamma*pi/180)*sin(betha*pi/180)))*180/pi;
    double G=(atan2(cos(alpha*pi/180)*sin(gamma*pi/180)-cos(gamma*pi/180)*sin(alpha*pi/180)*sin(betha*pi/180),cos(betha*pi/180)*cos(gamma*pi/180)))*180/pi;
    std::cout<<std::endl<<"angles"<<A<<std::endl<<B<<std::endl<<G<<std::endl;



    grip_right(0.3,-0.42,-0.2,0,-90,0);

    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));


    double a=0.15*cos((gamma+teta)*pi/180);
    double b=0.15*sin((gamma+teta)*pi/180);
    grip_right(X-a,Y-b-0.05,Z,A,B,G);

    // boost::this_thread::sleep(boost::posix_time::millisecon5ds(1000));

    grip_right(X,Y,Z,A,B,G);

    Qgrip_send=2492;
    speed_grip=150;
    gClinet->setMotor_rightGripper(Qgrip_send,speed_grip);

    boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
    grip_right(X-0.05,Y,Z+0.1,A,B,G);

    grip_right(0.3,-0.42,-0.25,0,-90,0);

    return 0;
}

//old
//int grip_right(float x,float y,float z,float a,float b,float g)
//{
//    // boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
//    // boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

//    double AsX;
//    double BsX;
//    double AsY;
//    double BsY;
//    double AsZ;
//    double BsZ;
//    double AeY;
//    double BeY;
//    double AwZ;
//    double BwZ;
//    double AwY;
//    double BwY;
//    double AwX;
//    double BwX;
//    gClinet->setMotorPid_rightElbowPitch(3,35,0);
//    gClinet->setMotorPid_rightShoulderPitch(3,35,0);
//    gClinet->setMotorPid_rightShoulderRoll(3,35,0);
//    gClinet->setMotorPid_rightShoulderYaw(3,35,0);
//    //  gClinet->setMotorPid_rightElbowPitch(3,35,0);
//    gClinet->setMotorPid_rightElbowRoll(3,35,0);
//    gClinet->setMotorPid_rightWristPitch(0,35,0);
//    gClinet->setMotorPid_rightWristRoll(0,35,0);


//    gClinet->setMotorPid_leftShoulderPitch(3,35,0);
//    gClinet->setMotorPid_leftShoulderRoll(3,35,0);
//    gClinet->setMotorPid_leftShoulderYaw(3,35,0);
//    gClinet->setMotorPid_leftElbowPitch(3,35,0);
//    gClinet->setMotorPid_leftElbowRoll(3,35,0);
//    gClinet->setMotorPid_leftWristPitch(0,35,0);
//    gClinet->setMotorPid_leftWristRoll(0,35,0);

//    if ( gClinet->allMotors.size() != 0 )
//    {

//        AsY = (float)-180/2048;
//        BsY = 1947;
//        upperbodyclient::motor_data MsY = gClinet->getMotor_rightShoulderYaw();
//        //Qsy0 =0;//AsY*(MsY.position-BsY)*pi/180;

//        AsX = (float)180/2048;
//        BsX = 3400;
//        upperbodyclient::motor_data MsX = gClinet->getMotor_rightShoulderPitch();
//        //Qsx0 =0;//AsX*(MsX.position-BsX)*pi/180;

//        AsZ = (float)-180/2048;
//        BsZ = 1540;
//        upperbodyclient::motor_data MsZ = gClinet->getMotor_rightShoulderRoll();
//        //Qsz0 =0;//AsZ*(MsZ.position-BsZ)*pi/180;

//        AeY = (float)-180/2048;
//        BeY = 2320;
//        upperbodyclient::motor_data MeY = gClinet->getMotor_rightElbowPitch();
//        //Qey0 =0;//AeY*(MeY.position-BeY)*pi/180;

//        AwZ = (float)-180/2048;
//        BwZ = 2544;
//        upperbodyclient::motor_data MwZ = gClinet->getMotor_rightElbowRoll();
//        //Qwz0 = 0;//AeZ*(MwZ.position-BwZ)*pi/180;

//        AwY = (float)-180/2048;
//        BwY = 1107;
//        upperbodyclient::motor_data MwY = gClinet->getMotor_rightWristPitch();
//        //Qwy0 = 0;//AwY*(MwY.position-BwY)*pi/180;

//        AwX = (float)-180/2048;
//        BwX = 2832;
//        upperbodyclient::motor_data MwX = gClinet->getMotor_rightWristRoll();
//        //Qwx0 = 0;//AwZ*(MwX.position-BwX)*pi/180;

//        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

//        //   std::cout<<"Qs_Y="<<Qsy0<<"   Qs_X="<<Qsx0<<"   Qs_Z="<<Qsz0<<"   Qe_Y="<<Qey0 <<"   Qe_Z="<<Qwx0<<"   Qw_y="<<Qwy0<<"   Qw_Z="<<Qwz0<<std::endl;


//    }
//    else
//        std::cout<<"nomotors"<<std::endl;


//    double a1=10*pi/180; // (rad)
//    double L1=0.24; // (m)
//    double L2=0.3; // (m)
//    double L3=0.2; // (m)
//    double L4=0.07; // (m)
//    double L5=0.12; // (m)
//    //double Lee=0.05;
//    //double Tc=2; // (s)
//    //3118
//    double Qs_Xrad;
//    double Qs_Yrad;
//    double Qs_Zrad;
//    double Qe_Yrad;
//    double Qw_Xrad;
//    double Qw_Yrad;
//    double Qw_Zrad;

//    //
//    double QsxL=-180*pi/180;
//    double QsxH=5*pi/180;
//    double QsyL=-180*pi/180;
//    double QsyH=40*pi/180;
//    double QszL=-180*pi/180;
//    double QszH=180*pi/180;
//    double QeyL=-180*pi/180;
//    double QeyH=180*pi/180;
//    double QwzL=-180*pi/180;
//    double QwzH=180*pi/180;
//    double QwyL=-180*pi/180;
//    double QwyH=180*pi/180;
//    double QwxL=-180*pi/180;
//    double QwxH=22.5*pi/180;

//    //
//    //double X=0.60;
//    //double Y=-0.30;
//    //double Z=-0.15;
//    //double A=0*pi/180;
//    //double B=-100*pi/180;
//    //double G=0*pi/180;

//    double X=x;
//    double Y=y;
//    double Z=z;
//    double A=a*pi/180;
//    double B=b*pi/180;
//    double G=g*pi/180;

//    /////reseting the refrence

//    upperbodyclient::motor_data pos1 = gClinet->getMotor_rightShoulderYaw();
//    double PR1=pos1.position;

//    upperbodyclient::motor_data pos2 = gClinet->getMotor_rightShoulderPitch();
//    double PR2=pos2.position;

//    upperbodyclient::motor_data pos3 = gClinet->getMotor_rightShoulderRoll();
//    double PR3=pos3.position;

//    upperbodyclient::motor_data pos4 = gClinet->getMotor_rightElbowPitch();
//    double PR4=pos4.position;

//    upperbodyclient::motor_data pos5 = gClinet->getMotor_rightElbowRoll();
//    double PR5=pos5.position;

//    upperbodyclient::motor_data pos6 = gClinet->getMotor_rightWristPitch();
//    double PR6=pos6.position;

//    upperbodyclient::motor_data pos7 = gClinet->getMotor_rightWristRoll();
//    double PR7=pos7.position;

//    std::cout<<"refrence positions are="<<PR1<<std::endl<<PR2<<std::endl<< PR3<<std::endl<< PR4<<std::endl<<PR5<<std::endl<<PR6<<std::endl<<PR7<<std::endl <<std::endl ;


//    //
//    int Error;
//    int Flag=0;

//    for (double Qwx=QwxL;Qwx<=QwxH;Qwx+=1*pi/180){

//        // IK
//        Eigen::MatrixXd P1;
//        Eigen::MatrixXd Pee;
//        Eigen::MatrixXd P;

//        double w[3]= {0,-L1/cos(a1),0};
//        P1=RotTrans('X',-a1)*RotTrans('P', w); // Shoulder Transfer Matrix with Respect to Base

//        double v[3]= {X,Y,Z};
//        Pee=RotTrans('P',v)*RotTrans('X',A)*RotTrans('Y',B)*RotTrans('Z',G); // Endeffector Transfer Matrix with Respect to Base
//        P=Pee.inverse()*P1;

//        double X1=P(0,3);
//        double  Y1=P(1,3);
//        double  Z1=P(2,3);
//        double  Qey=-acos((pow(X1,2)+pow(Y1-L4*sin(Qwx),2)+pow(Z1-L5-L4*cos(Qwx),2)-pow(L2,2)-pow(L3,2))/(2*L2*L3));
//        double  t1=(Y1*cos(Qwx)-Z1*sin(Qwx)+L5*sin(Qwx))/L2; // t1=sin(Qey)*sin(Qwz)
//        double  Qwz=asin(t1/sin(Qey));
//        //
//        double C1=-L3-L2*cos(Qey);
//        double C2=-L2*cos(Qwz)*sin(Qey);
//        double Qwy=asin(-X1/pow((pow(C1,2)+pow(C2,2)),0.5))-atan(C2/C1);

//        Eigen::MatrixXd P2;
//        double g1[3]={0,0,-L2};
//        double g2[3]={0,0,-L3};
//        double g3[3]={0,0,-L4};
//        double g4[3]={0,0,-L5};
//        P2=RotTrans('P',g1)*RotTrans('Y',Qey)*RotTrans('P',g2)*RotTrans('Z',Qwz)*RotTrans('Y',Qwy)*RotTrans('P',g3)*RotTrans('X',Qwx)*RotTrans('P',g4); // Endeffector Transfer Matrix with Respect to Shoulder
//        Eigen::MatrixXd f2=RotTrans('Y',Qwy);

//        Eigen::MatrixXd Px;
//        Px=P1.inverse()*Pee*P2.inverse();

//        double Qsx=asin(-Px(1,2));
//        double Qsy=atan(Px(0,2)/Px(2,2));
//        double Qsz=atan(Px(1,0)/Px(1,1));

//        Eigen::MatrixXd P3;
//        double h1[3]={0,-L1/cos(a1),0};
//        double h2[3]={0,0,-L2};
//        double h3[3]={0,0,-L3};
//        double h4[3]={0,0,-L4};
//        double h5[3]={0,0,-L5};
//        P3=RotTrans('X',-a1)*RotTrans('P',h1)*RotTrans('Y',Qsy)*RotTrans('X',Qsx)*RotTrans('Z',Qsz)*RotTrans('P',h2)*RotTrans('Y',Qey)*RotTrans('P',h3)*RotTrans('Z',Qwz)*RotTrans('Y',Qwy)*RotTrans('P',h4)*RotTrans('X',Qwx)*RotTrans('P',h5);
//        double Xee=P3(0,3);
//        double Yee=P3(1,3);
//        double Zee=P3(2,3);
//        //Y

//        double aa=pow(pow((Xee-X),2)+pow((Yee-Y),2)+pow((Zee-Z),2),0.5); //norm

//        if (aa>1e-6)
//        {
//            Error=1;
//            //     disp('IK has Error!');
//        }
//        else if (std::isnan(Qey)||std::isnan(Qwz)||std::isnan(Qwy)||std::isnan(Qsx)||std::isnan(Qsy)||std::isnan(Qsz)||std::isinf(Qey)||std::isinf(Qwz)||std::isinf(Qwy)||std::isinf(Qsx)||std::isinf(Qsy)||std::isinf(Qsz))
//        {
//            Error=2;
//        }
//        //     disp('Not Reachable!')
//        else if ((QsxL>Qsx)||(Qsx>QsxH)||(QsyL>Qsy)||(Qsy>QsyH)||(QszL>Qsz)||(Qsz>QszH)||(QeyL>Qey)||(Qey>QeyH)||(QwxL>Qwx)||(Qwx>QwxH)||(QwyL>Qwy)||(Qwy>QwyH)||(QwzL>Qwz)||(Qwz>QwzH))
//        {
//            Error=3;
//        }
//        //     disp('Joint Limit!')
//        else
//        {
//            Error=0;
//        }

//        if (Error==0){
//            if (Flag==0){

//                Qs_Xrad=Qsx;
//                Qs_Yrad=Qsy;
//                Qs_Zrad=Qsz;
//                Qe_Yrad=Qey;
//                Qw_Zrad=Qwz;
//                Qw_Yrad=Qwy;
//                Qw_Xrad=Qwx;


//                Flag=1;
//            }
//            else{

//                double AngleMovement=fabs(Qsx0-Qsx)+fabs(Qsy0-Qsy)+fabs(Qsz0-Qsz)+fabs(Qey0-Qey)+fabs(Qwx0-Qwx)+fabs(Qwy0-Qwy)+fabs(Qwz0-Qwz);
//                double AngleMovement1=fabs(Qsx0-Qs_Xrad)+fabs(Qsy0-Qs_Yrad)+fabs(Qsz0-Qs_Zrad)+fabs(Qey0-Qe_Yrad)+fabs(Qwx0-Qw_Xrad)+fabs(Qwy0-Qw_Yrad)+fabs(Qwz0-Qw_Zrad);

//                if (AngleMovement<AngleMovement1){
//                    Qs_Xrad=Qsx;Qs_Yrad=Qsy;Qs_Zrad=Qsz;Qe_Yrad=Qey;Qw_Xrad=Qwx;Qw_Yrad=Qwy;Qw_Zrad=Qwz;

//                }
//            }
//        }
//    }

//    if (Flag==0)
//    {
//        std::cout << "There is not any answer."<<std::endl;
//        double Qs_Ysend=1947;
//        double Qs_Xsend=3400;
//        double Qs_Zsend=1540;
//        double Qe_Ysend=2320;
//        double Qw_Zsend=2544;
//        double Qw_Ysend=1107;
//        double Qw_Xsend=2832;
//    }
//    std::cout<<"desired angles are="<<Qs_Yrad<<std::endl<<Qs_Xrad<<std::endl<< Qs_Zrad<<std::endl<< Qe_Yrad<<std::endl<< Qw_Zrad<<std::endl<< Qw_Yrad<<std::endl<<Qw_Xrad<<std::endl <<std::endl ;


//    double Time=2;

//    double Qs_Ysend=((Qs_Yrad*180/pi)/AsY)+BsY;
//    int speeds_Y=((fabs(Qs_Ysend-PR1)*fabs(AsY)/6)/Time)/0.01;
//    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

//    double Qs_Xsend=((Qs_Xrad*180/pi)/AsX)+BsX;
//    int speeds_X=((fabs(Qs_Xsend-PR2)*fabs(AsX)/6)/Time)/0.01;
//    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

//    double Qs_Zsend=((Qs_Zrad*180/pi)/AsZ)+BsZ;
//    int speeds_Z=((fabs(Qs_Zsend-PR3)*fabs(AsZ)/6)/Time)/0.01;
//    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

//    double Qe_Ysend=((Qe_Yrad*180/pi)/AeY)+BeY;
//    int speede_Y=((fabs(Qe_Ysend-PR4)*fabs(AeY)/6)/Time)/0.01;
//    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

//    double Qw_Zsend=((Qw_Zrad*180/pi)/AwZ)+BwZ;
//    int speedw_Z=((fabs(Qw_Zsend-PR5)*fabs(AwZ)/6)/Time)/0.01;
//    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

//    double Qw_Ysend=((Qw_Yrad*180/pi)/AwY)+BwY;
//    int speedw_Y=((fabs(Qw_Ysend-PR6)*fabs(AwY)/6)/Time)/0.01;
//    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

//    double Qw_Xsend=((Qw_Xrad*180/pi)/AwX)+BwX;
//    int speedw_X=((fabs(Qw_Xsend-PR7)*fabs(AwX)/6)/Time)/0.01;
//    gClinet->setMotor_rightWristRoll(Qw_Xsend,speedw_X);

//    // boost::this_thread::sleep(boost::posix_time::milliseconds(6000));
//    // double Qgrip_send=1420;
//    // double speed_grip=1;
//    // gClinet->setMotor_rightGripper(Qgrip_send,speed_grip);
//    std::cout<<"desired velocities are="<<speeds_Y<<std::endl<<speeds_X<<std::endl<<speeds_Z<<std::endl<< speede_Y<<std::endl<<speedw_Z<<std::endl<<speedw_Y<<std::endl<<speedw_X<<std::endl <<std::endl ;



//    std::cout<<"motor sent commands are="<<std::endl<<Qs_Ysend<<std::endl<<Qs_Xsend<<std::endl<<Qs_Zsend<<std::endl<<Qe_Ysend<<std::endl<<Qw_Zsend<<std::endl<<Qw_Ysend<<std::endl<<Qw_Xsend<<std::endl<<std::endl ;

//    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));


//    upperbodyclient::motor_data MssY = gClinet->getMotor_rightShoulderYaw();
//    Qsy0 =AsY*(MssY.position-BsY)*pi/180;
//    double bb=MssY.position;

//    upperbodyclient::motor_data MssX = gClinet->getMotor_rightShoulderPitch();
//    Qsx0 =AsX*(MssX.position-BsX)*pi/180;
//    double aa=MssX.position;


//    upperbodyclient::motor_data MssZ = gClinet->getMotor_rightShoulderRoll();
//    Qsz0 =AsZ*(MssZ.position-BsZ)*pi/180;
//    double cc=MssZ.position;

//    upperbodyclient::motor_data MeeY = gClinet->getMotor_rightElbowPitch();
//    Qey0 =AeY*(MeeY.position-BeY)*pi/180;
//    double dd=MeeY.position;

//    upperbodyclient::motor_data MwwZ = gClinet->getMotor_rightElbowRoll();
//    Qwz0 = AwZ*(MwwZ.position-BwZ)*pi/180;
//    double ee=MwwZ.position;

//    upperbodyclient::motor_data MwwY = gClinet->getMotor_rightWristPitch();
//    Qwy0 = AwY*(MwwY.position-BwY)*pi/180;
//    double ff=MwwY.position;

//    upperbodyclient::motor_data MwwX = gClinet->getMotor_rightWristRoll();
//    Qwx0 = AwX*(MwwX.position-BwX)*pi/180;
//    double gg=MwwX.position;

//    std::cout<<"motor read positions are="<<std::endl<<bb<<std::endl<<aa<<std::endl<<cc<<std::endl<<dd<<std::endl<<ee<<std::endl<<ff<<std::endl<<gg<<std::endl <<std::endl;
//    /*
//       double speed=5;
//    double Qs_Ysend=((0*180/pi)/AsY)+BsY;
//    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speed);

//    double Qs_Xsend=((0*180/pi)/AsX)+BsX;
//    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speed);

//    double Qs_Zsend=((0*180/pi)/AsZ)+BsZ;
//    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speed);

//    double Qe_Ysend=((0*180/pi)/AeY)+BeY;
//    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speed);

//    double Qw_Zsend=((0*180/pi)/AwZ)+BwZ;
//    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speed);

//    double Qw_Ysend=((0*180/pi)/AwY)+BwY;
//    gClinet->setMotor_rightWristPitch(Qw_Ysend,speed);

//    double Qw_Xsend=((0*180/pi)/AwX)+BwX;
//    gClinet->setMotor_rightWristRoll(Qw_Xsend,speed);
//*/
//    //setting the initial value of tetas
//    // Qsy0 =bb;
//    //Qsx0 =aa;
//    //Qsz0 =cc;
//    // Qey0 =dd;
//    // Qwz0 =ee;
//    // Qwy0 =ff;
//    //Qwx0 =gg;
//    //FK

//    Eigen::MatrixXd P4;
//    double k1[3]={0,-L1/cos(a1),0};
//    double k2[3]={0,0,-L2};
//    double k3[3]={0,0,-L3};
//    double k4[3]={0,0,-L4};
//    double k5[3]={0,0,-L5};
//    P4=RotTrans('X',-a1)*RotTrans('P',k1)*RotTrans('Y',Qsy0)*RotTrans('X',Qsx0)*RotTrans('Z',Qsz0)*RotTrans('P',k2)*RotTrans('Y',Qey0)*RotTrans('P',k3)*RotTrans('Z',Qwz0)*RotTrans('Y',Qwy0)*RotTrans('P',k4)*RotTrans('X',Qwx0)*RotTrans('P',k5);
//    double Xfk=P4(0,3);
//    double Yfk=P4(1,3);
//    double Zfk=P4(2,3);
//    std::cout<<std::endl<<Xfk<<std::endl<<Yfk<<std::endl<<Zfk<<std::endl;

//    double fasele=pow(pow(Xfk-X,2)+pow(Yfk-Y,2)+pow(Zfk-Z,2),0.5);
//    std::cout<<std::endl<<"fasele is ="<<fasele<<std::endl;
//    if (fasele>0.03)
//        std::cout<<std::endl<<"not in domain"<<std::endl;

//    //boost::this_thread::sleep(boost::posix_time::milliseconds(2500));

//    ////checking the motors position

//    upperbodyclient::motor_data CP1 = gClinet->getMotor_rightShoulderYaw();
//    double cp1=MssY.position;

//    upperbodyclient::motor_data CP2 = gClinet->getMotor_rightShoulderPitch();
//    double cp2=MssX.position;


//    upperbodyclient::motor_data CP3 = gClinet->getMotor_rightShoulderRoll();
//    double cp3=MssZ.position;

//    upperbodyclient::motor_data CP4 = gClinet->getMotor_rightElbowPitch();
//    double cp4=MeeY.position;

//    upperbodyclient::motor_data CP5 = gClinet->getMotor_rightElbowRoll();
//    double cp5=MwwZ.position;

//    upperbodyclient::motor_data CP6 = gClinet->getMotor_rightWristPitch();
//    double cp6=MwwY.position;

//    upperbodyclient::motor_data CP7 = gClinet->getMotor_rightWristRoll();
//    double cp7=MwwX.position;

//    if (cp1>3000 || cp1<1040){
//        std::cout<<std::endl<<"angle cp1 out of domain"<<std::endl;
//    }
//    if (cp2>3570 || cp2<2030){
//        std::cout<<std::endl<<"angle cp2 out of domain"<<std::endl;
//    }
//    if (cp3>2000 || cp3<70){
//        std::cout<<std::endl<<"angle cp3 out of domain"<<std::endl;
//    }
//    if (cp4>2870 || cp4<1730){
//        std::cout<<std::endl<<"angle cp4 out of domain"<<std::endl;
//    }
//    if (cp5>2820 || cp5<300){
//        std::cout<<std::endl<<"angle cp5 out of domain"<<std::endl;
//    }
//    if (cp6>3370 || cp6<1430){
//        std::cout<<std::endl<<"angle cp6 out of domain"<<std::endl;
//    }
//    if (cp7>2750 || cp7<1530){
//        std::cout<<std::endl<<"angle cp7 out of domain"<<std::endl;
//    }

//    if (Flag==0)
//    {
//        //std::cout << "There is not any answer. [Error!] "<<std::endl;

//        if ( Error == 1)
//            std::cout << "There is not any answer. IK has Error!"<<std::endl;
//        if ( Error == 2)
//            std::cout << "There is not any answer. Not Reachable!"<<std::endl;
//        if ( Error == 3)
//            std::cout << "There is not any answer. Joint Limit!"<<std::endl;

//        return Error;
//    }
//    else
//    {
//        return 0;
//    }

//}

//int grip_process_right(double xend,double yend,double zend)
//{

//    double al;
//    double be;
//    double ga;

//    //double Xkinect=0.14;
//    //double Ykinect=0.16;
//    //double Zkinect=0.66;

//    double Qgrip_send=2200;
//    double speed_grip=150;

//    gClinet->setMotor_rightGripper(Qgrip_send,speed_grip);

//    //double Xend=0.5;
//    //double Yend=-0.35;
//    //double Zend=-0.2;

//    //for kinect axes
//    double Yend=-xend;
//    double Zend=-yend;
//    double Xend=zend;

//    double KinOffX=0.16;
//    double KinOffY=0.05;
//    //TODO change this if we dont have any move forward
//    double RobotGo=0.4; //if Robot moves forward

//    double teta=0;// rotation of torso
//    //double RobotUp=0.09;

//    double Zoffset=0.02;
//    double Yoffset=-0.02;
//    double L1=0.24;

//    double Xk=L1*sin(teta*pi/180)*cos(10*pi/180);
//    double Yk=L1*(1-cos(teta*pi/180))*cos(10*pi/180);

//    std::cout<<std::endl<<"Xk is = "<<Xk<<std::endl;
//    std::cout<<std::endl<<"Yk is = "<<Yk<<std::endl;

//    double Xg=Xend+KinOffX-RobotGo;
//    double Yg=Yend+Yoffset+KinOffY;
//    double Z=Zend+Zoffset;  //-RobotUp;

//    ga = 10;
//    be = -90;
//    al = 0;

//    //    if (Yend<-0.1 && Yend>=-0.2)
//    //        ga=30;
//    //    else if(Yend<-0.20 && Yend>=-0.3)
//    //        ga=10;
//    //    else if (Yend<-0.3)
//    //        ga=0;

//    //    if (Zend>0)
//    //        be=-120;
//    //    else if (Zend<-0.3)
//    //        be=-70;
//    //    else
//    //        be=-90;

//    //default grip
//    // al = 0;

//    double alpha=al;
//    double betha=be;
//    double gamma=ga-teta;

//    double X=Xg*cos(teta*pi/180)+Yg*sin(teta*pi/180)-Xk;
//    double Y=-Xg*sin(teta*pi/180)+Yg*cos(teta*pi/180)-Yk;


//    std::cout<<std::endl<<"X is = "<<X<<std::endl;
//    std::cout<<std::endl<<"Y is = "<<Y<<std::endl;
//    std::cout<<std::endl<<"Z is = "<<Z<<std::endl;

//    double A=(atan2(cos(gamma*pi/180)*sin(alpha*pi/180)-cos(alpha*pi/180)*sin(betha*pi/180)*sin(gamma*pi/180),cos(alpha*pi/180)*cos(betha*pi/180)))*180/pi;
//    double B=(asin(sin(alpha*pi/180)*sin(gamma*pi/180)+cos(alpha*pi/180)*cos(gamma*pi/180)*sin(betha*pi/180)))*180/pi;
//    double G=(atan2(cos(alpha*pi/180)*sin(gamma*pi/180)-cos(gamma*pi/180)*sin(alpha*pi/180)*sin(betha*pi/180),cos(betha*pi/180)*cos(gamma*pi/180)))*180/pi;
//    std::cout<<std::endl<<"angles"<<A<<std::endl<<B<<std::endl<<G<<std::endl;



//    grip_right(0.3,-0.42,-0.25,0,-90,0);



//    double a=0.15*cos((gamma+teta)*pi/180);
//    double b=0.15*sin((gamma+teta)*pi/180);
//    grip_right(X-a,Y-b-0.05,Z,A,B,G);

//    // boost::this_thread::sleep(boost::posix_time::millisecon5ds(1000));

//    grip_right(X,Y,Z,A,B,G);

//    Qgrip_send=1700;
//    speed_grip=150;
//    gClinet->setMotor_rightGripper(Qgrip_send,speed_grip);

//    boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
//    grip_right(X-0.05,Y,Z+0.1,A,B,G);

//    grip_right(0.3,-0.42,-0.25,0,-90,0);

//    return 0;
//}

int grip_left(float x,float y,float z,float a,float b,float g)
{
    // boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
    // boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    double AsX;
    double BsX;
    double AsY;
    double BsY;
    double AsZ;
    double BsZ;
    double AeY;
    double BeY;
    double AwZ;
    double BwZ;
    double AwY;
    double BwY;
    double AwX;
    double BwX;
    gClinet->setMotorPid_leftElbowPitch(3,35,0);
    gClinet->setMotorPid_leftShoulderPitch(3,35,0);
    gClinet->setMotorPid_leftShoulderRoll(3,35,0);
    gClinet->setMotorPid_leftShoulderYaw(3,35,0);
    //  gClinet->setMotorPid_leftElbowPitch(3,35,0);
    gClinet->setMotorPid_leftElbowRoll(3,35,0);
    gClinet->setMotorPid_leftWristPitch(0,35,0);
    gClinet->setMotorPid_leftWristRoll(0,35,0);


    gClinet->setMotorPid_leftShoulderPitch(3,35,0);
    gClinet->setMotorPid_leftShoulderRoll(3,35,0);
    gClinet->setMotorPid_leftShoulderYaw(3,35,0);
    gClinet->setMotorPid_leftElbowPitch(3,35,0);
    gClinet->setMotorPid_leftElbowRoll(3,35,0);
    gClinet->setMotorPid_leftWristPitch(0,35,0);
    gClinet->setMotorPid_leftWristRoll(0,35,0);


    // if (left) {
    if ( gClinet->allMotors.size() != 0 )
    {

        AsY = (float)-180/2048;
        BsY = 1370;
        upperbodyclient::motor_data MsY = gClinet->getMotor_leftShoulderYaw();
        Qsy0 =0;//AsY*(MsY.position-BsY)*pi/180;

        AsX = (float)180/2048;
        BsX = 937;
        upperbodyclient::motor_data MsX = gClinet->getMotor_leftShoulderPitch();
        Qsx0 =0;//AsX*(MsX.position-BsX)*pi/180;

        AsZ = (float)-180/2048;
        BsZ = 2432;
        upperbodyclient::motor_data MsZ = gClinet->getMotor_leftShoulderRoll();
        Qsz0 =0;//AsZ*(MsZ.position-BsZ)*pi/180;

        AeY = (float)-180/2048;
        BeY = 1356;
        upperbodyclient::motor_data MeY = gClinet->getMotor_leftElbowPitch();
        Qey0 =0;//AeY*(MeY.position-BeY)*pi/180;

        AwZ = (float)-180/2048;
        BwZ = 1944;
        upperbodyclient::motor_data MwZ = gClinet->getMotor_leftElbowRoll();
        Qwz0 = 0;//AeZ*(MwZ.position-BwZ)*pi/180;

        AwY = (float)-180/2048;
        BwY = 1291;
        upperbodyclient::motor_data MwY = gClinet->getMotor_leftWristPitch();
        Qwy0 = 0;//AwY*(MwY.position-BwY)*pi/180;

        AwX = (float)-180/2048;
        BwX = 2011;
        upperbodyclient::motor_data MwX = gClinet->getMotor_leftWristRoll();
        Qwx0 = 0;//AwZ*(MwX.position-BwX)*pi/180;

        //     boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

        //   std::cout<<"Qs_Y="<<Qsy0<<"   Qs_X="<<Qsx0<<"   Qs_Z="<<Qsz0<<"   Qe_Y="<<Qey0 <<"   Qe_Z="<<Qwx0<<"   Qw_y="<<Qwy0<<"   Qw_Z="<<Qwz0<<std::endl;


    }
    else
        std::cout<<"nomotors"<<std::endl;

    double a1=10*pi/180; // (rad)
    double L1=0.24; // (m)
    double L2=0.3; // (m)
    double L3=0.2; // (m)
    double L4=0.06; // (m)
    double L5=0.12; // (m)
    //double Lee=0.05;
    //double Tc=2; // (s)
    //
    double Qs_Xrad;
    double Qs_Yrad;
    double Qs_Zrad;
    double Qe_Yrad;
    double Qw_Xrad;
    double Qw_Yrad;
    double Qw_Zrad;

    //
    double QsxL=-5*pi/180;
    double QsxH=180*pi/180;
    double QsyL=-180*pi/180;
    double QsyH=40*pi/180;
    double QszL=-180*pi/180;
    double QszH=180*pi/180;
    double QeyL=-120*pi/180;
    double QeyH=0*pi/180;
    double QwzL=-180*pi/180;
    double QwzH=180*pi/180;
    double QwyL=-180*pi/180;
    double QwyH=180*pi/180;
    double QwxL=-22.5*pi/180;
    double QwxH=180*pi/180;


    double X=x;
    double Y=y;
    double Z=z;
    double A=a*pi/180;
    double B=b*pi/180;
    double G=g*pi/180;

    /////reseting the refrence

    upperbodyclient::motor_data pos1 = gClinet->getMotor_leftShoulderYaw();
    double PR1=pos1.position;

    upperbodyclient::motor_data pos2 = gClinet->getMotor_leftShoulderPitch();
    double PR2=pos2.position;

    upperbodyclient::motor_data pos3 = gClinet->getMotor_leftShoulderRoll();
    double PR3=pos3.position;

    upperbodyclient::motor_data pos4 = gClinet->getMotor_leftElbowPitch();
    double PR4=pos4.position;

    upperbodyclient::motor_data pos5 = gClinet->getMotor_leftElbowRoll();
    double PR5=pos5.position;

    upperbodyclient::motor_data pos6 = gClinet->getMotor_leftWristPitch();
    double PR6=pos6.position;

    upperbodyclient::motor_data pos7 = gClinet->getMotor_leftWristRoll();
    double PR7=pos7.position;

    std::cout<<"refrence positions are="<<PR1<<std::endl<<PR2<<std::endl<< PR3<<std::endl<< PR4<<std::endl<<PR5<<std::endl<<PR6<<std::endl<<PR7<<std::endl <<std::endl ;


    int Error;
    int Flag=0;

    for (double Qwx=QwxL;Qwx<=QwxH;Qwx+=1*pi/180){

        // IK
        Eigen::MatrixXd P1l;
        Eigen::MatrixXd Peel;
        Eigen::MatrixXd Pl;

        double w[3]= {0,L1/cos(a1),0};
        P1l=RotTrans('X',a1)*RotTrans('P', w); // Shoulder Transfer Matrix with Respect to Base

        double v[3]= {X,Y,Z};
        Peel=RotTrans('P',v)*RotTrans('X',A)*RotTrans('Y',B)*RotTrans('Z',G); // Endeffector Transfer Matrix with Respect to Base
        Pl=Peel.inverse()*P1l;

        double  X1=Pl(0,3);
        double  Y1=Pl(1,3);
        double  Z1=Pl(2,3);
        double  Qey=-acos((pow(X1,2)+pow(Y1-L4*sin(Qwx),2)+pow(Z1-L5-L4*cos(Qwx),2)-pow(L2,2)-pow(L3,2))/(2*L2*L3));
        double  t1=(Y1*cos(Qwx)-Z1*sin(Qwx)+L5*sin(Qwx))/L2; // t1=sin(Qey)*sin(Qwz)
        double  Qwz=asin(t1/sin(Qey));
        //
        double C1=-L3-L2*cos(Qey);
        double C2=-L2*cos(Qwz)*sin(Qey);
        double Qwy=asin(-X1/pow((pow(C1,2)+pow(C2,2)),0.5))-atan(C2/C1);

        Eigen::MatrixXd P2l;
        double g1[3]={0,0,-L2};
        double g2[3]={0,0,-L3};
        double g3[3]={0,0,-L4};
        double g4[3]={0,0,-L5};
        P2l=RotTrans('P',g1)*RotTrans('Y',Qey)*RotTrans('P',g2)*RotTrans('Z',Qwz)*RotTrans('Y',Qwy)*RotTrans('P',g3)*RotTrans('X',Qwx)*RotTrans('P',g4); // Endeffector Transfer Matrix with Respect to Shoulder
        Eigen::MatrixXd f2=RotTrans('Y',Qwy);

        Eigen::MatrixXd Px;
        Px=P1l.inverse()*Peel*P2l.inverse();

        double Qsx=asin(-Px(1,2));
        double Qsy=atan(Px(0,2)/Px(2,2));
        double Qsz=atan(Px(1,0)/Px(1,1));

        Eigen::MatrixXd P3l;
        double h1[3]={0,L1/cos(a1),0};
        double h2[3]={0,0,-L2};
        double h3[3]={0,0,-L3};
        double h4[3]={0,0,-L4};
        double h5[3]={0,0,-L5};
        P3l=RotTrans('X',a1)*RotTrans('P',h1)*RotTrans('Y',Qsy)*RotTrans('X',Qsx)*RotTrans('Z',Qsz)*RotTrans('P',h2)*RotTrans('Y',Qey)*RotTrans('P',h3)*RotTrans('Z',Qwz)*RotTrans('Y',Qwy)*RotTrans('P',h4)*RotTrans('X',Qwx)*RotTrans('P',h5);
        double Xee=P3l(0,3);
        double Yee=P3l(1,3);
        double Zee=P3l(2,3);
        //Y

        double aa=pow(pow((Xee-X),2)+pow((Yee-Y),2)+pow((Zee-Z),2),0.5); //norm

        if (aa>1e-6)
        {
            Error=1;
            //     disp('IK has Error!');
        }
        else if (std::isnan(Qey)||std::isnan(Qwz)||std::isnan(Qwy)||std::isnan(Qsx)||std::isnan(Qsy)||std::isnan(Qsz)||std::isinf(Qey)||std::isinf(Qwz)||std::isinf(Qwy)||std::isinf(Qsx)||std::isinf(Qsy)||std::isinf(Qsz))
        {
            Error=2;
        }
        //     disp('Not Reachable!')
        else if ((QsxL>Qsx)||(Qsx>QsxH)||(QsyL>Qsy)||(Qsy>QsyH)||(QszL>Qsz)||(Qsz>QszH)||(QeyL>Qey)||(Qey>QeyH)||(QwxL>Qwx)||(Qwx>QwxH)||(QwyL>Qwy)||(Qwy>QwyH)||(QwzL>Qwz)||(Qwz>QwzH))
        {
            Error=3;
        }
        //     disp('Joint Limit!')
        else
        {
            Error=0;
        }

        if (Error==0){
            if (Flag==0){

                Qs_Xrad=Qsx;
                Qs_Yrad=Qsy;
                Qs_Zrad=Qsz;
                Qe_Yrad=Qey;
                Qw_Zrad=Qwz;
                Qw_Yrad=Qwy;
                Qw_Xrad=Qwx;


                Flag=1;
            }
            else{

                double AngleMovement=fabs(Qsx0-Qsx)+fabs(Qsy0-Qsy)+fabs(Qsz0-Qsz)+fabs(Qey0-Qey)+fabs(Qwx0-Qwx)+fabs(Qwy0-Qwy)+fabs(Qwz0-Qwz);
                double AngleMovement1=fabs(Qsx0-Qs_Xrad)+fabs(Qsy0-Qs_Yrad)+fabs(Qsz0-Qs_Zrad)+fabs(Qey0-Qe_Yrad)+fabs(Qwx0-Qw_Xrad)+fabs(Qwy0-Qw_Yrad)+fabs(Qwz0-Qw_Zrad);

                if (AngleMovement<AngleMovement1){
                    Qs_Xrad=Qsx;Qs_Yrad=Qsy;Qs_Zrad=Qsz;Qe_Yrad=Qey;Qw_Xrad=Qwx;Qw_Yrad=Qwy;Qw_Zrad=Qwz;

                }
            }
        }
    }

    if (Flag==0)
    {
        std::cout << "There is not any answer."<<std::endl;
    }
    std::cout<<"desired angles are="<<Qs_Yrad<<std::endl<<Qs_Xrad<<std::endl<< Qs_Zrad<<std::endl<< Qe_Yrad<<std::endl<< Qw_Zrad<<std::endl<< Qw_Yrad<<std::endl<<Qw_Xrad<<std::endl <<std::endl ;


    double Time=2;

    double Qs_Ysend=((Qs_Yrad*180/pi)/AsY)+BsY;
    int speeds_Y=((fabs(Qs_Ysend-PR1)*fabs(AsY)/6)/Time)/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    double Qs_Xsend=((Qs_Xrad*180/pi)/AsX)+BsX;
    int speeds_X=((fabs(Qs_Xsend-PR2)*fabs(AsX)/6)/Time)/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    double Qs_Zsend=((Qs_Zrad*180/pi)/AsZ)+BsZ;
    int speeds_Z=((fabs(Qs_Zsend-PR3)*fabs(AsZ)/6)/Time)/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    double Qe_Ysend=((Qe_Yrad*180/pi)/AeY)+BeY;
    int speede_Y=((fabs(Qe_Ysend-PR4)*fabs(AeY)/6)/Time)/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    double Qw_Zsend=((Qw_Zrad*180/pi)/AwZ)+BwZ;
    int speedw_Z=((fabs(Qw_Zsend-PR5)*fabs(AwZ)/6)/Time)/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    double Qw_Ysend=((Qw_Yrad*180/pi)/AwY)+BwY;
    int speedw_Y=((fabs(Qw_Ysend-PR6)*fabs(AwY)/6)/Time)/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    double Qw_Xsend=((Qw_Xrad*180/pi)/AwX)+BwX;
    int speedw_X=((fabs(Qw_Xsend-PR7)*fabs(AwX)/6)/Time)/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);

    std::cout<<"motor sent commands are="<<std::endl<<Qs_Ysend<<std::endl<<Qs_Xsend<<std::endl<<Qs_Zsend<<std::endl<<Qe_Ysend<<std::endl<<Qw_Zsend<<std::endl<<Qw_Ysend<<std::endl<<Qw_Xsend<<std::endl<<std::endl ;

    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));


    upperbodyclient::motor_data MssY = gClinet->getMotor_leftShoulderYaw();
    Qsy0 =AsY*(MssY.position-BsY)*pi/180;
    double bb=MssY.position;

    upperbodyclient::motor_data MssX = gClinet->getMotor_leftShoulderPitch();
    Qsx0 =AsX*(MssX.position-BsX)*pi/180;
    double aa=MssX.position;


    upperbodyclient::motor_data MssZ = gClinet->getMotor_leftShoulderRoll();
    Qsz0 =AsZ*(MssZ.position-BsZ)*pi/180;
    double cc=MssZ.position;

    upperbodyclient::motor_data MeeY = gClinet->getMotor_leftElbowPitch();
    Qey0 =AeY*(MeeY.position-BeY)*pi/180;
    double dd=MeeY.position;

    upperbodyclient::motor_data MwwZ = gClinet->getMotor_leftElbowRoll();
    Qwz0 = AwZ*(MwwZ.position-BwZ)*pi/180;
    double ee=MwwZ.position;

    upperbodyclient::motor_data MwwY = gClinet->getMotor_leftWristPitch();
    Qwy0 = AwY*(MwwY.position-BwY)*pi/180;
    double ff=MwwY.position;

    upperbodyclient::motor_data MwwX = gClinet->getMotor_leftWristRoll();
    Qwx0 = AwX*(MwwX.position-BwX)*pi/180;
    double gg=MwwX.position;

    std::cout<<"motor read positions are="<<std::endl<<bb<<std::endl<<aa<<std::endl<<cc<<std::endl<<dd<<std::endl<<ee<<std::endl<<ff<<std::endl<<gg<<std::endl <<std::endl;

    //FK

    Eigen::MatrixXd P4l;
    double k1[3]={0,L1/cos(a1),0};
    double k2[3]={0,0,-L2};
    double k3[3]={0,0,-L3};
    double k4[3]={0,0,-L4};
    double k5[3]={0,0,-L5};
    P4l=RotTrans('X',a1)*RotTrans('P',k1)*RotTrans('Y',Qsy0)*RotTrans('X',Qsx0)*RotTrans('Z',Qsz0)*RotTrans('P',k2)*RotTrans('Y',Qey0)*RotTrans('P',k3)*RotTrans('Z',Qwz0)*RotTrans('Y',Qwy0)*RotTrans('P',k4)*RotTrans('X',Qwx0)*RotTrans('P',k5);
    double Xfk=P4l(0,3);
    double Yfk=P4l(1,3);
    double Zfk=P4l(2,3);
    std::cout<<std::endl<<Xfk<<std::endl<<Yfk<<std::endl<<Zfk<<std::endl;


    double fasele=pow(pow(Xfk-X,2)+pow(Yfk-Y,2)+pow(Zfk-Z,2),0.5);
    std::cout<<std::endl<<"fasele is ="<<fasele<<std::endl;
    if (fasele>0.03)
        std::cout<<std::endl<<"not in domain"<<std::endl;

    ////checking the motors position

    upperbodyclient::motor_data CP1 = gClinet->getMotor_leftShoulderYaw();
    double cp1=MssY.position;

    upperbodyclient::motor_data CP2 = gClinet->getMotor_leftShoulderPitch();
    double cp2=MssX.position;


    upperbodyclient::motor_data CP3 = gClinet->getMotor_leftShoulderRoll();
    double cp3=MssZ.position;

    upperbodyclient::motor_data CP4 = gClinet->getMotor_leftElbowPitch();
    double cp4=MeeY.position;

    upperbodyclient::motor_data CP5 = gClinet->getMotor_leftElbowRoll();
    double cp5=MwwZ.position;

    upperbodyclient::motor_data CP6 = gClinet->getMotor_leftWristPitch();
    double cp6=MwwY.position;

    upperbodyclient::motor_data CP7 = gClinet->getMotor_leftWristRoll();
    double cp7=MwwX.position;

    if (cp1>2750 || cp1<1100){
        std::cout<<std::endl<<"angle cp1 out of domain"<<std::endl;
    }
    if (cp2>2000 || cp2<807){
        std::cout<<std::endl<<"angle cp2 out of domain"<<std::endl;
    }
    if (cp3>3260 || cp3<1500){
        std::cout<<std::endl<<"angle cp3 out of domain"<<std::endl;
    }
    if (cp4>2515 || cp4<1356){
        std::cout<<std::endl<<"angle cp4 out of domain"<<std::endl;
    }
    if (cp5>2350 || cp5<100){
        std::cout<<std::endl<<"angle cp5 out of domain"<<std::endl;
    }
    if (cp6>2200 || cp6<400){
        std::cout<<std::endl<<"angle cp6 out of domain"<<std::endl;
    }
    if (cp7>2255 || cp7<1400){
        std::cout<<std::endl<<"angle cp7 out of domain"<<std::endl;
    }

    if (Flag==0)
    {
        //std::cout << "There is not any answer. [Error!] "<<std::endl;

        if ( Error == 1)
            std::cout << "There is not any answer. IK has Error!"<<std::endl;
        if ( Error == 2)
            std::cout << "There is not any answer. Not Reachable!"<<std::endl;
        if ( Error == 3)
            std::cout << "There is not any answer. Joint Limit!"<<std::endl;

        return Error;
    }
    else
    {
        return 0;
    }

}

int grip_process_left(double xend,double yend,double zend)
{

    double al;
    double be;
    double ga;

    //     double Xkinect=0.14;
    //     double Ykinect=0.16;
    //     double Zkinect=0.66;


    double Qgrip_send=2200;
    double speed_grip=150;

    gClinet->setMotor_leftGripper(Qgrip_send,speed_grip);

    //double Xend=0.5;
    //double Yend=-0.35;
    //double Zend=-0.2;

    //for kinect axes
    double Yend=-xend;
    double Zend=-yend;
    double Xend= zend;

    // for my axes
    //     double Yend=yend;
    //     double Zend=zend;
    //     double Xend=xend;

    double KinOffX=0.16;
    double KinOffY=0.05;
    //TODO change this if we dont have any move forward
    double RobotGo=0; //if Robot moves forward

    double teta=0;// rotation of torso
    //double RobotUp=0.09;

    double Zoffset=0;
    double Yoffset=-0.02;
    double L1=0.24;

    double Xk=L1*sin(teta*pi/180)*cos(10*pi/180);
    double Yk=L1*(1-cos(teta*pi/180))*cos(10*pi/180);

    std::cout<<std::endl<<"Xk is = "<<Xk<<std::endl;
    std::cout<<std::endl<<"Yk is = "<<Yk<<std::endl;

    double Xg=Xend+KinOffX-RobotGo;
    double Yg=Yend+Yoffset+KinOffY;
    double Z=Zend+Zoffset;  //-RobotUp;



    double X=Xg*cos(teta*pi/180)+Yg*sin(teta*pi/180)-Xk;
    double Y=-Xg*sin(teta*pi/180)+Yg*cos(teta*pi/180)-Yk;

    if (Yg>=0.2)
        ga=0;
    else if(Yg<0.20 && Yg>=0.3)
        ga=-10;
    else if (Yg<0.3)
        ga=-30;

    if (Z>0)
        be=-110;
    else if (Z<-0.3)
        be=-70;
    else
        be=-90;

    //default grip
    al = 0;

    double alpha=al;
    double betha=be;
    double gamma=ga-teta;

    std::cout<<std::endl<<"X is = "<<X<<std::endl;
    std::cout<<std::endl<<"Y is = "<<Y<<std::endl;
    std::cout<<std::endl<<"Z is = "<<Z<<std::endl;
    std::cout<<std::endl<<"alpha is = "<<alpha<<std::endl;
    std::cout<<std::endl<<"betha is = "<<betha<<std::endl;
    std::cout<<std::endl<<"gamma is = "<<gamma<<std::endl;

    double A=(atan2(cos(gamma*pi/180)*sin(alpha*pi/180)-cos(alpha*pi/180)*sin(betha*pi/180)*sin(gamma*pi/180),cos(alpha*pi/180)*cos(betha*pi/180)))*180/pi;
    double B=(asin(sin(alpha*pi/180)*sin(gamma*pi/180)+cos(alpha*pi/180)*cos(gamma*pi/180)*sin(betha*pi/180)))*180/pi;
    double G=(atan2(cos(alpha*pi/180)*sin(gamma*pi/180)-cos(gamma*pi/180)*sin(alpha*pi/180)*sin(betha*pi/180),cos(betha*pi/180)*cos(gamma*pi/180)))*180/pi;
    std::cout<<std::endl<<"angles"<<A<<std::endl<<B<<std::endl<<G<<std::endl;



    grip_left(0.3,0.42,-0.22,0,-90,0);



    double a=0.15*cos((gamma+teta)*pi/180);
    double b=0.15*sin((gamma+teta)*pi/180);
    grip_left(X-a,Y-b+0.05,Z,A,B,G);

    // boost::this_thread::sleep(boost::posix_time::millisecon5ds(1000));

    grip_left(X,Y,Z,A,B,G);

    Qgrip_send=1700;
    speed_grip=150;
    gClinet->setMotor_leftGripper(Qgrip_send,speed_grip);

    boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
    grip_left(X+0.05,Y,Z+0.1,A,B,G);

    grip_left(0.3,0.42,-0.22,0,-90,0);

    return 0;
}

//=========================== GRIP IK FK [RIGHT|LEFT]

bool callback_grip(upperbodycore_msgs::grip::Request& request, upperbodycore_msgs::grip::Response& response)
{
    end_position.x = request.x;
    end_position.y = request.y;
    end_position.z = request.z;
    end_position.a = request.a;
    end_position.b = request.b;
    end_position.g = request.g;

    std::cout<<"get"<<std::endl;
    std::cout<<end_position.x<<std::endl;
    std::cout<<end_position.y<<std::endl;
    std::cout<<end_position.z<<std::endl;
    std::cout<<end_position.a<<std::endl;
    std::cout<<end_position.b<<std::endl;
    std::cout<<end_position.g<<std::endl;

    if ( request.right_left == "right" || request.right_left == "left" )
    {
        right_left = request.right_left;
    }
    else
    {
        //set default
        right_left = "right";
    }


    start_grip = true;

    return true;
}

std::vector<std::string> amg_vector;
int first_time = 1;

bool callback_amg(upperbodycore_msgs::grip::Request& request, upperbodycore_msgs::grip::Response& response)
{
    cout<<coutcolor_green<<"AMG GET RAW COMMAND :"<<request.right_left<<coutcolor0<<std::endl;

    if ( request.right_left == "XSTART" )
    {
        first_time = 1;
    }
    else
    {
        amg_vector.push_back(request.right_left);
    }
}

void grip_thread()
{

    while (ros::ok() && appExit == false )
    {
        if ( start_grip )
        {
            grip_busy = true;

            int result = 0;

            if ( right_left == "right")
                result = grip_process_right(end_position.x,end_position.y,end_position.z);
            else if ( right_left == "left")
                result = grip_process_left(end_position.x,end_position.y,end_position.z);

            if ( result == 0 )
                std::cout<<"ok"<<std::endl;
            if ( result == 1 )
                std::cout<<"IK has Error"<<std::endl;
            if ( result == 2 )
                std::cout<<"Not Reachable"<<std::endl;
            if ( result == 3 )
                std::cout<<"Joint Limit"<<std::endl;

            start_grip = false;

            grip_busy = false;
        }
        else
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            std::cout<<coutcolor_magenta<<"wait for request"<<coutcolor0<<std::endl;
        }
    }

}

void amg_thread()
{

    while (ros::ok() && appExit == false )
    {
        if ( amg_vector.size() != 0 )
        {
            if ( first_time == 1 )
            {
                first_time = 0;
                //cout<<coutcolor_red<<"WAIT..."<<coutcolor0<<endl;
                //boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

            }

            std::vector<std::string> strs;
            boost::algorithm::split(strs,amg_vector.at(0),boost::algorithm::is_any_of("_"));

            amg_vector.erase(amg_vector.begin());

            string command = strs[0];

            if ( command == "Flat" || command == "FlatReverse" )
            {
                string type = strs[1];
                string v = strs[2];
                string d = strs[3];

                string vv = v.substr(1);
                string dd = d.substr(1);

                int vvv = boost::lexical_cast<int>(vv);
                int ddd = boost::lexical_cast<int>(dd);

                cout<<"param :"<<vvv<<" "<<ddd<<endl;

                if ( type == "start" )
                {
                    start(ddd);
                }

                if ( type == "cycle" )
                {
                    cycle(vvv,ddd);
                }

                if ( type == "end" )
                {
                    end();
                }
            }

            if ( command == "Global" )
            {
                string type = strs[1];

                if ( type == "start")
                {
                    Global_start();
                }

                if ( type == "end")
                {
                    Global_end();
                }
            }

            if ( command == "Round" )
            {
                string dir = strs[1];
                string type = strs[2];

                string v = strs[3];
                string d = strs[4];

                string vv = v.substr(1);
                string dd = d.substr(1);

                int rrr = boost::lexical_cast<int>(vv);
                int ttt = boost::lexical_cast<int>(dd);

                if ( dir == "Right" )
                {
                    if ( type == "start")
                    {
                         start_left(30); //?
                    }
                    if ( type == "cycle")
                    {
                         cycle_left(20,30);
                    }
                    if ( type == "end" )
                    {
                         end();
                    }
                }

                if ( dir == "Left" )
                {

                    if ( type == "start")
                    {
                         start(30);
                    }
                    if ( type == "cycle")
                    {
                         cycle(20,30);
                    }
                    if ( type == "end" )
                    {
                         end();
                    }
                }

            }

            if ( command == "stair")
            {
                string dir = strs[1];

                if ( dir == "down" )
                {
                    start(35);
                    stair_down();
                    end();
                }
            }

            if ( command == "Slope")
            {
                string dir = strs[1];

                if ( dir == "up")
                {
                    string type = strs[2];

                    string v = strs[3];
                    string d = strs[4];

                    string vv = v.substr(1);
                    string dd = d.substr(1);

                    int vvv = boost::lexical_cast<int>(vv);
                    int ddd = boost::lexical_cast<int>(dd);

                    if ( type == "start")
                    {
                        start(ddd);
                        cycle(20,ddd);
                    }

                    if ( type == "cycle" )
                    {
                        cycle(20,ddd);
                    }

                    if ( type == "end" )
                    {
                        cycle(20,ddd);
                        end();
                    }
                }
            }

            if ( command == "show")
            {
                string dir = strs[1];

                if ( dir == "sitting")
                {
                    sitting();
                }

                if ( dir == "compound")
                {
                    compound();
                }

                if ( dir == "shooting")
                {
                    shooting();
                }

                if ( dir == "footforward")
                {
                    foot_forward_body_backward();
                }
            }

            if ( command == "online")
            {
                string dir = strs[1];

                if ( dir == "walking")
                {
                    online();
                }

                if ( dir == "shooting")
                {
                    shooting();
                }
            }
        }
        else
        {

            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }
    }
}

//=========================== MAIN

int main(int argc, char **argv)
{
    boost::thread _thread_logic1(&grip_thread);
    boost::thread _thread_logic2(&amg_thread);

    ros::init(argc, argv, "upperbody_client");
    std::cout<<"upperbody client started"<<std::endl;

    ros::Time::init();
    ros::Rate loop_rate(20);

    upperbodyclient Client;
    gClinet = &Client;

    ros::NodeHandle n,n1,n2;

    service_grip = n.advertiseService("/grip/position", callback_grip);
    service_amg =  n.advertiseService("/grip/amg", callback_amg);

    pub_grip_busy = n2.advertise<std_msgs::String>("/grip/busy",10);

    chatter_pub_ack = n.advertise<std_msgs::String>("/core_ikfk/ack",10);
    chatter_pub_log = n.advertise<std_msgs::String>("/core_ikfk/log",10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        std_msgs::String msg_string;

        msg_string.data = "false";
        if ( grip_busy )
            msg_string.data = "true";

        pub_grip_busy.publish(msg_string);
        send_ack();
    }

    appExit = true;
    _thread_logic1.interrupt();
    _thread_logic1.join();

    _thread_logic2.interrupt();
    _thread_logic2.join();

    return 0;
}

