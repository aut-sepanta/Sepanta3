#include "upperbodyclient.h"
#include <upperbodycore_msgs/objectsposition.h>
#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <math.h>

bool appExit = false;
upperbodyclient *gClinet;
ros::Subscriber sub_position_objects;
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





    std::cout<<"motor sent commands are="<<std::endl<<Qs_Ysend<<std::endl<<Qs_Xsend<<std::endl<<Qs_Zsend<<std::endl<<Qe_Ysend<<std::endl<<Qw_Zsend<<std::endl<<Qw_Ysend<<std::endl<<Qw_Xsend<<std::endl<<std::endl ;

}


int start(int d)
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
    double LH1=1424;
    double LH2=905;
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

}


int end()
{
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

    double gl1=1424;
    double gl2=905;
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


    std::cout<<"3"<<std::endl;
}


///
int Global_start()
{

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

    double gl1=1424;
    double gl2=905;
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
    double LH1=1424;
    double LH2=905;
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

}



int stair_down(float v,float d)
{
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




    std::cout<<"motor sent commands are="<<std::endl<<Qs_Ysend<<std::endl<<Qs_Xsend<<std::endl<<Qs_Zsend<<std::endl<<Qe_Ysend<<std::endl<<Qw_Zsend<<std::endl<<Qw_Ysend<<std::endl<<Qw_Xsend<<std::endl<<std::endl ;

}

/////////////////


int sitting()
{
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
    //    QWaist_send=WL;
    //    speedW=((fabs(Qw_Xsend-WP)*fabs(AwXl)/6)/(1000))/0.01;
    //    gClinet->setMotor_WaistRoll(QWaist_send,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    //    QWaist_send=WR;
    //    speedW=((fabs(Qw_Xsend-WL)*fabs(AwXl)/6)/(2000))/0.01;
    //    gClinet->setMotor_WaistRoll(QWaist_send,speedw_X);
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    //    QWaist_send=WP;
    //    speedW=((fabs(Qw_Xsend-WR)*fabs(AwXl)/6)/(1000))/0.01;
    //    gClinet->setMotor_WaistRoll(QWaist_send,speedw_X);
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

    //        QWaist_send=WL;
    //        speedW=((fabs(Qw_Xsend-WP)*fabs(AwXl)/6)/(Time3))/0.01;
    //        gClinet->setMotor_WaistRoll(QWaist_send,speedw_X);




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

    //    QWaist_send=WP;
    //    speedW=((fabs(Qw_Xsend-WL)*fabs(AwXl)/6)/(1000))/0.01;
    //    gClinet->setMotor_WaistRoll(QWaist_send,speedw_X);

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

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


    double Qs_Ysend=2400;
    int speeds_Y=((fabs(Qs_Ysend-LH1)*fabs(AsYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speeds_Y);

    double Qs_Xsend=2000;
    int speeds_X=((fabs(Qs_Xsend-LH2)*fabs(AsXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speeds_X);

    double Qs_Zsend=3484;
    int speeds_Z=((fabs(Qs_Zsend-LH3)*fabs(AsZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speeds_Z);

    double Qe_Ysend=1449;
    int speede_Y=((fabs(Qe_Ysend-LH4)*fabs(AeYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speede_Y);

    double  Qw_Zsend=1000;
    int speedw_Z=((fabs(Qw_Zsend-LH5)*fabs(AwZl)/6)/(Time))/0.01;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speedw_Z);

    double Qw_Ysend=1280;
    int speedw_Y=((fabs(Qw_Ysend-LH6)*fabs(AwYl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speedw_Y);

    double  Qw_Xsend=1995;
    int speedw_X=((fabs(Qw_Xsend-LH7)*fabs(AwXl)/6)/(Time))/0.01;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speedw_X);
    //right hand

    Qs_Ysend=2952;
    speeds_Y=((fabs(Qs_Ysend-RH1)*fabs(AsYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speeds_Y);

    Qs_Xsend=2385;
    speeds_X=((fabs(Qs_Xsend-RH2)*fabs(AsXr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speeds_X);

    Qs_Zsend=553;
    speeds_Z=((fabs(Qs_Zsend-RH3)*fabs(AsZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speeds_Z);

    Qe_Ysend=2260;
    speede_Y=((fabs(Qe_Ysend-RH4)*fabs(AeYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speede_Y);

    Qw_Zsend=3650;
    speedw_Z=((fabs(Qw_Zsend-RH5)*fabs(AwZr)/6)/(Time))/0.01;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speedw_Z);

    Qw_Ysend=1100;
    speedw_Y=((fabs(Qw_Ysend-RH6)*fabs(AwYr)/6)/(Time))/0.01;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speedw_Y);

    Qw_Xsend=2680;
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
///

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


/////


bool start_grip = false;



void grip_thread()
{

    //    while (ros::ok() && appExit == false )
    //    {
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    //   int result = Global_start();

    //        if(==Flat || ==Flat-revers){
    //         int result = start(45);

       //       int result =cycle(70,45);

             int result = end();
    //        }

    //        if(==round)
    //            if(==Right){
    //  int result = start_left(45);

    //    int result =cycle_left(70,45);
    //    int result = end();
    //            }
    //       if(==left){

    //           int result = start(45);

    //            int result =cycle(20,30);

    //            int result = end();
    //        }
    //       if(==slope_up){
    //    if(==start){
    //     int result = start(30);

    //  int result =cycle(20,30);
    //}

    // if(==cycle)
    //  int result =cycle(20,30);

    //  if(==end){
    //int result =cycle(20,30);
    //int result = end();
    //}
    //}
    //       if (==stair_down)
    //  int result = start(35);
    //  int result = stair_down(20,35);
    // int result = end();

    //         int result = sitting();

   //  int result= compound();

   // int result= foot_forward_body_backward();
    // if (==online_shooting || show_shooting)
    // int result= shooting();

    // if (==online)
    // int result= Global_start();
    //   int result= start(35);
    //  int result= online();
    //  int result= end();
    //int result= core();
    ///////////////////////////////////////////////////
    if ( result == 0 )
        std::cout<<"ok"<<std::endl;
    if ( result == 1 )
        std::cout<<"IK has Error"<<std::endl;
    if ( result == 2 )
        std::cout<<"Not Reachable"<<std::endl;
    if ( result == 3 )
        std::cout<<"Joint Limit"<<std::endl;

    start_grip = false;

    //  }

}

//_+_+_+_+_+______________________________________objrct __________________

//_+_+_+_+_+_+______________________________________________________________

int main(int argc, char **argv)
{
    boost::thread _thread_logic(&grip_thread);

    ros::init(argc, argv, "upperbody_client");
    std::cout<<"upperbody client started"<<std::endl;

    ros::Time::init();
    ros::Rate loop_rate(20);

    upperbodyclient Client;
    gClinet = &Client;

    ros::NodeHandle n,n1;
    //service_grip = n.advertiseService("/grip/position", callback_grip);
    //sub_position_objects=n1.subscribe("/OBJECTOUT_objectposition",1,callback_positionobjects);


    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        //your code here
        //get motor feedback
        //logic2();
    }

    appExit = true;
    _thread_logic.interrupt();
    _thread_logic.join();

    return 0;
}


