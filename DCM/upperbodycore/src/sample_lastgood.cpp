#include "upperbodyclient.h"
#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <math.h>


ros::ServiceServer service_grip;
bool start_grip = false;


struct grip_position
{
public:
    float x;
    float y;
    float z;
    float a;
    float b;
    float g;
};

grip_position end_position;

bool appExit = false;
upperbodyclient *gClinet;

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

void logic2()
{
    /*
    if ( gClinet->allMotors.size() != 0 )
    {
       upperbodyclient::motor_data x = gClinet->getMotor_headYaw();
       int a =  x.position;

       std::cout<<a<<std::endl;
    }
    else
        std::cout<<"no motors"<<std::endl;
*/

}


double Qsx0=0;
double Qsy0=0;
double Qsz0=0;
double Qey0=0;
double Qwz0=0;
double Qwy0=0;
double Qwx0=0;
//============================

int grip(float x,float y,float z,float a,float b,float g)
{
    //  boost::this_thread::sleep(boost::posix_time::milliseconds(3000));


   // boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

std::cout<<"X:"<< x<<std::endl;
std::cout<<"y:"<< y<<std::endl;
std::cout<<"z:"<< z<<std::endl;
std::cout<<"a:"<< a<<std::endl;
std::cout<<"b:"<< b<<std::endl;
std::cout<<"g:"<< g<<std::endl;

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
    gClinet->setMotorPid_rightElbowPitch(3,35,0);
    gClinet->setMotorPid_rightShoulderPitch(3,35,0);
    gClinet->setMotorPid_rightShoulderRoll(3,35,0);
    gClinet->setMotorPid_rightShoulderYaw(3,35,0);
  //  gClinet->setMotorPid_rightElbowPitch(3,35,0);
    gClinet->setMotorPid_rightElbowRoll(3,35,0);
    gClinet->setMotorPid_rightWristPitch(0,35,0);
    gClinet->setMotorPid_rightWristRoll(0,35,0);


    gClinet->setMotorPid_leftShoulderPitch(3,35,0);
    gClinet->setMotorPid_leftShoulderRoll(3,35,0);
    gClinet->setMotorPid_leftShoulderYaw(3,35,0);
    gClinet->setMotorPid_leftElbowPitch(3,35,0);
    gClinet->setMotorPid_leftElbowRoll(3,35,0);
    gClinet->setMotorPid_leftWristPitch(0,35,0);
    gClinet->setMotorPid_leftWristRoll(0,35,0);

    /*
   // if (left) {
    if ( gClinet->allMotors.size() != 0 )
    {

        AsY = (float)-180/2048;
        BsY = 1415;
        upperbodyclient::motor_data MsY = gClinet->getMotor_leftShoulderYaw();
        Qsy0 =0;//AsY*(MsY.position-BsY)*pi/180;

        AsX = (float)-180/2048;
        BsX = 2914;
        upperbodyclient::motor_data MsX = gClinet->getMotor_leftShoulderPitch();
        Qsx0 =0;//AsX*(MsX.position-BsX)*pi/180;

        AsZ = (float)-180/2048;
        BsZ = 2442;
        upperbodyclient::motor_data MsZ = gClinet->getMotor_leftShoulderRoll();
        Qsz0 =0;//AsZ*(MsZ.position-BsZ)*pi/180;

        AeY = (float)-180/2048;
        BeY = 1900;
        upperbodyclient::motor_data MeY = gClinet->getMotor_leftElbowPitch();
        Qey0 =0;//AeY*(MeY.position-BeY)*pi/180;

        AwZ = (float)-150/512;
        BwZ = 520;
        upperbodyclient::motor_data MwZ = gClinet->getMotor_leftElbowRoll();
        Qwz0 = 0;//AeZ*(MwZ.position-BwZ)*pi/180;

        AwY = (float)180/2048;
        BwY = 1692;
        upperbodyclient::motor_data MwY = gClinet->getMotor_leftWristPitch();
        Qwy0 = 0;//AwY*(MwY.position-BwY)*pi/180;

        AwX = (float)150/512;
        BwX = 514;
        upperbodyclient::motor_data MwX = gClinet->getMotor_leftWristRoll();
        Qwx0 = 0;//AwZ*(MwX.position-BwX)*pi/180;

        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

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
    double QsxL=-10*pi/180;
    double QsxH=180*pi/180;
    double QsyL=-180*pi/180;
    double QsyH=180*pi/180;
    double QszL=-180*pi/180;
    double QszH=180*pi/180;
    double QeyL=-120*pi/180;
    double QeyH=0*pi/180;
    double QwzL=-180*pi/180;
    double QwzH=180*pi/180;
    double QwyL=-90*pi/180;
    double QwyH=90*pi/180;
    double QwxL=-120*pi/180;
    double QwxH=0*pi/180;

    //
    double X=0.45;
    double Y=0.4;
    double Z=-0.26;
    double A=0*pi/180;
    double B=-90*pi/180;
    double G=0*pi/180;
  //  std::cout<<"X?"<<std::endl;
   // std::cin>>X;
    //std::cout<<"Y?"<<std::endl;
    //std::cin>>Y;
    //std::cout<<"Z?"<<std::endl;
    //std::cin>>Z;
   // std::cout<<"A?"<<std::endl;
 //   std::cin>>A;
   // std::cout<<"B?"<<std::endl;
   // std::cin>>B;
    //std::cout<<"G?"<<std::endl;
    //std::cin>>G;
   // std::cout<<"X="<<X<<" Y="<<Y <<" Z="<<Z<<std::endl<<"A="<<A<<" B="<<B <<" G="<<G<<std::endl;

    //
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

double speed=5;
  double Qs_Ysend=((Qs_Yrad*180/pi)/AsY)+BsY;
  gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speed);


  double Qs_Xsend=((Qs_Xrad*180/pi)/AsX)+BsX;
  gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speed);


  double Qs_Zsend=((Qs_Zrad*180/pi)/AsZ)+BsZ;
  gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speed);

  double Qe_Ysend=((Qe_Yrad*180/pi)/AeY)+BeY;
  gClinet->setMotor_leftElbowPitch(Qe_Ysend,speed);

  double Qw_Zsend=((Qw_Zrad*180/pi)/AwZ)+BwZ;
  gClinet->setMotor_leftElbowRoll(Qw_Zsend,speed);

  double Qw_Ysend=((Qw_Yrad*180/pi)/AwY)+BwY;
  gClinet->setMotor_leftWristPitch(Qw_Ysend,speed);

  double Qw_Xsend=((Qw_Xrad*180/pi)/AwX)+BwX;
  gClinet->setMotor_leftWristRoll(Qw_Xsend,speed);

  std::cout<<"motor sent commands are="<<std::endl<<Qs_Ysend<<std::endl<<Qs_Xsend<<std::endl<<Qs_Zsend<<std::endl<<Qe_Ysend<<std::endl<<Qw_Zsend<<std::endl<<Qw_Ysend<<std::endl<<Qw_Xsend<<std::endl<<std::endl ;

  boost::this_thread::sleep(boost::posix_time::milliseconds(5000));


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

       double speed=5;
    double Qs_Ysend=((0*180/pi)/AsY)+BsY;
    gClinet->setMotor_leftShoulderYaw(Qs_Ysend,speed);

    double Qs_Xsend=((0*180/pi)/AsX)+BsX;
    gClinet->setMotor_leftShoulderPitch(Qs_Xsend,speed);

    double Qs_Zsend=((0*180/pi)/AsZ)+BsZ;
    gClinet->setMotor_leftShoulderRoll(Qs_Zsend,speed);

    double Qe_Ysend=((0*180/pi)/AeY)+BeY;
    gClinet->setMotor_leftElbowPitch(Qe_Ysend,speed);

    double Qw_Zsend=((0*180/pi)/AwZ)+BwZ;
    gClinet->setMotor_leftElbowRoll(Qw_Zsend,speed);

    double Qw_Ysend=((0*180/pi)/AwY)+BwY;
    gClinet->setMotor_leftWristPitch(Qw_Ysend,speed);

    double Qw_Xsend=((0*180/pi)/AwX)+BwX;
    gClinet->setMotor_leftWristRoll(Qw_Xsend,speed);


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
}

//
*/
    if ( gClinet->allMotors.size() != 0 )
    {

        AsY = (float)180/2048;
        BsY = 2726;
        upperbodyclient::motor_data MsY = gClinet->getMotor_rightShoulderYaw();
        //Qsy0 =0;//AsY*(MsY.position-BsY)*pi/180;

        AsX = (float)180/2048;
        BsX = 3420;
        upperbodyclient::motor_data MsX = gClinet->getMotor_rightShoulderPitch();
        //Qsx0 =0;//AsX*(MsX.position-BsX)*pi/180;

        AsZ = (float)-180/2048;
        BsZ = 1023;
        upperbodyclient::motor_data MsZ = gClinet->getMotor_rightShoulderRoll();
        //Qsz0 =0;//AsZ*(MsZ.position-BsZ)*pi/180;

        AeY = (float)-180/2048;
        BeY = 1630;
        upperbodyclient::motor_data MeY = gClinet->getMotor_rightElbowPitch();
        //Qey0 =0;//AeY*(MeY.position-BeY)*pi/180;

        AwZ = (float)-180/2048;
        BwZ = 1783;
        upperbodyclient::motor_data MwZ = gClinet->getMotor_rightElbowRoll();
        //Qwz0 = 0;//AeZ*(MwZ.position-BwZ)*pi/180;

        AwY = (float)-180/2048;
        BwY = 2420;
        upperbodyclient::motor_data MwY = gClinet->getMotor_rightWristPitch();
        //Qwy0 = 0;//AwY*(MwY.position-BwY)*pi/180;

        AwX = (float)180/2048;
        BwX = 2540;
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
    //
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

    std::cout<<"X:"<< X<<std::endl;
    std::cout<<"y:"<< Y<<std::endl;
    std::cout<<"z:"<< Z<<std::endl;
    std::cout<<"a:"<< A<<std::endl;
    std::cout<<"b:"<< B<<std::endl;
    std::cout<<"g:"<< G<<std::endl;

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
     double Qs_Xsend=3118;
     double Qs_Ysend=2729;
     double Qs_Zsend=644;
     double Qe_Ysend=2750;
     double Qw_Zsend=1995;
     double Qw_Ysend=2758;
     double Qw_Xsend=2791;
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

 // boost::this_thread::sleep(boost::posix_time::milliseconds(6000));
 // double Qgrip_send=1420;
 // double speed_grip=1;
 // gClinet->setMotor_rightGripper(Qgrip_send,speed_grip);
  std::cout<<"desired velocities are="<<speeds_Y<<std::endl<<speeds_X<<std::endl<<speeds_Z<<std::endl<< speede_Y<<std::endl<<speedw_Z<<std::endl<<speedw_Y<<std::endl<<speedw_X<<std::endl <<std::endl ;



  std::cout<<"motor sent commands are="<<std::endl<<Qs_Ysend<<std::endl<<Qs_Xsend<<std::endl<<Qs_Zsend<<std::endl<<Qe_Ysend<<std::endl<<Qw_Zsend<<std::endl<<Qw_Ysend<<std::endl<<Qw_Xsend<<std::endl<<std::endl ;

 boost::this_thread::sleep(boost::posix_time::milliseconds(2500));


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
/*
       double speed=5;
    double Qs_Ysend=((0*180/pi)/AsY)+BsY;
    gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speed);

    double Qs_Xsend=((0*180/pi)/AsX)+BsX;
    gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speed);

    double Qs_Zsend=((0*180/pi)/AsZ)+BsZ;
    gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speed);

    double Qe_Ysend=((0*180/pi)/AeY)+BeY;
    gClinet->setMotor_rightElbowPitch(Qe_Ysend,speed);

    double Qw_Zsend=((0*180/pi)/AwZ)+BwZ;
    gClinet->setMotor_rightElbowRoll(Qw_Zsend,speed);

    double Qw_Ysend=((0*180/pi)/AwY)+BwY;
    gClinet->setMotor_rightWristPitch(Qw_Ysend,speed);

    double Qw_Xsend=((0*180/pi)/AwX)+BwX;
    gClinet->setMotor_rightWristRoll(Qw_Xsend,speed);
*/
//setting the initial value of tetas
 // Qsy0 =bb;
  //Qsx0 =aa;
  //Qsz0 =cc;
 // Qey0 =dd;
 // Qwz0 =ee;
 // Qwy0 =ff;
  //Qwx0 =gg;
//FK

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

    //start_grip = true;

    int result = grip(end_position.x,end_position.y,end_position.z,end_position.a,end_position.b,end_position.g);
    if ( result == 0 )
        response.result = "ok";
    if ( result == 1 )
        response.result = "IK has Error";
    if ( result == 2 )
        response.result = "Not Reachable";
    if ( result == 3 )
        response.result = "Joint Limit";

    std::cout<<result<<std::endl;
    return true;
}

void logic()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    //first time
  // double speed=500;
  // double speed3=500;
  // double Qs_Yinit=2982;
  // gClinet->setMotor_rightShoulderYaw(Qs_Yinit,speed);

   //double Qs_Xinit=3320;
   //gClinet->setMotor_rightShoulderPitch(Qs_Xinit,speed);

   //double Qs_Zinit=1514;
   //gClinet->setMotor_rightShoulderRoll(Qs_Zinit,speed);

   //double Qe_Yinit=2900;
   //gClinet->setMotor_rightElbowPitch(Qe_Yinit,speed3);

   //double Qw_Zinit=3120;
   //gClinet->setMotor_rightElbowRoll(Qw_Zinit,speed);

  // double Qw_Yinit=2280;
  // gClinet->setMotor_rightWristPitch(Qw_Yinit,speed);

  // double Qw_Xinit=2780;
  // gClinet->setMotor_rightWristRoll(Qw_Xinit,speed);

   //double Qgrip_send=1840;
  // double speed_grip=1;
  // gClinet->setMotor_rightGripper(Qgrip_send,speed_grip);

//    boost::this_thread::sleep(boost::posix_time::milliseconds(6000));


//***
   //  double Xkinect=0.14;
   //  double Ykinect=0.16;
   //  double Zkinect=0.66;

   //  double Qgrip_send=1840;
   //  double speed_grip=100;
   //  gClinet->setMotor_rightGripper(Qgrip_send,speed_grip);

   //  double Yend=-Xkinect;
   //  double Zend=-Ykinect;
   //  double Xend=Zkinect;

   //  double KinOffX=0.16;
   //  double RobotGo=0.3;
   //  double teta=0;
   //  double RobotUp=0.09;
   //  double Zoffset=-0.02;
   //  double Yoffset=-0.02;
   //  double L1=0.24;

   //  double Xk=L1*sin(teta*pi/180)*cos(10*pi/180);
   //  double Yk=L1*(1-cos(teta*pi/180))*cos(10*pi/180);

   //  std::cout<<std::endl<<"Xk is = "<<Xk<<std::endl;
   //  std::cout<<std::endl<<"Yk is = "<<Yk<<std::endl;

   //  double Xg=Xend+KinOffX-RobotGo;
   //  double Yg=Yend+Yoffset;
   //  double Z=Zend-RobotUp+Zoffset;
   //  double alpha=0;
   //  double betha=-90;
   //  double gamma=30-teta;

   //  double X=Xg*cos(teta*pi/180)+Yg*sin(teta*pi/180)-Xk;
   //  double Y=-Xg*sin(teta*pi/180)+Yg*cos(teta*pi/180)-Yk;


   //  std::cout<<std::endl<<"X is = "<<X<<std::endl;
   //  std::cout<<std::endl<<"Y is = "<<Y<<std::endl;
   //  std::cout<<std::endl<<"Z is = "<<Z<<std::endl;



   //  double A=(atan2(cos(gamma*pi/180)*sin(alpha*pi/180)-cos(alpha*pi/180)*sin(betha*pi/180)*sin(gamma*pi/180),cos(alpha*pi/180)*cos(betha*pi/180)))*180/pi;
   //  double B=(asin(sin(alpha*pi/180)*sin(gamma*pi/180)+cos(alpha*pi/180)*cos(gamma*pi/180)*sin(betha*pi/180)))*180/pi;
   //  double G=(atan2(cos(alpha*pi/180)*sin(gamma*pi/180)-cos(gamma*pi/180)*sin(alpha*pi/180)*sin(betha*pi/180),cos(betha*pi/180)*cos(gamma*pi/180)))*180/pi;
   //  std::cout<<std::endl<<"angles"<<A<<std::endl<<B<<std::endl<<G<<std::endl;

   //  //grip(X-0.2,Y,Z-0.1,A,B,G);

   //  grip(0.3,-0.42,-0.25,0,-70,0);

   // // boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

   //  double a=0.15*cos((gamma+teta)*pi/180);
   //  double b=0.15*sin((gamma+teta)*pi/180);
   //  grip(X-a,Y-b,Z,A,B,G);

   // // boost::this_thread::sleep(boost::posix_time::millisecon5ds(1000));

   //  grip(X,Y,Z,A,B,G);

   //  Qgrip_send=1500;
   //  speed_grip=100;
   // gClinet->setMotor_rightGripper(Qgrip_send,speed_grip);

   // boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
   // grip(X-0.05,Y,Z+0.1,A,B,G);

   // //grip(X,Y,Z+0.1,A,B,G);

   //***
}


int main(int argc, char **argv)
{
    boost::thread _thread_logic(&logic);

    ros::init(argc, argv, "upperbody_client");
    std::cout<<"upperbody client started"<<std::endl;

    ros::Time::init();
    ros::Rate loop_rate(20);

    upperbodyclient Client;
    gClinet = &Client;

    ros::NodeHandle n;
    service_grip = n.advertiseService("/grip/position", callback_grip);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        //your code here
        //get motor feedback
        logic2();

    }

    appExit = true;
    _thread_logic.interrupt();
    _thread_logic.join();

    return 0;
}
