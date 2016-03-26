#include "upperbodyclient.h"
#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <upperbodycore_msgs/grip.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp>


bool appExit = false;
upperbodyclient *gClinet;
int solve(float x,float y,float z,float a,float b,float g);

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

void logic2()
{


}

int user_data;

void
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
//    viewer.setBackgroundColor (0.1, 0.1, 0.1);
//    pcl::PointXYZ o;

//     for ( int xx = -50 ; xx < 50 ; xx += 10)
//     {
//         for ( int yy = -80 ; yy < 0 ; yy += 10 )
//         {
//             for ( int zz = -50 ; zz < 50 ; zz += 10)
//             {
//                 o.x = xx;
//                 o.y = yy;
//                 o.z = zz;

//                 int rr = solve(((float)xx)/100,((float)yy)/100,((float)zz)/100,0,-1.57,0);

//                 std::string s = boost::lexical_cast<std::string>(xx);
//                 std::string s1 = boost::lexical_cast<std::string>(yy);
//                 std::string s2 = boost::lexical_cast<std::string>(zz);

// //                if ( rr == 1)
// //                {
// //                    viewer.addSphere(o.x - 1,o.x + 1,o.y - 1,o.y + 1,o.z - 1,o.z + 1,1,0,0,"c" + s + s1 + s2,0);
// //                }


//                 if ( rr == 0)
//                 {
//                  //viewer.addCube(o.x - 1,o.x + 1,o.y - 1,o.y + 1,o.z - 1,o.z + 0.1,0,1,0,"c" + s + s1 + s2,0);
//                     viewer.addSphere(o, 1,"c" + s + s1 + s2,0);
//                  //viewer.updateCamera();
//                 }

//             }
//         }
//     }

//     viewer.addCube(0 ,30,0,0,0,0,1,0,0,"cube1",0);
//     viewer.addCube(0 ,0,0,30,0,0,0,1,0,"cube2",0);
//     viewer.addCube(0 ,0,0,0,0,30,0,0,1,"cube3",0);

//    std::cout << "i only run once" << std::endl;

}

void
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}

int solve(float x,float y,float z,float a,float b,float g)
{
    double Qsx0;
    double Qsy0;
    double Qsz0;
    double Qey0;
    double Qwz0;
    double Qwy0;
    double Qwx0;

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

    gClinet->setMotorPid_leftShoulderPitch(3,35,0);
    gClinet->setMotorPid_leftShoulderRoll(3,35,0);
    gClinet->setMotorPid_leftShoulderYaw(3,35,0);
    gClinet->setMotorPid_leftElbowPitch(3,35,0);
    gClinet->setMotorPid_leftElbowRoll(3,35,0);
    gClinet->setMotorPid_leftWristPitch(0,35,0);
    gClinet->setMotorPid_leftWristRoll(0,35,0);

    upperbodyclient::motor_data MsY;
    upperbodyclient::motor_data MsX;
    upperbodyclient::motor_data MsZ;
    upperbodyclient::motor_data MeY;
    upperbodyclient::motor_data MwZ;
    upperbodyclient::motor_data MwY;
    upperbodyclient::motor_data MwX;

    if ( gClinet->allMotors.size() != 0 )
    {
        MsY = gClinet->getMotor_rightShoulderYaw();
        MsX = gClinet->getMotor_rightShoulderPitch();
        MsZ = gClinet->getMotor_rightShoulderRoll();
        MeY = gClinet->getMotor_rightElbowPitch();
        MwZ = gClinet->getMotor_rightElbowRoll();
        MwY = gClinet->getMotor_rightWristPitch();
        MwX = gClinet->getMotor_rightWristRoll();

        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
    }
    else
    {
       std::cout<<"nomotors"<<std::endl;
    }

    AsY = (float)180/2048;
    BsY = 2170;
    Qsy0 =0;//AsY*(MsY.position-BsY)*pi/180;

    AsX = (float)180/2048;
    BsX = 3470;
    Qsx0 =0;//AsX*(MsX.position-BsX)*pi/180;

    AsZ = (float)-180/2048;
    BsZ = 1000;
    Qsz0 =0;//AsZ*(MsZ.position-BsZ)*pi/180;

    AeY = (float)-180/2048;
    BeY = 1630;
    Qey0 =0;//AeY*(MeY.position-BeY)*pi/180;

    AwZ = (float)-150/512;
    BwZ = 512;
    Qwz0 = 0;//AeZ*(MwZ.position-BwZ)*pi/180;

    AwY = (float)180/2048;
    BwY = 2435;
    Qwy0 = 0;//AwY*(MwY.position-BwY)*pi/180;

    AwX = (float)150/512;
    BwX = 200;
    Qwx0 = 0;//AwZ*(MwX.position-BwX)*pi/180;

    double a1=10*pi/180; // (rad)
    double L1=0.24;      // (m)
    double L2=0.3;       // (m)
    double L3=0.2;       // (m)
    double L4=0.06;      // (m)
    double L5=0.12;      // (m)

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
    double QsxH=10*pi/180;
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
    double QwxL=0*pi/180;
    double QwxH=120*pi/180;

    //
    double speed=10;
    int Error=0;
    int Flag=0;
//    double X=0.49;
//    double Y=-0.38;
//    double Z=-0.23;
//    double A=0*pi/180;
//    double B=-90*pi/180;
//    double G=0*pi/180;

        double X=x;
        double Y=y;
        double Z=z;
        double A=a*pi/180;
        double B=b*pi/180;
        double G=g*pi/180;


            start_grip = false;
//            X = x;
//            Y = y;
//            Z = z;
//            A = a;
//            B = b;
//            G = g;

        std::cout<<"X="<<X<<" Y="<<Y <<" Z="<<Z<<" A="<<A<<" B="<<B <<" G="<<G<<std::endl;


        //=============================================================== [ IK ]

        for (double Qwx=QwxL;Qwx<=QwxH;Qwx+=1*pi/180)
        {

            Eigen::MatrixXd P1;
            Eigen::MatrixXd Pee;
            Eigen::MatrixXd P;

            double w[3]= {0,-L1/cos(a1),0};
            P1=RotTrans('X',-a1)*RotTrans('P', w); // Shoulder Transfer Matrix with Respect to Base

            double v[3]= {X,Y,Z};
            Pee=RotTrans('P',v)*RotTrans('X',A)*RotTrans('Y',B)*RotTrans('Z',G); // Endeffector Transfer Matrix with Respect to Base
            P=Pee.inverse()*P1;

            double  X1=P(0,3);
            double  Y1=P(1,3);
            double  Z1=P(2,3);
            double  Qey=-acos((pow(X1,2)+pow(Y1-L4*sin(Qwx),2)+pow(Z1-L5-L4*cos(Qwx),2)-pow(L2,2)-pow(L3,2))/(2*L2*L3));
            double  t1=(Y1*cos(Qwx)-Z1*sin(Qwx)+L5*sin(Qwx))/L2; // t1=sin(Qey)*sin(Qwz)
            double  Qwz=asin(t1/sin(Qey));

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

            double aa=pow(pow((Xee-X),2)+pow((Yee-Y),2)+pow((Zee-Z),2),0.5); //norm

            if (aa>1e-6)
            {
                Error=1;
                //disp('IK has Error!');
            }
            else if (std::isnan(Qey)||std::isnan(Qwz)||std::isnan(Qwy)||std::isnan(Qsx)||std::isnan(Qsy)||std::isnan(Qsz)||std::isinf(Qey)||std::isinf(Qwz)||std::isinf(Qwy)||std::isinf(Qsx)||std::isinf(Qsy)||std::isinf(Qsz))
            {
                Error=2;
                //disp('Not Reachable!')
            }
            else if ((QsxL>Qsx)||(Qsx>QsxH)||(QsyL>Qsy)||(Qsy>QsyH)||(QszL>Qsz)||(Qsz>QszH)||(QeyL>Qey)||(Qey>QeyH)||(QwxL>Qwx)||(Qwx>QwxH)||(QwyL>Qwy)||(Qwy>QwyH)||(QwzL>Qwz)||(Qwz>QwzH))
            {
                Error=3;
                //disp('Joint Limit!')
            }
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
            if ( Error == 1)
             std::cout << "There is not any answer. IK has Error!"<<std::endl;
            if ( Error == 2)
             std::cout << "There is not any answer. Not Reachable!"<<std::endl;
            if ( Error == 3)
             std::cout << "There is not any answer. Joint Limit!"<<std::endl;

            return Error;
        }

        std::cout<<"desired angles are="<<Qs_Xrad<<","<<Qs_Yrad<<","<< Qs_Zrad<<","<< Qe_Yrad<<","<< Qw_Zrad<<","<< Qw_Yrad<<","<<Qw_Xrad<<std::endl;

        double Qs_Ysend=((Qs_Yrad*180/pi)/AsY)+BsY;
        double Qs_Xsend=((Qs_Xrad*180/pi)/AsX)+BsX;
        double Qs_Zsend=((Qs_Zrad*180/pi)/AsZ)+BsZ;
        double Qe_Ysend=((Qe_Yrad*180/pi)/AeY)+BeY;
        double Qw_Zsend=((Qw_Zrad*180/pi)/AwZ)+BwZ;
        double Qw_Ysend=((Qw_Yrad*180/pi)/AwY)+BwY;
        double Qw_Xsend=((Qw_Xrad*180/pi)/AwX)+BwX;

        gClinet->setMotor_rightShoulderYaw(Qs_Ysend,speed);
        gClinet->setMotor_rightShoulderPitch(Qs_Xsend,speed);
        gClinet->setMotor_rightShoulderRoll(Qs_Zsend,speed);
        gClinet->setMotor_rightElbowPitch(Qe_Ysend,speed);
        gClinet->setMotor_rightElbowRoll(Qw_Zsend,speed);
        gClinet->setMotor_rightWristPitch(Qw_Ysend,speed);
        gClinet->setMotor_rightWristRoll(Qw_Xsend,speed);

        std::cout<<"motor sent commands are="<<Qs_Xsend<<","<<Qs_Ysend<<","<<Qs_Zsend<<","<<Qe_Ysend<<","<<Qw_Zsend<<","<<Qw_Ysend<<","<<Qw_Xsend<<std::endl;

        upperbodyclient::motor_data MssY ;
        upperbodyclient::motor_data MssX ;
        upperbodyclient::motor_data MssZ ;
        upperbodyclient::motor_data MeeY ;
        upperbodyclient::motor_data MwwZ ;
        upperbodyclient::motor_data MwwY ;
        upperbodyclient::motor_data MwwX ;

        if ( gClinet->allMotors.size() != 0 )
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

            MssY = gClinet->getMotor_rightShoulderYaw();
            MssX = gClinet->getMotor_rightShoulderPitch();
            MssZ = gClinet->getMotor_rightShoulderRoll();
            MeeY = gClinet->getMotor_rightElbowPitch();
            MwwZ = gClinet->getMotor_rightElbowRoll();
            MwwY = gClinet->getMotor_rightWristPitch();
            MwwX = gClinet->getMotor_rightWristRoll();

            boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
        }
        else
        {
            std::cout<<"nomotors"<<std::endl;

            MssY.position = Qs_Ysend;
            MssX.position = Qs_Xsend;
            MssZ.position = Qs_Zsend;
            MeeY.position = Qe_Ysend;
            MwwZ.position = Qw_Zsend;
            MwwY.position = Qw_Ysend;
            MwwX.position = Qw_Xsend;

        }

        double bb=MssY.position;
        double aa=MssX.position;
        double cc=MssZ.position;
        double dd=MeeY.position;
        double ee=MwwZ.position;
        double ff=MwwY.position;
        double gg=MwwX.position;

        Qsy0 =AsY*(MssY.position-BsY)*pi/180;
        Qsx0 =AsX*(MssX.position-BsX)*pi/180;
        Qsz0 =AsZ*(MssZ.position-BsZ)*pi/180;
        Qey0 =AeY*(MeeY.position-BeY)*pi/180;
        Qwz0 = AwZ*(MwwZ.position-BwZ)*pi/180;
        Qwy0 = AwY*(MwwY.position-BwY)*pi/180;
        Qwx0 = AwX*(MwwX.position-BwX)*pi/180;

        std::cout<<"motor read positions are="<<aa<<","<<bb<<","<<cc<<","<<dd<<","<<ee<<","<<ff<<","<<gg<<std::endl;

        //========================================================================= [ FK ]

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

        std::cout<<"FK: ";
        std::cout<<Xfk<<" , "<<Yfk<<" , "<<Zfk<<std::endl;

        return 0;
}

void send_result()
{

}

void logic()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    while ( ros::ok() && appExit == false )
    {
//        if ( start_grip )
//        {
//            //start_grip = false;
//            //int result = solve(end_position.x,end_position.y,end_position.z,end_position.a,end_position.b,end_position.g);


//        }
//        else
//        {
//           boost::this_thread::sleep(boost::posix_time::milliseconds(100));
//        }

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

    //start_grip = true;

    int result = solve(end_position.x,end_position.y,end_position.z,end_position.a,end_position.b,end_position.g);
    if ( result == 0 )
        response.result = "ok";
    if ( result == 1 )
        response.result = "IK has Error";
    if ( result == 2 )
        response.result = "Not Reachable";
    if ( result == 3 )
        response.result = "Joint Limit";

    return true;
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


    //pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //viewer.runOnVisualizationThreadOnce (viewerOneOff);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        logic2();

    }

    appExit = true;
    _thread_logic.interrupt();
    _thread_logic.join();

    return 0;
}
