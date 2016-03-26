#include "upperbodyclient.h"

bool appExit = false;
upperbodyclient *gClinet;

void logic2()
{

    // if ( gClinet->allMotors.size() != 0 )
    // {
    //    upperbodyclient::motor_data x = gClinet->getMotor_headYaw();
    //    int a =  x.position;

    //    std::cout<<a<<std::endl;
    // }
    // else
    //     std::cout<<"no motors"<<std::endl;

}



//============================

void logic()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

    //i p d
    //gClinet->setMotorPid_headYaw(0,64,0);
    //pid for mx & ex motors
    //p for ax motors

    while( !appExit )
    {
      boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
      if ( gClinet == NULL ) continue;


       gClinet->right_down();
       gClinet->left_down();

      boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

       //gClinet->right_front();
       //gClinet->left_front();

       // gClinet->right_side();
       // gClinet-> left_side();

        gClinet->right_up();
        gClinet->left_up();

      boost::this_thread::sleep(boost::posix_time::milliseconds(10000));

      //  gClinet->right_up();
      //  gClinet->left_up();

      // boost::this_thread::sleep(boost::posix_time::milliseconds(5000));


      //  gClinet->right_side();
      //  gClinet-> left_side();

      // boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

      // gClinet->setMotor_rightShoulderYaw(2100,5);
      // gClinet->setMotor_rightWristPitch(386,5);
      // gClinet->setMotor_rightWristRoll(241,5);
      // gClinet->setMotor_rightElbowRoll(554,5);
      // gClinet->setMotor_rightElbowPitch(2391,5);
      // boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

      // gClinet->setMotor_rightGripper(300,10);
      // boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

      // gClinet->resetMotor_rightGripper();
      // boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

      // gClinet->resetMotor_rightArm();
      // boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

    }
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
