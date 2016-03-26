
#include <iostream>
#include <string>
#include "sepanta_msgs/sound.h"
#include <boost/thread.hpp>
#include "client_sound.h"

using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpgitic_sound_sample");
    ros::Time::init();
    ros::Rate loop_rate(20);
    client_sound soundClient;

    //__________________ play from defatule path sample_______________
    //Default path is Desktop if you want to change it:
    soundClient.setDefaultPath("/media/SONY_16GM/SampleMusic"); 
    soundClient.play("Kalimba.mp3");
    //    
    // //________________________play from direct path______________//
    // soundClient.play("NO_RESIDEH.MP3");
    //_______________________________________________________________
    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
    soundClient.pause();
    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
    soundClient.resume();
    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
    soundClient.stop();

    return 0;
}
