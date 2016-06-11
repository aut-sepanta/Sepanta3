#include <SepantaFollow.h>

SepantaFollowEngine::SepantaFollowEngine() : 
App_exit(false),
_thread_Logic(&SepantaFollowEngine::logic_thread,this),
_thread_10hz_publisher(&SepantaFollowEngine::scan10hz_thread,this)
{
    init();
}

SepantaFollowEngine::~SepantaFollowEngine()
{
	kill();
}

bool SepantaFollowEngine::isidexist(int id)
{
   for ( int i = 0 ; i < list_persons.size() ; i++ )
   {
      if ( list_persons.at(i).ID == id ) 
      {
         target_person = list_persons.at(i);
         return true;
      }

       
   }
   return false;
}

double SepantaFollowEngine::Quat2Rad(double orientation[])
{
    tf::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void SepantaFollowEngine::GetPos(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    Position[0] = msg->pose.position.x;
    Position[1] = msg->pose.position.y;
   
    orientation[0] = msg->pose.orientation.x;
    orientation[1] = msg->pose.orientation.y;
    orientation[2] = msg->pose.orientation.z;
    orientation[3] = msg->pose.orientation.w;
   
    Tetha = Quat2Rad(orientation);
    //if (Tetha < 0) Tetha += 2 * M_PI;

    //cout<<"POS : "<<Position[0]<<" "<<Position[1]<<" "<<Tetha<<endl;
}

double SepantaFollowEngine::GetDistance(double x1, double y1, double x2, double y2)
{
    double x = x2-x1;
    double y = y2-y1;
    return sqrt(x*x + y*y);
}

bool SepantaFollowEngine::find_user_for_follow()
{
    int dist_min = 100;
    bool valid = false;
    for ( int i = 0 ; i < list_persons.size() ; i++ )
    {   
        double _x = list_persons.at(i).pose.position.x;
        double _y = list_persons.at(i).pose.position.y;

        if ( _x > 0 && abs(_y) < 0.3 )
        {
            double dist = GetDistance(0,0,_x,_y);
            if ( dist < dist_min )
            {
                dist_min = dist;
                target_person = list_persons.at(i);
                valid = true;
            }
        }
    }

    return valid;
}

void SepantaFollowEngine::send_omni(double x,double y ,double w)
{
        geometry_msgs::Twist myTwist;

        myTwist.linear.x = x;
        myTwist.linear.y = -y;
        myTwist.angular.z = -w;
     
        mycmd_vel_pub.publish(myTwist); 
}

void SepantaFollowEngine::force_stop()
{
    send_omni(0,0,0);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}

int follow_state;
int find_state;

void SepantaFollowEngine::logic_thread()
{
    follow_state = 0;
    find_state = 0;
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    std::cout<<"logic thread started"<<endl;
 
    while(ros::ok() && !App_exit)
    {
         boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

         if ( follow_state == 0 )
         {
            cout<<"[state = 0] : wait for user "<<endl;
            follow_state = 1;
            find_state = 0;
         }
         else if ( follow_state == 1 )
         {
             cout<<"[state = 1] : find_state : "<<find_state<<endl;

             bool result = find_user_for_follow();

             if ( result )
             {

                find_state++;

                if ( find_state > 2)
                {
                    find_state = 0;
                    follow_state = 2;
                }
             }
             else
             {
                find_state = 0;
             }
         }
         else if ( follow_state == 2 )
         {
             bool result = isidexist(target_person.ID);
             if ( result == false )
             {
                 cout<<"[state = 2] User Lost"<<endl;
                 follow_state = 0;
             }
             else
             {

                 cout<<"theta : "<<Rad2Deg(Tetha)<<endl;

                  //double e_x1 = target_person.pose.position.x * cos(0) + target_person.pose.position.y * sin(0);
                  //double e_y1 = - target_person.pose.position.x * sin(0) + target_person.pose.position.y * cos(0);

                 // double e_x =  (target_person.pose.position.x -Position[0])* cos(Tetha) - (target_person.pose.position.y-Position[1]) * sin(Tetha);
                 // double e_y =  (target_person.pose.position.x -Position[0]) *sin(Tetha) + (target_person.pose.position.y-Position[1])* cos(Tetha);

                

                  double e_x = Position[0]+ (target_person.pose.position.x+0.27)* cos(Tetha) - (target_person.pose.position.y)* sin(Tetha);
                  double e_y = Position[1]+ (target_person.pose.position.x+0.27) *sin(Tetha) + (target_person.pose.position.y)* cos(Tetha);

                  cout<<"[state = 2] TRACK : "<<target_person.ID<<" "<<e_x<<" "<<e_y<<endl;

                  bool result = isidexist(target_person.ID);
                 if ( result == false )
                 {
                     cout<<"[state = 2] User Lost"<<endl;
                     follow_state = 0;
                 }
             }

         }
        
    }
}

void SepantaFollowEngine::scan10hz_thread()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    std::cout<<"10hz publisher thread started"<<endl;
 
    while(ros::ok() && !App_exit)
    {
         scan10hz_can_send = true;
         boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
}

void SepantaFollowEngine::chatterCallback_laser(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   if ( scan10hz_can_send )
   {
      scan10hz_can_send = false;
      scan10hz_pub.publish(msg);
   }

}

void SepantaFollowEngine::chatterCallback_persons(const sepanta_msgs::PersonArray::ConstPtr &msg)
{
     list_persons.clear();

     for  ( int i = 0 ; i < msg->people.size() ; i++ )
     {
        person p;
        p.pose = msg->people.at(i).pose;
        p.ID = msg->people.at(i).id;
        list_persons.push_back(p);
     }

    // cout<<"people detected : "<<list_persons.size()<<endl;
}

void SepantaFollowEngine::init()
{

ROS_INFO("SepantaFollowEngine Version 1.0.0 :*");

bool say_enable;
App_exit = false;
say_enable = false;
isttsready = false;
scan10hz_can_send = false;
//============================================================================================
sub_handles[0] = node_handles[0].subscribe("/slam_out_pose", 10, &SepantaFollowEngine::GetPos,this);
sub_handles[1] = node_handles[1].subscribe("/scan",10,&SepantaFollowEngine::chatterCallback_laser,this);
sub_handles[2] = node_handles[2].subscribe("/people_tracked",10,&SepantaFollowEngine::chatterCallback_persons,this);
//============================================================================================
mycmd_vel_pub = node_handles[3].advertise<geometry_msgs::Twist>("SepantaFollowEngine/cmd_vel", 10);
scan10hz_pub = node_handles[3].advertise<sensor_msgs::LaserScan>("/scan_10hz", 10);
//============================================================================================
//pub_tts = node_handles[6].advertise<std_msgs::String>("/texttospeech/message", 10);
//============================================================================================
//sub_handles[3] = node_handles[3].subscribe("/texttospeech/queue", 10, &SepantaFollowEngine::chatterCallback_ttsfb,this);
//say_service = node_handles[11].serviceClient<sepanta_msgs::command>("texttospeech/say");
//============================================================================================
    
ROS_INFO("Init done");

}

void SepantaFollowEngine::kill()
{
    _thread_Logic.interrupt();
    _thread_Logic.join();
}