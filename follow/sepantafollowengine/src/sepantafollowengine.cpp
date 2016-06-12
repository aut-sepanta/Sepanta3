#include <SepantaFollow.h>


actionlib::SimpleActionClient<sepanta_msgs::MasterAction> * ac;


SepantaFollowEngine::SepantaFollowEngine() : 
App_exit(false),
_thread_Logic(&SepantaFollowEngine::logic_thread,this),
_thread_10hz_publisher(&SepantaFollowEngine::scan10hz_thread,this),
_thread_logic_action(&SepantaFollowEngine::action_thread,this)
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

double SepantaFollowEngine::Quat2Rad2(tf::Quaternion q)
{
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

        if ( _x > 0.5 && _x < 3 && abs(_y) < 0.4 )
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

void go_to_location(double x,double y)
{
  ac = new actionlib::SimpleActionClient<sepanta_msgs::MasterAction>("SepantaMoveBaseAction", true);
  // wait for the action server to start
  ac->waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  sepanta_msgs::MasterGoal goal;
  goal.action = "exe";
  goal.id = "";
  //goal.x = (int)(x * 100);
  //goal.y = (int)(y * 100);
  //goal.yaw = 0;

  ac->sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac->waitForResult(ros::Duration(1000));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac->getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());

    sepanta_msgs::MasterResult::ConstPtr _res = ac->getResult();

     ROS_INFO("Action result : %s",_res->result.c_str());

  }
  else
  {
     ac->cancelGoal ();
     ROS_INFO("Action did not finish before the time out.");
  }
}

double goal_x;
double goal_y;
int action_state = 0;

double old_goal_x;
double old_goal_y;

void SepantaFollowEngine::action_thread()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    std::cout<<"action thread started"<<endl;

    while ( ros::ok() && !App_exit )
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        if ( action_state == 0 )
        {
            cout<<"wait for red dot to go there"<<endl;
            action_state = 1;
        }
        else if ( action_state == 2)
        {
            cout<<"Go to : "<<goal_x<<" "<<goal_y<<endl;
            go_to_location(goal_x,goal_y);
            action_state = 0;
        }
    }
}

void SepantaFollowEngine::logic_thread()
{
    follow_state = 0;
    find_state = 0;
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    std::cout<<"logic thread started"<<endl;
 
    while(ros::ok() && !App_exit)
    {
         boost::this_thread::sleep(boost::posix_time::milliseconds(500));

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
                  double e_x = Position[0]+ (target_person.pose.position.x+0.27)* cos(Tetha) - (target_person.pose.position.y)* sin(Tetha);
                  double e_y = Position[1]+ (target_person.pose.position.x+0.27) *sin(Tetha) + (target_person.pose.position.y)* cos(Tetha);
                  double g[4];
                  g[0] = target_person.pose.orientation.x;
                  g[1] = target_person.pose.orientation.y;
                  g[2] = target_person.pose.orientation.z;
                  g[3] = target_person.pose.orientation.w;

                  double e_yaw = Rad2Deg(Quat2Rad(g));

                  cout<<"YAW : "<<e_yaw<<endl;

                  double r_costmap = 1;
                  double Y = e_y - Position[0];
                  double X = e_x - Position[1];
                  double R = sqrt(X*X + Y*Y);
                  double r = R - r_costmap;

                  double x_goal = ( X * r ) / R;
                  double y_goal = ( x_goal * Y ) / X;


                visualization_msgs::Marker points;

                points.header.frame_id =  "map";
                points.header.stamp = ros::Time::now();
                points.ns = "point";
                points.action = visualization_msgs::Marker::ADD;
                points.pose.orientation.w = 1.0;
                points.id = 0;
                points.type = visualization_msgs::Marker::POINTS;
                points.scale.x = 0.1;
                points.scale.y = 0.1;
                points.color.r = 1;
                points.color.a = 1.0;

                geometry_msgs::Point p;
                p.x = x_goal;
                p.y = y_goal;
                p.z = 0;

                points.points.push_back(p);

                marker_pub.publish(points);

                  if ( r > 1 )
                  {
                       goal_x = e_x;
                       goal_y = e_y;

                       double delta = (old_goal_x - goal_x) * (old_goal_x - goal_x) + (old_goal_y - goal_y) * (old_goal_y - goal_y);
                       delta = sqrt(delta);

                       if ( delta > 0.5 )
                       {
                            if ( action_state == 1 )
                            {
                                old_goal_y = goal_y;
                                old_goal_x = goal_x;

                                action_state = 2;
                            }
                            else
                            if ( action_state == 2)
                            {
                                   ac->cancelGoal();
                                   action_state = 1;
                            }

                       }
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
 marker_pub =  node_handles[7].advertise<visualization_msgs::Marker>("visualization_marker_follow_target", 10);
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