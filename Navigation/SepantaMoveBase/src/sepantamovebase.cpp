#include <sepantamovebase.h>


// #define VIRTUALMODE

SepantaMoveBase::SepantaMoveBase() : 
<<<<<<< HEAD
App_exit(false),
=======
>>>>>>> 2de9ae01f4fb663c8ab2a23d0f3ffc555dc43eb2
_thread_PathFwr(&SepantaMoveBase::PathFwr,this),
_thread_Logic(&SepantaMoveBase::logic_thread,this),
_thread_Vis(&SepantaMoveBase::vis_thread,this)
{
<<<<<<< HEAD
    init();
}

SepantaMoveBase::~SepantaMoveBase()
{
	kill();
}

double SepantaMoveBase::Quat2Rad(double orientation[])
{
    tf::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void SepantaMoveBase::publish_isrobotmove()
{
    std_msgs::Bool _msg;
    _msg.data = getrobotmove();
    pub_move.publish(_msg);
}

void SepantaMoveBase::setsystemstate(int value,bool forced = false)
{
   if ( getstatemutex() || forced)
   		system_state = value;
}

void SepantaMoveBase::setlogicstate(int value,bool forced = false)
{
	if ( getstatemutex() || forced)
   		logic_state = value;
}

int SepantaMoveBase::getsystemstate()
{
   return system_state;
}

int SepantaMoveBase::getlogicstate()
{
  return logic_state;
}

bool SepantaMoveBase::getstatemutex()
{
   return statemutex;
}

void SepantaMoveBase::setstatemutex(bool value)
{
    statemutex = value;
}

=======

    init();
}

SepantaMoveBase::~SepantaMoveBase()
{
	kill();
}

double SepantaMoveBase::Quat2Rad(double orientation[])
{
    tf::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void SepantaMoveBase::publish_isrobotmove()
{
    std_msgs::Bool _msg;
    _msg.data = getrobotmove();
    pub_move.publish(_msg);
}

void SepantaMoveBase::setsystemstate(int value,bool forced = false)
{
   if ( getstatemutex() || forced)
   		system_state = value;
}

void SepantaMoveBase::setlogicstate(int value,bool forced = false)
{
	if ( getstatemutex() || forced)
   		logic_state = value;
}

int SepantaMoveBase::getsystemstate()
{
   return system_state;
}

int SepantaMoveBase::getlogicstate()
{
  return logic_state;
}

bool SepantaMoveBase::getstatemutex()
{
   return statemutex;
}

void SepantaMoveBase::setstatemutex(bool value)
{
    statemutex = value;
}

>>>>>>> 2de9ae01f4fb663c8ab2a23d0f3ffc555dc43eb2
void SepantaMoveBase::say_message(string data)
{
    if ( say_enable == false ) return;
    isttsready = false;
    sepanta_msgs::command _msg;
   _msg.request.command = data;
    say_service.call(_msg);
    sayMessageId = _msg.response.result;
    while(!isttsready)
    {
        cout<<"wait for tts id : "<<sayMessageId<<endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    }
}

void SepantaMoveBase::send_omni(double x,double y ,double w)
{
        geometry_msgs::Twist myTwist;



        #ifdef VIRTUALMODE
       
            myTwist.linear.x = x;
            myTwist.linear.y = y;
            myTwist.angular.z = w;
        #else
            myTwist.linear.x = x;
            myTwist.linear.y = -y;
            myTwist.angular.z = -w;
        #endif
       
        mycmd_vel_pub.publish(myTwist); 
}

void SepantaMoveBase::force_stop()
{
    send_omni(0,0,0);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}

double SepantaMoveBase::GetDistance(double x1, double y1, double x2, double y2)
{
    double x = x2-x1;
    double y = y2-y1;
    return sqrt(x*x + y*y);
}

int SepantaMoveBase::GetCurrentStep()
{
    for(int i=0;i<globalPathSize-1;i++)
    {
        if(GetDistance(position[0],position[1],globalPath.poses[i].pose.position.x, globalPath.poses[i].pose.position.y) < GetDistance(position[0],position[1],globalPath.poses[i+1].pose.position.x, globalPath.poses[i+1].pose.position.y))
            return i;
    }
    return globalPathSize-1;
}

void SepantaMoveBase::sepantamapengine_savemap()
{
   std_srvs::Empty _s;
   client_map_save.call(_s);
}

void SepantaMoveBase::sepantamapengine_loadmap()
{
   std_srvs::Empty _s;
   client_map_load.call(_s);
}

void SepantaMoveBase::clean_costmaps()
{
   std_srvs::Empty _s;
   client_resetcostmap.call(_s);
}

//cm cm degree
void SepantaMoveBase::update_hector_origin(float x,float y,float yaw)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    pub_slam_origin.publish(msg);
}

void SepantaMoveBase::reset_hector_slam()
{
	std_msgs::String _msg;
	_msg.data = "reset";
	pub_slam_reset.publish(_msg);
}


void SepantaMoveBase::read_file()
{

        std::string path_points =  ros::package::getPath("managment") + "/maps/points.txt";
        //cout<<path_points<<endl;
        std::string line;
        std::ifstream text;

        goal_list.clear();
        text.open(path_points.c_str(), ios_base::in);

        if (text.is_open())
        {
        	
            getline(text,line);

            while (text.good())
            {
                vector <string> fields;

                boost::split( fields, line, boost::is_any_of( "," ) );

                goal_data gdata;

                 gdata.id = fields[0].c_str();
                 gdata.x = atoi(fields[1].c_str());
                 gdata.y = atoi(fields[2].c_str());
                 gdata.yaw = atoi(fields[3].c_str());
                 gdata.height = atoi(fields[4].c_str());

                goal_list.push_back(gdata);
               
                getline(text,line);

            }
            text.close();

        }
        else
        {
            std::cout << coutcolor_blue << "[Unable to open file]" << coutcolor0 <<std::endl << std::endl;
        }

        std::cout << coutcolor_blue << "read done : " << coutcolor0 << goal_list.size()<<std::endl << std::endl;
}

int SepantaMoveBase::find_goal_byname(string name)
{
    for ( int i = 0 ; i < goal_list.size() ; i++ )
    {
        if ( name == goal_list.at(i).id )
        {
           return i;
        }
    }

    return -1;
}

nav_msgs::Path SepantaMoveBase::call_make_plan()
{
    nav_msgs::GetPlan srv;

    srv.request.start.header.frame_id = "map";
    srv.request.start.pose.position.x = position[0];
    srv.request.start.pose.position.y = position[1];
    srv.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(tetha);

    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = goalPos[0];
    srv.request.goal.pose.position.y = goalPos[1];
    srv.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(goalTetha);

    srv.request.tolerance = 0.1;
    client_makeplan.call(srv);

    return srv.response.plan;
}

void SepantaMoveBase::logic_thread()
{
     boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    std::cout<<"logic thread started"<<endl;
    nav_msgs::Path result;

    while(ros::ok() && !App_exit)
    {
         boost::this_thread::sleep(boost::posix_time::milliseconds(500));
         //cout<<"issepantamove : "<< isrobotmove << endl;

         if ( getlogicstate() == -1)
         {
            cout<<"Get Error With Hector Status"<<endl;
            say_message("There is a problem with my laser");
           
            reset_hector_slam();
            update_hector_origin(position[0],position[1],tetha);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));   
            clean_costmaps();     
            say_message("My laser recovered successfuly");
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            setsystemstate(tempSystemState);
            setlogicstate(tempLogicState);
       

         }
         if ( getlogicstate() == 0 )
         {
            IsGoalReached = false;
            if ( idle_flag == false )
            {
                idle_flag = true;
                cout<<"wait for exe , idle"<<endl;
            }
            
         }

         else if ( getlogicstate() == 1 )
         {
               idle_flag = false;
               //operation loop
               cout<<coutcolor_green<<"Planning... " <<coutcolor0<<endl;
               result = call_make_plan();
               setlogicstate(2);
              
         }

        else if ( getlogicstate() == 2 )
        {
            //check the plan
            cout<<"Check the plan from global planner"<<endl;
           
            if( result.poses.size() == 0 )
            {
                    cout<<coutcolor_red<<"Error in PATH ! "<<coutcolor0<<endl;
                    
                  
                    setlogicstate(3);
                    setsystemstate(-1); //wait
                    force_stop();
                    if(!IsRecoveryState)
                        say_message("Error in path generation");
            }
            else
            {
                globalPath = result;
                globalPathSize = globalPath.poses.size();
                cout<<coutcolor_green<<"get a new PATH from GPLANNER Points : "<< globalPathSize <<coutcolor0<<endl;
                setsystemstate(1);
                setlogicstate(4);
                force_stop();   

            }
        }

        else if ( getlogicstate() == 3 )
        {
        	say_message("Let me think");
            cout<<coutcolor_red<<" Recovery state " <<coutcolor0<<endl;
            //path error handler 
            //revocery state
           
            if(IsRecoveryState)
            {
            	if(IsHectorReset)
            	{
            		IsRecoveryState = false;
	            	IsHectorReset = false;
	            	setlastnavigationresult("GOAL IS UNREACHABLE");
	            	say_message("Goal is unreachable");
	            	setsystemstate(0);
	            	setlogicstate(0);
            	}
            	else
            	{
            		IsHectorReset = true;
	            	say_message("reseting hector");
            		reset_hector_slam();
            		update_hector_origin(position[0],position[1],tetha);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                    clean_costmaps();   
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            		setlogicstate(1);
	            }

            }
            else
            {
            	IsRecoveryState = true;
            	say_message("reseting cost map");
            	clean_costmaps();
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            	setlogicstate(1);
         	}
             
        }

        else if ( getlogicstate() == 4 ) //controlling mode
        {
        	IsRecoveryState = false;
        	IsHectorReset = false;
            cout<<coutcolor_magenta<<" getlogicstate() == 4 " <<coutcolor0<<endl;

<<<<<<< HEAD
            if(!IsPoseStimated && IsamclReady && getsystemstate() == 4)
=======
            if(!IsPoseStimated && IsamclReady)
>>>>>>> 2de9ae01f4fb663c8ab2a23d0f3ffc555dc43eb2
            {
            	 setlogicstate(5);
            	 setsystemstate(-1); //wait
            	IsPoseStimated = true;
            	force_stop();
            }

           if ( IsGoalReached == true)
           {
               cout<<coutcolor_red<<" Goal reached " <<coutcolor0<<endl;
               setlogicstate(0);
             
               continue;
           }
            
            result = call_make_plan();
            if( result.poses.size() != 0 )
             {
                int currentStep = GetCurrentStep();
                for(int i=0;i<result.poses.size()-1 && i+currentStep<globalPathSize-1;i++)
                {
                    if(GetDistance(result.poses[i].pose.position.x, result.poses[i].pose.position.y,globalPath.poses[i+currentStep].pose.position.x, globalPath.poses[i+currentStep].pose.position.y)>0.2)
                    {
                        globalPath = result;
                        globalPathSize = result.poses.size();
                         setsystemstate(1);
                        cout<<coutcolor_red<<"PATH changed : "<< globalPathSize <<coutcolor0<<endl;
                        break;
                    }
                }

             }
             else
             {
                    cout<<coutcolor_red<<"Error in PATH ! "<<coutcolor0<<endl;
                    setlogicstate(2);
             }
        

        }
        else if ( getlogicstate() == 5 ) //Estimating position
        {
        	say_message("Estimating Position");
        	cout<<coutcolor_green<<"Estimating Position"<<coutcolor0<<endl;
    		reset_hector_slam();
<<<<<<< HEAD
    		update_hector_origin(estimatedPosition[0],estimatedPosition[1],estimatedOrientation);
=======
    		update_hector_origin(amclPosition[0],amclPosition[1],Quat2Rad(amclOrientation));
>>>>>>> 2de9ae01f4fb663c8ab2a23d0f3ffc555dc43eb2
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            clean_costmaps();   
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    		setlogicstate(1);
        }
    }
}

void SepantaMoveBase::exe_slam(goal_data g)
{
    bool valid_point = false;
    //this function get goal and move robot to there
    if ( g.id == "")
    {
       goalPos[0] = (float)(g.x) / 100; //cm => m
       goalPos[1] = (float)(g.y) / 100; //cm => m
       goalTetha = Deg2Rad(g.yaw); //rad => deg

       valid_point = true;
       ROS_INFO("EXECUTE MANUAL POINT");
    }
    else
    {
       int index = find_goal_byname(g.id);
       if ( index != -1)
       {
           goal_data data = (goal_data)goal_list.at(index);
           goalPos[0] = (float)(data.x)/ 100;
           goalPos[1] = (float)(data.y)/ 100;
           goalTetha = Deg2Rad(data.yaw);

           valid_point = true;
           ROS_INFO("EXECUTE LOCAL POINT");
       }
       else
       {
           valid_point = false;
           ROS_ERROR("LOCAL POINT NOT FOUND");
       }
    }

    if ( valid_point && getlogicstate() == 0)
    {
       
        // say_message("Got a new goal");
       setlogicstate(1);
    }  
}

void SepantaMoveBase::exe_cancel()
{
	 setstatemutex(false);
	 setlogicstate(0,true);
     setsystemstate(0,true);
     
     cout<<"Cancel requested"<<endl;
     // say_message("cancel requested , operation canceled!");
     setlastnavigationresult("CANCEL REQUEST");

      force_stop();
    
}

int SepantaMoveBase::sign(double data)
{
    if(data > 0) return 1;
    else if(data < 0) return -1;
    else return 0;
}

int SepantaMoveBase::roundData(double data)
{
    if(data>=0)
        return ceil(data);
    else
        return floor(data);
}

double SepantaMoveBase::GetToPointsAngle(double x1, double y1, double x2, double y2)
{
    return atan2(y2-y1,x2-x1);
}

void SepantaMoveBase::ResetLimits()
{
    desireErrorX = normal_desire_errorX;
    desireErrorY = normal_desire_errorY;
    desireErrorTetha = normal_desire_errorTetha;

    LKpX = normal_kp_linearX;
    LKpY = normal_kp_linearY;
    WKp = norma_kp_angular;

    LKiX = normal_ki_linearX;
    LKiY = normal_ki_linearY;
    WKi = normal_ki_angular;

    maxLinSpeedX = normal_max_linear_speedX;
    maxLinSpeedY = normal_max_linear_speedY;
    maxTethaSpeed = normal_max_angular_speed;
}

void SepantaMoveBase::ReduceLimits()
{
    desireErrorX = goal_desire_errorX;
    desireErrorY = goal_desire_errorY;
    desireErrorTetha = goal_desire_errorTetha;

    maxLinSpeedX = goal_max_linear_speedX;
    maxLinSpeedY = goal_max_linear_speedY;
    maxTethaSpeed = goal_max_angular_speed;

    LKpX = goal_kp_linearX;
    LKpY = goal_kp_linearY;
    WKp = goal_kp_angular;

    LKiX = goal_ki_linearX;
    LKiY = goal_ki_linearY;
    WKi = goal_ki_angular;
}

//0 => wait for goal
//2 => turn to target
//4 => go on path
//6 => turn to goal
//8 => reached

void SepantaMoveBase::setrobotmove(bool value)
{
	isrobotmove = value;
}
<<<<<<< HEAD

bool SepantaMoveBase::getrobotmove()
{
	return isrobotmove;
}

void SepantaMoveBase::setlastnavigationresult(string value)
{
	last_navigation_result = value;
}

string SepantaMoveBase::getlastnavigationresult()
{
	return last_navigation_result;
}

=======

bool SepantaMoveBase::getrobotmove()
{
	return isrobotmove;
}

void SepantaMoveBase::setlastnavigationresult(string value)
{
	last_navigation_result = value;
}

string SepantaMoveBase::getlastnavigationresult()
{
	return last_navigation_result;
}

>>>>>>> 2de9ae01f4fb663c8ab2a23d0f3ffc555dc43eb2
int SepantaMoveBase::calc_next_point()
{
            bool isgoalnext = false;
            if ( step == globalPathSize-1)
            {
                on_the_goal = true;
               
            }
                
            if(step+step_size >= globalPathSize)
            {
                step = globalPathSize - 1;
                //we are very near to goal so next is the goalstep = globalPathSize-1;
                tempGoalPos[0] = goalPos[0];
                tempGoalPos[1] = goalPos[1];
                tempGoalTetha = goalTetha;
                if (tempGoalTetha < 0) tempGoalTetha += 2*M_PI;

                isgoalnext = true;
                //on_the_goal = true;

                cout<<coutcolor_magenta<<"goal calc"<<coutcolor0<<endl;

               
            }
            else
            {
                //select the point in next 20 step and calc all errors
                step +=step_size;

                tempGoalPos[0] = globalPath.poses[step].pose.position.x;
                tempGoalPos[1] = globalPath.poses[step].pose.position.y;
                tempGoalTetha = GetToPointsAngle(position[0], position[1], globalPath.poses[step].pose.position.x, globalPath.poses[step].pose.position.y);
                if (tempGoalTetha < 0) tempGoalTetha += 2*M_PI;


                 cout<<coutcolor_magenta<<"step calc"<<coutcolor0<<endl;
                 
            }

           

            return isgoalnext;
}

void SepantaMoveBase::errors_update()
{
            //calc errorX , errorY , errorTetha 
            errorX = tempGoalPos[0]-position[0];
            errorY = tempGoalPos[1]-position[1];
            errorTetha = tempGoalTetha-tetha;

            if (errorTetha >= M_PI) errorTetha =  errorTetha - 2*M_PI;
            if (errorTetha < -M_PI) errorTetha =  errorTetha + 2*M_PI;
            //?
            if (errorTetha > 0.833*M_PI) errorTetha = 0.833*M_PI;
            if (errorTetha < -0.833*M_PI) errorTetha = -0.833*M_PI;

            errorX_R = cos(tetha)*errorX+sin(tetha)*errorY;
            errorY_R = -sin(tetha)*errorX+cos(tetha)*errorY;

            float x1 = position[0];
            float y1 = position[1];
            float x2 = goalPos[0];
            float y2 = goalPos[1];
            float d_1 = (x2 - x1); 
            float d_2 = (y2 - y1);
            distacne_to_goal = d_1 * d_1 + d_2 * d_2;
            distacne_to_goal = sqrt(distacne_to_goal);
}

void SepantaMoveBase::publish_info()
{
        info_counter++;
        if ( info_counter>50)
        {
            info_counter= 0;

            cout << "Speed: " << xSpeed << " - " << ySpeed << " - " << tethaSpeed << endl;
            cout << "Step: " << step << endl;
            cout << "TError: " << errorX << " - " << errorY << " - " << errorTetha << endl;
            cout << "Goal: " << fabs(goalPos[0]-position[0]) << " - " << fabs(goalPos[1]-position[1]) << " - " << fabs(goalTetha-tetha)<< endl; 
        }
}

void SepantaMoveBase::controller_update(int x,bool y,bool theta)
{
    if ( x == 1)
    xSpeed = (fabs(errorX_R*LKpX)<=maxLinSpeedX)?(errorX_R*LKpX):sign(errorX_R)*maxLinSpeedX;
    else if ( x == 0)
    xSpeed = 0;
    else if ( x == 2)
    xSpeed = 0.3;

    if ( y )
    ySpeed = (fabs(errorY_R*LKpY)<=maxLinSpeedY)?(errorY_R*LKpY):sign(errorY_R)*maxLinSpeedY;
    else
    ySpeed = 0;

    if ( theta )
    tethaSpeed = (fabs(errorTetha*WKp)<=maxTethaSpeed)?(errorTetha*WKp):sign(errorTetha)*maxTethaSpeed;
    else
    tethaSpeed = 0;

    send_omni(xSpeed,ySpeed,tethaSpeed); 
}

void SepantaMoveBase::PathFwr()
{
	
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    force_stop();
   
    #ifndef VIRTUALMODE
    say_message("Sepanta Move Base Started");
    #else
    say_message("Sepanta Move Base Started in virtual mode");
    #endif

    while (ros::ok() && !App_exit)
    {
        errors_update();

        if ( getsystemstate() == 0 )
        {
        	if ( getstatemutex() == false )
        	{
        		boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        		setstatemutex(true);
        	}
        	setrobotmove(false);
        }
        else
        {
        	setrobotmove(true);
        }

        if ( getsystemstate() == -1) //wait state
        {

           boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
        if ( getsystemstate() == 0)
        {

            if ( wait_flag == false)
            {
               wait_flag = true;
               cout<< coutcolor_green <<"Wait for goal ! ... "<< coutcolor0 <<endl;
               say_message("I am waiting for new goal");
            }
          
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
        else
        if ( getsystemstate() == 1)
        {
           force_stop();
           IsGoalReached = false;
           on_the_goal = false;
           ResetLimits();
           step = 0;
           wait_flag = false;
           cout<<"State = 1 -turn to target-"<<endl;

            bool resutl = calc_next_point();
            if ( resutl)
            {
                //next is the goal !
                cout<<"Next is goal =>3"<<endl;
              
                setsystemstate(3);
            }
            else
            {
                cout<<"Next is step =>2"<<endl;
                setsystemstate(2);
            }

        }
        else
        if ( getsystemstate() == 2)
        {
            //turn to goal <loop>
          
            if(fabs(errorTetha)<=desireErrorTetha)
            {
                
                cout<<"DONE ! "<<tetha<<" "<<tempGoalTetha<<" "<<errorTetha<<endl;
                setsystemstate(3);
                force_stop();
                
            }
            else
            {
                controller_update(0,false,true);
            }

        }
        else
        if ( getsystemstate() == 3)
        {
           cout<<"State = 3 -go on path- Step = "<<step<<endl;
           setsystemstate(4);
        }
        else
        if ( getsystemstate() == 4)
        {
           
            if(fabs(errorX_R)<=desireErrorX && fabs(errorY_R)<=desireErrorY && fabs(errorTetha)<=desireErrorTetha)
            {
                bool resutl = calc_next_point();
                if ( resutl )
                {
                	//next is the goal
                	ReduceLimits();

                    if ( on_the_goal )
                    {
                       setsystemstate(5);
                    }
                    else
                    {
                       setsystemstate(3);
                    }
                    
                }
                else
                {
                    cout<<"Temp point reached"<<endl;
                    setsystemstate(3);
                }
            }
            else
            {
            	if ( step < 40 || (globalPathSize - step) < 40 )
                controller_update(1,true,true); //p (X,Y,T)
                else 
                {
                	if ( fabs(errorTetha)<=desireErrorTetha )
                	{
                		controller_update(2,true,true); //fixed (F,Y,T)
                	}
                	else
                	{
                		//if we are going away ! 
                		controller_update(1,true,true); //p  fixed (X,Y,T)
                	}
                }

                	
                
            }
        }
        else
        if ( getsystemstate() == 5)
        {

            cout<<"State = 5 -turn to goal-"<<endl;
            setsystemstate(6);
        }
        else
        if ( getsystemstate() == 6)
        {
           
            if(fabs(errorTetha)<=desireErrorTetha)
            {
                cout<<"DONE ! "<<tetha<<" "<<tempGoalTetha<<" "<<errorTetha<<endl;
                setsystemstate(7);
                force_stop();
                
            }
            else
            {
                controller_update(0,false,true);
            }
        }
        else
        if ( getsystemstate() == 7)
        {
            cout<<"State = 7 -goal reached-"<<endl;
            setsystemstate(8);
        }
        else
        if ( getsystemstate() == 8)
        {
            cout<<"Finished !"<<endl;
            
            IsGoalReached = true;

            on_the_goal = false;
            wait_flag = false;
            force_stop();
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            setsystemstate(0);
            setlastnavigationresult("GOAL REACHED");
            say_message("Goal reached");
        }

       
        //publish_info();
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    }
}

void SepantaMoveBase::GetCostmap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	if(!IsCmValid)
	{
		costmap = *msg;
		IsCmValid = true;
	}
}

bool SepantaMoveBase::calc_error(double x1,double y1,double t1,double x2,double y2,double t2,double delta_t)
{
   bool valid = true;
   double v1 = fabs(x2-x1) / delta_t;
   double v2 = fabs(y2-y1) / delta_t;

   double dt = t2-t1;
   if (dt >= M_PI) dt =  dt - 2*M_PI;
   if (dt < -M_PI) dt =  dt + 2*M_PI;

   double v3 = fabs(dt) / delta_t;

   if ( v1 > maxLinSpeedX + 0.1 ) valid = false;
   else if ( v2 > maxLinSpeedY + 0.1 ) valid = false;
   else if ( v3 > maxTethaSpeed + 0.1 ) valid = false;

   // cout<<v1<<" "<<v2<<" "<<v3<<" "<<delta_t<<endl;

   return valid;
}

void SepantaMoveBase::hector_problem_detected()
{
    if ( getlogicstate() != -1)
    {
        
            tempLogicState = getlogicstate();
            setlogicstate(-1);

            tempSystemState = getsystemstate();
            setsystemstate(-1);

            force_stop();
        }
}

void SepantaMoveBase::GetAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
	amclCovariance = msg->pose.covariance;
	amclPosition[0] = msg->pose.pose.position.x;
	amclPosition[1] = msg->pose.pose.position.y;
    amclOrientation[0] = msg->pose.pose.orientation.x;
    amclOrientation[1] = msg->pose.pose.orientation.y;
    amclOrientation[2] = msg->pose.pose.orientation.z;
    amclOrientation[3] = msg->pose.pose.orientation.w;
<<<<<<< HEAD
    if(abs(amclCovariance[0]) < 0.04 && amclCovariance[0] != 0 && abs(amclCovariance[7]) < 0.02 && amclCovariance[7] != 0)
    {
        estimatedPosition[0] = amclPosition[0];
        estimatedPosition[1] = amclPosition[1];
        estimatedOrientation = Quat2Rad(amclOrientation);
        IsamclReady = true;
        if(IsPoseStimated)
        {
            if(sqrt((position[0]-estimatedPosition[0])*(position[0]-estimatedPosition[0]) +
                (position[1]-estimatedPosition[1])*(position[1]-estimatedPosition[1])) > 0.1 && getsystemstate() == 4)
            {
                cout<<coutcolor_green<<"Estimating Position without Orientation"<<coutcolor0<<endl;
                update_hector_origin(estimatedPosition[0],estimatedPosition[1],tetha);
            }
        }
    }
    else
    {
    	IsamclReady = false;
    	if(abs(amclCovariance[0]) > 0.06 || abs(amclCovariance[7]) > 0.04)
    		IsPoseStimated = false;
    }

=======
    if(abs(amclCovariance[0]) < 0.02 && amclCovariance[0] != 0 && abs(amclCovariance[7]) < 0.02 && amclCovariance[7] != 0)
        IsamclReady = true;
    else
    {
    	IsamclReady = false;
    	if(abs(amclCovariance[0]) > 0.05 || abs(amclCovariance[7]) > 0.05)
    		IsPoseStimated = false;
    }
>>>>>>> 2de9ae01f4fb663c8ab2a23d0f3ffc555dc43eb2
    // for(int i=0;i<10;i++)
    // 	cout<<amclCovariance[i]<<"\t";
    // cout<<"\n";
}

void SepantaMoveBase::GetPos(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    oldposition[0] = msg->pose.position.x;
    oldposition[1] = msg->pose.position.y;
    orientation[0] = msg->pose.orientation.x;
    orientation[1] = msg->pose.orientation.y;
    orientation[2] = msg->pose.orientation.z;
    orientation[3] = msg->pose.orientation.w;
    oldtetha = Quat2Rad(orientation);
    if (oldtetha < 0) oldtetha += 2*M_PI;

    ros::Duration _delta_t = msg->header.stamp - old_time; //delta t in sec
    double delta_t = _delta_t.toSec();

    //cout<<"Temp POSE : "<<oldposition[0]<<" "<<oldposition[1]<<" "<<oldtetha<<endl;
    //cout<<"POSITION"<<oldposition[0]*100<<" | "<<oldposition[1]*100<<" | "<<Rad2Deg(oldtetha)<<endl;
    bool valid = calc_error(oldposition[0],oldposition[1],oldtetha,position[0],position[1],tetha,delta_t);
    if ( true )
    {
        position[0] = oldposition[0];
        position[1] = oldposition[1];
        tetha = oldtetha;
    }
    else
    {
        cout<<"Keeped POSE : "<<position[0]<<" "<<position[1]<<" "<<tetha<<endl;
        if( getlogicstate() != 3) //if we are not in revcovery state and this is not caused by out request or hector reseting
        {
            cout<<coutcolor_red<<"problem with position"<<coutcolor0<<endl;
            //hector_problem_detected();
        }

       
    }

    old_time = msg->header.stamp;
    //===================================== 
}

void SepantaMoveBase::CheckHectorStatus(const std_msgs::Bool::ConstPtr &msg)
{ 
    if(msg->data == false)
    {
        hector_problem_detected(); 
    }
}

void SepantaMoveBase::chatterCallback_ttsfb(const std_msgs::String::ConstPtr &msg)
{
    if(!isttsready && msg->data == sayMessageId)
    {
        cout<<coutcolor_brown<<"text to speech is ready!"<<coutcolor0<<endl;
        isttsready = true;
    }
}

bool SepantaMoveBase::checkcommand(sepanta_msgs::command::Request  &req,sepanta_msgs::command::Response &res)
{

	ROS_INFO("Service Request....");

    std::string _cmd = req.command;
    std::string _id = req.id;
    int _value1 = req.value1;
    int _value2 = req.value2;
    int _value3 = req.value3;

    if ( _cmd == "reload_points")
    {
    	read_file();
    }

    if ( _cmd == "save_map")
    {
    	sepantamapengine_savemap();
    }

    if ( _cmd == "load_map")
    {
    	sepantamapengine_loadmap();
    }

    if ( _cmd == "exe")
    {

        goal_data g;
        g.id = _id;
        g.x = _value1;
        g.y = _value2;
        g.yaw = _value3;

        exe_slam(g);
    }

    if ( _cmd == "cancel")
    {

        exe_cancel();
    }

    if ( _cmd == "reset_hector")
    {
        reset_hector_slam();
    }

    if ( _cmd == "update_hector_origin")
    {

        update_hector_origin(0,0,0);
    }

    res.result = "done";
    return true;
}

void SepantaMoveBase::test_vis()
{

    visualization_msgs::Marker points , points2 , points3;

    points.header.frame_id =  "map";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.g = 1.0f;
    points.color.a = 1.0;

    points2.header.frame_id =  "map";
    points2.header.stamp = ros::Time::now();
    points2.action = visualization_msgs::Marker::ADD;
    points2.id = 1;
    points2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    points2.scale.x = 1;
    points2.scale.y = 1;
    points2.scale.z = 0.4;
    points2.color.b = 0.5;
    points2.color.r = 1;
    points2.color.a = 1;

    points3.header.frame_id =  "map";
    points3.header.stamp = ros::Time::now();
    points3.action = visualization_msgs::Marker::ADD;
    points3.id = 2;
    points3.type = visualization_msgs::Marker::ARROW;
    points3.scale.x = 0.5;
    points3.scale.y = 0.05;
    points3.scale.z = 0.05;
    points3.color.b = 0.5;
    points3.color.r = 1;
    points3.color.a = 1;


     // Create the vertices for the points and lines
    for (int i = 0; i < globalPath.poses.size(); i += step_size)
    {
      
      geometry_msgs::Point p;
      p.x = globalPath.poses[i].pose.position.x;
      p.y = globalPath.poses[i].pose.position.y;
      p.z = 0;

      points.points.push_back(p);
      
    }

    for ( int i = 0 ; i < goal_list.size() ; i++)
    {
     
     points2.pose.position.x =  (float)goal_list[i].x / 100;
     points2.pose.position.y =  (float)goal_list[i].y / 100;
     points2.pose.position.z = 0;
     points2.text = goal_list[i].id;
     points2.ns = "text " + goal_list[i].id;
     marker_pub2.publish(points2);


     points3.pose.position.x =  (float)goal_list[i].x / 100;
     points3.pose.position.y =  (float)goal_list[i].y / 100;
     points3.pose.position.z = 0; 
     points3.pose.orientation = tf::createQuaternionMsgFromYaw(Deg2Rad(goal_list[i].yaw));
     points3.ns = "arrow " + goal_list[i].id;

     marker_pub3.publish(points3);
     
    }


    marker_pub.publish(points);
}

void SepantaMoveBase::vis_thread()
{
<<<<<<< HEAD
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
=======
>>>>>>> 2de9ae01f4fb663c8ab2a23d0f3ffc555dc43eb2
    while (ros::ok() && App_exit == false)
    {
    	test_vis();
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    }
}

void SepantaMoveBase::init()
{

ROS_INFO("SepantaMoveBase Version 2.2 :*");

coutcolor0 = "\033[0;0m";
coutcolor_red = "\033[0;31m";
coutcolor_green = "\033[0;32m";
coutcolor_blue = "\033[0;34m";
coutcolor_magenta = "\033[0;35m";
coutcolor_brown = "\033[0;33m";
say_enable = false;
App_exit = false;
IsCmValid = false;
IsGoalReached = false;
IsRecoveryState = false;
IsHectorReset = false;
isttsready = true;
IsPoseStimated = false;
IsamclReady = false;
statemutex = true;
maxLinSpeedX = normal_max_linear_speedX;
maxLinSpeedY = normal_max_linear_speedY;
maxTethaSpeed = normal_max_angular_speed;
globalPathSize = 0;
temp_path_size = 0;
xSpeed=0;
ySpeed=0;
tethaSpeed=0;
desireErrorX = normal_desire_errorX;
desireErrorY = normal_desire_errorY;
desireErrorTetha = normal_desire_errorTetha;
errorX = 0;
errorY = 0;
errorTetha = 0;
errorX_R = 0;
errorY_R = 0;
LKpX = normal_kp_linearX;
LKpY = normal_kp_linearY;
WKp = norma_kp_angular;
LKiX = normal_ki_linearX;
LKiY = normal_ki_linearY;
WKi = normal_ki_angular;
step = 0;
position[2] = {0};
oldposition[2] = {0};
orientation[4] = {0};
amclPosition[2] = {0};
amclOrientation[4] = {0};
<<<<<<< HEAD
estimatedPosition[2] = {0};
estimatedOrientation = 0;
=======
>>>>>>> 2de9ae01f4fb663c8ab2a23d0f3ffc555dc43eb2
tetha = 0;
oldtetha = 0;
tempGoalPos[2] = {0};
tempGoalTetha = 0;
goalPos[2] = {0};
goalOri[4] = {0};
goalTetha = 0;
distacne_to_goal = 0;
maxErrorX = 0;
maxErrorY = 0;
maxErrorTetha = 0;
info_counter = 0;
system_state = 0;
logic_state = 0;
on_the_goal = false;
step_size  = 40;
wait_flag = false;
idle_flag = false;
isrobotmove = false;
last_navigation_result = "";
f = 0.0;



    //============================================================================================
    sub_handles[0] = node_handles[0].subscribe("/slam_out_pose", 10, &SepantaMoveBase::GetPos,this);
<<<<<<< HEAD
    //============================================================================================
    sub_handles[1] = node_handles[1].subscribe("/move_base/global_costmap/costmap", 10, &SepantaMoveBase::GetCostmap,this);
    //============================================================================================
    sub_handles[2] = node_handles[2].subscribe("/HectorStatus", 10, &SepantaMoveBase::CheckHectorStatus,this);
    //============================================================================================
=======
    //============================================================================================
    sub_handles[1] = node_handles[1].subscribe("/move_base/global_costmap/costmap", 10, &SepantaMoveBase::GetCostmap,this);
    //============================================================================================
    sub_handles[2] = node_handles[2].subscribe("/HectorStatus", 10, &SepantaMoveBase::CheckHectorStatus,this);
    //============================================================================================
>>>>>>> 2de9ae01f4fb663c8ab2a23d0f3ffc555dc43eb2
    sub_handles[4] = node_handles[13].subscribe("/amcl_pose", 10, &SepantaMoveBase::GetAmclPose,this);
    //============================================================================================
    mycmd_vel_pub = node_handles[3].advertise<geometry_msgs::Twist>("sepantamovebase/cmd_vel", 10);
    pub_slam_origin = node_handles[4].advertise<geometry_msgs::PoseWithCovarianceStamped>("/slam_origin", 1);
    pub_slam_reset = node_handles[5].advertise<std_msgs::String>("syscommand", 1);
    //============================================================================================
    ros::ServiceServer service_command = n_service.advertiseService("sepantamovebase/command", &SepantaMoveBase::checkcommand,this);
    //============================================================================================
    pub_tts = node_handles[6].advertise<std_msgs::String>("/texttospeech/message", 10);
    //============================================================================================
    pub_current_goal = node_handles[7].advertise<geometry_msgs::PoseStamped>("current_goal", 0 );
    pub_move = node_handles[7].advertise<std_msgs::Bool>("lowerbodycore/isrobotmove", 10);
    //============================================================================================
    marker_pub =  node_handles[7].advertise<visualization_msgs::Marker>("visualization_marker_steps", 10);
    marker_pub2 =  node_handles[7].advertise<visualization_msgs::Marker>("visualization_marker_goals", 10);
    marker_pub3 =  node_handles[7].advertise<visualization_msgs::Marker>("visualization_marker_goals_arrow", 10);
    //============================================================================================
    client_makeplan = node_handles[9].serviceClient<nav_msgs::GetPlanRequest>("move_base/make_plan");
 	client_resetcostmap = node_handles[10].serviceClient<std_srvs::EmptyRequest>("move_base/clear_costmaps");
 	client_map_save = node_handles[11].serviceClient<std_srvs::EmptyRequest>("sepantamapengenine/save");
    client_map_load = node_handles[12].serviceClient<std_srvs::EmptyRequest>("sepantamapengenine/load");
    //============================================================================================
    sub_handles[3] = node_handles[3].subscribe("/texttospeech/queue", 10, &SepantaMoveBase::chatterCallback_ttsfb,this);
    say_service = node_handles[11].serviceClient<sepanta_msgs::command>("texttospeech/say");
    //============================================================================================
    read_file();
    ROS_INFO("Init done");
}

void SepantaMoveBase::kill()
{
	_thread_PathFwr.interrupt();
    _thread_PathFwr.join();

    _thread_Logic.interrupt();
    _thread_Logic.join();

    _thread_Vis.interrupt();
    _thread_Vis.join();
}