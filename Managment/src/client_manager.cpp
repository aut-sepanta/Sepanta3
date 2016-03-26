#include "client_manager.h"

using namespace std;

bool client_manager::task_getStatus(string run)
{
    if ( run == "upperbody" ) //1
    {
        if ( process_getStatus(names[1]) == 2 && process_getStatus(names[5]) == 2 && process_getStatus(names[34]) == 2)
            return true;
    }

    if ( run == "downerbody" ) //2
    {
        if ( process_getStatus(names[7]) == 2 )
            return true;
    }

    if ( run == "speech" ) //3
    {
        if ( process_getStatus(names[9]) == 2 )
            return true;
    }

    if ( run == "sound" ) //4
    {
        if ( process_getStatus(names[10]) == 2 )
            return true;
    }

    if ( run == "skeleton") //5
    {
        if ( process_getStatus(names[2]) == 2 && process_getStatus(names[3]) == 2 && process_getStatus(names[8]) == 2)
            return true;
    }

    if ( run == "face") //6
    {
        if ( process_getStatus(names[0]) == 2 && process_getStatus(names[4]) == 2)
            return true;
    }

    if ( run == "object" ) //7
    {
        if ( process_getStatus(names[0]) == 2 && process_getStatus(names[13]) == 2)
            return true;
    }

    if ( run == "stair") //8
    {
        if ( process_getStatus(names[0]) == 2 && process_getStatus(names[6]) == 2 )
            return true;
    }
}

int client_manager::process_getStatus(string name)
{
    int mode = 0;

    //-1 => invalid name
    //0 => stop
    //1 => start without ack
    //2 => start with ack

    int index = -1;
    for ( int i = 0 ; i < process_size ; i++ )
    {
        if ( name == names[i])
        {
            index = i;
            break;
        }
    }

    if ( index != -1 )
    {
        if ( node_list.at(index).status == "start" )
        {
            mode = 1; //start

            if ( node_list.at(index).ack == true )
                mode = 2; //ack

        }
        else
            mode = 0; //stop

    }
    else
        mode = -1;

    return mode;
}

int client_manager::task_start(string run,int max_try,int max_sec_timeout)
{
    std::vector<run_object> commands;
    run_object robject;

    if ( run == "upperbody" ) //1
    {
        robject.name = names[1]; robject.index = 1; commands.push_back(robject);
        robject.name = names[5]; robject.index = 5; commands.push_back(robject);
        robject.name = names[34]; robject.index = 34; commands.push_back(robject);
    }

    if ( run == "downerbody" ) //2
    {
        robject.name = names[7]; robject.index = 7; commands.push_back(robject);
    }

    if ( run == "speech" ) //3
    {
        robject.name = names[9]; robject.index = 9; commands.push_back(robject);
    }

    if ( run == "tcp" ) //37
    {
        robject.name = names[37]; robject.index = 37; commands.push_back(robject);
    }

    if ( run == "sound" ) //4
    {
        robject.name = names[10]; robject.index = 10; commands.push_back(robject);
    }

    if ( run == "skeleton") //5
    {
        robject.name = names[2]; robject.index = 2; commands.push_back(robject);
        robject.name = names[3]; robject.index = 3; commands.push_back(robject);
        robject.name = names[8]; robject.index = 8; commands.push_back(robject);
    }

    if ( run == "face") //6
    {
        robject.name = names[0]; robject.index = 0; commands.push_back(robject);
        robject.name = names[4]; robject.index = 4; commands.push_back(robject);
    }

    if ( run == "object" ) //7
    {
        robject.name = names[0]; robject.index = 0; commands.push_back(robject);
        robject.name = names[13]; robject.index = 13; commands.push_back(robject);
    }

    if ( run == "stair") //8
    {
        robject.name = names[0]; robject.index = 0; commands.push_back(robject);
        robject.name = names[6]; robject.index = 6; commands.push_back(robject);
    }

    if ( commands.size() != 0 )
    {

        bool status = false;

        for ( int i = 0 ; i < commands.size() ; i++ )
        {
            status = false;
            //========================= check exeption for kinect / marker
            if (  commands.at(i).index == 0 )
            {
                if ( node_list.at(2).status == "start" )
                {
                    //marker is run
                    status = process_stop_timeout(node_list.at(2).name,2,max_try,max_sec_timeout); //kill marker

                    if ( status == false)
                    {
                        return -100; //marker kill error while trying to open kinect
                    }
                }
            }
            else if (  commands.at(i).index == 2 )
            {
                if ( node_list.at(0).status == "start" )
                {
                    //kinect is run
                    status = process_stop_timeout(node_list.at(0).name,0,max_try,max_sec_timeout); //kill kinect

                    if ( status == false)
                    {
                        return -200; //kinect kill error while trying to open marker
                    }
                }
            }
            //================================================================


            status = process_start_timeout(commands.at(i).name,commands.at(i).index,max_try,max_sec_timeout);
            //cout<<status<<endl;

            if ( status == false)
            {
                return -1 * (i+1);
            }
        }
    }

    return 0;
}

int client_manager::task_stop(string run,int max_try,int max_sec_timeout)
{
    std::vector<run_object> commands;
    run_object robject;

    if ( run == "upperbody" ) //1
    {
        robject.name = names[1]; robject.index = 1; commands.push_back(robject);
        robject.name = names[5]; robject.index = 5; commands.push_back(robject);
        robject.name = names[34]; robject.index = 34; commands.push_back(robject);
    }

    if ( run == "downerbody" ) //2
    {
        robject.name = names[7]; robject.index = 7; commands.push_back(robject);
    }

    if ( run == "speech" ) //3
    {
        robject.name = names[9]; robject.index = 9; commands.push_back(robject);
    }

    if ( run == "tcp" ) //37
    {
        robject.name = names[37]; robject.index = 37; commands.push_back(robject);
    }

    if ( run == "sound" ) //4
    {
        robject.name = names[10]; robject.index = 10; commands.push_back(robject);
    }

    if ( run == "skeleton") //5
    {
        robject.name = names[2]; robject.index = 2; commands.push_back(robject);
        robject.name = names[3]; robject.index = 3; commands.push_back(robject);
        robject.name = names[8]; robject.index = 8; commands.push_back(robject);
    }

    if ( run == "face") //6
    {
        robject.name = names[0]; robject.index = 0; commands.push_back(robject);
        robject.name = names[4]; robject.index = 4; commands.push_back(robject);
    }

    if ( run == "object" ) //7
    {
        robject.name = names[0]; robject.index = 0; commands.push_back(robject);
        robject.name = names[13]; robject.index = 13; commands.push_back(robject);
    }

    if ( run == "stair") //8
    {
        robject.name = names[0]; robject.index = 0; commands.push_back(robject);
        robject.name = names[6]; robject.index = 6; commands.push_back(robject);
    }

    if ( commands.size() != 0 )
    {

        bool status = false;

        for ( int i = 0 ; i < commands.size() ; i++ )
        {
            status = false;
            //============================== check for exeptions ====================
            if ( commands.at(i).index == 0 ) //kinect
            {
                if ( run == "face" )
                {
                    if ( node_list.at(13).status == "start" || node_list.at(6).status == "start")
                    {
                        continue;
                    }
                }

                if ( run == "object" )
                {
                    if ( node_list.at(4).status == "start" || node_list.at(6).status == "start")
                    {
                        continue;
                    }
                }

                if ( run == "stair" )
                {
                    if ( node_list.at(13).status == "start" || node_list.at(4).status == "start")
                    {
                        continue;
                    }
                }
            }

            status = process_stop_timeout(commands.at(i).name,commands.at(i).index,max_try,max_sec_timeout);

            //=======================================================================
            if ( status == false)
            {
                return -1 * (i+1);
            }
        }
    }

    return 0;
}

bool client_manager::process_start_timeout(string name,int index,int max_try,int max_sec_timeout)
{

    int time_out_counter = 0;
    bool status = false;

    for ( int j = 0 ; j < max_try && status == false; j++ )
    {
        process_start(name);

        time_out_counter = 0;
        while( node_list.at(index).ack == false && time_out_counter < max_sec_timeout )
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            time_out_counter++;
        }

        if ( node_list.at(index).ack )
            status = true;
        else
        {
            process_stop(name);
            boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
        }

    }

    return status;
}

bool client_manager::process_stop_timeout(string name , int index,int max_try,int max_sec_timeout)
{
    int time_out_counter = 0;
    bool status = false;

    for ( int j = 0 ; j < max_try && status == false; j++ )
    {
        process_stop(name);

        time_out_counter = 0;
        while( node_list.at(index).status == "start" && time_out_counter < max_sec_timeout )
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            time_out_counter++;
        }

        if ( node_list.at(index).status == "stop" )
            status = true;
    }

    return status;
}

void client_manager::init()
{
    process_size = 33;
    first_time = false;

    names[0] = "kinect.sh";
    names[1] = "motor.sh";
    names[2] = "marker.sh";
    names[3] = "core_skeleton";
    names[4] = "core_faceDetect";
    names[5] = "upperbodycore";
    names[6] = "core_rampStairDetection";
    names[7] = "lowerbody.sh";
    names[8] = "core_waveDetect";
    names[9] = "core_speech";
    names[10] = "core_sound";
    names[11] = "core_recordPlayMotors";
    names[12] = "kinect_aux_node";
    names[13] = "core_objectRecognition";
    names[14] = "robot_byby";
    names[15] = "hand_imitation";
    names[16] = "anounce_pose";
    names[17] = "introduction";
    names[18] = "hand_track";
    names[19] = "senario";
    names[20] = "stair";
    names[21] = "logic2";
    names[22] = "sample_stair_ramp";      //ramp
    names[23] = "sample";                 //upperbody
    names[24] = "sample1";                //downerbody
    names[25] = "sample_use";             //sound
    names[26] = "sample_skeleton";    //skeleton
    names[27] = "sample_recordmotor";     //record
    names[28] = "sample_speech";     //record
    names[29] = "sample_aux";     //record
    names[30] = "core_createModel";     //record
    names[31] = "sample_wavedetect";     //record
    names[32] = "sample_faceDetect";     //record
    names[33] = "core"; //in hichi nist ! :)
    names[34] = "ikfk";
    names[35] = "model.sh";
    names[36] = "object.sh";
    names[37] = "core_tcp";

    service_startall = node_handles[0].serviceClient<sepanta_msgs::NodeAction>("/node_manager/startall");
    service_stopall = node_handles[1].serviceClient<sepanta_msgs::NodeAction>("/node_manager/stopall");
    service_start = node_handles[2].serviceClient<sepanta_msgs::NodeAction>("/node_manager/start");
    service_stop = node_handles[3].serviceClient<sepanta_msgs::NodeAction>("/node_manager/stop");

    sub_node_status = node_handles[6].subscribe("/node_manager/node_status", 1, &client_manager::callback_manager,this);

}

void client_manager::callback_manager(const sepanta_msgs::nodestatuslist::ConstPtr &msg)
{
    if ( first_time == false )
    {
        for ( int i = 0 ; i < msg->statuslist.size() ; i++ )
        {
            node_status ns;
            ns.ack = msg->statuslist.at(i).ack;
            ns.name = msg->statuslist.at(i).name;
            ns.status = msg->statuslist.at(i).status;
            node_list.push_back(ns);
        }
    }
    else
    {
        // ROS_INFO("get %d",msg->statuslist.size());

        for ( int i = 0 ; i < msg->statuslist.size() ; i++ )
        {
            node_list.at(i).ack = msg->statuslist.at(i).ack;
            node_list.at(i).name = msg->statuslist.at(i).name;
            node_list.at(i).status = msg->statuslist.at(i).status;

            //cout<<msg->statuslist.at(i).name<<endl;
        }

        //cout<<"======================================================"<<endl;
    }

    first_time = true;
}

void client_manager::process_start(std::string name)
{
    sepanta_msgs::NodeAction srv_start;
    srv_start.request.node_file_name = name;
    service_start.call(srv_start);
}

void client_manager::process_stop(std::string name)
{
    sepanta_msgs::NodeAction srv_stop;
    srv_stop.request.node_file_name = name;
    service_stop.call(srv_stop);
}


void client_manager::process_startall(string run)
{

    sepanta_msgs::NodeAction na;
    na.request.node_file_name = run;
    service_startall.call(na);

}

void client_manager::process_stopall(string run)
{
    sepanta_msgs::NodeAction na;
    na.request.node_file_name = run;
    service_stopall.call(na);
}













