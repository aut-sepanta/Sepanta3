
/***************************************************************************
 *  sepanta_msgs.cpp - sepanta_msgs main application
 *
 *  Created: Tue Aug  3 17:06:44 2010
 *  Copyright  2010  Tim Niemueller [www.niemueller.de]
 *             2010  Carnegie Mellon University
 *             2010  Intel Labs Pittsburgh, Intel Research
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free
 *
 * Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE file in the base directory.
 */


#include "sepanta_msgs/NodeAction.h"
#include "sepanta_msgs/ListAvailable.h"
#include "sepanta_msgs/ListLoaded.h"
#include "sepanta_msgs/NodeEvent.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <map>
#include <string>
#include <utility>
#include <fstream>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>


#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <dirent.h>
#include <signal.h>
#include <unistd.h>
#include <regex.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <ros/package.h>

#include <iostream>
#include <string>
#include <stdlib.h>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

#include <sensor_msgs/Image.h>

#include <rosspawn/watchdog.h>
#include <sepanta_msgs/nodestatus.h>
#include <sepanta_msgs/nodestatuslist.h>
#include <sepanta_msgs/kill_marker.h>
#include <fstream>

#include "ros/ros.h"
#include <math.h>
#include <sstream>
#include <cstdio>
#include <unistd.h>
#include <cmath>
#include <tbb/atomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;
// For now we only accept binaries which are in a bin directory. This makes
// thinks much less clutered, otherwise add  "%s/%s" to the array
const char *test_paths[] = { "%s/bin/%s" };

std::vector<std::string > models_fpath;
std::vector<std::string > models_ename;

std::vector<std::string > shell_fpath;
std::vector<std::string > shell_ename;

bool App_exit = false;
ros::Subscriber sub_log[34];
std::vector<std::string> logs[34];

struct node_status
{
public :

    std::string name;
    std::string status;
    bool ack;
};

std::vector<node_status> node_list;
ros::Publisher     __pub_node_status;


class sepanta_msgsMain
{


public:
    sepanta_msgsMain(ros::NodeHandle &n)
        : __n(n)
    {
        __children_wait_thread = boost::thread(boost::bind(&sepanta_msgsMain::wait_thread, this));
        if (regcomp(&__re_alnum, "^[[:alnum:]]+$", REG_EXTENDED) != 0) {
            throw ros::Exception("Failed to compile regex");
        }

        __use_acceptable_modules_file = n.getParam("/sepanta_msgs/acceptable_modules_file",
                                                   __acceptable_modules_file);

        //ros::package rospack;

        if (__use_acceptable_modules_file) {
            ROS_INFO("Using acceptable modules file %s", __acceptable_modules_file.c_str());
            std::ifstream f(__acceptable_modules_file.c_str());
            while (! (f.fail() || f.eof()) ) {
                std::string mod;
                f >> mod;
                //                for (rospack::VecPkg::iterator i = rospack::Package::pkgs.begin();
                //                     i != rospack::Package::pkgs.end(); ++i) {
                //                    if ((*i)->name == mod) {
                //                        __search_paths.push_back((*i)->path);
                //                    }
                //                }
            }
        } else {
            std::string ros_root_bin = getenv("ROS_ROOT");
            if (ros_root_bin == "") {
                throw ros::Exception("Failed to read ROS_ROOT environment variable");
            }
            ros_root_bin += "/bin";
            __search_paths.push_back(ros_root_bin);
            //            for (rospack::VecPkg::iterator i = rospack::Package::pkgs.begin();
            //                 i != rospack::Package::pkgs.end(); ++i) {
            //                __search_paths.push_back((*i)->path);
            //            }
        }

        ros::NodeHandle nn;
        __srv_startall = n.advertiseService("/node_manager/startall", &sepanta_msgsMain::startall_node, this);
        __srv_stopall = n.advertiseService("/node_manager/stopall", &sepanta_msgsMain::stopall_node, this);
        __srv_start = n.advertiseService("/node_manager/start", &sepanta_msgsMain::start_node, this);
        __srv_stop = n.advertiseService("/node_manager/stop", &sepanta_msgsMain::stop_node, this);
        __srv_pause = n.advertiseService("/node_manager/pause", &sepanta_msgsMain::pause_node, this);
        __srv_continue = n.advertiseService("/node_manager/continue", &sepanta_msgsMain::continue_node, this);
        __srv_list_loaded = n.advertiseService("/node_manager/list_loaded",&sepanta_msgsMain::list_loaded, this);
        __srv_list_avail = n.advertiseService("/node_manager/list_available", &sepanta_msgsMain::list_available, this);

        __pub_node_events = n.advertise<sepanta_msgs::NodeEvent>("node_manager/node_events", 10);
        service_markerkill = nn.serviceClient<sepanta_msgs::kill_marker>("pgitic_kill_marker");

    }

    ~sepanta_msgsMain()
    {
        // We use native pthread calls here since Boost does not allow to cancel a thread.
        // I'm not going back to the stoneage and do a polling wait() in the thread and
        // waste power just to account for the sluggish Boost API.
        void *dont_care;
        pthread_cancel(__children_wait_thread.native_handle());
        pthread_join(__children_wait_thread.native_handle(), &dont_care);
    }

    std::string find_valid(std::string &progname)
    {
        //        if (regexec(&__re_alnum, progname.c_str(), 0, NULL, 0) == REG_NOMATCH) {
        //            throw ros::Exception("Invalid program name");
        //        }

        for (ChildrenMap::iterator i = __children.begin(); i != __children.end(); ++i) {
            if (i->second.first == progname) {
                throw ros::Exception("Program is already running");
            }
        }

        for (int i = 0 ; i < models_ename.size() ; i++)
        {
            if ( models_ename.at(i) == progname.c_str())
            {
                return models_fpath.at(i);
            }
        }

        throw ros::Exception("No program with the requested name found");
    }

    void kill_marker()
    {
        sepanta_msgs::kill_marker srv_kill;
        service_markerkill.call(srv_kill);
    }

    std::string find_valid2(std::string &progname)
    {


        for (ChildrenMap::iterator i = __children.begin(); i != __children.end(); ++i) {
            if (i->second.first == progname) {
                throw ros::Exception("Program is already running");
            }
        }

        for (int i = 0 ; i < shell_ename.size() ; i++)
        {
            if ( shell_ename.at(i) == progname.c_str())
            {
                return shell_fpath.at(i);
            }
        }

        throw ros::Exception("No program with the requested name found");
    }

    void update_status(std::string name,std::string status)
    {
        for ( int i = 0 ; i < node_list.size() ; i++ )
        {
            if ( node_list.at(i).name == name )
            {
                node_list.at(i).status = status; break;
            }
        }
    }

    bool fork_and_exec(std::string progname)
    {
        std::cout<<"try:"<<progname<<std::endl;
        std::string p;
        if ( progname != "kinect.sh" && progname != "kinectkill.sh" && progname != "kinectkill2.sh" &&
             progname != "motor.sh"  && progname != "motorkill.sh"  && progname != "modelkill.sh" && progname != "objectkill.sh" &&
             progname != "marker.sh" && progname != "markerkill.sh" &&
             progname != "lowerbody.sh" && progname != "lowerbodykill.sh" && progname != "speechkill.sh" && progname != "tcpkill.sh")

            p = find_valid(progname);
        else
            p = find_valid2(progname);

        pid_t pid = fork();
        if (pid == -1) {
            return false;
        } else if (pid == 0) {
            // child
            setsid();
            signal(SIGINT, SIG_IGN);
            ROS_INFO("Running %s from path %s", progname.c_str(), p.c_str());
            fclose(stdout);
            fclose(stdin);
            fclose(stderr);
            execl(p.c_str(), p.c_str(), NULL);
        } else {
            ROS_DEBUG("Child PID %i", pid);
            boost::mutex::scoped_lock lock(__children_mutex);
            __children[pid] = make_pair(progname, p);
            if (__children.size() == 1) {
                __children_cond.notify_all();
            }
            sepanta_msgs::NodeEvent msg;
            msg.header.stamp.setNow(ros::Time::now());
            msg.event_type = sepanta_msgs::NodeEvent::NODE_STARTED;
            msg.node_name = progname;
            __pub_node_events.publish(msg);

            update_status(progname,"start");


        }

        return true;
    }

    void wait_thread()
    {
        while (ros::ok()) {
            boost::unique_lock<boost::mutex> lock(__children_mutex);
            while (__children.empty()) {
                __children_cond.wait(lock);
            }

            int status = 0;
            lock.unlock();
            pid_t pid = waitpid(-1, &status, WUNTRACED | WCONTINUED);
            if (pid == -1)  continue;
            lock.lock();

            sepanta_msgs::NodeEvent msg;
            msg.event_type = sepanta_msgs::NodeEvent::NODE_DIED;
            msg.node_name = __children[pid].first;
            update_status(msg.node_name,"stop");

            // Debug output
            if (WIFEXITED(status)) {
                ROS_INFO("%i/%s exited, status=%d", pid,
                         __children[pid].first.c_str(), WEXITSTATUS(status));
                char *tmp;
                if (asprintf(&tmp, "%s (PID %i) exited, status=%d",
                             __children[pid].first.c_str(), pid, WEXITSTATUS(status)) != -1) {
                    msg.message = tmp;
                    free(tmp);
                }
            } else if (WIFSIGNALED(status)) {
                ROS_INFO("%i/%s killed by signal %d", pid,
                         __children[pid].first.c_str(), WTERMSIG(status));
                char *tmp;
                if (asprintf(&tmp, "%s (PID %i) killed by signal %d",
                             __children[pid].first.c_str(), pid, WTERMSIG(status)) != -1) {
                    msg.message = tmp;
                    free(tmp);
                }
            } else if (WIFSTOPPED(status)) {
                ROS_INFO("%i/%s stopped by signal %d", pid,
                         __children[pid].first.c_str(), WSTOPSIG(status));
                char *tmp;
                msg.event_type = sepanta_msgs::NodeEvent::NODE_PAUSED;
                update_status(msg.node_name,"pause");
                if (asprintf(&tmp, "%s (PID %i) stopped by signal %d",
                             __children[pid].first.c_str(), pid, WSTOPSIG(status)) != -1) {
                    msg.message = tmp;
                    free(tmp);
                }
            } else if (WIFCONTINUED(status)) {
                ROS_INFO("%i/%s continued", pid, __children[pid].first.c_str());
                char *tmp;
                msg.event_type = sepanta_msgs::NodeEvent::NODE_CONTINUED;
                update_status(msg.node_name,"resume");
                if (asprintf(&tmp, "%s (PID %i) continued",
                             __children[pid].first.c_str(), pid) != -1) {
                    msg.message = tmp;
                    free(tmp);
                }
            }

            if (WIFEXITED(status) || WIFSIGNALED(status)) {
                if (WIFSIGNALED(status)) {
                    int sig = WTERMSIG(status);
                    if (sig == SIGSEGV) {
                        // inform about faulty program
                        ROS_WARN("Program %s (%s) died with segfault", __children[pid].first.c_str(),
                                 __children[pid].second.c_str());

                        char *tmp;
                        msg.event_type |= sepanta_msgs::NodeEvent::NODE_SEGFAULT;
                        update_status(msg.node_name,"segfault");
                        if (asprintf(&tmp, "%s (PID %i) died with segfault",
                                     __children[pid].first.c_str(), pid) != -1) {
                            msg.message = tmp;
                            free(tmp);
                        }
                    }
                }
                __children.erase(pid);
            }

            __pub_node_events.publish(msg);


        }
    }

    std::string get_process_state(pid_t pid)
    {
        char *procpath;
        if (asprintf(&procpath, "/proc/%i/stat", pid) != -1) {
            FILE *f = fopen(procpath, "r");
            if (f) {
                int pid;
                char *program;
                char state[2]; state[1] = 0;
                if (fscanf(f, "%d %as %c", &pid, &program, state) == 3) {
                    free(program);
                    return state;
                }
                fclose(f);
            }
            free(procpath);
        }

        return "?";
    }

    pid_t get_pid(std::string &node_file_name)
    {
        for (ChildrenMap::iterator i = __children.begin(); i != __children.end(); ++i) {
            if (i->second.first == node_file_name) {
                return i->first;
            }
        }
        return 0;
    }

    bool start_node(sepanta_msgs::NodeAction::Request &req,
                    sepanta_msgs::NodeAction::Response &resp)
    {
        return fork_and_exec(req.node_file_name);
    }

    bool send_signal(std::string &node_file_name, int signum)
    {
        pid_t pid = get_pid(node_file_name);
        if (pid != 0) {
            ROS_INFO("Sending signal %s (%i) to %s (PID %i)", strsignal(signum), signum,
                     __children[pid].first.c_str(), pid);
            ::kill(pid, signum);
            return true;
        } else {
            return false;
        }
    }

    bool pause_node(sepanta_msgs::NodeAction::Request &req,
                    sepanta_msgs::NodeAction::Response &resp)
    {
        return send_signal(req.node_file_name, SIGSTOP);
    }

    bool continue_node(sepanta_msgs::NodeAction::Request &req,
                       sepanta_msgs::NodeAction::Response &resp)
    {
        return send_signal(req.node_file_name, SIGCONT);
    }

    bool startall_node(sepanta_msgs::NodeAction::Request &req,sepanta_msgs::NodeAction::Response &resp)
    {
        std::cout<<"startall"<<std::endl;
        std::string line = req.node_file_name;

        std::vector<std::string> strs;
        boost::algorithm::split(strs,line,boost::algorithm::is_any_of(","));
        for ( int i = 0 ; i < strs.size() ; i++ )
        {
            fork_and_exec(strs.at(i));
        }

    }

    bool stopnode(std::string name)
    {
        pid_t pid = get_pid(name);
        if (pid != 0) {
            std::string state = get_process_state(pid);
            ROS_INFO("Sending signal %s (%i) to %s (PID %i)", strsignal(SIGINT), SIGINT,
                     __children[pid].first.c_str(), pid);
            ::kill(pid, SIGINT);

            if ( name == "kinect.sh")
            {
                fork_and_exec("kinectkill.sh");
                fork_and_exec("kinectkill2.sh");
            }

            if ( name == "lowerbody.sh")
            {
                fork_and_exec("lowerbodykill.sh");
            }

            if ( name == "marker.sh")
            {
                kill_marker();
                fork_and_exec("markerkill.sh");
            }

            std::cout<<"speech"<<std::endl;
            if ( name == "core_speech")
            {
                fork_and_exec("speechkill.sh");
            }

            if ( name == "core_tcp")
            {
                fork_and_exec("tcpkill.sh");
            }

            if ( name == "motor.sh")
            {
                fork_and_exec("motorkill.sh");
            }

            if ( name == "core_objectRecognition")
            {
                fork_and_exec("objectkill.sh");
            }

            if ( name == "core_createModel")
            {
                fork_and_exec("modelkill.sh");
            }

            return true;
        } else {
            return false;
        }
    }

    bool stopall_node(sepanta_msgs::NodeAction::Request &req, sepanta_msgs::NodeAction::Response &resp)
    {
        std::cout<<"stopall"<<std::endl;
        std::string line = req.node_file_name;

        std::vector<std::string> strs;
        boost::algorithm::split(strs,line,boost::algorithm::is_any_of(","));

        for ( int i = 0 ; i < strs.size() ; i++ )
        {
            stopnode(strs.at(i));
        }
    }

    bool stop_node(sepanta_msgs::NodeAction::Request &req,sepanta_msgs::NodeAction::Response &resp)
    {
        return stopnode(req.node_file_name);
    }

    bool list_loaded(sepanta_msgs::ListLoaded::Request &req,
                     sepanta_msgs::ListLoaded::Response &resp)
    {
        for (ChildrenMap::iterator i = __children.begin(); i != __children.end(); ++i) {
            resp.nodes.clear();
            resp.nodes.push_back(i->second.first);
        }
        return true;
    }

    bool list_available(sepanta_msgs::ListAvailable::Request &req,
                        sepanta_msgs::ListAvailable::Response &resp)
    {
        resp.bin_files.clear();
        for (std::list<std::string>::iterator i = __search_paths.begin();
             i != __search_paths.end(); ++i) {
            for (unsigned int j = 0; (j < sizeof(test_paths) / sizeof(const char *)); ++j) {
                char *tmp;
                if (asprintf(&tmp, test_paths[j], i->c_str(), "") != -1) {
                    struct stat s;
                    if (stat(tmp, &s) == 0) {
                        if (S_ISDIR(s.st_mode) && (access(tmp, X_OK) == 0)) {
                            // check for files
                            DIR *d = opendir(tmp);
                            if (d != NULL) {
                                struct dirent de, *deres;
                                if ((readdir_r(d, &de, &deres) == 0) && (deres != NULL)) {
                                    do {
                                        char *tmp2;
                                        if (asprintf(&tmp2, test_paths[j], i->c_str(), de.d_name) != -1) {
                                            struct stat filestat;
                                            if (stat(tmp2, &filestat) == 0) {
                                                if (S_ISREG(filestat.st_mode) && (access(tmp2, X_OK) == 0)) {
                                                    resp.bin_files.push_back(de.d_name);
                                                }
                                            }
                                            free(tmp2);
                                        }
                                    } while ((readdir_r(d, &de, &deres) == 0) && (deres != NULL));
                                }
                                closedir(d);
                            }
                        }
                    }
                    free(tmp);
                }
            }
        }
        return true;
    }



private:
    ros::NodeHandle &__n;
    ros::Publisher     __pub_node_events;
    
    ros::ServiceServer __srv_startall;
    ros::ServiceServer __srv_stopall;

    ros::ServiceServer __srv_start;
    ros::ServiceServer __srv_stop;
    ros::ServiceServer __srv_pause;
    ros::ServiceServer __srv_continue;
    ros::ServiceServer __srv_list_loaded;
    ros::ServiceServer __srv_list_avail;
    ros::ServiceClient service_markerkill;

    std::list<std::string> __search_paths;


    typedef std::map<int, std::pair<std::string, std::string> > ChildrenMap;
    ChildrenMap                __children;
    boost::mutex               __children_mutex;
    boost::condition_variable  __children_cond;
    boost::thread              __children_wait_thread;

    bool                   __use_acceptable_modules_file;
    std::string            __acceptable_modules_file;
    std::list<std::string> __acceptable_modules;


    regex_t __re_alnum;
};


void loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, std::vector<std::string> &models)
{
    if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    {
        ROS_INFO("Not found");
        return;
    }

    for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
    {
        if (boost::filesystem::is_directory (it->status ()))
        {
            //ROS_INFO("in folder");
            loadFeatureModels (it->path (), extension, models);
        }
        if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
        {
            //ROS_INFO("pushed");
            std::string m;
            boost::filesystem::path path;
            path=base_dir / it->path ().filename ();
            m=path.string();
            models.push_back (m);
        }
    }
}

const int names_count = 38;
std::string names[names_count];

int init_nodes()
{

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

    for ( int i = 0 ; i < names_count ; i++ )
    {
        node_status nd_st;
        nd_st.name = names[i];
        nd_st.status = "stop";
        nd_st.ack = false;
        node_list.push_back(nd_st);
    }
}

void publisher()
{
    watchdog wd1(1000,"/camera/rgb/image_color","kinect","image");
    watchdog wd2(1000,"/core_upperbody/ack","upperbodycore","string");
    watchdog wd3(1000,"/core_speech/ack","core_speech","string");
    watchdog wd4(1000,"/core_aux/ack","core_aux","string");
    watchdog wd5(1000,"/core_sound/ack","core_sound","string");
    watchdog wd6(1000,"/core_lowerbody/ack","core_lowerbody","string");

    watchdog wd7(1000,"/senario_byby/ack","senario_byby","string");
    watchdog wd8(1000,"/senario_stair/ack","senario_stair","string");
    watchdog wd9(1000,"/senario_walk/ack","senario_walk","string");
    watchdog wd10(1000,"/senario_introduction/ack","senario_introduction","string");
    watchdog wd11(1000,"/senario_anouncepose/ack","senario_anouncepose","string");
    watchdog wd12(1000,"/senario_imitation/ack","senario_imitation","string");
    watchdog wd13(1000,"/senario_handtrack/ack","senario_handtrack","string");
    watchdog wd14(1000,"/senario_logic2/ack","senario_logic2","string");

    watchdog wd15(1000,"/core_object/ack","core_object","string");
    watchdog wd16(1000,"/core_face/ack","core_face","string");
    watchdog wd17(1000,"/motor_states/dx_port","core_motor","motor");
    watchdog wd18(1000,"/skeleton_markers","core_marker","marker");
    watchdog wd19(1000,"/core_wavedetection/ack","core_wavedetection","string");
    watchdog wd20(1000,"/core_skeleton/ack","core_skeleton","string");
    watchdog wd21(1000,"/core_stair/ack","core_stair","string");
    watchdog wd22(1000,"/core_recordmotor/ack","core_recordmotor","string");

    watchdog wd23(1000,"/OBJECTOUT/createmodel/ack","core_createModel","string");
    watchdog wd24(1000,"/core_ikfk/ack","ikfk","string");
    watchdog wd25(1000,"/core_tcp/ack","core_tcp","string");

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    while ( App_exit == false )
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        
        node_list.at(0).ack = wd1.ack;
        node_list.at(1).ack = true;
        node_list.at(5).ack = wd2.ack;
        node_list.at(9).ack = wd3.ack;
        node_list.at(12).ack = wd4.ack;
        node_list.at(10).ack = wd5.ack;
        node_list.at(7).ack = wd6.ack;
        node_list.at(14).ack = wd7.ack;
        node_list.at(20).ack = wd8.ack;
        node_list.at(19).ack = wd9.ack;
        node_list.at(17).ack = wd10.ack;
        node_list.at(16).ack = wd11.ack;
        node_list.at(15).ack = wd12.ack;
        node_list.at(18).ack = wd13.ack;
        node_list.at(21).ack = wd14.ack;
        node_list.at(11).ack = wd22.ack;
        node_list.at(2).ack = wd18.ack;
        node_list.at(8).ack = wd19.ack;
        node_list.at(3).ack = wd20.ack;
        node_list.at(6).ack = wd21.ack;
        node_list.at(4).ack = wd16.ack;
        node_list.at(13).ack = wd15.ack;
        node_list.at(30).ack = wd23.ack;
        node_list.at(34).ack = wd24.ack;
        node_list.at(37).ack = wd25.ack;

        //=========
        sepanta_msgs::nodestatuslist list;

        for ( int i = 0 ; i < node_list.size() ; i++ )
        {
            sepanta_msgs::nodestatus ns;
            ns.name = node_list.at(i).name;
            ns.status = node_list.at(i).status;
            ns.ack = node_list.at(i).ack;
            list.statuslist.push_back(ns);
        }

        __pub_node_status.publish(list);

        // std::cout<<"io"<<std::endl;

    }

    wd1.kill();
    wd2.kill();
    wd3.kill();
    wd4.kill();
    wd5.kill();
    wd6.kill();
    wd7.kill();
    wd8.kill();
    wd9.kill();
    wd10.kill();
    wd11.kill();
    wd12.kill();
    wd13.kill();
    wd14.kill();
    wd15.kill();
    wd16.kill();
    wd17.kill();
    wd18.kill();
    wd19.kill();
    wd20.kill();
    wd21.kill();
    wd22.kill();
    wd23.kill();
    wd24.kill();
    wd25.kill();
}

int log_state[33] = {0};

string get_time()
{
    time_t now = time(0);
    tm* localtm = localtime(&now);

    int day = localtm->tm_mday;
    int month = localtm->tm_mon;
    int year = localtm->tm_year - 100 + 2000;

    int hour = localtm->tm_hour;
    int min = localtm->tm_min;
    int sec = localtm->tm_sec;

    std::string sday = boost::lexical_cast<string>(day);
    std::string smonth = boost::lexical_cast<string>(month);
    std::string syear = boost::lexical_cast<string>(year);
    std::string shour = boost::lexical_cast<string>(hour);
    std::string smin = boost::lexical_cast<string>(min);
    std::string ssec = boost::lexical_cast<string>(sec);

    if ( day < 10 ) sday = "0" + sday;
    if ( month < 10 ) smonth = "0" + smonth;
    if ( year < 10 ) syear = "0" + syear;

    if ( hour < 10 ) shour = "0" + shour;
    if ( min < 10 ) smin = "0" + smin;
    if ( sec < 10 ) ssec = "0" + ssec;

    std::string time_stamp =  syear  + "/" + smonth + "/" + sday + " ";
    time_stamp += shour  + ":" + smin + ":" + ssec + " => ";

    return time_stamp;
}

void save_log(int index,string message)
{
    std::string homedir = getenv("HOME");
    std::string path_points =  homedir + "/catkin_ws/log/" + names[index] + ".txt";
    std::string line;
    std::ofstream text;

    string time_stamp =  get_time();

    if ( log_state[index] == 0)
        text.open(path_points.c_str(), std::fstream::out ); //first time create a new log txt
    else
        text.open(path_points.c_str(), std::fstream::out | std::fstream::app); //second time append the current log txt

    if (text.is_open())
    {
        log_state[index] = 1;
        text<<time_stamp<<message<<endl;
        text.flush();
        text.close();
    }
    else
    {
        std::cout << "Unable to open file" << std::endl << std::endl;
    }

}

void callback_log1(const std_msgs::String::ConstPtr &msg)
{
    save_log(0,msg->data);
}
void callback_log2(const std_msgs::String::ConstPtr &msg)
{
    save_log(1,msg->data);
}
void callback_log3(const std_msgs::String::ConstPtr &msg)
{
    save_log(2,msg->data);

}
void callback_log4(const std_msgs::String::ConstPtr &msg)
{
    save_log(3,msg->data);

}
void callback_log5(const std_msgs::String::ConstPtr &msg)
{
    save_log(4,msg->data);

}
void callback_log6(const std_msgs::String::ConstPtr &msg)
{
    save_log(5,msg->data);

}
void callback_log7(const std_msgs::String::ConstPtr &msg)
{
    save_log(6,msg->data);

}
void callback_log8(const std_msgs::String::ConstPtr &msg)
{
    save_log(7,msg->data);

}
void callback_log9(const std_msgs::String::ConstPtr &msg)
{
    save_log(8,msg->data);

}
void callback_log10(const std_msgs::String::ConstPtr &msg)
{
    save_log(9,msg->data);

}
void callback_log11(const std_msgs::String::ConstPtr &msg)
{
    save_log(10,msg->data);

}
void callback_log12(const std_msgs::String::ConstPtr &msg)
{
    save_log(11,msg->data);

}
void callback_log13(const std_msgs::String::ConstPtr &msg)
{
    save_log(12,msg->data);

}
void callback_log14(const std_msgs::String::ConstPtr &msg)
{
    save_log(13,msg->data);

}
void callback_log15(const std_msgs::String::ConstPtr &msg)
{
    save_log(14,msg->data);

}
void callback_log16(const std_msgs::String::ConstPtr &msg)
{
    save_log(15,msg->data);

}
void callback_log17(const std_msgs::String::ConstPtr &msg)
{
    save_log(16,msg->data);

}
void callback_log18(const std_msgs::String::ConstPtr &msg)
{
    save_log(17,msg->data);

}
void callback_log19(const std_msgs::String::ConstPtr &msg)
{
    save_log(18,msg->data);

}
void callback_log20(const std_msgs::String::ConstPtr &msg)
{
    save_log(19,msg->data);

}
void callback_log21(const std_msgs::String::ConstPtr &msg)
{
    save_log(20,msg->data);

}
void callback_log22(const std_msgs::String::ConstPtr &msg)
{
    save_log(21,msg->data);

}
void callback_log23(const std_msgs::String::ConstPtr &msg)
{
    save_log(22,msg->data);

}
void callback_log24(const std_msgs::String::ConstPtr &msg)
{
    save_log(23,msg->data);

}
void callback_log25(const std_msgs::String::ConstPtr &msg)
{
    save_log(24,msg->data);

}
void callback_log26(const std_msgs::String::ConstPtr &msg)
{
    save_log(25,msg->data);

}
void callback_log27(const std_msgs::String::ConstPtr &msg)
{
    save_log(26,msg->data);

}
void callback_log28(const std_msgs::String::ConstPtr &msg)
{
    save_log(27,msg->data);

}
void callback_log29(const std_msgs::String::ConstPtr &msg)
{
    save_log(28,msg->data);

}
void callback_log30(const std_msgs::String::ConstPtr &msg)
{
    save_log(29,msg->data);

}
void callback_log31(const std_msgs::String::ConstPtr &msg)
{
    save_log(30,msg->data);

}
void callback_log32(const std_msgs::String::ConstPtr &msg)
{
    save_log(31,msg->data);

}
void callback_log33(const std_msgs::String::ConstPtr &msg)
{
    save_log(32,msg->data);

}
void callback_log34(const std_msgs::String::ConstPtr &msg)
{
    save_log(33,msg->data);
}

int main(int argc, char **argv)
{
    init_nodes();
    ros::init(argc, argv, "rosspawn");
    ros::Time::init();

    ros::NodeHandle n;
    //=========================================================================
    std::string homedir = getenv("HOME");
    std::string root_path = homedir + "/catkin_ws/devel/lib/";
    std::string extension = "";
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
    loadFeatureModels (root_path, extension, models_fpath);

    for (int i = 0 ; i < models_fpath.size() ; i++ )
    {
        std::vector<std::string> strs;
        std::string path = models_fpath[i];
        boost::algorithm::split(strs,path,boost::algorithm::is_any_of("/"));

        std::string name = strs.at(strs.size()-1);
        models_ename.push_back(name);
        std::cout << name << std::endl;
    }

    std::cout<< models_ename.size() << std::endl;
    //=========================================================================
    root_path = homedir + "/catkin_ws/shell/";
    extension = ".sh";

    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
    loadFeatureModels (root_path, extension, shell_fpath);

    for (int i = 0 ; i < shell_fpath.size() ; i++ )
    {
        std::vector<std::string> strs;
        std::string path = shell_fpath[i];
        boost::algorithm::split(strs,path,boost::algorithm::is_any_of("/"));

        std::string name = strs.at(strs.size()-1);
        shell_ename.push_back(name);
        std::cout << name << std::endl;
    }

    std::cout<< shell_ename.size() << std::endl;

    //init nodes
    __pub_node_status = n.advertise<sepanta_msgs::nodestatuslist>("node_manager/node_status", 10);

    
    boost::thread _thread_wd(&publisher);

    sepanta_msgsMain sepanta_msgs(n);
    ros::NodeHandle _n;

    sub_log[0] = _n.subscribe("/core_kinect/log", 1, callback_log1);
    sub_log[1] = _n.subscribe("/core_motor/log", 1, callback_log2);
    sub_log[2] = _n.subscribe("/core_marker/log", 1, callback_log3);
    sub_log[3] = _n.subscribe("/core_skeleton/log", 1, callback_log4);
    sub_log[4] = _n.subscribe("/core_face/log", 1, callback_log5);
    sub_log[5] = _n.subscribe("/core_upperbody/log", 1, callback_log6);
    sub_log[6] = _n.subscribe("/core_stair/log", 1, callback_log7);
    sub_log[7] = _n.subscribe("/core_lowerbody/log", 1, callback_log8);
    sub_log[8] = _n.subscribe("/core_wavedetect/log", 1, callback_log9);
    sub_log[9] = _n.subscribe("/core_speech/log", 1, callback_log10);
    sub_log[10] = _n.subscribe("/core_sound/log", 1, callback_log11);
    sub_log[11] = _n.subscribe("/core_recordmotor/log", 1, callback_log12);
    sub_log[12] = _n.subscribe("/core_aux/log", 1, callback_log13);
    sub_log[13] = _n.subscribe("/core_object/log", 1, callback_log14);
    sub_log[14] = _n.subscribe("/senario_byby/log", 1, callback_log15);
    sub_log[15] = _n.subscribe("/senario_imitation/log", 1, callback_log16);
    sub_log[16] = _n.subscribe("/senario_anouncepose/log", 1, callback_log17);
    sub_log[17] = _n.subscribe("/senario_introduction/log", 1, callback_log18);
    sub_log[18] = _n.subscribe("/senario_handtrack/log", 1, callback_log19);
    sub_log[19] = _n.subscribe("/senario_walk/log", 1, callback_log20);
    sub_log[20] = _n.subscribe("/senario_stair/log", 1, callback_log21);
    sub_log[21] = _n.subscribe("/senario_logic2/log", 1, callback_log22);
    sub_log[22] =  _n.subscribe("/sample_stair/log", 1, callback_log23);
    sub_log[23] =  _n.subscribe("/sample_upperbody/log", 1, callback_log24);
    sub_log[24] =  _n.subscribe("/sample_lowerbody/log", 1, callback_log25);
    sub_log[25] =  _n.subscribe("/sample_sound/log", 1, callback_log26);
    sub_log[26] =  _n.subscribe("/sample_skeleton/log", 1, callback_log27);
    sub_log[27] =  _n.subscribe("/sample_recordmotor/log", 1, callback_log28);
    sub_log[28] =  _n.subscribe("/sample_speech/log", 1, callback_log29);
    sub_log[29] =  _n.subscribe("/sample_aux/log", 1, callback_log30);
    sub_log[30] =  _n.subscribe("/sample_object/log", 1, callback_log31);
    sub_log[31] =  _n.subscribe("/sample_wavedetect/log", 1, callback_log32);
    sub_log[32] =  _n.subscribe("/sample_face/log", 1, callback_log33);
    sub_log[33] =  _n.subscribe("/core_tcp/log", 1, callback_log34);

    // save_log(0);

    ros::spin();

    _thread_wd.interrupt();
    _thread_wd.join();

    App_exit = true;
    return 0;

}
