
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

#include <sepanta_msgs/managercommand.h>
#include <sepanta_msgs/NodeAction.h>
#include <sepanta_msgs/nodestatus.h>
#include <sepanta_msgs/nodestatuslist.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

class client_manager 
{

private:
    bool manager_init;
    ros::NodeHandle node_handles[100];
    ros::Subscriber sub_handles[50];

    ros::Subscriber sub_node_status;

    ros::Subscriber sub_log[34];
    std::vector<std::string> logs[34];
    std::string names[38];
    int process_size;
    void init();

    bool first_time;

    std::vector<std::string> run_vector;

    struct node_status
    {
    public :

        std::string name;
        std::string status;
        bool ack;
    };

    struct run_object
    {
    public :
        std::string name;
        int index;
    };

    std::vector<node_status> node_list;

public:

    ros::ServiceClient service_start;
    ros::ServiceClient service_stop;
    ros::ServiceClient service_startall;
    ros::ServiceClient service_stopall;

int task_start(string run,int max_try,int max_sec_timeout);
int task_stop(string run,int max_try,int max_sec_timeout);

bool process_start_timeout(string name,int index,int max_try,int max_sec_timeout);
bool process_stop_timeout(string name,int index,int max_try,int max_sec_timeout);

void process_start(string name);
void process_stop(string name);
void process_startall(string name);
void process_stopall(string name);
int  process_getStatus(string name);
bool task_getStatus(string name);

void callback_manager(const sepanta_msgs::nodestatuslist::ConstPtr &msg);

client_manager()
{
    manager_init = false;

    init();
}

~client_manager()
{

}

};
