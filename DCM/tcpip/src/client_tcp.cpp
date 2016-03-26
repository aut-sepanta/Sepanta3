
#include "client_tcp.h"

using namespace std;

void client_tcp::init()
{ 
    tcp_init = false;
    isconnected = false;
    iscommandthere = false;

    service_tcp = node_handles[1].serviceClient<sepanta_msgs::sound>("pgitic_service_tcp");
    sub_tcp = node_handles[2].subscribe("/pgitic_tcp_out", 1, &client_tcp::callback_tcp,this);
    sub_tcp_ack =  node_handles[3].subscribe("/core_tcp/ack", 1, &client_tcp::callback_tcpack,this);


}

void client_tcp::callback_tcp(const std_msgs::String::ConstPtr &msg)
{
   command = msg->data;
   iscommandthere = true;
}

void client_tcp::callback_tcpack(const std_msgs::String::ConstPtr &msg)
{
    if ( msg->data == "ok")
    {
        isconnected = true;
    }
    else
    {
        isconnected = false;
    }
}


void client_tcp::send_tcp(string msg)
{
    sepanta_msgs::sound _srv;
    _srv.request.command = msg;
    service_tcp.call(_srv);
}

string client_tcp::get_command()
{
    if ( isconnected )
    {
        while(ros::ok() && iscommandthere == false && isconnected)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }

    	iscommandthere = false;

        if ( isconnected && ros::ok() )
            return command;
        else
            return "null";

    }
    else
    {
    	return "null";
    }
}










