struct motor_data
{
public :
    std::string name;
    std::string status;
    int speed;
    int position;
    float load;
    int voltage;
    int temp;
    int id;
    int min;
    int max;
    int init;
    std::string model;
    int p;
    int i;
    int d;
};

struct motor_config
{
public :
    int id;
    std::string name;
    int max;
    int min;
    int init;
    double speed;
    std::string model;
};

std::vector<motor_data> motor_list;
std::vector<motor_config> motorconfig_list;

const int right_motor_count = 6;
const int left_motor_count = 6;
const int head_motor_count = 2;
const int total_motor_count = 14;

int g_Motor_left[left_motor_count] = {0};
int g_Motortemp_left[left_motor_count] = {0};

int g_Motor_right[right_motor_count] = {0};
int g_Motortemp_right[right_motor_count] = {0};

int g_Motor_head[head_motor_count] = {0};
int g_Motortemp_head[head_motor_count] = {0};

int sp_Motor_left[left_motor_count] = {0};
int sp_Motortemp_left[left_motor_count] = {0};

int sp_Motor_right[right_motor_count] = {0};
int sp_Motortemp_right[right_motor_count] = {0};

int sp_Motor_head[head_motor_count] = {0};
int sp_Motortemp_head[head_motor_count] = {0};

//=================================================
//left

void chatterCallback_right_1(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_right[0] = msg->position;
    sp_Motor_right[0] = msg->speed;
}

void chatterCallback_right_2(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_right[1] = msg->position;
    sp_Motor_right[1] = msg->speed;
}

void chatterCallback_right_3(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_right[2] = msg->position;
    sp_Motor_right[2] = msg->speed;
}

void chatterCallback_right_4(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_right[3] = msg->position;
    sp_Motor_right[3] = msg->speed;
}

void chatterCallback_right_5(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_right[4] = msg->position;
    sp_Motor_right[4] = msg->speed;
}

void chatterCallback_right_gripper(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_right[5] = msg->position;
    sp_Motor_right[5] = msg->speed;
}

void reset_right_1()
{
    g_Motor_right[0] = motorconfig_list.at(0).init;
}

void reset_right_2()
{
    g_Motor_right[1] = motorconfig_list.at(1).init;
}

void reset_right_3()
{
    g_Motor_right[2] = motorconfig_list.at(2).init;
}

void reset_right_4()
{
    g_Motor_right[3] = motorconfig_list.at(3).init;
}

void reset_right_5()
{
    g_Motor_right[4] = motorconfig_list.at(4).init;
}

void reset_right_gripper()
{
    g_Motor_right[5] = motorconfig_list.at(9).init;
}

//==============================================================
//left

void chatterCallback_left_1(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[0] = msg->position;
    sp_Motor_left[0] = msg->speed;
}

void chatterCallback_left_2(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[1] = msg->position;
    sp_Motor_left[1] = msg->speed;
}

void chatterCallback_left_3(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[2] = msg->position;
    sp_Motor_left[2] = msg->speed;
}

void chatterCallback_left_4(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[3] = msg->position;
    sp_Motor_left[3] = msg->speed;
}

void chatterCallback_left_5(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[4] = msg->position;
    sp_Motor_left[4] = msg->speed;
}


void chatterCallback_left_gripper(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[5] = msg->position;
    sp_Motor_left[5] = msg->speed;
}

void reset_left_1()
{
    g_Motor_left[0] = motorconfig_list.at(6).init;
}

void reset_left_2()
{
    g_Motor_left[1] = motorconfig_list.at(7).init;
}

void reset_left_3()
{
    g_Motor_left[2] = motorconfig_list.at(8).init;
}

void reset_left_4()
{
    g_Motor_left[3] = motorconfig_list.at(9).init;
}

void reset_left_5()
{
    g_Motor_left[4] = motorconfig_list.at(10).init;
}

void reset_left_gripper()
{
    g_Motor_left[5] = motorconfig_list.at(11).init;
}

//============================================================
//head

void chatterCallback_head_yaw(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_head[0] = msg->position;
    sp_Motor_head[0] = msg->speed;
}

void chatterCallback_head_pitch(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_head[1] = msg->position;
    sp_Motor_head[1] = msg->speed;
}

void reset_head_yaw()
{
    g_Motor_head[0] = motorconfig_list.at(12).init;
}

void reset_head_pitch()
{
    g_Motor_head[1] = motorconfig_list.at(13).init;
}
//===============================================================

void reset_right_arm()
{
    for ( int i = 0 ; i < right_motor_count ; i++)
        g_Motor_right[i] = motorconfig_list.at(i).init;
}

void reset_left_arm()
{
    for ( int i = 0 ; i < left_motor_count ; i++)
        g_Motor_left[i] = motorconfig_list.at(i + 6).init;
}

void reset_head()
{
    for ( int i = 0 ; i < head_motor_count ; i++)
        g_Motor_head[i] = motorconfig_list.at(i + 12).init;
}



