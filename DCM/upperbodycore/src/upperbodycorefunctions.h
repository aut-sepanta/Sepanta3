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

const int right_motor_count = 10;
const int left_motor_count = 10;
const int head_motor_count = 3;

int g_Motor_right[right_motor_count] = {0};
int g_Motortemp_right[right_motor_count] = {0};

int g_Motor_left[left_motor_count] = {0};
int g_Motortemp_left[left_motor_count] = {0};

int g_Motor_head[head_motor_count] = {0};
int g_Motortemp_head[head_motor_count] = {0};

int sp_Motor_right[right_motor_count] = {0};
int sp_Motortemp_right[right_motor_count] = {0};

int sp_Motor_left[left_motor_count] = {0};
int sp_Motortemp_left[left_motor_count] = {0};

int sp_Motor_head[head_motor_count] = {0};
int sp_Motortemp_head[head_motor_count] = {0};

//=================================================
//right

void chatterCallback_right_shoulder_yaw(const sepanta_msgs::motor::ConstPtr &msg)
{
    std::cout<<"get "<<msg->position<<" "<<msg->speed<<std::endl;
    g_Motor_right[0] = msg->position;
    sp_Motor_right[0] = msg->speed;
}

void chatterCallback_right_shoulder_pitch(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_right[2] = msg->position;
    sp_Motor_right[2] = msg->speed;
}

void chatterCallback_right_shoulder_roll(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_right[4] = msg->position;
    sp_Motor_right[4] = msg->speed;
}

void chatterCallback_right_elbow_pitch(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_right[5] = msg->position;
    sp_Motor_right[5] = msg->speed;
}

void chatterCallback_right_elbow_roll(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_right[6] = msg->position;
    sp_Motor_right[6] = msg->speed;
}

void chatterCallback_right_wrist_pitch(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_right[7] = msg->position;
    sp_Motor_right[7] = msg->speed;
}

void chatterCallback_right_wrist_roll(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_right[8] = msg->position;
    sp_Motor_right[8] = msg->speed;
}

void chatterCallback_right_gripper(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_right[9] = msg->position;
    sp_Motor_right[9] = msg->speed;
}

void reset_right_shoulder_yaw()
{
    g_Motor_right[0] = motorconfig_list.at(0).init;
}

void reset_right_shoulder_pitch()
{
    g_Motor_right[2] = motorconfig_list.at(2).init;
}

void reset_right_shoulder_roll()
{
    g_Motor_right[4] =  motorconfig_list.at(4).init;
}

void reset_right_elbow_pitch()
{
    g_Motor_right[5] = motorconfig_list.at(5).init;
}

void reset_right_elbow_roll()
{
    g_Motor_right[6] = motorconfig_list.at(6).init;
}

void reset_right_wrist_pitch()
{
    g_Motor_right[7] = motorconfig_list.at(7).init;
}

void reset_right_wrist_roll()
{
    g_Motor_right[8] = motorconfig_list.at(8).init;
}

void reset_right_gripper()
{
    g_Motor_right[9] = motorconfig_list.at(9).init;
}

//==============================================================
//left

void chatterCallback_left_shoulder_yaw(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[0] = msg->position;
    sp_Motor_left[0] = msg->speed;
}

void chatterCallback_left_shoulder_pitch(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[2] = msg->position;
    sp_Motor_left[2] = msg->speed;
}

void chatterCallback_left_shoulder_roll(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[4] = msg->position;
    sp_Motor_left[4] = msg->speed;
}

void chatterCallback_left_elbow_pitch(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[5] = msg->position;
    sp_Motor_left[5] = msg->speed;
}

void chatterCallback_left_elbow_roll(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[6] = msg->position;
    sp_Motor_left[6] = msg->speed;
}

void chatterCallback_left_wrist_pitch(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[7] = msg->position;
    sp_Motor_left[7] = msg->speed;
}

void chatterCallback_left_wrist_roll(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[8] = msg->position;
    sp_Motor_left[8] = msg->speed;
}

void chatterCallback_left_gripper(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_left[9] = msg->position;
    sp_Motor_left[9] = msg->speed;
}

void reset_left_shoulder_yaw()
{
    g_Motor_left[0] = motorconfig_list.at(10).init;
}

void reset_left_shoulder_pitch()
{
    g_Motor_left[2] = motorconfig_list.at(12).init;
}

void reset_left_shoulder_roll()
{
    g_Motor_left[4] = motorconfig_list.at(14).init;
}

void reset_left_elbow_pitch()
{
    g_Motor_left[5] = motorconfig_list.at(15).init;
}

void reset_left_elbow_roll()
{
    g_Motor_left[6] = motorconfig_list.at(16).init;
}

void reset_left_wrist_pitch()
{
    g_Motor_left[7] = motorconfig_list.at(17).init;
}

void reset_left_wrist_roll()
{
    g_Motor_left[8] = motorconfig_list.at(18).init;
}

void reset_left_gripper()
{
    g_Motor_left[9] = motorconfig_list.at(19).init;
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

void chatterCallback_waist(const sepanta_msgs::motor::ConstPtr &msg)
{
    g_Motor_head[2] = msg->position;
    sp_Motor_head[2] = msg->speed;
}

void reset_head_yaw()
{
    g_Motor_head[0] = motorconfig_list.at(20).init;
}

void reset_head_pitch()
{
    g_Motor_head[1] = motorconfig_list.at(21).init;
}

void reset_waist()
{
    g_Motor_head[2] = motorconfig_list.at(22).init;
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
        g_Motor_left[i] = motorconfig_list.at(i + 10).init;
}

void reset_head()
{
    for ( int i = 0 ; i < head_motor_count ; i++)
        g_Motor_head[i] = motorconfig_list.at(i + 20).init;
}



