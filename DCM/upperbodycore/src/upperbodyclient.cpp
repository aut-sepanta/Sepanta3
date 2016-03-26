#include "upperbodyclient.h"

void upperbodyclient::init()
{
    grip_wait = false;
    chatter_pub_motor_right[0] = node_handles[0].advertise<sepanta_msgs::motor>("upperbodycorein_right_shoulder_yaw", 10);
    chatter_pub_motor_right[1] = node_handles[1].advertise<sepanta_msgs::motor>("upperbodycorein_right_shoulder_pitch", 10);
    chatter_pub_motor_right[2] = node_handles[2].advertise<sepanta_msgs::motor>("upperbodycorein_right_shoulder_roll", 10);
    chatter_pub_motor_right[3] = node_handles[3].advertise<sepanta_msgs::motor>("upperbodycorein_right_elbow_pitch", 10);
    chatter_pub_motor_right[4] = node_handles[4].advertise<sepanta_msgs::motor>("upperbodycorein_right_elbow_roll", 10);
    chatter_pub_motor_right[5] = node_handles[5].advertise<sepanta_msgs::motor>("upperbodycorein_right_wrist_pitch", 10);
    chatter_pub_motor_right[6] = node_handles[6].advertise<sepanta_msgs::motor>("upperbodycorein_right_wrist_roll", 10);
    chatter_pub_motor_right[7] = node_handles[7].advertise<sepanta_msgs::motor>("upperbodycorein_right_gripper", 10);

    chatter_pub_motor_left[0] = node_handles[8].advertise<sepanta_msgs::motor>("upperbodycorein_left_shoulder_yaw", 10);
    chatter_pub_motor_left[1] = node_handles[9].advertise<sepanta_msgs::motor>("upperbodycorein_left_shoulder_pitch", 10);
    chatter_pub_motor_left[2] = node_handles[10].advertise<sepanta_msgs::motor>("upperbodycorein_left_shoulder_roll", 10);
    chatter_pub_motor_left[3] = node_handles[11].advertise<sepanta_msgs::motor>("upperbodycorein_left_elbow_pitch", 10);
    chatter_pub_motor_left[4] = node_handles[12].advertise<sepanta_msgs::motor>("upperbodycorein_left_elbow_roll", 10);
    chatter_pub_motor_left[5] = node_handles[13].advertise<sepanta_msgs::motor>("upperbodycorein_left_wrist_pitch", 10);
    chatter_pub_motor_left[6] = node_handles[14].advertise<sepanta_msgs::motor>("upperbodycorein_left_wrist_roll", 10);
    chatter_pub_motor_left[7] = node_handles[15].advertise<sepanta_msgs::motor>("upperbodycorein_left_gripper", 10);

    chatter_pub_motor_head[0] = node_handles[16].advertise<sepanta_msgs::motor>("upperbodycorein_head_yaw", 10);
    chatter_pub_motor_head[1] = node_handles[17].advertise<sepanta_msgs::motor>("upperbodycorein_head_pitch", 10);
    chatter_pub_motor_head[2] = node_handles[17].advertise<sepanta_msgs::motor>("upperbodycorein_waist", 10);

    chatter_pub_allmotors_right = node_handles[18].advertise<sepanta_msgs::upperbodymotors>("/upperbodycorein_right_motors", 10);
    chatter_pub_allmotors_left = node_handles[19].advertise<sepanta_msgs::upperbodymotors>("/upperbodycorein_left_motors", 10);
    chatter_pub_allmotors_head = node_handles[20].advertise<sepanta_msgs::upperbodymotors>("/upperbodycorein_head_motors", 10);

    service_reset_motor = node_handles[21].serviceClient<sepanta_msgs::motorreset>("upperbodycorein_resetmotor");
    service_pid_motor = node_handles[23].serviceClient<sepanta_msgs::motorpid>("upperbodycorein_pidmotor");
    sub_motors = node_handles[22].subscribe("/upperbodycoreout_feedback", 1, &upperbodyclient::callback_motors,this);
    service_torquemotor = node_handles[24].serviceClient<sepanta_msgs::motortorque>("upperbodycorein_torquemotor");
    service_grip = node_handles[25].serviceClient<sepanta_msgs::grip>("/grip/position");

    service_amg = node_handles[26].serviceClient<sepanta_msgs::grip>("/grip/amg");
    sub_gripwait = node_handles[27].subscribe("/grip/busy", 1, &upperbodyclient::chatterCallback_gripbusy,this);

}

std::string upperbodyclient::grip_right(float X, float Y, float Z, float alpha, float beta, float gama)
{
    // double X=0.49;
    // double Y=-0.38;
    // double Z=-0.23;
    // double A=0*pi/180;
    // double B=-90*pi/180;
    // double G=0*pi/180;

    std::cout<<"test ..."<<std::endl;
    sepanta_msgs::grip msg;
    msg.request.x = (float)(X);
    msg.request.y = (float)(Y) ;
    msg.request.z = (float)(Z) ;
    msg.request.a = (float)(alpha) ;
    msg.request.b = (float)(beta) ;
    msg.request.g = (float)(gama);
    msg.request.right_left = "right";

    service_grip.call(msg);


    return msg.response.result;
}

std::string upperbodyclient::grip_left(float X, float Y, float Z, float alpha, float beta, float gama)
{
    // double X=0.49;
    // double Y=-0.38;
    // double Z=-0.23;
    // double A=0*pi/180;
    // double B=-90*pi/180;
    // double G=0*pi/180;

    std::cout<<"test ..."<<std::endl;
    sepanta_msgs::grip msg;
    msg.request.x = (float)(X);
    msg.request.y = (float)(Y) ;
    msg.request.z = (float)(Z) ;
    msg.request.a = (float)(alpha) ;
    msg.request.b = (float)(beta) ;
    msg.request.g = (float)(gama);
    msg.request.right_left = "left";

    service_grip.call(msg);


    return msg.response.result;
}

void upperbodyclient::callback_motors(const sepanta_msgs::upperbodymotorsfeedback::ConstPtr &msg)
{

    if ( motor_init == false)
    {
        for ( int i = 0 ; i < msg->motorfeedbacks.size() ; i++ )
        {
            motor_data motor;
            motor.speed = -1;
            motor.position = -1;
            motor.load = -1;
            motor.voltage = -1;
            motor.id = -1;
            motor.temp = -1;
            motor.model = "none";
            motor.name = "none";
            motor.status = "not found";
            motor.max = -1;
            motor.min = -1;
            motor.init = -1;
            allMotors.push_back(motor);
        }
    }

    for ( int i = 0 ; i < msg->motorfeedbacks.size() ; i++ )
    {
        allMotors.at(i).speed = msg->motorfeedbacks[i].speed;
        allMotors.at(i).position = msg->motorfeedbacks[i].position;
        allMotors.at(i).load = msg->motorfeedbacks[i].load;
        allMotors.at(i).voltage = msg->motorfeedbacks[i].voltage;
        allMotors.at(i).id = msg->motorfeedbacks[i].id;
        allMotors.at(i).temp = msg->motorfeedbacks[i].temp;
        allMotors.at(i).name = msg->motorfeedbacks[i].name;
        allMotors.at(i).status = msg->motorfeedbacks[i].status;
        allMotors.at(i).min = msg->motorfeedbacks[i].min;
        allMotors.at(i).max = msg->motorfeedbacks[i].max;
        allMotors.at(i).init = msg->motorfeedbacks[i].init;
        allMotors.at(i).model = msg->motorfeedbacks[i].model;
    }

    motor_init = true;
}

void upperbodyclient::setMotor_rightShoulderYaw(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_right[0].publish(msg);
}

void upperbodyclient::setMotor_rightShoulderPitch(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_right[1].publish(msg);
}

void upperbodyclient::setMotor_rightShoulderRoll(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_right[2].publish(msg);
}

void upperbodyclient::setMotor_rightElbowPitch(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_right[3].publish(msg);
}

void upperbodyclient::setMotor_rightElbowRoll(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_right[4].publish(msg);
}

void upperbodyclient::setMotor_rightWristPitch(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_right[5].publish(msg);
}

void upperbodyclient::setMotor_rightWristRoll(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_right[6].publish(msg);
}

void upperbodyclient::setMotor_rightGripper(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_right[7].publish(msg);
}

void upperbodyclient::setMotors_rightArm(int positions[8], int speeds[8])
{
    sepanta_msgs::upperbodymotors motor_msg;

    motor_msg.shoulder_yawm_position = positions[0];
    motor_msg.shoulder_pitchm_position = positions[1];
    motor_msg.shoulder_roll_position = positions[2];
    motor_msg.elbow_pitch_position = positions[3];
    motor_msg.elbow_roll_position = positions[4];
    motor_msg.wrist_pitch_position = positions[5];
    motor_msg.wrist_roll_position = positions[6];
    motor_msg.gripper_position = positions[7];

    motor_msg.shoulder_yawm_speed = speeds[0];
    motor_msg.shoulder_pitchm_speed = speeds[1];
    motor_msg.shoulder_roll_speed = speeds[2];
    motor_msg.elbow_pitch_speed = speeds[3];
    motor_msg.elbow_roll_speed = speeds[4];
    motor_msg.wrist_pitch_speed = speeds[5];
    motor_msg.wrist_roll_speed = speeds[6];
    motor_msg.gripper_speed = speeds[7];

    chatter_pub_allmotors_right.publish(motor_msg);
}

void upperbodyclient::resetMotor_rightShoulderYaw()
{
    reset.request.id = "rightShoulderYaw";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_rightShoulderPitch()
{
    reset.request.id = "rightShoulderPitch";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_rightShoulderRoll()
{
    reset.request.id = "rightShoulderRoll";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_rightElbowPitch()
{
    reset.request.id = "rightElbowPitch";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_rightElbowRoll()
{
    reset.request.id = "rightElbowRoll";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_rightWristPitch()
{
    reset.request.id = "rightWristPitch";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_rightWristRoll()
{
    reset.request.id = "rightWristRoll";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_rightGripper()
{
    reset.request.id = "rightGripper";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_rightArm()
{
    reset.request.id = "rightArm";
    service_reset_motor.call(reset);
}

//=====================================

void upperbodyclient::setMotor_leftShoulderYaw(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_left[0].publish(msg);
}

void upperbodyclient::setMotor_leftShoulderPitch(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_left[1].publish(msg);
}

void upperbodyclient::setMotor_leftShoulderRoll(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_left[2].publish(msg);
}

void upperbodyclient::setMotor_leftElbowPitch(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_left[3].publish(msg);
}

void upperbodyclient::setMotor_leftElbowRoll(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_left[4].publish(msg);
}

void upperbodyclient::setMotor_leftWristPitch(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_left[5].publish(msg);
}

void upperbodyclient::setMotor_leftWristRoll(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_left[6].publish(msg);
}

void upperbodyclient::setMotor_leftGripper(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_left[7].publish(msg);
}

void upperbodyclient::setMotors_leftArm(int positions[8], int speeds[8])
{
    sepanta_msgs::upperbodymotors motor_msg;

    motor_msg.shoulder_yawm_position = positions[0];
    motor_msg.shoulder_pitchm_position = positions[1];
    motor_msg.shoulder_roll_position = positions[2];
    motor_msg.elbow_pitch_position = positions[3];
    motor_msg.elbow_roll_position = positions[4];
    motor_msg.wrist_pitch_position = positions[5];
    motor_msg.wrist_roll_position = positions[6];
    motor_msg.gripper_position = positions[7];

    motor_msg.shoulder_yawm_speed = speeds[0];
    motor_msg.shoulder_pitchm_speed = speeds[1];
    motor_msg.shoulder_roll_speed = speeds[2];
    motor_msg.elbow_pitch_speed = speeds[3];
    motor_msg.elbow_roll_speed = speeds[4];
    motor_msg.wrist_pitch_speed = speeds[5];
    motor_msg.wrist_roll_speed = speeds[6];
    motor_msg.gripper_speed = speeds[7];

    chatter_pub_allmotors_left.publish(motor_msg);
}

void upperbodyclient::resetMotor_leftShoulderYaw()
{
    reset.request.id = "leftShoulderYaw";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_leftShoulderPitch()
{
    reset.request.id = "leftShoulderPitch";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_leftShoulderRoll()
{
    reset.request.id = "leftShoulderRoll";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_leftElbowPitch()
{
    reset.request.id = "leftElbowPitch";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_leftElbowRoll()
{
    reset.request.id = "leftElbowRoll";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_leftWristPitch()
{
    reset.request.id = "leftWristPitch";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_leftWristRoll()
{
    reset.request.id = "leftWristRoll";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_leftGripper()
{
    reset.request.id = "leftGripper";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_leftArm()
{
    reset.request.id = "leftArm";
    service_reset_motor.call(reset);
}

//==================================
void upperbodyclient::setMotor_headYaw(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_head[0].publish(msg);
}

void upperbodyclient::setMotor_headPitch(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_head[1].publish(msg);
}

void upperbodyclient::setMotor_waist(int position,int speed)
{
    sepanta_msgs::motor msg;
    msg.position = position;
    msg.speed = speed;
    chatter_pub_motor_head[2].publish(msg);
}

void upperbodyclient::setMotors_head(int positions[2], int speeds[2])
{
    sepanta_msgs::upperbodymotors motor_msg;

    motor_msg.head_yaw_position = positions[0];
    motor_msg.head_pitch_position = positions[1];

    motor_msg.head_yaw_speed = speeds[0];
    motor_msg.head_pitch_speed = speeds[1];

    chatter_pub_allmotors_head.publish(motor_msg);
}

void upperbodyclient::resetMotor_headYaw()
{
    reset.request.id = "headYaw";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_headPitch()
{
    reset.request.id = "headPitch";
    service_reset_motor.call(reset);
}

void upperbodyclient::resetMotor_waist()
{
    reset.request.id = "waist";
    service_reset_motor.call(reset);
}
//========================================================

upperbodyclient::motor_data upperbodyclient::getMotor_rightShoulderYaw()
{
    motor_data item;
    item = allMotors.at(0);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_rightShoulderPitch()
{
    motor_data item;
    item = allMotors.at(2);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_rightShoulderRoll()
{
    motor_data item;
    item = allMotors.at(4);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_rightElbowPitch()
{
    motor_data item;
    item = allMotors.at(5);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_rightElbowRoll()
{
    motor_data item;
    item = allMotors.at(6);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_rightWristPitch()
{
    motor_data item;
    item = allMotors.at(7);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_rightWristRoll()
{
    motor_data item;
    item = allMotors.at(8);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_rightGripper()
{
    motor_data item;
    item = allMotors.at(9);
    return item;
}

//-


upperbodyclient::motor_data upperbodyclient::getMotor_leftShoulderYaw()
{
    motor_data item;
    item = allMotors.at(10);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_leftShoulderPitch()
{
    motor_data item;
    item = allMotors.at(12);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_leftShoulderRoll()
{
    motor_data item;
    item = allMotors.at(14);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_leftElbowPitch()
{
    motor_data item;
    item = allMotors.at(15);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_leftElbowRoll()
{
    motor_data item;
    item = allMotors.at(16);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_leftWristPitch()
{
    motor_data item;
    item = allMotors.at(17);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_leftWristRoll()
{
    motor_data item;
    item = allMotors.at(18);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_leftGripper()
{
    motor_data item;
    item = allMotors.at(19);
    return item;
}

//-

upperbodyclient::motor_data upperbodyclient::getMotor_headYaw()
{
    motor_data item;
    item = allMotors.at(20);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_headPitch()
{
    motor_data item;
    item = allMotors.at(21);
    return item;
}

upperbodyclient::motor_data upperbodyclient::getMotor_waist()
{
    motor_data item;
    item = allMotors.at(22);
    return item;
}

//=========== * pid

void upperbodyclient::setMotorPid_rightShoulderYaw(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "rightShoulderYawm";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);

}

void upperbodyclient::setMotorPid_rightShoulderPitch(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "rightShoulderPitchm";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);

}

void upperbodyclient::setMotorPid_rightShoulderRoll(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "rightShoulderRoll";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);

}

void upperbodyclient::setMotorPid_rightElbowPitch(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "rightElbowPitch";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}
bool enable_usb = false;
void upperbodyclient::setMotorPid_rightElbowRoll(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "rightElbowRoll";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

void upperbodyclient::setMotorPid_rightWristPitch(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "rightWristPitch";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

void upperbodyclient::setMotorPid_rightWristRoll(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "rightWristRoll";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

void upperbodyclient::setMotorPid_rightGripper(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "rightGripper";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

//-

void upperbodyclient::setMotorPid_leftShoulderYaw(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "leftShoulderYawm";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

void upperbodyclient::setMotorPid_leftShoulderPitch(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "leftShoulderPitchm";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

void upperbodyclient::setMotorPid_leftShoulderRoll(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "leftShoulderRoll";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

void upperbodyclient::setMotorPid_leftElbowPitch(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "leftElbowPitch";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

void upperbodyclient::setMotorPid_leftElbowRoll(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "leftElbowRoll";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

void upperbodyclient::setMotorPid_leftWristPitch(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "leftWristPitch";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

void upperbodyclient::setMotorPid_leftWristRoll(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "leftWristRoll";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

void upperbodyclient::setMotorPid_leftGripper(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "leftGripper";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

//-

void upperbodyclient::setMotorPid_headPitch(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "headPitch";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

void upperbodyclient::setMotorPid_headYaw(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "headYaw";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

void upperbodyclient::setMotorPid_waist(uint8_t i,uint8_t p,uint8_t d)
{
    pid.request.id = "waist";
    pid.request.margin = i;
    pid.request.slope = p;
    pid.request.punch = d;
    service_pid_motor.call(pid);
}

//=========== p;id

void upperbodyclient::torqueToggle_allMotors(bool value)
{
    torque.request.status = value;
    service_torquemotor.call(torque);
}

//============= left right

void upperbodyclient::left_down()
{
    resetMotor_leftArm();
}

void upperbodyclient::left_up()
{
    setMotor_leftShoulderYaw(2390,500);
    setMotor_leftShoulderPitch(1640,500);
    setMotor_leftShoulderRoll(2446,500);
    setMotor_leftElbowPitch(2302,500);
    setMotor_leftElbowRoll(1914,500);
    setMotor_leftWristPitch(1291,500);
    setMotor_leftWristRoll(991,500);
}

void upperbodyclient::left_front()
{
    setMotor_leftShoulderYaw(1836,500);
    setMotor_leftShoulderRoll(2432,500);
    // boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    setMotor_leftShoulderPitch(1003,500);
    setMotor_leftElbowPitch(1946,500);
    setMotor_leftElbowRoll(1914,500);
    setMotor_leftWristPitch(1291,500);
    setMotor_leftWristRoll(991,500);
}

void upperbodyclient::left_side()
{
    setMotor_leftShoulderYaw(2390,500);
    setMotor_leftShoulderPitch(1660,500);
    setMotor_leftShoulderRoll(3800,500);
    setMotor_leftElbowPitch(1356,500);
    setMotor_leftElbowRoll(1914,500);
    setMotor_leftWristPitch(1291,500);
    setMotor_leftWristRoll(991,500);
}

//============================

void upperbodyclient::right_down()
{
    resetMotor_rightArm();
}

void upperbodyclient::right_up()
{
    setMotor_rightShoulderYaw(1660,500);
    setMotor_rightShoulderPitch(2590,500);
    setMotor_rightShoulderRoll(1687,500);
    setMotor_rightElbowPitch(2004,500);
    setMotor_rightElbowRoll(2594,500);
    setMotor_rightWristPitch(2217,500);
    setMotor_rightWristRoll(1442,500);
}

void upperbodyclient::right_front()
{
    setMotor_rightShoulderYaw(2135,500);
    setMotor_rightShoulderRoll(1540,500);
    //     boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    setMotor_rightShoulderPitch(3360,500);
    setMotor_rightElbowPitch(1724,500);
    setMotor_rightElbowRoll(2594,500);
    setMotor_rightWristPitch(2217,500);
    setMotor_rightWristRoll(1442,500);
}

void upperbodyclient::right_side()
{
    setMotor_rightShoulderYaw(1660,500);
    setMotor_rightShoulderPitch(2590,500);
    setMotor_rightShoulderRoll(1109,500);
    setMotor_rightElbowPitch(1107,500);
    setMotor_rightElbowRoll(2594,500);
    setMotor_rightWristPitch(2217,500);
    setMotor_rightWristRoll(1442,500);
}

bool upperbodyclient::getgripwait()
{
    return grip_wait;
}

void upperbodyclient::setamg(std::string command)
{
    sepanta_msgs::grip gsrv;
    gsrv.request.right_left = command;
    service_amg.call(gsrv);
}

void upperbodyclient::chatterCallback_gripbusy(const std_msgs::String::ConstPtr &msg)
{
    if ( msg->data == "true")
    {
        grip_wait = true;
    }
    else
    {
        grip_wait = false;
    }
}

