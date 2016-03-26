#include "ros/ros.h"
#include <ros/package.h>
#include "rqt_camera_calib_gui/CameraCalibView.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <sepanta_msgs/arm.h>
#include <sepanta_msgs/omnidata.h>
#include <sepanta_msgs/head.h>
#include <sepanta_msgs/irsensor.h>
#include <sepanta_msgs/motortorques.h>
#include <std_msgs/Int32.h>
#include <QTimer>
#include <QDebug>
#include <QFile>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sepanta_msgs/slamactionAction.h>

#include <QProcess>
#include <QThread>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "std_msgs/String.h"

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "sepanta_msgs/stop.h" //facestop and manualauto service
#include "sepanta_msgs/command.h" //command service
#include "sepanta_msgs/maptools.h" //command service

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace rqt_camera_calib_gui
{

struct goal_data
{
  public :
    float x; //cm
    float y; //cm
    int yaw; //angle
    std::string id;
};

std::vector<goal_data> goal_list;

int reset_kinect = 0;

ros::Publisher chatter_pub1;
ros::Publisher chatter_pub2;
ros::Publisher chatter_pub3;
ros::Publisher chatter_pub4;
ros::Publisher chatter_pub5;
ros::Publisher chatter_pub6;
ros::Publisher chatter_pub7;
ros::Publisher chatter_pub8;
ros::Publisher chatter_pub9;

ros::ServiceClient serviceclient_odometryslam;
ros::ServiceClient serviceclient_facestop;
ros::ServiceClient serviceclient_manualauto;
ros::ServiceClient serviceclient_map;


int speedx = 150;
int speedy = 150;
int speedt = -100;

int Compass = -1;
int Currentz = -1;

int slam_position_yaw[3] = {-1,-1,-1};
int laser_IR[8] = {250,250,250,250,250,250,250,250};
int s_Motor[4] = {128,128,128,128};
int c_Motor[4] = {512,512,512,512};

long basic_time;

QTimer *mtimer;
QTimer *mtimer_basicfunction;

QProcess *sepanta_process[21];
QString sepanta_process_commands[21];

CameraCalibView::CameraCalibView()
: rqt_gui_cpp::Plugin()
, widget(0)
{
	setObjectName("CameraCalibView");
}

float ConvertQuatToYaw(const geometry_msgs::Quaternion &quat)
{
    float yaw = tf::getYaw(quat);
    return isnan(yaw) ? 0 : yaw;
}

inline float Deg2Rad(float deg)
{
    return deg * M_PI / 180;
}

inline float Rad2Deg(float rad)
{
    return rad * 180 / M_PI;
}

void CameraCalibView::process_init()
{
    sepanta_process_commands[0] = "roslaunch athomerobot laser.launch";
    sepanta_process_commands[1] = "roslaunch openni_launch openni.launch";
    sepanta_process_commands[2] = "roslaunch athomerobot motor.launch";
    sepanta_process_commands[3] = "rosrun athomerobot athomerobot";
    sepanta_process_commands[4] = "rosrun athomerobot odometry";
    sepanta_process_commands[5] = "rosrun athomerobot slam";
    sepanta_process_commands[6] = "rosrun athomerobot slam_client";
    sepanta_process_commands[7] = "roslaunch athomerobot hector_slam.launch";
    sepanta_process_commands[8] = "roslaunch athomerobot move_base.launch";
    sepanta_process_commands[9] = "rosrun map_server map_server";
    sepanta_process_commands[10] = "rosrun pick_and_place logic_pick_place";
    sepanta_process_commands[11] = "rosrun pick_and_place object_recognition_shotcolor";
    sepanta_process_commands[12] = "rosrun pick_and_place grip_pick_place";
    sepanta_process_commands[13] = "rosrun pick_and_place build_pcd_list_pic";
    sepanta_process_commands[14] = "rosrun avoidthat avoidthat";
    sepanta_process_commands[15] = "rosrun what_did_you_say what_did_you_say_logic";
    sepanta_process_commands[16] = "rosrun what_did_you_say find_person";
    sepanta_process_commands[17] = "rosrun athomerobot communication";
    sepanta_process_commands[18] = "rosrun athomerobot gesture";
    sepanta_process_commands[19] = "rosrun basic_function client_stop";
    sepanta_process_commands[20] = "rosrun basic_function basicFunction";

    for ( int i = 0 ; i < 21 ; i++ )
    {
        sepanta_process[i] = new QProcess(this);
    }

}

void CameraCalibView::rosImageCallBack(const sensor_msgs::ImageConstPtr &msg)
{
   reset_kinect = 0;
}

void CameraCalibView::process_start(int id)
{

       //QString   prog  = "gnome-terminal";
       //QStringList args;
       //args << "-c" << "rosrun athomerobot athomerobot";
       //sepanta_process[id]->start(prog, args);
       //sepanta_process[id]->waitForFinished();

}

void CameraCalibView::process_kill(int id)
{
    sepanta_process[id]->terminate();
    sepanta_process[id]->kill();

}

void CameraCalibView::set_omni(int x,int y,int w)
{
 sepanta_msgs::omnidata msg;
 msg.d0 = x;
 msg.d1 = y;
 msg.d2 = w;

 chatter_pub1.publish(msg);
}

void CameraCalibView::robot_forward()
{
    CameraCalibView::set_omni(speedx,0,0);
}

void CameraCalibView::robot_backward()
{
    CameraCalibView::set_omni(-speedx,0,0);
}

void CameraCalibView::robot_left()
{
    CameraCalibView::set_omni(0,speedy,0);
}

void CameraCalibView::robot_right()
{
    CameraCalibView::set_omni(0,-speedy,0);
}

void CameraCalibView::robot_turn_left()
{
   CameraCalibView::set_omni(0,0,speedt);
}

void CameraCalibView::robot_turn_right()
{
   CameraCalibView::set_omni(0,0,-speedt);
}

void CameraCalibView::robot_stop()
{
   CameraCalibView::set_omni(0,0,0);
}

void CameraCalibView::facestart()
{
   sepanta_msgs::stop srv_stop;
   srv_stop.request.command = "Start";
   serviceclient_facestop.call(srv_stop);
}


void CameraCalibView::device_start()
{
   process_start(0);
   process_start(1);
   process_start(2);
}

void CameraCalibView::device_stop()
{
   process_kill(0);
   process_kill(1);
   process_kill(2);
}

void CameraCalibView::core_start()
{
    process_start(3);
    process_start(4);
    process_start(5);
    process_start(6);
}

void CameraCalibView::core_stop()
{
    process_kill(3);
    process_kill(4);
    process_kill(5);
    process_kill(6);
}

void CameraCalibView::map_start()
{
    process_start(7);
    process_start(8);
    process_start(9);
}

void CameraCalibView::map_stop()
{
    process_kill(7);
    process_kill(8);
    process_kill(9);
}

void CameraCalibView::node_start()
{
    process_start(10);
    process_start(11);
    process_start(12);
    process_start(13);
    process_start(14);
    process_start(15);
    process_start(16);
    process_start(17);
    process_start(18);
    process_start(19);

}

void CameraCalibView::node_stop()
{
    process_kill(10);
    process_kill(11);
    process_kill(12);
    process_kill(13);
    process_kill(14);
    process_kill(15);
    process_kill(16);
    process_kill(17);
    process_kill(18);
    process_kill(19);
}

void CameraCalibView::basic_start()
{
process_start(20);
}

void CameraCalibView::basic_stop()
{
process_kill(20);
}

bool kinect_connect = false;
bool kinect_watchdogen = false;

void CameraCalibView::kinect_watchdog()
{
   kinect_watchdogen = true;
}

//void CameraCalibView::pstart()
//{



   //connect(sepanta_process[0], SIGNAL(readyReadStandardError()), this, SLOT(wrongmessage()));

//    //    child = new QProcess(this);
//    //    QString   prog  = "gnome-terminal";
//    //    QStringList args;
//    //    args << "-e" << "/home/edwin/robot.sh";
//    //    // set the process env and working dir
//    //    child->start(prog, args);


//    //    QString   prog  = "gnome-terminal";
//    //    QStringList args;
//    //    args << "-e" << "rosrun athomerobot athomerobot";
//    //    // set the process env and working dir
//    //    child->start(prog, args);


//    //    child->waitForFinished(); // didn't work without this

//    //    QTextStream stream(child);
//    //    while (!stream.atEnd()) {
//    //        ui.list_log->addItem(cstream.readLine());
//    //    }

//        \
//}

void CameraCalibView::savemap()
{
   sepanta_msgs::maptools srv_map;
   srv_map.request.command = "savepgmmap";
   serviceclient_map.call(srv_map);


}

void CameraCalibView::facestop()
{
   sepanta_msgs::stop srv_stop;
   srv_stop.request.command = "Stop";
   serviceclient_facestop.call(srv_stop);
}

void CameraCalibView::manualmode()
{
     sepanta_msgs::stop srv_stop;
   srv_stop.request.command = "Manual";
   serviceclient_manualauto.call(srv_stop);
}

void CameraCalibView::automode()
{
   sepanta_msgs::stop srv_stop;
   srv_stop.request.command = "Slam";
   serviceclient_manualauto.call(srv_stop);
}

void CameraCalibView::manualreached()
{
   sepanta_msgs::stop srv_stop;
   srv_stop.request.command = "ManualReached";
   serviceclient_manualauto.call(srv_stop);
}

void CameraCalibView::movex()
{
    sepanta_msgs::command srv_command;
    srv_command.request.command ="movex";
    srv_command.request.value = ui.spin_movex->value();
    serviceclient_odometryslam.call(srv_command);
}

void CameraCalibView::movey()
{
    sepanta_msgs::command srv_command;
    srv_command.request.command ="movey";
    srv_command.request.value = ui.spin_movey->value();
    serviceclient_odometryslam.call(srv_command);
}

void CameraCalibView::turngl()
{
    sepanta_msgs::command srv_command;
    srv_command.request.command ="turngl";
    srv_command.request.value = ui.spin_turngl->value();
    serviceclient_odometryslam.call(srv_command);
}

void CameraCalibView::turngllocal()
{
    sepanta_msgs::command srv_command;
    srv_command.request.command ="turngllocal";
    srv_command.request.value = ui.spin_turngllocal->value();
    serviceclient_odometryslam.call(srv_command);
}

void CameraCalibView::odometrycancle()
{
    sepanta_msgs::command srv_command;
    srv_command.request.command ="odometrycancle";
    serviceclient_odometryslam.call(srv_command);
}

void CameraCalibView::gotolocation()
{
    sepanta_msgs::command srv_command;
    srv_command.request.command ="goto";
    srv_command.request.id = ui.txt_where->toPlainText().toStdString();
    serviceclient_odometryslam.call(srv_command);
}

void CameraCalibView::slamcancle()
{
    sepanta_msgs::command srv_command;
    srv_command.request.command ="gotocancle";
    serviceclient_odometryslam.call(srv_command);
}

void CameraCalibView::point_add()
{
    goal_data gdata;
    gdata.id = ui.txt_point_id->toPlainText().toStdString();
    gdata.x = ui.spin_point_x->value();
    gdata.y = ui.spin_point_y->value();
    gdata.yaw = ui.spin_point_yaw->value();


    goal_list.push_back(gdata);
    /////////////////////////////////////////
    QString str = QString::fromUtf8(gdata.id.c_str());
    ui.list_points->addItem(str);


    point_save();

}

void CameraCalibView::point_addcurrent()
{

    goal_data gdata;
    int size = goal_list.size();

    size++;

    QString str = QString::number(size);
    QString name = "point";
    gdata.id =  name.toStdString() + str.toStdString();

    gdata.x = slam_position_yaw[0];
    gdata.y = slam_position_yaw[1];
    gdata.yaw = slam_position_yaw[2];

    goal_list.push_back(gdata);

    QString strf = QString::fromUtf8(gdata.id.c_str());
    ui.list_points->addItem(strf);

    point_save();
}

void CameraCalibView::point_delete()
{

    int index = ui.list_points->currentRow();

    if ( index != -1 )
    {


        std::vector<goal_data> goal_list_temp;
        int i = 0;
        for (  ; i < index ; i++)
        {
            goal_data gdata = goal_list.at(i);
            goal_list_temp.push_back(gdata);
        }


        i++;

        for ( ; i < goal_list.size();i++)
        {
            goal_data gdata = goal_list.at(i);
            goal_list_temp.push_back(gdata);
        }


        goal_list = goal_list_temp;

        ui.list_points->clear();

        for ( int i = 0 ; i < goal_list.size() ; i++)
        {
            goal_data gdata = goal_list.at(i);
            QString str = QString::fromUtf8(gdata.id.c_str());
            ui.list_points->addItem(str);
        }
    }

   point_save();
}

void CameraCalibView::point_edit()
{
    int index = ui.list_points->currentRow();

    if ( index != -1 )
    {
        std::vector<goal_data> goal_list_temp;
        int i = 0;
        for (  ; i < index ; i++)
        {
            goal_data gdata = goal_list.at(i);
            goal_list_temp.push_back(gdata);
        }

        goal_data cgdata;
        cgdata.id = ui.txt_point_id->toPlainText().toStdString();
        cgdata.x = ui.spin_point_x->value();
        cgdata.y = ui.spin_point_y->value();
        cgdata.yaw = ui.spin_point_yaw->value();
        goal_list_temp.push_back(cgdata);
        i++;

        for ( ; i < goal_list.size();i++)
        {
            goal_data gdata = goal_list.at(i);
            goal_list_temp.push_back(gdata);
        }


        goal_list = goal_list_temp;

        ui.list_points->clear();

        for ( int i = 0 ; i < goal_list.size() ; i++)
        {
            goal_data gdata = goal_list.at(i);
            QString str = QString::fromUtf8(gdata.id.c_str());
            ui.list_points->addItem(str);
        }
    }

   point_save();
}

void CameraCalibView::point_selected()
{
  int index = ui.list_points->currentRow();

  if ( index != -1 )
  {
      goal_data gdata = goal_list.at(index);
      ui.txt_point_id->setText(gdata.id.c_str());
      ui.spin_point_x->setValue(gdata.x);
      ui.spin_point_y->setValue(gdata.y);
      ui.spin_point_yaw->setValue(gdata.yaw);

  }
}

void CameraCalibView::point_save()
{

  std::string path_points = ros::package::getPath("athomerobot") + "/points.txt";
  QString str = QString::fromUtf8(path_points.c_str());

  QFile file(str);
      if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
          return;

      QTextStream out(&file);

      for ( int i = 0 ; i < goal_list.size() ; i++)
      {
          goal_data gdata = goal_list.at(i);
          QString data1 = gdata.id.c_str();
          QString data2 = QString::number(gdata.x);
          QString data3 = QString::number(gdata.y);
          QString data4 = QString::number(gdata.yaw);

          out<<data1<<","<<data2<<","<<data3<<","<<data4<<endl;
      }


      file.close();
}

void CameraCalibView::point_load()
{
    std::string path_points = ros::package::getPath("athomerobot") + "/points.txt";

    QString str = QString::fromUtf8(path_points.c_str());


   QFile file(str);
       if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
           return;

       QTextStream in(&file);
       QString line = in.readLine();
       while (!line.isNull())
       {
           /////////////////////// split the string

           QStringList array = line.split(","); //id , x , y , yaw

           if (array.length() == 4)
           {
           goal_data gdata;

           gdata.id = array.at(0).toStdString();
           gdata.x = array.at(1).toInt();
           gdata.y = array.at(2).toInt();
           gdata.yaw = array.at(3).toInt();

           goal_list.push_back(gdata);
           /////////////////////////////////////////
           QString str = QString::fromUtf8(gdata.id.c_str());
           ui.list_points->addItem(str);
           }

           line = in.readLine();

       }

       file.close();
}

void CameraCalibView::chatterCallback_log(const std_msgs::String::ConstPtr& msg)
{
   QString nmsg = QString::fromUtf8(msg->data.c_str());
   ui.list_log->addItem(nmsg);
}

void CameraCalibView::z_update()
{
  std_msgs::Int32 msg;
  msg.data = ui.spin_desirez->value();
  chatter_pub9.publish(msg);
}

void CameraCalibView::head_update()
{
   sepanta_msgs::head msg;
   msg.pan = ui.spin_head_pan->value();
   msg.tilt = ui.spin_head_tilt->value();
   chatter_pub8.publish(msg);
}

void CameraCalibView::head_reset()
{
    sepanta_msgs::head msg;
    msg.pan = 512;
    msg.tilt = 512;
    chatter_pub8.publish(msg);
}

void CameraCalibView::reset()
{
    std_msgs::String msg;
    msg.data = "reset";
    chatter_pub3.publish(msg);
}

void CameraCalibView::left_arm_update()
{
    sepanta_msgs::arm arm_msg;
    arm_msg.shoulder_pitch = ui.spin_left_shoulderPitch->value();
    arm_msg.shoulder_roll = ui.spin_left_shoulderRoll->value();
    arm_msg.elbow = ui.spin_left_shoulderRoll->value();
    arm_msg.wrist_pitch = ui.spin_left_wristPitch->value();
    arm_msg.wrist_roll = ui.spin_left_wristRoll->value();

    chatter_pub6.publish(arm_msg);
}

void CameraCalibView::left_arm_reset()
{
    sepanta_msgs::arm arm_msg;
    arm_msg.shoulder_pitch = 3100;
    arm_msg.shoulder_roll = 2048;
    arm_msg.elbow = 2048;
    arm_msg.wrist_pitch = 512;
    arm_msg.wrist_roll = 512;

    chatter_pub6.publish(arm_msg);
}

void CameraCalibView::left_gripclose()
{
    std_msgs::Int32 msg;
    msg.data = 5;
    chatter_pub7.publish(msg);
}

void CameraCalibView::left_gripopen()
{
    std_msgs::Int32 msg;
    msg.data = 0;
    chatter_pub7.publish(msg);
}

void CameraCalibView::right_arm_update()
{
    sepanta_msgs::arm arm_msg;
    arm_msg.shoulder_pitch = ui.spin_right_shoulderPitch->value();
    arm_msg.shoulder_roll = ui.spin_right_shoulderRoll->value();
    arm_msg.elbow = ui.spin_right_shoulderRoll->value();
    arm_msg.wrist_pitch = ui.spin_right_wristPitch->value();
    arm_msg.wrist_roll = ui.spin_right_wristRoll->value();

    chatter_pub4.publish(arm_msg);
}

void CameraCalibView::right_arm_reset()
{
    sepanta_msgs::arm arm_msg;
    arm_msg.shoulder_pitch = 3100;
    arm_msg.shoulder_roll = 2048;
    arm_msg.elbow = 2048;
    arm_msg.wrist_pitch = 2048;
    arm_msg.wrist_roll = 2048;

    chatter_pub4.publish(arm_msg);
}

void CameraCalibView::right_gripclose()
{
   std_msgs::Int32 msg;
   msg.data = 5;
   chatter_pub5.publish(msg);
}

void CameraCalibView::right_gripopen()
{
    std_msgs::Int32 msg;
    msg.data = 0;
    chatter_pub5.publish(msg);
}

void CameraCalibView::origin_update()
{
    float ox = (float)ui.spin_x->value() / 100;
    float oy = (float)ui.spin_y->value() / 100;
    float oyaw = (float)ui.spin_yaw->value();
    oyaw = Deg2Rad(oyaw);

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(oyaw);
    msg.pose.pose.position.x = ox;
    msg.pose.pose.position.y = oy;

    chatter_pub2.publish(msg);
}

void CameraCalibView::chatterCallback_omnispeed(const sepanta_msgs::omnidata::ConstPtr &msg)
{
    s_Motor[0] = msg->d0;
    s_Motor[1] = msg->d1;
    s_Motor[2] = msg->d2;
}

void CameraCalibView::chatterCallback_lasersensor(const sepanta_msgs::irsensor::ConstPtr &msg)
{
    laser_IR[0] = (int)msg->d0;
    laser_IR[1] = (int)msg->d1;
    laser_IR[2] = (int)msg->d2;
    laser_IR[3] = (int)msg->d3;
    laser_IR[4] = (int)msg->d4;
    laser_IR[5] = (int)msg->d5;
    laser_IR[6] = (int)msg->d6;
    laser_IR[7] = (int)msg->d7;
}

void CameraCalibView::chatterCallback_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
   //cout<<"get"<<endl;
    geometry_msgs::Pose pose;
    geometry_msgs::Point point;
    geometry_msgs::Quaternion quaternion;
    pose = msg->pose;

    slam_position_yaw[0] = (int)(pose.position.x * 100); //meter
    slam_position_yaw[1] = (int)(pose.position.y * 100); //meter
    slam_position_yaw[2] = (int)Rad2Deg(ConvertQuatToYaw(pose.orientation)); //radians
}

void CameraCalibView::chatterCallback_compass(const std_msgs::Int32::ConstPtr& msg)
{
     Compass = msg->data;
}

void CameraCalibView::chatterCallback_z(const std_msgs::Int32::ConstPtr& msg)
{
     Currentz = msg->data;
}

void CameraCalibView::basic_timer_update()
{
    int m = basic_time / 60;
    int s = basic_time % 60;

    QString str =  QString::number(m) + ":" + QString::number(s);

    ui.lcd_timer->display(str);

    basic_time++;
}

void CameraCalibView::update()
{

  ui.progress_laser1->setValue(laser_IR[0]);
  ui.progress_laser2->setValue(laser_IR[1]);
  ui.progress_laser3->setValue(laser_IR[2]);
  ui.progress_laser4->setValue(laser_IR[3]);
  ui.progress_laser5->setValue(laser_IR[4]);
  ui.progress_laser6->setValue(laser_IR[5]);
  ui.progress_laser7->setValue(laser_IR[6]);
  ui.progress_laser8->setValue(laser_IR[7]);

  ui.progress_speed1->setValue(s_Motor[0]);
  ui.progress_speed2->setValue(s_Motor[1]);
  ui.progress_speed3->setValue(s_Motor[2]);
  ui.progress_speed4->setValue(s_Motor[3]);

  ui.lcd_compass->display(Compass);
  ui.lcd_currentz->display(Currentz);

  ui.lcd_x->display(slam_position_yaw[0]);
  ui.lcd_y->display(slam_position_yaw[1]);
  ui.lcd_yaw->display(slam_position_yaw[2]);

  if ( ui.check_loop->isChecked() )
  {
      std_msgs::String msg;
      msg.data = "reset";

      chatter_pub3.publish(msg);
  }

  if ( kinect_watchdogen == true )
  {
      if ( kinect_connect == false )
      {
          process_kill(1);
          
          process_start(1);
          kinect_connect = true;
      }
  }

  reset_kinect++;

  if ( reset_kinect > 300)
  {
     kinect_watchdogen = true;
     kinect_connect = false;
     reset_kinect = 0;
  }
}



void CameraCalibView::cls()
{
   ui.list_log->clear();
}

void CameraCalibView::chatterCallback_timer(const std_msgs::Int32::ConstPtr& msg)
{
    if ( msg->data == 2 )
    mtimer_basicfunction->stop();

        if ( msg->data == 1 )
        {
          basic_time = 0;
          mtimer_basicfunction->start(1000);
        }
}

void CameraCalibView::initPlugin(qt_gui_cpp::PluginContext& context)
{
	QStringList argv = context.argv();
	widget = new QWidget();
	ui.setupUi(widget);
	context.addWidget(widget);

    //sepanta navigation
    connect(ui.btn_forward, SIGNAL(clicked()), this, SLOT(robot_forward()));
    connect(ui.btn_backward, SIGNAL(clicked()), this, SLOT(robot_backward()));
    connect(ui.btn_left, SIGNAL(clicked()), this, SLOT(robot_left()));
    connect(ui.btn_right, SIGNAL(clicked()), this, SLOT(robot_right()));
    connect(ui.btn_stop, SIGNAL(clicked()), this, SLOT(robot_stop()));
    connect(ui.btn_turn_left, SIGNAL(clicked()), this, SLOT(robot_turn_left()));
    connect(ui.btn_turn_right, SIGNAL(clicked()), this, SLOT(robot_turn_right()));

    //sepanta slam calib
    connect(ui.spin_x,SIGNAL(valueChanged(int)),this,SLOT(origin_update()));
    connect(ui.spin_y,SIGNAL(valueChanged(int)),this,SLOT(origin_update()));
    connect(ui.spin_yaw,SIGNAL(valueChanged(int)),this,SLOT(origin_update()));
    connect(ui.btn_reset,SIGNAL(clicked()),this,SLOT(reset()));

    connect(ui.btn_facestart,SIGNAL(clicked()),this,SLOT(facestart()));
    connect(ui.btn_facestop,SIGNAL(clicked()),this,SLOT(facestop()));
    connect(ui.btn_manualmode,SIGNAL(clicked()),this,SLOT(manualmode()));
    connect(ui.btn_automode,SIGNAL(clicked()),this,SLOT(automode()));
    connect(ui.btn_manualreached,SIGNAL(clicked()),this,SLOT(manualreached()));

    connect(ui.btn_movex,SIGNAL(clicked()),this,SLOT(movex()));
    connect(ui.btn_movey,SIGNAL(clicked()),this,SLOT(movey()));
    connect(ui.btn_btnturngl,SIGNAL(clicked()),this,SLOT(turngl()));
    connect(ui.btn_btnturngllocal,SIGNAL(clicked()),this,SLOT(turngllocal()));
    connect(ui.btn_odometry_cancle,SIGNAL(clicked()),this,SLOT(odometrycancle()));

    connect(ui.btn_goto,SIGNAL(clicked()),this,SLOT(gotolocation()));
    connect(ui.btn_slam_cancle,SIGNAL(clicked()),this,SLOT(slamcancle()));

    connect(ui.btn_point_add,SIGNAL(clicked()),this,SLOT(point_add()));
    connect(ui.btn_point_edit,SIGNAL(clicked()),this,SLOT(point_edit()));
    connect(ui.btn_point_delete,SIGNAL(clicked()),this,SLOT(point_delete()));
    connect(ui.btn_point_addcurrent,SIGNAL(clicked()),this,SLOT(point_addcurrent()));
    connect(ui.btn_savemap , SIGNAL(clicked()),this,SLOT(savemap()));

    connect(ui.list_points,SIGNAL(currentRowChanged(int)),this,SLOT(point_selected()));

    connect(ui.btn_right_arm_reset,SIGNAL(clicked()),this,SLOT(right_arm_reset()));
    connect(ui.btn_right_grip_open,SIGNAL(clicked()),this,SLOT(right_gripopen()));
    connect(ui.btn_right_grip_close,SIGNAL(clicked()),this,SLOT(right_gripclose()));

    //connect(ui.spin_right_shoulderPitch,SIGNAL(valueChanged(int)),this,SLOT(right_arm_update()));
    //connect(ui.spin_right_shoulderRoll,SIGNAL(valueChanged(int)),this,SLOT(right_arm_update()));
    //connect(ui.spin_right_elbow,SIGNAL(valueChanged(int)),this,SLOT(right_arm_update()));
    //connect(ui.spin_right_wristPitch,SIGNAL(valueChanged(int)),this,SLOT(right_arm_update()));
    //connect(ui.spin_right_wristRoll,SIGNAL(valueChanged(int)),this,SLOT(right_arm_update()));

    //connect(ui.spin_left_shoulderPitch,SIGNAL(valueChanged(int)),this,SLOT(left_arm_update()));
    //connect(ui.spin_left_shoulderRoll,SIGNAL(valueChanged(int)),this,SLOT(left_arm_update()));
    //connect(ui.spin_left_elbow,SIGNAL(valueChanged(int)),this,SLOT(left_arm_update()));
    //connect(ui.spin_left_wristPitch,SIGNAL(valueChanged(int)),this,SLOT(left_arm_update()));
    //connect(ui.spin_left_wristRoll,SIGNAL(valueChanged(int)),this,SLOT(left_arm_update()));

    //connect(ui.spin_head_pan,SIGNAL(valueChanged(int)),this,SLOT(head_update()));
    //connect(ui.spin_head_tilt,SIGNAL(valueChanged(int)),this,SLOT(head_update()));

    connect(ui.btn_head_update,SIGNAL(clicked()),this,SLOT(head_update()));
    connect(ui.btn_left_update,SIGNAL(clicked()),this,SLOT(left_arm_update()));
    connect(ui.btn_right_update,SIGNAL(clicked()),this,SLOT(right_arm_update()));

    connect(ui.btn_head_reset,SIGNAL(clicked()),this,SLOT(head_reset()));

    //=========================================================

    connect(ui.btn_device_start,SIGNAL(clicked()),this,SLOT(device_start()));
    connect(ui.btn_device_stop,SIGNAL(clicked()),this,SLOT(device_stop()));

    connect(ui.btn_core_start,SIGNAL(clicked()),this,SLOT(core_start()));
    connect(ui.btn_core_stop,SIGNAL(clicked()),this,SLOT(core_stop()));

    connect(ui.btn_map_start,SIGNAL(clicked()),this,SLOT(map_start()));
    connect(ui.btn_map_stop,SIGNAL(clicked()),this,SLOT(map_stop()));

    connect(ui.btn_node_start,SIGNAL(clicked()),this,SLOT(node_start()));
    connect(ui.btn_node_stop,SIGNAL(clicked()),this,SLOT(node_stop()));

    connect(ui.btn_basic_start,SIGNAL(clicked()),this,SLOT(basic_start()));
    connect(ui.btn_basic_stop,SIGNAL(clicked()),this,SLOT(basic_stop()));

    connect(ui.btn_kinect,SIGNAL(clicked()),this,SLOT(kinect_watchdog()));

    //=========================================================

    connect(ui.btn_cls,SIGNAL(clicked()),this,SLOT(cls()));

    ros::NodeHandle n1;
    ros::NodeHandle n2;
    ros::NodeHandle n3;
    ros::NodeHandle n4;
    ros::NodeHandle n5;
    ros::NodeHandle n6;
    ros::NodeHandle n7;
    ros::NodeHandle n8;
    ros::NodeHandle n9;

    chatter_pub1 = n1.advertise<sepanta_msgs::omnidata>("/AUTROBOTIN_omnidrive", 1);
    chatter_pub2 = n2.advertise<geometry_msgs::PoseWithCovarianceStamped>("/slam_origin", 1);
    chatter_pub3 = n3.advertise<std_msgs::String>("/syscommand", 1);
    chatter_pub4 = n4.advertise<sepanta_msgs::arm>("/AUTROBOTIN_arm_right", 1);
    chatter_pub5 = n5.advertise<std_msgs::Int32>("AUTROBOTIN_gripper_right", 1); //grip right
    chatter_pub6 = n6.advertise<sepanta_msgs::arm>("/AUTROBOTIN_arm_left", 1);
    chatter_pub7 = n7.advertise<std_msgs::Int32>("AUTROBOTIN_gripper_left", 1); //grip left
    chatter_pub8 = n8.advertise<sepanta_msgs::head>("/AUTROBOTIN_head", 1); //head
    chatter_pub9 = n9.advertise<std_msgs::Int32>("/AUTROBOTIN_desirez", 1); //desirez

    sub_compass = getNodeHandle().subscribe("/AUTROBOTOUT_compass", 1, &CameraCalibView::chatterCallback_compass,this);
    sub_motor_speeds = getNodeHandle().subscribe("/AUTROBOTOUT_omnispeed", 1, &CameraCalibView::chatterCallback_omnispeed,this);
    //sub_motor_positions = getNodeHandle().subscribe("/AUTROBOTOUT_omniposition", 1, &CameraCalibView::chatterCallback_omniposition,this);
    sub_laser = getNodeHandle().subscribe("/AUTROBOTOUT_lasersensor", 1, &CameraCalibView::chatterCallback_lasersensor,this);
    sub_slam = getNodeHandle().subscribe("/slam_out_pose", 1, &CameraCalibView::chatterCallback_pose,this);
    sub_timer = getNodeHandle().subscribe("/timer", 1, &CameraCalibView::chatterCallback_timer,this);
    sub_log = getNodeHandle().subscribe("/AUTROBOTOUT_log", 1, &CameraCalibView::chatterCallback_log,this);
    sub_z = getNodeHandle().subscribe("/AUTROBOTOUT_currentz",1,&CameraCalibView::chatterCallback_z,this);
    sub_img = getNodeHandle().subscribe("/camera/rgb/image_color", 1, &CameraCalibView::rosImageCallBack,this);

    //timer GUI update
    mtimer = new QTimer(this);
    connect(mtimer, SIGNAL(timeout()), this, SLOT(update()));
    mtimer->start(100);

    mtimer_basicfunction = new QTimer(this);
    connect(mtimer_basicfunction, SIGNAL(timeout()), this, SLOT(basic_timer_update()));
    //mtimer_basicfunction->start(1000);

    //service clients
    ros::NodeHandle n_client1;
    serviceclient_facestop = n_client1.serviceClient<sepanta_msgs::stop>("speechOrFace_Stop");

    ros::NodeHandle n_client2;
    serviceclient_manualauto = n_client2.serviceClient<sepanta_msgs::stop>("manualOrAuto");

    ros::NodeHandle n_client3;
    serviceclient_odometryslam = n_client3.serviceClient<sepanta_msgs::command>("AUTROBOTINSRV_command");

    ros::NodeHandle n_client4;
    serviceclient_map = n_client4.serviceClient<sepanta_msgs::maptools>("AUTROBOTINSRV_maptools");

    //load point from HDD
    point_load();


    process_init();

}

void CameraCalibView::shutdownPlugin()
{
       mtimer->stop();
       sub_compass.shutdown();
       sub_motor_speeds.shutdown();
       sub_laser.shutdown();
       sub_slam.shutdown();
       sub_timer.shutdown();
       sub_log.shutdown();
       sub_z.shutdown();
       sub_img.shutdown();
}

void CameraCalibView::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
}

void CameraCalibView::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
}

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_camera_calib_gui, CameraCalibView, rqt_camera_calib_gui::CameraCalibView, rqt_gui_cpp::Plugin)

