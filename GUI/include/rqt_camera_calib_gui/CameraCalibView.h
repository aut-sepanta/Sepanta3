#ifndef CAMERA_CALIB_VIEW_H
#define CAMERA_CALIB_VIEW_H

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <rqt_gui_cpp/plugin.h>
#include "ui_camera_calib_view.h"

#include <QWidget>
#include <QMessageBox>
#include <vector>

#include <sepanta_msgs/arm.h>
#include <sepanta_msgs/omnidata.h>
#include <sepanta_msgs/head.h>
#include <sepanta_msgs/irsensor.h>
#include <sepanta_msgs/motortorques.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <qt4/QtNetwork/QTcpServer>
#include <qt4/QtNetwork/QTcpSocket>

#include <sepanta_msgs/stop.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace rqt_camera_calib_gui {

class CameraCalibView
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  CameraCalibView();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
  


public slots:
  
  void kinect_watchdog();
  void z_update();
  void robot_forward();
  void robot_backward();
  void robot_left();
  void robot_right();
  void robot_turn_left();
  void robot_turn_right();
  void robot_stop();
  void update();
  void basic_timer_update();
  //slam calibration  
  void reset();
  void origin_update();
  

  
  //point edit
  void point_add();
  void point_delete();
  void point_edit();
  void point_addcurrent();
  
  //service calls
  void savemap();
  void facestop();
  void facestart();
  void manualmode();
  void automode();
  void manualreached();
  void movex();
  void movey();
  void turngl();
  void turngllocal();
  void gotolocation();
  void slamcancle();
  void odometrycancle();

  void point_save();
  void point_load();

  void point_selected();


  
  //arm
  void right_arm_update();
  void right_arm_reset();
  void right_gripopen();
  void right_gripclose();
  
  void left_arm_update();
  void left_arm_reset();
  void left_gripopen();
  void left_gripclose();
  
  void head_update();
  void head_reset();
  
  void cls();

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  void process_start(int id);
  void process_kill(int id);

  void rightmessage_laser();
  void wrongmessage_laser();

  void rightmessage_kinect();
  void wrongmessage_kinect();

  void rightmessage_dynamixel();
  void wrongmessage_dynamixel();

  void rightmessage_core();
  void wrongmessage_core();

  void rightmessage_odometry();
  void wrongmessage_odometry();

  void rightmessage_slam();
  void wrongmessage_slam();

  void rightmessage_slamclient();
  void wrongmessage_slamclient();

  void rightmessage_map();
  void wrongmessage_map();

  void rightmessage_move();
  void wrongmessage_move();

  void rightmessage_mapserver();
  void wrongmessage_mapserver();

  void rightmessage_pickandplace();
  void wrongmessage_pickandplace();

  void rightmessage_object();
  void wrongmessage_object();

  void rightmessage_grip();
  void wrongmessage_grip();

  void rightmessage_pcd();
  void wrongmessage_pcd();

  void rightmessage_avoid();
  void wrongmessage_avoid();

  void rightmessage_whatdidyousay();
  void wrongmessage_whatdidyousay();

  void rightmessage_person();
  void wrongmessage_person();

  void rightmessage_windows();
  void wrongmessage_windows();

  void rightmessage_gesture();
  void wrongmessage_gesture();

  void rightmessage_clientstop();
  void wrongmessage_clientstop();


  void rightmessage_basictask();
  void wrongmessage_basictask();

  void process_init();

  void device_start();
  void device_stop();

  void core_start();
  void core_stop();

  void map_start();
  void map_stop();

  void node_start();
  void node_stop();

  void basic_start();
  void basic_stop();
  
private:
  Ui::CameraCalibViewWidget ui;
  QWidget* widget;


  ros::Subscriber sub_compass;
  ros::Subscriber sub_motor_speeds;
  ros::Subscriber sub_laser;
  ros::Subscriber sub_slam;
  ros::Subscriber sub_timer;
  ros::Subscriber sub_log;
  ros::Subscriber sub_z;
  ros::Subscriber sub_keypad;
  ros::Subscriber sub_img;

  virtual void set_omni(int x,int y,int w);
  virtual void chatterCallback_compass(const std_msgs::Int32::ConstPtr& msg);
  virtual void chatterCallback_pose(const geometry_msgs::PoseStamped::ConstPtr &msg);
  virtual void chatterCallback_lasersensor(const sepanta_msgs::irsensor::ConstPtr &msg);
  virtual void chatterCallback_omnispeed(const sepanta_msgs::omnidata::ConstPtr &msg);
  virtual void chatterCallback_timer(const std_msgs::Int32::ConstPtr& msg);
  virtual void chatterCallback_log(const std_msgs::String::ConstPtr& msg);
  virtual void chatterCallback_z(const std_msgs::Int32::ConstPtr& msg);
  virtual void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg);
  //virtual void chatterCallback_keypad(const std_msgs::Int32::ConstPtr& msg);
   
};
} // namespace
#endif // my_namespace__my_plugin_H
