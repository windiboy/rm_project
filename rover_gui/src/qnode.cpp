/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include "../include/rover_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rover_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
{}

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"rover_gui");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.
  signal_subscriber = n.subscribe("/gesture/command", 1000, &QNode::signal_callback, this);
  vel_publisher = n.advertise<std_msgs::String>("/command/write", 1000);
  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings,"rover_gui");
  if ( ! ros::master::check() ) {
    return false;
  }

  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  speed_value = 0;
  vel_cmd = string("!M ") + float_to_string(0) + string(" ") + float_to_string(0) + string("\r");
  // Add your ros communications here.
  signal_subscriber = n.subscribe("/gesture/command", 1000, &QNode::signal_callback, this);
  vel_publisher = n.advertise<std_msgs::String>("/command/write", 1000);
  start();
  return true;
}

void QNode::run() {
  ros::Rate loop_rate(10);
  int count = 0;
  while ( ros::ok() ) {
    std_msgs::String vel_msg;
    vel_msg.data = vel_cmd;
    vel_publisher.publish(vel_msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::signal_callback(const std_msgs::String::ConstPtr &msg){
  log(Info,std::string("I receive: ")+msg->data);
  if (msg->data == string("MoveForward"))
  {
    set_speed(MoveForward);
  }
  else if (msg->data == string("MoveBackward"))
  {
    set_speed(MoveBackward);
  }
  else if (msg->data == string("TurnLeft"))
  {
    set_speed(TurnLeft);
  }
  else if (msg->data == string("TurnRight"))
  {
    set_speed(TurnRight);
  }
  else if (msg->data == string("Stop"))
  {
    set_speed(Stop);
  }
  else if (msg->data == string("TakePhoto"))
  {
    take_photo();
  }
  else if (msg->data == string("StartRecord"))
  {
    start_record();
  }
  else if (msg->data == string("StopRecord"))
  {
    stop_record();
  }
  else if (msg->data == string("HeadUp"))
  {
    set_head_move(HeadUp);
  }
  else if (msg->data == string("HeadDown"))
  {
    set_head_move(HeadDown);
  }
  else if (msg->data == string("HeadLeft"))
  {
    set_head_move(HeadLeft);
  }
  else if (msg->data == string("HeadRight"))
  {
    set_head_move(HeadRight);
  }
}

void QNode::log( const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  switch ( level ) {
  case(Debug) : {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Info) : {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Warn) : {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Error) : {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Fatal) : {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

string QNode::float_to_string(float num){
  ostringstream oss;
  oss<<num;
  string str(oss.str());
  return str;
}

void QNode::set_speed(int direction){
  string cmd;
  switch (direction) {
  case MoveForward:
  {
    //cmd = string("!M ") + float_to_string(-speed_value) + string("\r");
    cmd = string("!M ") + float_to_string(0) + string(" ") + float_to_string(-speed_value) + string("\r");
    log(Info, string("I send:  ") + cmd);
  }
    break;
  case MoveBackward:
  {
    //cmd = string("!M ") + float_to_string(speed_value) + string("\r");
    cmd = string("!M ") + float_to_string(0) + string(" ") + float_to_string(speed_value) + string("\r");
    log(Info, string("I send:  ") + cmd);
  }
    break;
  case TurnLeft:
  {
    cmd = string("!M ") + float_to_string(speed_value) + string("\r");
    //cmd = string("!M ") + float_to_string(-speed_value) + string(" ") + float_to_string(speed_value) + string("\r");
    log(Info, string("I send:  ") + cmd);
  }
    break;
  case TurnRight:
  {
    cmd = string("!M ") + float_to_string(-speed_value) + string("\r");
    //cmd = string("!M ") + float_to_string(speed_value) + string(" ") + float_to_string(-speed_value) + string("\r");
    log(Info, string("I send:  ") + cmd);
  }
    break;
  case Stop:
  {
    cmd = string("!M ") + float_to_string(0) + string(" ") + float_to_string(0) + string("\r");
    log(Info, string("I send:  ") + cmd);
  }
    break;
  default:
    break;
  }
  vel_cmd = cmd;
}

void QNode::set_head_move(int direction){
  if (direction == HeadUp){
    std_msgs::String headup_msg;
    headup_msg.data = string("HeadUp");
    log(Info, string("I send:  ") + string("HeadUp"));
    vel_publisher.publish(headup_msg);
  }
  else if (direction == HeadDown){
    std_msgs::String headdown_msg;
    headdown_msg.data = string("HeadDown");
    log(Info, string("I send:  ") + string("HeadDown"));
    vel_publisher.publish(headdown_msg);
  }
  else if (direction == HeadLeft){
    std_msgs::String headleft_msg;
    headleft_msg.data = string("HeadLeft");
    log(Info, string("I send:  ") + string("HeadLeft"));
    vel_publisher.publish(headleft_msg);
  }
  else if (direction == HeadRight){
    std_msgs::String headright_msg;
    headright_msg.data = string("HeadRight");
    log(Info, string("I send:  ") + string("HeadRight"));
    vel_publisher.publish(headright_msg);
  }
}

void QNode::take_photo(){
  std_msgs::String photo_msg;
  photo_msg.data = string("TakePhoto");
  log(Info, string("I send:  ") + string("TakePhoto"));
  vel_publisher.publish(photo_msg);
}

void QNode::start_record(){
  std_msgs::String photo_msg;
  photo_msg.data = string("StartRecord");
  log(Info, string("I send:  ") + string("StartRecord"));
  vel_publisher.publish(photo_msg);
}

void QNode::stop_record(){
  std_msgs::String photo_msg;
  photo_msg.data = string("StopRecord");
  log(Info, string("I send:  ") + string("StopRecord"));
  vel_publisher.publish(photo_msg);
}

void QNode::update_speed(float speed){
  speed_value = speed;
}

}  // namespace rover_gui
