/**
 * @file /include/rover_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rover_gui_QNODE_HPP_
#define rover_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <sstream>
#include <QThread>
#include <std_msgs/String.h>
#include <QStringListModel>

#define MoveForward 1
#define MoveBackward 2
#define TurnLeft 3
#define TurnRight 4
#define Stop 5

#define HeadUp 101
#define HeadDown 102
#define HeadLeft 103
#define HeadRight 104
/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;

namespace rover_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
  void signal_callback(const std_msgs::String::ConstPtr& msg);

  void update_speed(float speed);
  void set_speed(int direction);
  void set_head_move(int direction);
  void take_photo();
  void start_record();
  void stop_record();

  string float_to_string(float num);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();

private:
	int init_argc;
	char** init_argv;
  ros::Publisher vel_publisher;
  ros::Subscriber signal_subscriber;
  QStringListModel logging_model;
  float speed_value;
  string vel_cmd;
};

}  // namespace rover_gui

#endif /* rover_gui_QNODE_HPP_ */
