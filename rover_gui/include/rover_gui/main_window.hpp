/**
 * @file /include/rover_gui/main_window.hpp
 *
 * @brief Qt based gui for rover_gui.
 *
 * @date November 2010
 **/
#ifndef rover_gui_MAIN_WINDOW_H
#define rover_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rover_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
  void on_btn_set_speed_clicked(bool check);
	void on_checkbox_use_environment_stateChanged(int state);

  void on_btn_move_forward_clicked(bool check);
  void on_btn_move_backward_clicked(bool check);
  void on_btn_turn_left_clicked(bool check);
  void on_btn_turn_right_clicked(bool check);
  void on_btn_stop_clicked(bool check);

  void on_btn_head_up_clicked(bool check);
  void on_btn_head_down_clicked(bool check);
  void on_btn_head_left_clicked(bool check);
  void on_btn_head_right_clicked(bool check);

  void on_btn_take_photo_clicked(bool check);
  void on_btn_start_record_clicked(bool check);
  void on_btn_stop_record_clicked(bool check);
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace rover_gui

#endif // rover_gui_MAIN_WINDOW_H
