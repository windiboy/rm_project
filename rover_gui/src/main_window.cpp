/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/rover_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rover_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    QObject::connect(ui.set_speed_btn, SIGNAL(clicked(bool)), this, SLOT(on_btn_set_speed_clicked(bool)));

    QObject::connect(ui.move_forward_btn, SIGNAL(clicked(bool)), this, SLOT(on_btn_move_forward_clicked(bool)));
    QObject::connect(ui.move_backward_btn, SIGNAL(clicked(bool)), this, SLOT(on_btn_move_backward_clicked(bool)));
    QObject::connect(ui.turn_left_btn, SIGNAL(clicked(bool)), this, SLOT(on_btn_turn_left_clicked(bool)));
    QObject::connect(ui.turn_right_btn, SIGNAL(clicked(bool)), this, SLOT(on_btn_turn_right_clicked(bool)));
    QObject::connect(ui.stop_btn, SIGNAL(clicked(bool)), this, SLOT(on_btn_stop_clicked(bool)));

    QObject::connect(ui.head_up_btn, SIGNAL(clicked(bool)), this, SLOT(on_btn_head_up_clicked(bool)));
    QObject::connect(ui.head_down_btn, SIGNAL(clicked(bool)), this, SLOT(on_btn_head_down_clicked(bool)));
    QObject::connect(ui.head_left_btn, SIGNAL(clicked(bool)), this, SLOT(on_btn_head_left_clicked(bool)));
    QObject::connect(ui.head_right_btn, SIGNAL(clicked(bool)), this, SLOT(on_btn_head_right_clicked(bool)));

    QObject::connect(ui.take_photo_btn, SIGNAL(clicked(bool)), this, SLOT(on_btn_take_photo_clicked(bool)));
    QObject::connect(ui.start_record_btn, SIGNAL(clicked(bool)), this, SLOT(on_btn_start_record_clicked(bool)));
    QObject::connect(ui.stop_record_btn, SIGNAL(clicked(bool)), this, SLOT(on_btn_stop_record_clicked(bool)));

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
                        ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}

void MainWindow::on_btn_set_speed_clicked(bool check){
  float vel = atof(ui.speed_edit->text().toStdString().c_str());
  qnode.update_speed(vel);
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
  bool enabled;
  if ( state == 0 ) {
    enabled = true;
  } else {
    enabled = false;
  }
  ui.line_edit_master->setEnabled(enabled);
  ui.line_edit_host->setEnabled(enabled);
}

void MainWindow::on_btn_move_forward_clicked(bool check){
  qnode.set_speed(MoveForward);
}

void MainWindow::on_btn_move_backward_clicked(bool check){
  qnode.set_speed(MoveBackward);
}

void MainWindow::on_btn_turn_left_clicked(bool check){
  qnode.set_speed(TurnLeft);
}

void MainWindow::on_btn_turn_right_clicked(bool check){
  qnode.set_speed(TurnRight);
}

void MainWindow::on_btn_stop_clicked(bool check){
  qnode.set_speed(Stop);
}

void MainWindow::on_btn_head_left_clicked(bool check){
  qnode.set_head_move(HeadLeft);
}

void MainWindow::on_btn_head_right_clicked(bool check){
  qnode.set_head_move(HeadRight);
}

void MainWindow::on_btn_head_up_clicked(bool check){
  qnode.set_head_move(HeadUp);
}

void MainWindow::on_btn_head_down_clicked(bool check){
  qnode.set_head_move(HeadDown);
}

void MainWindow::on_btn_take_photo_clicked(bool check){
  qnode.take_photo();
}

void MainWindow::on_btn_start_record_clicked(bool check){
  qnode.start_record();
}

void MainWindow::on_btn_stop_record_clicked(bool check){
  qnode.stop_record();
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "rover_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    //restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "rover_gui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace rover_gui

