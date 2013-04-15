/**
 * @file /include/bioloid_gui/main_window.hpp
 *
 * @brief Qt based gui for bioloid_gui.
 *
 * @date November 2010
 **/
#ifndef bioloid_gui_MAIN_WINDOW_H
#define bioloid_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <QFile>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace bioloid_gui {

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
	virtual ~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
	void showNotPossibleToSave();
	void valuesChanged();
public slots:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_save_configuration_clicked(bool check);
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
	void on_jslider_1_sliderMoved();
	void on_jslider_2_sliderMoved();
	void on_jslider_3_sliderMoved();
	void on_jslider_4_sliderMoved();
	void on_jslider_5_sliderMoved();
	void on_jslider_6_sliderMoved();
	void on_jslider_7_sliderMoved();
	void on_jslider_8_sliderMoved();
	void on_jslider_9_sliderMoved();
	void on_jslider_10_sliderMoved();
	void on_jslider_11_sliderMoved();
	void on_jslider_12_sliderMoved();
	void on_jslider_13_sliderMoved();
	void on_jslider_14_sliderMoved();
	void on_jslider_15_sliderMoved();
	void on_jslider_16_sliderMoved();
	void on_jslider_17_sliderMoved();
	void on_jslider_18_sliderMoved();	
	void on_resetAllButton_clicked();
	void on_testAllButton_clicked();
	void on_torqueAll_stateChanged(int state);
	void on_torque_1_clicked(bool checked);
	void on_torque_2_stateChanged(int state);
	void on_torque_3_stateChanged(int state);
	void on_torque_4_stateChanged(int state);
	void on_torque_5_stateChanged(int state);
	void on_torque_6_stateChanged(int state);
	void on_torque_7_stateChanged(int state);
	void on_torque_8_stateChanged(int state);
	void on_torque_9_stateChanged(int state);
	void on_torque_10_stateChanged(int state);
	void on_torque_11_stateChanged(int state);
	void on_torque_12_stateChanged(int state);
	void on_torque_13_stateChanged(int state);
	void on_torque_14_stateChanged(int state);
	void on_torque_15_stateChanged(int state);
	void on_torque_16_stateChanged(int state);
	void on_torque_17_stateChanged(int state);
	void on_torque_18_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
	void valuesRefresh();	
signals:
	void jointUpdateReq();
	void jointTestReq();
private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	boost::shared_ptr<QFile> configuration_file;
};

}  // namespace bioloid_gui

#endif // bioloid_gui_MAIN_WINDOW_H
