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
#include "bioloid_gui/main_window.hpp"

#define Qsettings_organization "Qt-Ros Package"
#define Qsettings_application "bioloid_gui"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace bioloid_gui {

using namespace Qt;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
	QMainWindow(parent),
	qnode(argc, argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	 ** Logging
	 **********************/
	ui.view_logging->setModel(qnode.loggingModel());
	QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,SLOT(updateLoggingView()));


	QObject::connect(&qnode, SIGNAL(jointUpdate()), this, SLOT(valuesRefresh()));
	QObject::connect(this, SIGNAL(jointUpdateReq()), &qnode,SLOT(updateJoints()));
	QObject::connect(this, SIGNAL(jointTestReq()), &qnode, SLOT(testJoints()));

	/*********************
	 ** Auto Start
	 **********************/
	if (ui.checkbox_remember_settings->isChecked()) {
		on_button_connect_clicked(true);
	}
}

MainWindow::~MainWindow() {
}

/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master. Make sure ros master is running and try to connect again.");
	msgBox.exec();
}

void MainWindow::showNotPossibleToSave() {
	QMessageBox msgBox;
	msgBox.setText("An error occurred. Not possible to save the configuration to a file. Check file permissions.");
	msgBox.exec();
}

void MainWindow::on_button_save_configuration_clicked(bool check){
	//Enable the button only after we are connected to the robot
	showNotPossibleToSave();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check) {
	if (ui.checkbox_use_environment->isChecked()) {
		if (!qnode.init()) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
//			ui.button_save_configuration->setEnabled(true);
		}
	} else {
		if (!qnode.init(ui.line_edit_master->text().toStdString(), ui.line_edit_host->text().toStdString())) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
//			ui.button_save_configuration->setEnabled(true);
		}
	}
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if (state == 0) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

void MainWindow::on_resetAllButton_clicked() {
	for (int i = 0; i < 18; i++) {
		qnode.des_pos[i] = 0;
	}
	emit jointUpdateReq();
}

void MainWindow::on_testAllButton_clicked() {
	emit jointTestReq();
}

void MainWindow::on_torqueAll_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	if (state == 0) {
		for (std::vector<double>::iterator i = qnode.des_eff.begin(); i < qnode.des_eff.end(); i++) {
			*i = 1;
		}
	} else {
		for (std::vector<double>::iterator i = qnode.des_eff.begin(); i	< qnode.des_eff.end(); i++) {
			*i = 0;
		}
	}
	emit jointUpdateReq();
}

void MainWindow::on_torque_1_clicked(bool checked) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[0] = (checked == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_2_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[1] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_3_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[2] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_4_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[3] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_5_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[4] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_6_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[5] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_7_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[6] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_8_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[7] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_9_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[8] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_10_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[9] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_11_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[10] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_12_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[11] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_13_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[12] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_14_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[13] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_15_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[14] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_16_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[15] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_17_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[16] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
}
void MainWindow::on_torque_18_stateChanged(int state) {
	qnode.des_eff = qnode.eff;
	qnode.des_eff[17] = (state == Qt::Checked) ? 0 : 1;
	ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	emit jointUpdateReq();
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

void MainWindow::valuesRefresh() {
	ui.jname_1->setText(QString(" 0. ").append(qnode.name[0].c_str()));
	ui.jstate_1->setText(QString::number(qnode.pos[0], 'f', 6));
	ui.jrawstate_1->setText(QString::number(qnode.pos_raw[0]));
	ui.jslider_1->setValue(qnode.pos[0] * 100);
	ui.torque_1->setCheckState((qnode.eff[0] == 1) ? Qt::Unchecked	: Qt::Checked);

	ui.jname_2->setText(QString(" 1. ").append(qnode.name[1].c_str()));
	ui.jstate_2->setText(QString::number(qnode.pos[1], 'f', 6));
	ui.jrawstate_2->setText(QString::number(qnode.pos_raw[1]));
	ui.jslider_2->setValue(qnode.pos[1] * 100);
	ui.torque_2->setCheckState((qnode.eff[1] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_3->setText(QString(" 2. ").append(qnode.name[2].c_str()));
	ui.jstate_3->setText(QString::number(qnode.pos[2], 'f', 6));
	ui.jrawstate_3->setText(QString::number(qnode.pos_raw[2]));
	ui.jslider_3->setValue(qnode.pos[2] * 100);
	ui.torque_3->setCheckState((qnode.eff[2] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_4->setText(QString(" 3. ").append(qnode.name[3].c_str()));
	ui.jstate_4->setText(QString::number(qnode.pos[3], 'f', 6));
	ui.jrawstate_4->setText(QString::number(qnode.pos_raw[3]));
	ui.jslider_4->setValue(qnode.pos[3] * 100);
	ui.torque_4->setCheckState((qnode.eff[3] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_5->setText(QString(" 4. ").append(qnode.name[4].c_str()));
	ui.jstate_5->setText(QString::number(qnode.pos[4], 'f', 6));
	ui.jrawstate_5->setText(QString::number(qnode.pos_raw[4]));
	ui.jslider_5->setValue(qnode.pos[4] * 100);
	ui.torque_5->setCheckState((qnode.eff[4] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_6->setText(QString(" 5. ").append(qnode.name[5].c_str()));
	ui.jstate_6->setText(QString::number(qnode.pos[5], 'f', 6));
	ui.jrawstate_6->setText(QString::number(qnode.pos_raw[5]));
	ui.jslider_6->setValue(qnode.pos[5] * 100);
	ui.torque_6->setCheckState((qnode.eff[5] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_7->setText(QString(" 6. ").append(qnode.name[6].c_str()));
	ui.jstate_7->setText(QString::number(qnode.pos[6], 'f', 6));
	ui.jrawstate_7->setText(QString::number(qnode.pos_raw[6]));
	ui.jslider_7->setValue(qnode.pos[6] * 100);
	ui.torque_7->setCheckState((qnode.eff[6] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_8->setText(QString(" 7. ").append(qnode.name[7].c_str()));
	ui.jstate_8->setText(QString::number(qnode.pos[7], 'f', 6));
	ui.jrawstate_8->setText(QString::number(qnode.pos_raw[7]));
	ui.jslider_8->setValue(qnode.pos[7] * 100);
	ui.torque_8->setCheckState((qnode.eff[7] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_9->setText(QString(" 8. ").append(qnode.name[8].c_str()));
	ui.jstate_9->setText(QString::number(qnode.pos[8], 'f', 6));
	ui.jrawstate_9->setText(QString::number(qnode.pos_raw[8]));
	ui.jslider_9->setValue(qnode.pos[8] * 100);
	ui.torque_9->setCheckState((qnode.eff[8] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_10->setText(QString(" 9. ").append(qnode.name[9].c_str()));
	ui.jstate_10->setText(QString::number(qnode.pos[9], 'f', 6));
	ui.jrawstate_10->setText(QString::number(qnode.pos_raw[9]));
	ui.jslider_10->setValue(qnode.pos[9] * 100);
	ui.torque_10->setCheckState((qnode.eff[9] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_11->setText(QString("10. ").append(qnode.name[10].c_str()));
	ui.jstate_11->setText(QString::number(qnode.pos[10], 'f', 6));
	ui.jrawstate_11->setText(QString::number(qnode.pos_raw[10]));
	ui.jslider_11->setValue(qnode.pos[10] * 100);
	ui.torque_11->setCheckState((qnode.eff[10] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_12->setText(QString("11. ").append(qnode.name[11].c_str()));
	ui.jstate_12->setText(QString::number(qnode.pos[11], 'f', 6));
	ui.jrawstate_12->setText(QString::number(qnode.pos_raw[11]));
	ui.jslider_12->setValue(qnode.pos[11] * 100);
	ui.torque_12->setCheckState((qnode.eff[11] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_13->setText(QString("12. ").append(qnode.name[12].c_str()));
	ui.jstate_13->setText(QString::number(qnode.pos[12], 'f', 6));
	ui.jrawstate_13->setText(QString::number(qnode.pos_raw[12]));
	ui.jslider_13->setValue(qnode.pos[12] * 100);
	ui.torque_13->setCheckState((qnode.eff[12] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_14->setText(QString("13. ").append(qnode.name[13].c_str()));
	ui.jstate_14->setText(QString::number(qnode.pos[13], 'f', 6));
	ui.jrawstate_14->setText(QString::number(qnode.pos_raw[13]));
	ui.jslider_14->setValue(qnode.pos[13] * 100);
	ui.torque_14->setCheckState((qnode.eff[13] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_15->setText(QString("14. ").append(qnode.name[14].c_str()));
	ui.jstate_15->setText(QString::number(qnode.pos[14], 'f', 6));
	ui.jrawstate_15->setText(QString::number(qnode.pos_raw[14]));
	ui.jslider_15->setValue(qnode.pos[14] * 100);
	ui.torque_15->setCheckState((qnode.eff[14] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_16->setText(QString("15. ").append(qnode.name[15].c_str()));
	ui.jstate_16->setText(QString::number(qnode.pos[15], 'f', 6));
	ui.jrawstate_16->setText(QString::number(qnode.pos_raw[15]));
	ui.jslider_16->setValue(qnode.pos[15] * 100);
	ui.torque_16->setCheckState((qnode.eff[15] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_17->setText(QString("16. ").append(qnode.name[16].c_str()));
	ui.jstate_17->setText(QString::number(qnode.pos[16], 'f', 6));
	ui.jrawstate_17->setText(QString::number(qnode.pos_raw[16]));
	ui.jslider_17->setValue(qnode.pos[16] * 100);
	ui.torque_17->setCheckState((qnode.eff[16] == 1) ? Qt::Unchecked
			: Qt::Checked);

	ui.jname_18->setText(QString("17. ").append(qnode.name[17].c_str()));
	ui.jstate_18->setText(QString::number(qnode.pos[17], 'f', 6));
	ui.jrawstate_18->setText(QString::number(qnode.pos_raw[17]));
	ui.jslider_18->setValue(qnode.pos[17] * 100);
	ui.torque_18->setCheckState((qnode.eff[17] == 1) ? Qt::Unchecked
			: Qt::Checked);

	if (ui.jmin_1->text().toDouble() >= qnode.pos[0]) {
		ui.jmin_1->setText(QString::number(qnode.pos[0], 'f', 6));
	}
	if (ui.jmax_1->text().toDouble() <= qnode.pos[0]) {
		ui.jmax_1->setText(QString::number(qnode.pos[0], 'f', 6));
	}

	if (ui.jmin_2->text().toDouble() >= qnode.pos[1]) {
		ui.jmin_2->setText(QString::number(qnode.pos[1], 'f', 6));
	}
	if (ui.jmax_2->text().toDouble() <= qnode.pos[1]) {
		ui.jmax_2->setText(QString::number(qnode.pos[1], 'f', 6));
	}

	if (ui.jmin_3->text().toDouble() >= qnode.pos[2]) {
		ui.jmin_3->setText(QString::number(qnode.pos[2], 'f', 6));
	}
	if (ui.jmax_3->text().toDouble() <= qnode.pos[2]) {
		ui.jmax_3->setText(QString::number(qnode.pos[2], 'f', 6));
	}

	if (ui.jmin_4->text().toDouble() >= qnode.pos[3]) {
		ui.jmin_4->setText(QString::number(qnode.pos[3], 'f', 6));
	}
	if (ui.jmax_4->text().toDouble() <= qnode.pos[3]) {
		ui.jmax_4->setText(QString::number(qnode.pos[3], 'f', 6));
	}

	if (ui.jmin_5->text().toDouble() >= qnode.pos[4]) {
		ui.jmin_5->setText(QString::number(qnode.pos[4], 'f', 6));
	}
	if (ui.jmax_5->text().toDouble() <= qnode.pos[4]) {
		ui.jmax_5->setText(QString::number(qnode.pos[4], 'f', 6));
	}

	if (ui.jmin_6->text().toDouble() >= qnode.pos[5]) {
		ui.jmin_6->setText(QString::number(qnode.pos[5], 'f', 6));
	}
	if (ui.jmax_6->text().toDouble() <= qnode.pos[5]) {
		ui.jmax_6->setText(QString::number(qnode.pos[5], 'f', 6));
	}

	if (ui.jmin_7->text().toDouble() >= qnode.pos[6]) {
		ui.jmin_7->setText(QString::number(qnode.pos[6], 'f', 6));
	}
	if (ui.jmax_7->text().toDouble() <= qnode.pos[6]) {
		ui.jmax_7->setText(QString::number(qnode.pos[6], 'f', 6));
	}

	if (ui.jmin_8->text().toDouble() >= qnode.pos[7]) {
		ui.jmin_8->setText(QString::number(qnode.pos[7], 'f', 6));
	}
	if (ui.jmax_8->text().toDouble() <= qnode.pos[7]) {
		ui.jmax_8->setText(QString::number(qnode.pos[7], 'f', 6));
	}

	if (ui.jmin_9->text().toDouble() >= qnode.pos[8]) {
		ui.jmin_9->setText(QString::number(qnode.pos[8], 'f', 6));
	}
	if (ui.jmax_9->text().toDouble() <= qnode.pos[8]) {
		ui.jmax_9->setText(QString::number(qnode.pos[8], 'f', 6));
	}

	if (ui.jmin_10->text().toDouble() >= qnode.pos[9]) {
		ui.jmin_10->setText(QString::number(qnode.pos[9], 'f', 6));
	}
	if (ui.jmax_10->text().toDouble() <= qnode.pos[9]) {
		ui.jmax_10->setText(QString::number(qnode.pos[9], 'f', 6));
	}

	if (ui.jmin_11->text().toDouble() >= qnode.pos[10]) {
		ui.jmin_11->setText(QString::number(qnode.pos[10], 'f', 6));
	}
	if (ui.jmax_11->text().toDouble() <= qnode.pos[10]) {
		ui.jmax_11->setText(QString::number(qnode.pos[10], 'f', 6));
	}

	if (ui.jmin_12->text().toDouble() >= qnode.pos[11]) {
		ui.jmin_12->setText(QString::number(qnode.pos[11], 'f', 6));
	}
	if (ui.jmax_12->text().toDouble() <= qnode.pos[11]) {
		ui.jmax_12->setText(QString::number(qnode.pos[11], 'f', 6));
	}

	if (ui.jmin_13->text().toDouble() >= qnode.pos[12]) {
		ui.jmin_13->setText(QString::number(qnode.pos[12], 'f', 6));
	}
	if (ui.jmax_13->text().toDouble() <= qnode.pos[12]) {
		ui.jmax_13->setText(QString::number(qnode.pos[12], 'f', 6));
	}

	if (ui.jmin_14->text().toDouble() >= qnode.pos[13]) {
		ui.jmin_14->setText(QString::number(qnode.pos[13], 'f', 6));
	}
	if (ui.jmax_14->text().toDouble() <= qnode.pos[13]) {
		ui.jmax_14->setText(QString::number(qnode.pos[13], 'f', 6));
	}

	if (ui.jmin_15->text().toDouble() >= qnode.pos[14]) {
		ui.jmin_15->setText(QString::number(qnode.pos[14], 'f', 6));
	}
	if (ui.jmax_15->text().toDouble() <= qnode.pos[14]) {
		ui.jmax_15->setText(QString::number(qnode.pos[14], 'f', 6));
	}

	if (ui.jmin_16->text().toDouble() >= qnode.pos[15]) {
		ui.jmin_16->setText(QString::number(qnode.pos[15], 'f', 6));
	}
	if (ui.jmax_16->text().toDouble() <= qnode.pos[15]) {
		ui.jmax_16->setText(QString::number(qnode.pos[15], 'f', 6));
	}

	if (ui.jmin_17->text().toDouble() >= qnode.pos[16]) {
		ui.jmin_17->setText(QString::number(qnode.pos[16], 'f', 6));
	}
	if (ui.jmax_17->text().toDouble() <= qnode.pos[16]) {
		ui.jmax_17->setText(QString::number(qnode.pos[16], 'f', 6));
	}

	if (ui.jmin_18->text().toDouble() >= qnode.pos[17]) {
		ui.jmin_18->setText(QString::number(qnode.pos[17], 'f', 6));
	}
	if (ui.jmax_18->text().toDouble() <= qnode.pos[17]) {
		ui.jmax_18->setText(QString::number(qnode.pos[17], 'f', 6));
	}

	bool allEqual = true;
	for (std::vector<double>::iterator i = qnode.eff.begin() + 1; i	< qnode.eff.end(); i++) {
		if (*i != qnode.eff.front()) {
			allEqual = false;
		}
	}
	if (allEqual == false) {
		ui.torqueAll->setCheckState(Qt::PartiallyChecked);
	} else {
		if (qnode.eff.front() == 0) {
			ui.torqueAll->setCheckState(Qt::Checked);
		} else {
			ui.torqueAll->setCheckState(Qt::Unchecked);
		}
	}

}

void MainWindow::on_jslider_1_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[0] = (float) ui.jslider_1->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_2_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[1] = (float) ui.jslider_2->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_3_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[2] = (float) ui.jslider_3->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_4_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[3] = (float) ui.jslider_4->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_5_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[4] = (float) ui.jslider_5->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_6_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[5] = (float) ui.jslider_6->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_7_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[6] = (float) ui.jslider_7->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_8_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[7] = (float) ui.jslider_8->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_9_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[8] = (float) ui.jslider_9->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_10_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[9] = (float) ui.jslider_10->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_11_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[10] = (float) ui.jslider_11->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_12_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[11] = (float) ui.jslider_12->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_13_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[12] = (float) ui.jslider_13->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_14_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[13] = (float) ui.jslider_14->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_15_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[14] = (float) ui.jslider_15->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_16_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[15] = (float) ui.jslider_16->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_17_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[16] = (float) ui.jslider_17->value() / 100;
	emit jointUpdateReq();
}
void MainWindow::on_jslider_18_sliderMoved() {
	qnode.des_pos = qnode.pos;
	qnode.des_pos[17] = (float) ui.jslider_18->value() / 100;
	emit jointUpdateReq();
}

void MainWindow::valuesChanged() {

}

/*****************************************************************************
 ** Implementation [Menu]
 *****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
	QMessageBox::about(
			this,
			tr("About ..."),
			tr(
					"<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
 ** Implementation [Configuration]
 *****************************************************************************/

void MainWindow::ReadSettings() {
	QSettings settings(QString(Qsettings_organization), QString(Qsettings_application));
	restoreGeometry(settings.value("geometry").toByteArray());
	restoreState(settings.value("windowState").toByteArray());
	QString master_url = settings.value("master_url", QString("http://192.168.1.2:11311/")).toString();
	QString host_url =	settings.value("host_url", QString("192.168.1.3")).toString();
	//QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
	ui.line_edit_master->setText(master_url);
	ui.line_edit_host->setText(host_url);
	//ui.line_edit_topic->setText(topic_name);
	bool remember = settings.value("remember_settings", false).toBool();
	ui.checkbox_remember_settings->setChecked(remember);
	bool checked = settings.value("use_environment_variables", false).toBool();
	ui.checkbox_use_environment->setChecked(checked);
	if (checked) {
		ui.line_edit_master->setEnabled(false);
		ui.line_edit_host->setEnabled(false);
		//ui.line_edit_topic->setEnabled(false);
	}
}

void MainWindow::WriteSettings() {
	QSettings settings(QString(Qsettings_organization), QString(Qsettings_application));
	settings.setValue("master_url", ui.line_edit_master->text());
	settings.setValue("host_url", ui.line_edit_host->text());
	//settings.setValue("topic_name",ui.line_edit_topic->text());
	settings.setValue("use_environment_variables", QVariant(ui.checkbox_use_environment->isChecked()));
	settings.setValue("geometry", saveGeometry());
	settings.setValue("windowState", saveState());
	settings.setValue("remember_settings", QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event) {
	WriteSettings();
	QMainWindow::closeEvent(event);
}

} // namespace bioloid_gui

