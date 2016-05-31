/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/


/* Author: Kayman Jung */

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/thormang3_demo/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace thor3_control {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode_thor3_(argc,argv)
  , is_updating_(false)
{
  // code to DEBUG
  DEBUG = false;

  if(argc >= 2)
  {
    std::string debug_code(argv[1]);
    if(debug_code == "debug")
      DEBUG = true;
    else
      DEBUG = false;
  }

  ui_.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  readSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui_.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode_thor3_, SIGNAL(rosShutdown()), this, SLOT(close()));

  qRegisterMetaType< std::vector<int> >("std::vector<int>");
  QObject::connect(&qnode_thor3_, SIGNAL(updatePresentJointControlModules(std::vector<int>)), this, SLOT(updatePresentJointModule(std::vector<int>)));
  QObject::connect(&qnode_thor3_, SIGNAL(updateHeadJointsAngle(double,double)), this, SLOT(updateHeadJointsAngle(double,double)));

  QObject::connect(ui_.head_pan_slider, SIGNAL(valueChanged(int)), this, SLOT(setHeadJointsAngle()));
  QObject::connect(ui_.head_tilt_slider, SIGNAL(valueChanged(int)), this, SLOT(setHeadJointsAngle()));

  QObject::connect(&qnode_thor3_, SIGNAL(updateCurrJoint(double)), this, SLOT(updateCurrJointSpinbox(double)));
  QObject::connect(&qnode_thor3_, SIGNAL(updateCurrPos(double , double , double)), this, SLOT(updateCurrPosSpinbox(double , double , double)));
  QObject::connect(&qnode_thor3_, SIGNAL(updateCurrOri(double , double , double, double)), this, SLOT(updateCurrOriSpinbox(double , double , double , double)));

  QObject::connect(ui_.tabWidget_control, SIGNAL(currentChanged(int)), &qnode_thor3_, SLOT(setCurrentControlUI(int)));
  QObject::connect(&qnode_thor3_, SIGNAL(havePoseToMakeFootstep()), this, SLOT(enableGetStepButton()));

  qRegisterMetaType<geometry_msgs::Point>("geometry_msgs::Point");
  qRegisterMetaType<geometry_msgs::Pose>("geometry_msgs::Pose");
  connect( &qnode_thor3_, SIGNAL(updateDemoPoint(geometry_msgs::Point)), this, SLOT(updatePointPanel(geometry_msgs::Point)));
  connect( &qnode_thor3_, SIGNAL(updateDemoPose(geometry_msgs::Pose)), this, SLOT(updatePosePanel(geometry_msgs::Pose)));

  /*********************
    ** Logging
    **********************/
  ui_.view_logging->setModel(qnode_thor3_.loggingModel());
  QObject::connect(&qnode_thor3_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  /*********************
    ** Auto Start
    **********************/
  qnode_thor3_.init();
  initModeUnit();
  setUserShortcut();
  updateModuleUI();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_assemble_lidar_clicked(bool check) { qnode_thor3_.assembleLidar(); }
void MainWindow::on_button_clear_log_clicked(bool check) { qnode_thor3_.clearLog(); }
void MainWindow::on_button_init_pose_clicked(bool check) { qnode_thor3_.moveInitPose(); }

void MainWindow::on_button_ft_air_clicked(bool check) { qnode_thor3_.initFTCommand("ft_air"); }
void MainWindow::on_button_ft_gnd_clicked(bool check) { qnode_thor3_.initFTCommand("ft_gnd"); }
void MainWindow::on_button_ft_calc_clicked(bool check)
{
  qnode_thor3_.initFTCommand("ft_send");
  qnode_thor3_.log(QNodeThor3::Info, "Apply new FT config");
}
void MainWindow::on_button_ft_save_clicked(bool check)
{
  qnode_thor3_.initFTCommand("ft_save");
  qnode_thor3_.log(QNodeThor3::Info, "Save FT config data.");
}

// Manipulation
void MainWindow::on_inipose_button_clicked( bool check )
{
  std_msgs::String msg;

  msg.data = "ini_pose";

  qnode_thor3_.sendInitPoseMsg( msg );
}

void MainWindow::on_currjoint_button_clicked( bool check )
{
  qnode_thor3_.getJointPose( ui_.joint_combobox->currentText().toStdString() );
}

void MainWindow::on_desjoint_button_clicked( bool check )
{
  thormang3_manipulation_module_msgs::JointPose msg;

  msg.name = ui_.joint_combobox->currentText().toStdString();
  msg.value = ui_.joint_spinbox->value() * M_PI / 180.0 ;

  qnode_thor3_.sendDestJointMsg( msg );
}

void MainWindow::on_currpos_button_clicked( bool check )
{
  qnode_thor3_.getKinematicsPose( ui_.group_combobox->currentText().toStdString() );
}

void MainWindow::on_despos_button_clicked( bool check )
{
  thormang3_manipulation_module_msgs::KinematicsPose msg;

  msg.name = ui_.group_combobox->currentText().toStdString();

  msg.pose.position.x = ui_.pos_x_spinbox->value();
  msg.pose.position.y = ui_.pos_y_spinbox->value();
  msg.pose.position.z = ui_.pos_z_spinbox->value();

  double roll = ui_.ori_roll_spinbox->value() * M_PI / 180.0;
  double pitch = ui_.ori_pitch_spinbox->value() * M_PI / 180.0;
  double yaw = ui_.ori_yaw_spinbox->value() * M_PI / 180.0;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  qnode_thor3_.sendIkMsg( msg );
}

void MainWindow::on_button_grip_on_clicked(bool check)
{
//  thormang3_manipulation_module_msgs::JointPose msg;

//  msg.name = ui_.gripper_comboBox->currentText().toStdString();
//  msg.value = 60 * M_PI / 180.0 ;

//  qnode_thor3_.sendDestJointMsg( msg );

  setGripper(60, ui_.gripper_comboBox->currentText().toStdString());
}

void MainWindow::on_button_grip_off_clicked(bool check)
{
//  thormang3_manipulation_module_msgs::JointPose msg;

//  msg.name = ui_.gripper_comboBox->currentText().toStdString();
//  msg.value = 0 * M_PI / 180.0 ;

//  qnode_thor3_.sendDestJointMsg( msg );

  setGripper(0, ui_.gripper_comboBox->currentText().toStdString());
}

void MainWindow::on_A0_button_fl_clicked(bool check) { sendWalkingCommand("turn left"); }

void MainWindow::on_A1_button_f_clicked(bool check) { sendWalkingCommand("forward"); }
void MainWindow::on_A2_button_fr_clicked(bool check) { sendWalkingCommand("turn right"); }

void MainWindow::on_B0_button_l_clicked(bool check) { sendWalkingCommand("left"); }
void MainWindow::on_B1_button_stop_clicked(bool check) { }  // disable
void MainWindow::on_B2_button_r_clicked(bool check) { sendWalkingCommand("right"); }

void MainWindow::on_C0_button_bl_clicked(bool check) { }    // disable
void MainWindow::on_C1_button_b_clicked(bool check) { sendWalkingCommand("backward"); }
void MainWindow::on_C2_button_br_clicked(bool check) { }    // disable

void MainWindow::on_button_balance_on_clicked(bool check) { qnode_thor3_.setWalkingBalance(true); }
void MainWindow::on_button_balance_off_clicked(bool check) { qnode_thor3_.setWalkingBalance(false); }
void MainWindow::on_button_balance_param_apply_clicked(bool check)
{
  double gyro_gain = ui_.dSpinBox_gyro_gain->value();
  double ft_gain = ui_.dSpinBox_gyro_gain->value();
  double imu_time_constant = ui_.dSpinBox_imu_time_constant->value();
  double ft_time_constant = ui_.dSpinBox_ft_time_constant->value();

  qnode_thor3_.setWalkingBalanceParam(gyro_gain, ft_gain, imu_time_constant, ft_time_constant);
}

void MainWindow::on_A0_button_get_step_clicked(bool check)
{
  qnode_thor3_.makeFootstepUsingPlanner();

  ui_.A0_button_get_step->setEnabled(false);
  ui_.A1_button_clear_step->setEnabled(true);
  ui_.A2_button_go_walking->setEnabled(true);
}

void MainWindow::on_A1_button_clear_step_clicked(bool check)
{
  qnode_thor3_.clearFootsteps();

  ui_.A0_button_get_step->setEnabled(false);
  ui_.A1_button_clear_step->setEnabled(false);
  ui_.A2_button_go_walking->setEnabled(false);
}

void MainWindow::on_A2_button_go_walking_clicked(bool check)
{
  qnode_thor3_.setWalkingFootsteps();

  ui_.A0_button_get_step->setEnabled(false);
  ui_.A1_button_clear_step->setEnabled(false);
  ui_.A2_button_go_walking->setEnabled(false);
}

void MainWindow::on_head_center_button_clicked(bool check)
{
  //    is_updating_ == true;
  //    ui.head_pan_slider->setValue(0.0);
  //    ui.head_tilt_slider->setValue(0.0);
  //    is_updating_ == false;

  qnode_thor3_.log(QNodeThor3::Info, "Go Head init position");
  setHeadJointsAngle(0, 0);
}

void MainWindow::on_dSpinBox_marker_pos_x_valueChanged(double value) { updateInteractiveMarker(); }
void MainWindow::on_dSpinBox_marker_pos_y_valueChanged(double value) { updateInteractiveMarker(); }
void MainWindow::on_dSpinBox_marker_pos_z_valueChanged(double value) { updateInteractiveMarker(); }

void MainWindow::on_dSpinBox_marker_ori_r_valueChanged(double value) { updateInteractiveMarker(); }
void MainWindow::on_dSpinBox_marker_ori_p_valueChanged(double value) { updateInteractiveMarker(); }
void MainWindow::on_dSpinBox_marker_ori_y_valueChanged(double value) { updateInteractiveMarker(); }

void MainWindow::on_button_marker_set_clicked() { makeInteractiveMarker(); }
void MainWindow::on_button_marker_clear_clicked() { clearMarkerPanel(); }

void MainWindow::on_button_manipulation_demo_0_clicked(bool check)
{
  // init pose : base
  qnode_thor3_.moveInitPose();
}

void MainWindow::on_button_manipulation_demo_1_clicked(bool check)
{
  // manipulation mode
  qnode_thor3_.enableControlModule("manipulation_module");
}

void MainWindow::on_button_manipulation_demo_2_clicked(bool check)
{
  // manipulation init pose
  on_inipose_button_clicked(false);
}

void MainWindow::on_button_manipulation_demo_3_clicked(bool check)
{
  // head control mode
  qnode_thor3_.enableControlModule("head_control_module");

  usleep(10 * 1000);

  // scan
  qnode_thor3_.assembleLidar();
}

void MainWindow::on_button_manipulation_demo_4_clicked(bool check)
{
  // set interactive marker
  geometry_msgs::Pose _current_pose;
  getPoseFromMarkerPanel(_current_pose);

  // set default value
  if(_current_pose.position.x == 0 && _current_pose.position.y == 0 && _current_pose.position.z == 0)
  {
    _current_pose.position.x = 0.305;
    _current_pose.position.y = (ui_.comboBox_arm_group->currentText().toStdString() == "Right Arm") ? -0.3 : 0.3;
    _current_pose.position.z = 0.8;
  }

  qnode_thor3_.makeInteractiveMarker(_current_pose);
}

void MainWindow::on_button_manipulation_demo_5_clicked(bool check)
{
  // send pose
  thormang3_manipulation_module_msgs::KinematicsPose msg;

  // arm group : left_arm_with_torso / right_arm_with_torso
  std::string arm_group = (ui_.comboBox_arm_group->currentText().toStdString() == "Right Arm") ? "right_arm_with_torso" : "left_arm_with_torso";
  msg.name = arm_group;

  msg.pose.position.x = ui_.dSpinBox_marker_pos_x->value() + ui_.dSpinBox_offset_x->value();
  msg.pose.position.y = ui_.dSpinBox_marker_pos_y->value() + ui_.dSpinBox_offset_y->value();
  msg.pose.position.z = ui_.dSpinBox_marker_pos_z->value() + ui_.dSpinBox_offset_z->value();

  double roll = ui_.dSpinBox_marker_ori_r->value() * M_PI / 180.0;
  double pitch = ui_.dSpinBox_marker_ori_p->value() * M_PI / 180.0;
  double yaw = ui_.dSpinBox_marker_ori_y->value() * M_PI / 180.0;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  qnode_thor3_.sendIkMsg( msg );

  // clear marker and foot steps
  qnode_thor3_.clearInteractiveMarker();
  qnode_thor3_.clearFootsteps();
}

void MainWindow::on_button_manipulation_demo_6_clicked(bool check)
{
  // grip on : l_arm_grip / r_arm_grip
  std::string arm_group = (ui_.comboBox_arm_group->currentText().toStdString() == "Right Arm") ? "r_arm_grip" : "l_arm_grip";
  setGripper(60, arm_group);
}

void MainWindow::on_button_manipulation_demo_7_clicked(bool check)
{
  // grip off : l_arm_grip / r_arm_grip
  std::string arm_group = (ui_.comboBox_arm_group->currentText().toStdString() == "Right Arm") ? "r_arm_grip" : "l_arm_grip";
  setGripper(0, arm_group);
}

void MainWindow::on_button_walking_demo_0_clicked(bool check)
{
  // init pose : base
  qnode_thor3_.moveInitPose();
}

void MainWindow::on_button_walking_demo_1_clicked(bool check)
{
  // head control mode
  qnode_thor3_.enableControlModule("head_control_module");

  usleep(10 * 1000);

  // scan
  qnode_thor3_.assembleLidar();
}

void MainWindow::on_button_walking_demo_2_clicked(bool check)
{
  // walking mode
  qnode_thor3_.enableControlModule("walking_module");

  usleep(10 * 1000);

  // balance on
  qnode_thor3_.setWalkingBalance(true);
}

void MainWindow::on_button_walking_demo_3_clicked(bool check)
{
  // set interactive marker
  makeInteractiveMarker();
}

void MainWindow::on_button_walking_demo_4_clicked(bool check)
{
  double y_offset = (ui_.comboBox_kick_foot->currentText().toStdString() == "Right Foot") ? 0.093 : -0.093;

  geometry_msgs::Pose target;
  target.position.x = ui_.dSpinBox_marker_pos_x->value();
  target.position.y = ui_.dSpinBox_marker_pos_y->value() + y_offset;
  target.position.z = ui_.dSpinBox_marker_pos_z->value();

  double roll = ui_.dSpinBox_marker_ori_r->value() * M_PI / 180.0;
  double pitch = ui_.dSpinBox_marker_ori_p->value() * M_PI / 180.0;
  double yaw = ui_.dSpinBox_marker_ori_y->value() * M_PI / 180.0;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  target.orientation.x = QR.x();
  target.orientation.y = QR.y();
  target.orientation.z = QR.z();
  target.orientation.w = QR.w();

  // generate foot steps
  qnode_thor3_.makeFootstepUsingPlanner(target);
}

void MainWindow::on_button_walking_demo_5_clicked(bool check)
{
  // start walking
  qnode_thor3_.setWalkingFootsteps();

  // clear marker and foot steps
  qnode_thor3_.clearInteractiveMarker();
  qnode_thor3_.clearFootsteps();
}

void MainWindow::on_button_walking_demo_6_clicked(bool check)
{
  // head control mode
  qnode_thor3_.enableControlModule("head_control_module");

  usleep(10 * 1000);

  // scan
  qnode_thor3_.assembleLidar();
}

void MainWindow::on_button_walking_demo_7_clicked(bool check)
{
  // foot : right kick / left kick
  std::string kick_command = (ui_.comboBox_kick_foot->currentText().toStdString() == "Right Foot") ? "right kick" : "left kick";
  qnode_thor3_.kickDemo(kick_command);
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
  ui_.view_logging->scrollToBottom();
}

// user shortcut
void MainWindow::setUserShortcut()
{
  // Setup a signal mapper to avoid creating custom slots for each tab
  QSignalMapper *sig_map = new QSignalMapper(this);

  // Setup the shortcut for the first tab : Mode
  QShortcut *short_tab1 = new QShortcut(QKeySequence("F1"), this);
  connect(short_tab1, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab1, 0);

  // Setup the shortcut for the second tab : Manipulation
  QShortcut *short_tab2 = new QShortcut(QKeySequence("F2"), this);
  connect(short_tab2, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab2, 1);

  // Setup the shortcut for the third tab : Walking
  QShortcut *short_tab3 = new QShortcut(QKeySequence("F3"), this);
  connect(short_tab3, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab3, 2);

  // Setup the shortcut for the fouth tab : Head control
  QShortcut *short_tab4 = new QShortcut(QKeySequence("F4"), this);
  connect(short_tab4, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab4, 3);

  // Setup the shortcut for the fouth tab : Motion
  QShortcut *short_tab5 = new QShortcut(QKeySequence("F5"), this);
  connect(short_tab5, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab5, 4);

  // Wire the signal mapper to the tab widget index change slot
  connect(sig_map, SIGNAL(mapped(int)), ui_.tabWidget_control, SLOT(setCurrentIndex(int)));
}

// mode control
// it's not used now
/*
void MainWindow::setMode(bool check)
{
    robotis_controller_msgs::JointCtrlModule control_msg;

    QList<QComboBox *> combo_children = ui.widget_mode->findChildren<QComboBox *>();
    for(int ix = 0; ix < combo_children.length(); ix++)
    {
        std::stringstream stream;
        std::string joint;
        int id;

        int control_index = combo_children.at(ix)->currentIndex();
        // if(control_index == QNodeThor3::Control_None) continue;

        std::string control_mode = combo_children.at(ix)->currentText().toStdString();

        if(qnode_thor3.getIDJointNameFromIndex(ix, id, joint) == true)
        {
            stream << "[" << (id < 10 ? "0" : "") << id << "] "<< joint <<" : " << control_mode;

            control_msg.joint_name.push_back(joint);
            control_msg.module_name.push_back(control_mode);
        }
        else
        {
            stream << "id " << ix << " : " << control_mode;
        }

        qnode_thor3.log(QNodeThor3::Info, stream.str());
    }

    // no control
    if(control_msg.joint_name.size() == 0) return;

    qnode_thor3.log(QNodeThor3::Info, "set mode");

    qnode_thor3.setJointControlMode(control_msg);
}
*/

void MainWindow::updatePresentJointModule(std::vector<int> mode)
{
  QList<QComboBox *> combo_children = ui_.widget_mode->findChildren<QComboBox *>();
  for(int ix = 0; ix < combo_children.length(); ix++)
  {
    int control_index = mode.at(ix);
    combo_children.at(ix)->setCurrentIndex(control_index);

    if(DEBUG)
    {
      std::stringstream stream;
      std::string joint;
      int id;

      std::string control_mode = combo_children.at(ix)->currentText().toStdString();

      if(qnode_thor3_.getIDJointNameFromIndex(ix, id, joint) == true)
      {
        stream << "[" << (id < 10 ? "0" : "") << id << "] "<< joint <<" : " << control_mode;
      }
      else
      {
        stream << "id " << ix << " : " << control_mode;
      }

      qnode_thor3_.log(QNodeThor3::Info, stream.str());
    }
  }

  // set module UI
  updateModuleUI();
}

void MainWindow::updateModuleUI()
{
  if(DEBUG) return;

  for(int index = 0; index < qnode_thor3_.getModuleTableSize(); index++)
  {
    std::string mode = qnode_thor3_.getModuleName(index);
    if(mode == "") continue;

    std::map< std::string, QList<QWidget *> >::iterator module_iter = module_ui_table_.find(mode);
    if(module_iter == module_ui_table_.end()) continue;

    QList<QWidget *> list = module_iter->second;
    for(int ix = 0; ix < list.size(); ix++)
    {
      bool is_enable = qnode_thor3_.isUsingModule(mode);
      list.at(ix)->setEnabled(is_enable);
    }
  }
}

void MainWindow::updateHeadJointsAngle(double pan, double tilt)
{
  if(ui_.head_pan_slider->underMouse() == true) return;
  if(ui_.head_pan_spinbox->underMouse() == true) return;
  if(ui_.head_tilt_slider->underMouse() == true) return;
  if(ui_.head_tilt_spinbox->underMouse() == true) return;

  is_updating_ = true;

  ui_.head_pan_slider->setValue( pan * 180.0 / M_PI );
  // ui.head_pan_spinbox->setValue( pan * 180.0 / M_PI );
  ui_.head_tilt_slider->setValue( tilt * 180.0 / M_PI );
  // ui.head_tilt_spinbox->setValue( tilt * 180.0 / M_PI );

  is_updating_ = false;
}

void MainWindow::setHeadJointsAngle()
{
  if(is_updating_ == true) return;
  qnode_thor3_.setHeadJoint(ui_.head_pan_slider->value() * M_PI / 180, ui_.head_tilt_slider->value() * M_PI / 180);
}

void MainWindow::setHeadJointsAngle(double pan, double tilt)
{
  qnode_thor3_.setHeadJoint(pan * M_PI / 180, tilt * M_PI / 180);
}

// manipulation
void MainWindow::updateCurrJointSpinbox( double value )
{
  ui_.joint_spinbox->setValue( value * 180.0 / M_PI );
}

void MainWindow::updateCurrPosSpinbox( double x, double y, double z )
{
  ui_.pos_x_spinbox->setValue( x );
  ui_.pos_y_spinbox->setValue( y );
  ui_.pos_z_spinbox->setValue( z );
}

void MainWindow::updateCurrOriSpinbox( double x , double y , double z , double w )
{
  Eigen::Quaterniond QR(w,x,y,z);

  Eigen::MatrixXd R = QR.toRotationMatrix();

  double roll = atan2( R.coeff(2,1), R.coeff(2,2) ) * 180.0 / M_PI;
  double pitch = atan2( -R.coeff(2,0), sqrt( pow(R.coeff(2,1),2) + pow(R.coeff(2,2),2) ) ) * 180.0 / M_PI;
  double yaw = atan2 ( R.coeff(1,0) , R.coeff(0,0) ) * 180.0 / M_PI;

  ui_.ori_roll_spinbox->setValue( roll );
  ui_.ori_pitch_spinbox->setValue( pitch );
  ui_.ori_yaw_spinbox->setValue( yaw );
}
void MainWindow::setGripper(const double &angle_deg, const std::string &arm_type)
{
  thormang3_manipulation_module_msgs::JointPose msg;

  msg.name = arm_type;
  msg.value = angle_deg * M_PI / 180.0 ;

  qnode_thor3_.sendDestJointMsg( msg );
}

// walking
void MainWindow::sendWalkingCommand(const std::string &command)
{
  thormang3_foot_step_generator::FootStepCommand msg;

  msg.command = command;
  msg.step_num = ui_.A1_spinbox_step_num->value();
  msg.step_length = ui_.B1_spinbox_f_step_length->value();
  msg.side_step_length = ui_.C1_spinbox_s_step_length->value();
  msg.step_angle_rad = ui_.D1_spinbox_r_angle->value() * M_PI / 180;

  qnode_thor3_.setWalkingCommand(msg);
}

void MainWindow::enableGetStepButton()
{
  ui_.A0_button_get_step->setEnabled(true);
}

// Update UI - position
void MainWindow::updatePointPanel(const geometry_msgs::Point point)
{
  is_updating_ = true;

  setPointToMarkerPanel(point);

  ROS_INFO("Update Position Panel");
  is_updating_ = false;
}

// Update UI - pose
void MainWindow::updatePosePanel(const geometry_msgs::Pose pose)
{
  is_updating_ = true;

  setPoseToMarkerPanel(pose);

  ROS_INFO("Update Pose Panel");
  is_updating_ = false;
}

void MainWindow::getPoseFromMarkerPanel(geometry_msgs::Pose &current)
{
  // position
  current.position.x = ui_.dSpinBox_marker_pos_x->value();
  current.position.y = ui_.dSpinBox_marker_pos_y->value();
  current.position.z = ui_.dSpinBox_marker_pos_z->value();

  // orientation
  Eigen::Vector3d _euler(ui_.dSpinBox_marker_ori_r->value(),
                         ui_.dSpinBox_marker_ori_p->value(),
                         ui_.dSpinBox_marker_ori_y->value());
  Eigen::Quaterniond _q = rpy2quaternion(deg2rad<Eigen::Vector3d>(_euler));

  tf::quaternionEigenToMsg(_q, current.orientation);
}

void MainWindow::setPoseToMarkerPanel(const geometry_msgs::Pose &current)
{
  // position
  ui_.dSpinBox_marker_pos_x->setValue(current.position.x);
  ui_.dSpinBox_marker_pos_y->setValue(current.position.y);
  ui_.dSpinBox_marker_pos_z->setValue(current.position.z);

  // orientation
  Eigen::Vector3d _euler = rad2deg<Eigen::Vector3d>(quaternion2rpy(current.orientation));

  ui_.dSpinBox_marker_ori_r->setValue(_euler[0]);
  ui_.dSpinBox_marker_ori_p->setValue(_euler[1]);
  ui_.dSpinBox_marker_ori_y->setValue(_euler[2]);
}

void MainWindow::getPointFromMarkerPanel(geometry_msgs::Point &current)
{
  // position
  current.x = ui_.dSpinBox_marker_pos_x->value();
  current.y = ui_.dSpinBox_marker_pos_y->value();
  current.z = ui_.dSpinBox_marker_pos_z->value();
}

void MainWindow::setPointToMarkerPanel(const geometry_msgs::Point &current)
{
  // position
  ui_.dSpinBox_marker_pos_x->setValue(current.x);
  ui_.dSpinBox_marker_pos_y->setValue(current.y);
  ui_.dSpinBox_marker_pos_z->setValue(current.z);

  // orientation
  ui_.dSpinBox_marker_ori_r->setValue(0.0);
  ui_.dSpinBox_marker_ori_p->setValue(0.0);
  ui_.dSpinBox_marker_ori_y->setValue(0.0);
}

// make interactive marker
void MainWindow::makeInteractiveMarker()
{
  geometry_msgs::Pose _current_pose;
  getPoseFromMarkerPanel(_current_pose);

  qnode_thor3_.makeInteractiveMarker(_current_pose);
}

// update interactive marker pose from ui
void MainWindow::updateInteractiveMarker()
{
  if(is_updating_ == true) return;

  geometry_msgs::Pose _current_pose;
  getPoseFromMarkerPanel(_current_pose);

  qnode_thor3_.updateInteractiveMarker(_current_pose);
}

void MainWindow::clearMarkerPanel()
{
  geometry_msgs::Pose _init;
  updatePosePanel(_init);

  ROS_INFO("Clear Panel");

  qnode_thor3_.clearInteractiveMarker();
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

void MainWindow::initModeUnit()
{
  int number_joint = qnode_thor3_.getJointTableSize();

  // preset module button
  QHBoxLayout *preset_layout = new QHBoxLayout;
  QSignalMapper *signalMapper = new QSignalMapper(this);

  // parse yaml : preset modules
  for(std::map< int, std::string >::iterator iter = qnode_thor3_.module_table_.begin(); iter != qnode_thor3_.module_table_.end(); ++iter)
  {
    std::string preset_name = iter->second;
    QPushButton *preset_button = new QPushButton(tr(preset_name.c_str()));
    if(DEBUG) std::cout << "name : " <<  preset_name << std::endl;

    preset_layout->addWidget(preset_button);

    signalMapper->setMapping(preset_button, preset_button->text());
    QObject::connect(preset_button, SIGNAL(clicked()), signalMapper, SLOT(map()));
  }

  // QObject::connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(setPreset(QString)));
  QObject::connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(enableModule(QString)));

  ui_.widget_mode_preset->setLayout(preset_layout);

  // joints
  QGridLayout *grid_mod = new QGridLayout;
  for(int ix = 0; ix < number_joint; ix++)
  {
    std::stringstream stream;
    std::string joint;
    int id;

    if(qnode_thor3_.getIDJointNameFromIndex(ix, id, joint) == false) continue;

    stream << "[" << (id < 10 ? "0" : "") << id << "] "<< joint ;
    QLabel *label = new QLabel(tr(stream.str().c_str()));

    QStringList list;
    for(int index = 0; index < qnode_thor3_.getModuleTableSize(); index++)
    {
      std::string mode = qnode_thor3_.getModuleName(index);
      if(mode != "")
        list << mode.c_str();
    }

    QComboBox *combo = new QComboBox();
    combo->setObjectName(tr(joint.c_str()));
    combo->addItems(list);
    combo->setEnabled(false);      // not changable
    int row = ix / 2 + 1;
    int col = (ix % 2) * 3;
    grid_mod->addWidget(label, row, col, 1, 1);
    grid_mod->addWidget(combo, row, col + 1, 1, 2);
  }

  // get buttons
  QPushButton *get_mode_button = new QPushButton(tr("Get Mode"));
  grid_mod->addWidget(get_mode_button, (number_joint / 2) + 2, 0, 1, 3);
  QObject::connect(get_mode_button, SIGNAL(clicked(bool)), &qnode_thor3_, SLOT(getJointControlModule()));

  ui_.widget_mode->setLayout(grid_mod);

  // make module widget table
  for(int index = 0; index < qnode_thor3_.getModuleTableSize(); index++)
  {
    std::string mode = qnode_thor3_.getModuleName(index);
    if(mode == "") continue;
    std::string mode_reg = "*" + mode;

    QRegExp rx(QRegExp(tr(mode_reg.c_str())));
    rx.setPatternSyntax(QRegExp::Wildcard);

    QList<QWidget *> list = ui_.centralwidget->findChildren<QWidget *>(rx);
    module_ui_table_[mode] = list;

    if(DEBUG) std::cout << "Module widget : " << mode << " [" << list.size() << "]" << std::endl;
  }

  // make motion tab
  if(qnode_thor3_.getModuleIndex("action_module") != -1) initMotionUnit();
}

void MainWindow::initMotionUnit()
{
  // preset button
  QGridLayout *motion_layout = new QGridLayout;
  QSignalMapper *signalMapper = new QSignalMapper(this);

  // yaml preset
  int index = 0;
  for(std::map< int, std::string >::iterator iter = qnode_thor3_.motion_table_.begin(); iter != qnode_thor3_.motion_table_.end(); ++iter)
  {
    int motion_index = iter->first;
    std::string motion_name = iter->second;
    QString q_motion_name = QString::fromStdString(motion_name);
    QPushButton *motion_button = new QPushButton(q_motion_name);
    // if(DEBUG) std::cout << "name : " <<  motion_name << std::endl;

    int size = (motion_index < 0) ? 2 : 1;
    int row = index / 4;
    int col = index % 4;
    motion_layout->addWidget(motion_button, row, col, 1, size);

    signalMapper->setMapping(motion_button, motion_index);
    QObject::connect(motion_button, SIGNAL(clicked()), signalMapper, SLOT(map()));

    index += size;
  }

  int row = index / 4;
  row = (index % 4 == 0) ? row : row + 1;
  QSpacerItem *verticalSpacer = new QSpacerItem(20, 400, QSizePolicy::Minimum, QSizePolicy::Expanding);
  motion_layout->addItem(verticalSpacer, row, 0, 1, 4);

  QObject::connect(signalMapper, SIGNAL(mapped(int)), &qnode_thor3_, SLOT(playMotion(int)));

  ui_.scroll_widget_motion->setLayout(motion_layout);
}

void MainWindow::enableModule(QString mode_name)
{
  qnode_thor3_.enableControlModule(mode_name.toStdString());
}

void MainWindow::readSettings()
{
  QSettings settings("Qt-Ros Package", "thor3_control");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::writeSettings()
{
  QSettings settings("Qt-Ros Package", "thor3_control");
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  writeSettings();
  QMainWindow::closeEvent(event);
}

/*****************************************************************************
** Implementation [Util]
*****************************************************************************/
// math : euler & quaternion & rotation mat
Eigen::Vector3d MainWindow::rotation2rpy(const Eigen::MatrixXd &rotation )
{
  Eigen::Vector3d _rpy;

  _rpy[0] = atan2( rotation.coeff( 2 , 1 ), rotation.coeff( 2 , 2 ) );
  _rpy[1] = atan2(-rotation.coeff( 2 , 0 ), sqrt( pow( rotation.coeff( 2 , 1 ) , 2 ) + pow( rotation.coeff( 2 , 2 ) , 2 ) ) );
  _rpy[2] = atan2( rotation.coeff( 1 , 0 ), rotation.coeff( 0 , 0 ) );

  return _rpy;
}

Eigen::MatrixXd MainWindow::rpy2rotation(const double &roll, const double &pitch, const double &yaw )
{
  Eigen::MatrixXd _rotation = rotationZ( yaw ) * rotationY( pitch ) * rotationX( roll );

  return _rotation;
}

Eigen::Quaterniond MainWindow::rpy2quaternion(const Eigen::Vector3d &euler)
{
  return rpy2quaternion(euler[0], euler[1], euler[2]);
}

Eigen::Quaterniond MainWindow::rpy2quaternion(const double &roll, const double &pitch, const double &yaw )
{
  Eigen::MatrixXd _rotation = rpy2rotation( roll, pitch, yaw );

  Eigen::Matrix3d _rotation3d;
  _rotation3d = _rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;

  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::Quaterniond MainWindow::rotation2quaternion(const Eigen::MatrixXd &rotation )
{
  Eigen::Matrix3d _rotation3d;

  _rotation3d = rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;
  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::Vector3d MainWindow::quaternion2rpy(const Eigen::Quaterniond &quaternion )
{
  Eigen::Vector3d _rpy = rotation2rpy( quaternion.toRotationMatrix() );

  return _rpy;
}

Eigen::Vector3d MainWindow::quaternion2rpy(const geometry_msgs::Quaternion &quaternion)
{
  Eigen::Quaterniond _quaternion;
  tf::quaternionMsgToEigen(quaternion, _quaternion);

  Eigen::Vector3d _rpy = rotation2rpy( _quaternion.toRotationMatrix() );

  return _rpy;
}

Eigen::MatrixXd MainWindow::quaternion2rotation(const Eigen::Quaterniond &quaternion )
{
  Eigen::MatrixXd _rotation = quaternion.toRotationMatrix();

  return _rotation;
}

Eigen::MatrixXd MainWindow::rotationX(const double &angle )
{
  Eigen::MatrixXd _rotation( 3 , 3 );

  _rotation << 1.0,          0.0,           0.0,
      0.0, cos( angle ), -sin( angle ),
      0.0, sin( angle ),  cos( angle );

  return _rotation;
}

Eigen::MatrixXd MainWindow::rotationY(const double &angle )
{
  Eigen::MatrixXd _rotation( 3 , 3 );

  _rotation << cos( angle ), 0.0, sin( angle ),
      0.0, 1.0,          0.0,
      -sin( angle ), 0.0, cos( angle );

  return _rotation;
}

Eigen::MatrixXd MainWindow::rotationZ(const double &angle )
{
  Eigen::MatrixXd _rotation(3,3);

  _rotation << cos( angle ), -sin( angle ), 0.0,
      sin( angle ),  cos( angle ), 0.0,
      0.0,           0.0, 1.0;

  return _rotation;
}

}  // namespace thor3_control
