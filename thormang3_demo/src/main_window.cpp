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
    , qnode_thor3(argc,argv)
    , is_updating_(false)
{
    // code to DEBUG
    DEBUG = false;

    if(argc >= 2)
    {
        std::string _debug_code(argv[1]);
        if(_debug_code == "debug")
            DEBUG = true;
        else
            DEBUG = false;
    }

    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode_thor3, SIGNAL(RosShutdown()), this, SLOT(close()));

    qRegisterMetaType< std::vector<int> >("std::vector<int>");
    QObject::connect(&qnode_thor3, SIGNAL(UpdatePresentJointControlModules(std::vector<int>)), this, SLOT(UpdatePresentJointModule(std::vector<int>)));
    QObject::connect(&qnode_thor3, SIGNAL(UpdateHeadJointsAngle(double,double)), this, SLOT(UpdateHeadJointsAngle(double,double)));

    QObject::connect(ui.head_pan_slider, SIGNAL(valueChanged(int)), this, SLOT(SetHeadJointsAngle()));
    QObject::connect(ui.head_tilt_slider, SIGNAL(valueChanged(int)), this, SLOT(SetHeadJointsAngle()));

    QObject::connect(&qnode_thor3, SIGNAL(UpdateCurrJoint(double)), this, SLOT(UpdateCurrJointSpinbox(double)));
    QObject::connect(&qnode_thor3, SIGNAL(UpdateCurrPos(double , double , double)), this, SLOT(UpdateCurrPosSpinbox(double , double , double)));
    QObject::connect(&qnode_thor3, SIGNAL(UpdateCurrOri(double , double , double, double)), this, SLOT(UpdateCurrOriSpinbox(double , double , double , double)));

    QObject::connect(ui.tabWidget_control, SIGNAL(currentChanged(int)), &qnode_thor3, SLOT(SetCurrentControlUI(int)));
    QObject::connect(&qnode_thor3, SIGNAL(HavePoseToMakeFootstep()), this, SLOT(EnableGetStepButton()));

    /*********************
    ** Logging
    **********************/
    ui.view_logging->setModel(qnode_thor3.LoggingModel());
    QObject::connect(&qnode_thor3, SIGNAL(LoggingUpdated()), this, SLOT(UpdateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    qnode_thor3.init();
    InitModeUnit();
    SetUserShortcut();
    UpdateModuleUI();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::ShowNoMasterMessage()
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

void MainWindow::on_button_assemble_lidar_clicked(bool check) { qnode_thor3.Assemble_lidar(); }
void MainWindow::on_button_clear_log_clicked(bool check) { qnode_thor3.ClearLog(); }
void MainWindow::on_button_init_pose_clicked(bool check) { qnode_thor3.MoveInitPose(); }

void MainWindow::on_button_ft_air_clicked(bool check) { qnode_thor3.InitFTCommand("ft_air"); }
void MainWindow::on_button_ft_gnd_clicked(bool check) { qnode_thor3.InitFTCommand("ft_gnd"); }
void MainWindow::on_button_ft_calc_clicked(bool check)
{
    qnode_thor3.InitFTCommand("ft_send");
    qnode_thor3.Log(QNodeThor3::Info, "Apply new FT config");
}
void MainWindow::on_button_ft_save_clicked(bool check)
{
    qnode_thor3.InitFTCommand("ft_save");
    qnode_thor3.Log(QNodeThor3::Info, "Save FT config data.");
}

// Manipulation
void MainWindow::on_inipose_button_clicked( bool check )
{
    std_msgs::String msg;

    msg.data = "ini_pose";

    qnode_thor3.SendInitPoseMsg( msg );
}

void MainWindow::on_currjoint_button_clicked( bool check )
{
    qnode_thor3.GetJointPose( ui.joint_combobox->currentText().toStdString() );
}

void MainWindow::on_desjoint_button_clicked( bool check )
{
    thormang3_manipulation_module_msgs::JointPose msg;

    msg.name = ui.joint_combobox->currentText().toStdString();
    msg.value = ui.joint_spinbox->value() * M_PI / 180.0 ;

    qnode_thor3.SendDestJointMsg( msg );
}

void MainWindow::on_currpos_button_clicked( bool check )
{
    qnode_thor3.GetKinematicsPose( ui.group_combobox->currentText().toStdString() );
}

void MainWindow::on_despos_button_clicked( bool check )
{
    thormang3_manipulation_module_msgs::KinematicsPose msg;

    msg.name = ui.group_combobox->currentText().toStdString();

    msg.pose.position.x = ui.pos_x_spinbox->value();
    msg.pose.position.y = ui.pos_y_spinbox->value();
    msg.pose.position.z = ui.pos_z_spinbox->value();

    double roll = ui.ori_roll_spinbox->value() * M_PI / 180.0;
    double pitch = ui.ori_pitch_spinbox->value() * M_PI / 180.0;
    double yaw = ui.ori_yaw_spinbox->value() * M_PI / 180.0;

    Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

    msg.pose.orientation.x = QR.x();
    msg.pose.orientation.y = QR.y();
    msg.pose.orientation.z = QR.z();
    msg.pose.orientation.w = QR.w();

    qnode_thor3.SendIkMsg( msg );
}

void MainWindow::on_button_grip_on_clicked(bool check)
{
    thormang3_manipulation_module_msgs::JointPose _msg;

    _msg.name = ui.gripper_comboBox->currentText().toStdString();
    _msg.value = 60 * M_PI / 180.0 ;

    qnode_thor3.SendDestJointMsg( _msg );
}

void MainWindow::on_button_grip_off_clicked(bool check)
{
    thormang3_manipulation_module_msgs::JointPose _msg;

    _msg.name = ui.gripper_comboBox->currentText().toStdString();
    _msg.value = 0 * M_PI / 180.0 ;

    qnode_thor3.SendDestJointMsg( _msg );
}

void MainWindow::on_A0_button_fl_clicked(bool check) { SendWalkingCommand("turn left"); }

void MainWindow::on_A1_button_f_clicked(bool check) { SendWalkingCommand("forward"); }
void MainWindow::on_A2_button_fr_clicked(bool check) { SendWalkingCommand("turn right"); }

void MainWindow::on_B0_button_l_clicked(bool check) { SendWalkingCommand("left"); }
void MainWindow::on_B1_button_stop_clicked(bool check) { }  // disable
void MainWindow::on_B2_button_r_clicked(bool check) { SendWalkingCommand("right"); }

void MainWindow::on_C0_button_bl_clicked(bool check) { }    // disable
void MainWindow::on_C1_button_b_clicked(bool check) { SendWalkingCommand("backward"); }
void MainWindow::on_C2_button_br_clicked(bool check) { }    // disable

void MainWindow::on_button_balance_on_clicked(bool check) { qnode_thor3.SetWalkingBalance(true); }
void MainWindow::on_button_balance_off_clicked(bool check) { qnode_thor3.SetWalkingBalance(false); }


void MainWindow::on_A0_button_get_step_clicked(bool check)
{
    qnode_thor3.MakeFootstepUsingPlanner();

    ui.A0_button_get_step->setEnabled(false);
    ui.A1_button_clear_step->setEnabled(true);
    ui.A2_button_go_walking->setEnabled(true);
}

void MainWindow::on_A1_button_clear_step_clicked(bool check)
{
    qnode_thor3.ClearFootsteps();

    ui.A0_button_get_step->setEnabled(false);
    ui.A1_button_clear_step->setEnabled(false);
    ui.A2_button_go_walking->setEnabled(false);
}

void MainWindow::on_A2_button_go_walking_clicked(bool check)
{
    qnode_thor3.SetWalkingFootsteps();

    ui.A0_button_get_step->setEnabled(false);
    ui.A1_button_clear_step->setEnabled(false);
    ui.A2_button_go_walking->setEnabled(false);
}

void MainWindow::on_head_center_button_clicked(bool check)
{
    //    is_updating_ == true;
    //    ui.head_pan_slider->setValue(0.0);
    //    ui.head_tilt_slider->setValue(0.0);
    //    is_updating_ == false;

    qnode_thor3.Log(QNodeThor3::Info, "Go Head init position");
    SetHeadJointsAngle(0, 0);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::UpdateLoggingView() {
    ui.view_logging->scrollToBottom();
}

// user shortcut
void MainWindow::SetUserShortcut()
{
    // Setup a signal mapper to avoid creating custom slots for each tab
    QSignalMapper *_sig_map = new QSignalMapper(this);

    // Setup the shortcut for the first tab : Mode
    QShortcut *_short_tab1 = new QShortcut(QKeySequence("F1"), this);
    connect(_short_tab1, SIGNAL(activated()), _sig_map, SLOT(map()));
    _sig_map->setMapping(_short_tab1, 0);

    // Setup the shortcut for the second tab : Manipulation
    QShortcut *_short_tab2 = new QShortcut(QKeySequence("F2"), this);
    connect(_short_tab2, SIGNAL(activated()), _sig_map, SLOT(map()));
    _sig_map->setMapping(_short_tab2, 1);

    // Setup the shortcut for the third tab : Walking
    QShortcut *_short_tab3 = new QShortcut(QKeySequence("F3"), this);
    connect(_short_tab3, SIGNAL(activated()), _sig_map, SLOT(map()));
    _sig_map->setMapping(_short_tab3, 2);

    // Setup the shortcut for the fouth tab : Head control
    QShortcut *_short_tab4 = new QShortcut(QKeySequence("F4"), this);
    connect(_short_tab4, SIGNAL(activated()), _sig_map, SLOT(map()));
    _sig_map->setMapping(_short_tab4, 3);

    // Setup the shortcut for the fouth tab : Motion
    QShortcut *_short_tab5 = new QShortcut(QKeySequence("F5"), this);
    connect(_short_tab5, SIGNAL(activated()), _sig_map, SLOT(map()));
    _sig_map->setMapping(_short_tab5, 4);

    // Wire the signal mapper to the tab widget index change slot
    connect(_sig_map, SIGNAL(mapped(int)), ui.tabWidget_control, SLOT(setCurrentIndex(int)));
}

void MainWindow::UpdatePresentJointModule(std::vector<int> mode)
{
    QList<QComboBox *> _combo_children = ui.widget_mode->findChildren<QComboBox *>();
    for(int ix = 0; ix < _combo_children.length(); ix++)
    {
        int _control_index = mode.at(ix);
        _combo_children.at(ix)->setCurrentIndex(_control_index);

        if(DEBUG)
        {
            std::stringstream _stream;
            std::string _joint;
            int _id;

            std::string _control_mode = _combo_children.at(ix)->currentText().toStdString();

            if(qnode_thor3.GetIDJointNameFromIndex(ix, _id, _joint) == true)
            {
                _stream << "[" << (_id < 10 ? "0" : "") << _id << "] "<< _joint <<" : " << _control_mode;
            }
            else
            {
                _stream << "id " << ix << " : " << _control_mode;
            }

            qnode_thor3.Log(QNodeThor3::Info, _stream.str());
        }
    }

    // set module UI
    UpdateModuleUI();
}

void MainWindow::UpdateModuleUI()
{
    if(DEBUG) return;

    for(int index = 0; index < qnode_thor3.GetModuleTableSize(); index++)
    {
        std::string _mode = qnode_thor3.GetModuleName(index);
        if(_mode == "") continue;

        std::map< std::string, QList<QWidget *> >::iterator _module_iter = module_ui_table_.find(_mode);
        if(_module_iter == module_ui_table_.end()) continue;

        QList<QWidget *> _list = _module_iter->second;
        for(int ix = 0; ix < _list.size(); ix++)
        {
            bool _is_enable = qnode_thor3.IsUsingModule(_mode);
            _list.at(ix)->setEnabled(_is_enable);
        }
    }
}

void MainWindow::UpdateHeadJointsAngle(double pan, double tilt)
{
    if(ui.head_pan_slider->underMouse() == true) return;
    if(ui.head_pan_spinbox->underMouse() == true) return;
    if(ui.head_tilt_slider->underMouse() == true) return;
    if(ui.head_tilt_spinbox->underMouse() == true) return;

    is_updating_ = true;

    ui.head_pan_slider->setValue( pan * 180.0 / M_PI );
    // ui.head_pan_spinbox->setValue( pan * 180.0 / M_PI );
    ui.head_tilt_slider->setValue( tilt * 180.0 / M_PI );
    // ui.head_tilt_spinbox->setValue( tilt * 180.0 / M_PI );

    is_updating_ = false;
}

void MainWindow::SetHeadJointsAngle()
{
    if(is_updating_ == true) return;
    qnode_thor3.SetHeadJoint(ui.head_pan_slider->value() * M_PI / 180, ui.head_tilt_slider->value() * M_PI / 180);
}

void MainWindow::SetHeadJointsAngle(double pan, double tilt)
{
    qnode_thor3.SetHeadJoint(pan * M_PI / 180, tilt * M_PI / 180);
}

// manipulation
void MainWindow::UpdateCurrJointSpinbox( double value )
{
    ui.joint_spinbox->setValue( value * 180.0 / M_PI );
}

void MainWindow::UpdateCurrPosSpinbox( double x, double y, double z )
{
    ui.pos_x_spinbox->setValue( x );
    ui.pos_y_spinbox->setValue( y );
    ui.pos_z_spinbox->setValue( z );
}

void MainWindow::UpdateCurrOriSpinbox( double x , double y , double z , double w )
{
    Eigen::Quaterniond QR(w,x,y,z);

    Eigen::MatrixXd R = QR.toRotationMatrix();

    double roll = atan2( R.coeff(2,1), R.coeff(2,2) ) * 180.0 / M_PI;
    double pitch = atan2( -R.coeff(2,0), sqrt( pow(R.coeff(2,1),2) + pow(R.coeff(2,2),2) ) ) * 180.0 / M_PI;
    double yaw = atan2 ( R.coeff(1,0) , R.coeff(0,0) ) * 180.0 / M_PI;

    ui.ori_roll_spinbox->setValue( roll );
    ui.ori_pitch_spinbox->setValue( pitch );
    ui.ori_yaw_spinbox->setValue( yaw );
}

// walking
void MainWindow::SendWalkingCommand(const std::string &command)
{
    thormang3_foot_step_generator::FootStepCommand _msg;

    _msg.command = command;
    _msg.step_num = ui.A1_spinbox_step_num->value();
    _msg.step_length = ui.B1_spinbox_f_step_length->value();
    _msg.side_step_length = ui.C1_spinbox_s_step_length->value();
    _msg.step_angle_rad = ui.D1_spinbox_r_angle->value() * M_PI / 180;

    qnode_thor3.SetWalkingCommand(_msg);
}

void MainWindow::EnableGetStepButton()
{
    ui.A0_button_get_step->setEnabled(true);
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

void MainWindow::InitModeUnit()
{
    int _number_joint = qnode_thor3.GetJointTableSize();

    // preset module button
    QHBoxLayout *_preset_layout = new QHBoxLayout;
    QSignalMapper *_signalMapper = new QSignalMapper(this);

    // parse yaml : preset modules
    for(std::map< int, std::string >::iterator _iter = qnode_thor3.module_table.begin(); _iter != qnode_thor3.module_table.end(); ++_iter)
    {
        std::string _preset_name = _iter->second;
        QPushButton *_preset_button = new QPushButton(tr(_preset_name.c_str()));
        if(DEBUG) std::cout << "name : " <<  _preset_name << std::endl;

        _preset_layout->addWidget(_preset_button);

        _signalMapper->setMapping(_preset_button, _preset_button->text());
        QObject::connect(_preset_button, SIGNAL(clicked()), _signalMapper, SLOT(map()));
    }

    QObject::connect(_signalMapper, SIGNAL(mapped(QString)), this, SLOT(EnableModule(QString)));

    ui.widget_mode_preset->setLayout(_preset_layout);

    // joints
    QGridLayout *_grid_mod = new QGridLayout;
    for(int ix = 0; ix < _number_joint; ix++)
    {
        std::stringstream _stream;
        std::string _joint;
        int _id;

        if(qnode_thor3.GetIDJointNameFromIndex(ix, _id, _joint) == false) continue;

        _stream << "[" << (_id < 10 ? "0" : "") << _id << "] "<< _joint ;
        QLabel *_label = new QLabel(tr(_stream.str().c_str()));

        QStringList _list;
        for(int index = 0; index < qnode_thor3.GetModuleTableSize(); index++)
        {
            std::string _mode = qnode_thor3.GetModuleName(index);
            if(_mode != "")
                _list << _mode.c_str();
        }

        QComboBox *_combo = new QComboBox();
        _combo->setObjectName(tr(_joint.c_str()));
        _combo->addItems(_list);
        _combo->setEnabled(false);      // not changable
        int _row = ix / 2 + 1;
        int _col = (ix % 2) * 3;
        _grid_mod->addWidget(_label, _row, _col, 1, 1);
        _grid_mod->addWidget(_combo, _row, _col + 1, 1, 2);
    }

    // get buttons
    QPushButton *_get_mode_button = new QPushButton(tr("Get Mode"));
    _grid_mod->addWidget(_get_mode_button, (_number_joint / 2) + 2, 0, 1, 3);
    QObject::connect(_get_mode_button, SIGNAL(clicked(bool)), &qnode_thor3, SLOT(GetJointControlModule()));

    ui.widget_mode->setLayout(_grid_mod);

    // make module widget table
    for(int index = 0; index < qnode_thor3.GetModuleTableSize(); index++)
    {
        std::string _mode = qnode_thor3.GetModuleName(index);
        if(_mode == "") continue;
        std::string _mode_reg = "*_" + _mode;

        QRegExp _rx(QRegExp(tr(_mode_reg.c_str())));
        _rx.setPatternSyntax(QRegExp::Wildcard);

        QList<QWidget *> _list = ui.centralwidget->findChildren<QWidget *>(_rx);
        module_ui_table_[_mode] = _list;

        if(DEBUG) std::cout << "Module widget : " << _mode << " [" << _list.size() << "]" << std::endl;
    }
}

void MainWindow::EnableModule(QString mode_name)
{
    qnode_thor3.EnableControlModule(mode_name.toStdString());
}

void MainWindow::ReadSettings()
{
    QSettings settings("Qt-Ros Package", "thor3_control");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings()
{
    QSettings settings("Qt-Ros Package", "thor3_control");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

/*****************************************************************************
** Implementation [Util]
*****************************************************************************/

Eigen::MatrixXd MainWindow::rx( double s )
{
    Eigen::MatrixXd R(3,3);

    R << 1.0, 	 0.0, 	  0.0,
            0.0, cos(s), -sin(s),
            0.0, sin(s),  cos(s);

    return R;
}

Eigen::MatrixXd MainWindow::ry( double s )
{
    Eigen::MatrixXd R(3,3);

    R << cos(s), 0.0, sin(s),
            0.0, 	 1.0, 	 0.0,
            -sin(s), 0.0, cos(s);

    return R;
}

Eigen::MatrixXd MainWindow::rz( double s )
{
    Eigen::MatrixXd R(3,3);

    R << cos(s), -sin(s), 0.0,
            sin(s),  cos(s), 0.0,
            0.0,     0.0, 1.0;

    return R;
}

Eigen::MatrixXd MainWindow::rpy2rotation( double r, double p, double y )
{
    Eigen::MatrixXd R = rz(y)*ry(p)*rx(r);

    return R;
}

Eigen::Quaterniond MainWindow::rpy2quaternion( double r, double p, double y )
{
    Eigen::MatrixXd R = rpy2rotation(r, p, y);

    Eigen::Matrix3d R_plus;
    R_plus = R.block(0,0,3,3);

    Eigen::Quaterniond QR;
    QR = R_plus;

    return QR;
}

}  // namespace thor3_control

