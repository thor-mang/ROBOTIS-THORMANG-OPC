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
#include "../include/thormang3_offset_tuner_client/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace thormang3_offset_tuner_client {

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

	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    all_torque_on = false;

    id_1_double_spinbox.append( ui.id_1_goal_spinbox );
    id_1_double_spinbox.append( ui.id_1_offset_spinbox );
    id_1_double_spinbox.append( ui.id_1_present_spinbox );
    id_1_int_spinbox.append( ui.id_1_p_spinbox );
    id_1_int_spinbox.append( ui.id_1_i_spinbox );
    id_1_int_spinbox.append( ui.id_1_d_spinbox );

    id_2_double_spinbox.append( ui.id_2_goal_spinbox );
    id_2_double_spinbox.append( ui.id_2_offset_spinbox );
    id_2_double_spinbox.append( ui.id_2_present_spinbox );
    id_2_int_spinbox.append( ui.id_2_p_spinbox );
    id_2_int_spinbox.append( ui.id_2_i_spinbox );
    id_2_int_spinbox.append( ui.id_2_d_spinbox );

    id_3_double_spinbox.append( ui.id_3_goal_spinbox );
    id_3_double_spinbox.append( ui.id_3_offset_spinbox );
    id_3_double_spinbox.append( ui.id_3_present_spinbox );
    id_3_int_spinbox.append( ui.id_3_p_spinbox );
    id_3_int_spinbox.append( ui.id_3_i_spinbox );
    id_3_int_spinbox.append( ui.id_3_d_spinbox );

    id_4_double_spinbox.append( ui.id_4_goal_spinbox );
    id_4_double_spinbox.append( ui.id_4_offset_spinbox );
    id_4_double_spinbox.append( ui.id_4_present_spinbox );
    id_4_int_spinbox.append( ui.id_4_p_spinbox );
    id_4_int_spinbox.append( ui.id_4_i_spinbox );
    id_4_int_spinbox.append( ui.id_4_d_spinbox );

    id_5_double_spinbox.append( ui.id_5_goal_spinbox );
    id_5_double_spinbox.append( ui.id_5_offset_spinbox );
    id_5_double_spinbox.append( ui.id_5_present_spinbox );
    id_5_int_spinbox.append( ui.id_5_p_spinbox );
    id_5_int_spinbox.append( ui.id_5_i_spinbox );
    id_5_int_spinbox.append( ui.id_5_d_spinbox );

//    for(int ix = 0; ix < id_5_double_spinbox.size(); ix++)
//    {
//        QDoubleSpinBox * _sb = id_5_double_spinbox[ix];
//        QObject::connect(_sb, SIGNAL(valueChanged(double)), this, SLOT(on_id_5_valueChanged()));
//    }

//    for(int ix = 0; ix < id_5_int_spinbox.size(); ix++)
//    {
//        QSpinBox * _sb = id_5_int_spinbox[ix];
//        QObject::connect(_sb, SIGNAL(valueChanged(int)), this, SLOT(on_id_5_valueChanged()));
//    }

    id_6_double_spinbox.append( ui.id_6_goal_spinbox );
    id_6_double_spinbox.append( ui.id_6_offset_spinbox );
    id_6_double_spinbox.append( ui.id_6_present_spinbox );
    id_6_int_spinbox.append( ui.id_6_p_spinbox );
    id_6_int_spinbox.append( ui.id_6_i_spinbox );
    id_6_int_spinbox.append( ui.id_6_d_spinbox );

    id_7_double_spinbox.append( ui.id_7_goal_spinbox );
    id_7_double_spinbox.append( ui.id_7_offset_spinbox );
    id_7_double_spinbox.append( ui.id_7_present_spinbox );
    id_7_int_spinbox.append( ui.id_7_p_spinbox );
    id_7_int_spinbox.append( ui.id_7_i_spinbox );
    id_7_int_spinbox.append( ui.id_7_d_spinbox );

    id_8_double_spinbox.append( ui.id_8_goal_spinbox );
    id_8_double_spinbox.append( ui.id_8_offset_spinbox );
    id_8_double_spinbox.append( ui.id_8_present_spinbox );
    id_8_int_spinbox.append( ui.id_8_p_spinbox );
    id_8_int_spinbox.append( ui.id_8_i_spinbox );
    id_8_int_spinbox.append( ui.id_8_d_spinbox );

    id_9_double_spinbox.append( ui.id_9_goal_spinbox );
    id_9_double_spinbox.append( ui.id_9_offset_spinbox );
    id_9_double_spinbox.append( ui.id_9_present_spinbox );
    id_9_int_spinbox.append( ui.id_9_p_spinbox );
    id_9_int_spinbox.append( ui.id_9_i_spinbox );
    id_9_int_spinbox.append( ui.id_9_d_spinbox );

    id_10_double_spinbox.append( ui.id_10_goal_spinbox );
    id_10_double_spinbox.append( ui.id_10_offset_spinbox );
    id_10_double_spinbox.append( ui.id_10_present_spinbox );
    id_10_int_spinbox.append( ui.id_10_p_spinbox );
    id_10_int_spinbox.append( ui.id_10_i_spinbox );
    id_10_int_spinbox.append( ui.id_10_d_spinbox );

    id_11_double_spinbox.append( ui.id_11_goal_spinbox );
    id_11_double_spinbox.append( ui.id_11_offset_spinbox );
    id_11_double_spinbox.append( ui.id_11_present_spinbox );
    id_11_int_spinbox.append( ui.id_11_p_spinbox );
    id_11_int_spinbox.append( ui.id_11_i_spinbox );
    id_11_int_spinbox.append( ui.id_11_d_spinbox );

    id_12_double_spinbox.append( ui.id_12_goal_spinbox );
    id_12_double_spinbox.append( ui.id_12_offset_spinbox );
    id_12_double_spinbox.append( ui.id_12_present_spinbox );
    id_12_int_spinbox.append( ui.id_12_p_spinbox );
    id_12_int_spinbox.append( ui.id_12_i_spinbox );
    id_12_int_spinbox.append( ui.id_12_d_spinbox );

    id_13_double_spinbox.append( ui.id_13_goal_spinbox );
    id_13_double_spinbox.append( ui.id_13_offset_spinbox );
    id_13_double_spinbox.append( ui.id_13_present_spinbox );
    id_13_int_spinbox.append( ui.id_13_p_spinbox );
    id_13_int_spinbox.append( ui.id_13_i_spinbox );
    id_13_int_spinbox.append( ui.id_13_d_spinbox );

    id_14_double_spinbox.append( ui.id_14_goal_spinbox );
    id_14_double_spinbox.append( ui.id_14_offset_spinbox );
    id_14_double_spinbox.append( ui.id_14_present_spinbox );
    id_14_int_spinbox.append( ui.id_14_p_spinbox );
    id_14_int_spinbox.append( ui.id_14_i_spinbox );
    id_14_int_spinbox.append( ui.id_14_d_spinbox );

    id_15_double_spinbox.append( ui.id_15_goal_spinbox );
    id_15_double_spinbox.append( ui.id_15_offset_spinbox );
    id_15_double_spinbox.append( ui.id_15_present_spinbox );
    id_15_int_spinbox.append( ui.id_15_p_spinbox );
    id_15_int_spinbox.append( ui.id_15_i_spinbox );
    id_15_int_spinbox.append( ui.id_15_d_spinbox );

    id_16_double_spinbox.append( ui.id_16_goal_spinbox );
    id_16_double_spinbox.append( ui.id_16_offset_spinbox );
    id_16_double_spinbox.append( ui.id_16_present_spinbox );
    id_16_int_spinbox.append( ui.id_16_p_spinbox );
    id_16_int_spinbox.append( ui.id_16_i_spinbox );
    id_16_int_spinbox.append( ui.id_16_d_spinbox );

    id_17_double_spinbox.append( ui.id_17_goal_spinbox );
    id_17_double_spinbox.append( ui.id_17_offset_spinbox );
    id_17_double_spinbox.append( ui.id_17_present_spinbox );
    id_17_int_spinbox.append( ui.id_17_p_spinbox );
    id_17_int_spinbox.append( ui.id_17_i_spinbox );
    id_17_int_spinbox.append( ui.id_17_d_spinbox );

    id_18_double_spinbox.append( ui.id_18_goal_spinbox );
    id_18_double_spinbox.append( ui.id_18_offset_spinbox );
    id_18_double_spinbox.append( ui.id_18_present_spinbox );
    id_18_int_spinbox.append( ui.id_18_p_spinbox );
    id_18_int_spinbox.append( ui.id_18_i_spinbox );
    id_18_int_spinbox.append( ui.id_18_d_spinbox );

    id_19_double_spinbox.append( ui.id_19_goal_spinbox );
    id_19_double_spinbox.append( ui.id_19_offset_spinbox );
    id_19_double_spinbox.append( ui.id_19_present_spinbox );
    id_19_int_spinbox.append( ui.id_19_p_spinbox );
    id_19_int_spinbox.append( ui.id_19_i_spinbox );
    id_19_int_spinbox.append( ui.id_19_d_spinbox );

    id_20_double_spinbox.append( ui.id_20_goal_spinbox );
    id_20_double_spinbox.append( ui.id_20_offset_spinbox );
    id_20_double_spinbox.append( ui.id_20_present_spinbox );
    id_20_int_spinbox.append( ui.id_20_p_spinbox );
    id_20_int_spinbox.append( ui.id_20_i_spinbox );
    id_20_int_spinbox.append( ui.id_20_d_spinbox );

    id_21_double_spinbox.append( ui.id_21_goal_spinbox );
    id_21_double_spinbox.append( ui.id_21_offset_spinbox );
    id_21_double_spinbox.append( ui.id_21_present_spinbox );
    id_21_int_spinbox.append( ui.id_21_p_spinbox );
    id_21_int_spinbox.append( ui.id_21_i_spinbox );
    id_21_int_spinbox.append( ui.id_21_d_spinbox );

    id_22_double_spinbox.append( ui.id_22_goal_spinbox );
    id_22_double_spinbox.append( ui.id_22_offset_spinbox );
    id_22_double_spinbox.append( ui.id_22_present_spinbox );
    id_22_int_spinbox.append( ui.id_22_p_spinbox );
    id_22_int_spinbox.append( ui.id_22_i_spinbox );
    id_22_int_spinbox.append( ui.id_22_d_spinbox );

    id_23_double_spinbox.append( ui.id_23_goal_spinbox );
    id_23_double_spinbox.append( ui.id_23_offset_spinbox );
    id_23_double_spinbox.append( ui.id_23_present_spinbox );
    id_23_int_spinbox.append( ui.id_23_p_spinbox );
    id_23_int_spinbox.append( ui.id_23_i_spinbox );
    id_23_int_spinbox.append( ui.id_23_d_spinbox );

    id_24_double_spinbox.append( ui.id_24_goal_spinbox );
    id_24_double_spinbox.append( ui.id_24_offset_spinbox );
    id_24_double_spinbox.append( ui.id_24_present_spinbox );
    id_24_int_spinbox.append( ui.id_24_p_spinbox );
    id_24_int_spinbox.append( ui.id_24_i_spinbox );
    id_24_int_spinbox.append( ui.id_24_d_spinbox );

    id_25_double_spinbox.append( ui.id_25_goal_spinbox );
    id_25_double_spinbox.append( ui.id_25_offset_spinbox );
    id_25_double_spinbox.append( ui.id_25_present_spinbox );
    id_25_int_spinbox.append( ui.id_25_p_spinbox );
    id_25_int_spinbox.append( ui.id_25_i_spinbox );
    id_25_int_spinbox.append( ui.id_25_d_spinbox );

    id_26_double_spinbox.append( ui.id_26_goal_spinbox );
    id_26_double_spinbox.append( ui.id_26_offset_spinbox );
    id_26_double_spinbox.append( ui.id_26_present_spinbox );
    id_26_int_spinbox.append( ui.id_26_p_spinbox );
    id_26_int_spinbox.append( ui.id_26_i_spinbox );
    id_26_int_spinbox.append( ui.id_26_d_spinbox );

    id_27_double_spinbox.append( ui.id_27_goal_spinbox );
    id_27_double_spinbox.append( ui.id_27_offset_spinbox );
    id_27_double_spinbox.append( ui.id_27_present_spinbox );
    id_27_int_spinbox.append( ui.id_27_p_spinbox );
    id_27_int_spinbox.append( ui.id_27_i_spinbox );
    id_27_int_spinbox.append( ui.id_27_d_spinbox );

    id_28_double_spinbox.append( ui.id_28_goal_spinbox );
    id_28_double_spinbox.append( ui.id_28_offset_spinbox );
    id_28_double_spinbox.append( ui.id_28_present_spinbox );
    id_28_int_spinbox.append( ui.id_28_p_spinbox );
    id_28_int_spinbox.append( ui.id_28_i_spinbox );
    id_28_int_spinbox.append( ui.id_28_d_spinbox );

    id_29_double_spinbox.append( ui.id_29_goal_spinbox );
    id_29_double_spinbox.append( ui.id_29_offset_spinbox );
    id_29_double_spinbox.append( ui.id_29_present_spinbox );
    id_29_int_spinbox.append( ui.id_29_p_spinbox );
    id_29_int_spinbox.append( ui.id_29_i_spinbox );
    id_29_int_spinbox.append( ui.id_29_d_spinbox );

    id_30_double_spinbox.append( ui.id_30_goal_spinbox );
    id_30_double_spinbox.append( ui.id_30_offset_spinbox );
    id_30_double_spinbox.append( ui.id_30_present_spinbox );
    id_30_int_spinbox.append( ui.id_30_p_spinbox );
    id_30_int_spinbox.append( ui.id_30_i_spinbox );
    id_30_int_spinbox.append( ui.id_30_d_spinbox );

    id_31_double_spinbox.append( ui.id_31_goal_spinbox );
    id_31_double_spinbox.append( ui.id_31_offset_spinbox );
    id_31_double_spinbox.append( ui.id_31_present_spinbox );
    id_31_int_spinbox.append( ui.id_31_p_spinbox );
    id_31_int_spinbox.append( ui.id_31_i_spinbox );
    id_31_int_spinbox.append( ui.id_31_d_spinbox );

    QObject::connect(ui.id_1_checkbox, SIGNAL(clicked(bool)), ui.id_1_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_1_checkbox, SIGNAL(clicked(bool)), ui.id_1_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_1_checkbox, SIGNAL(clicked(bool)), ui.id_1_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_1_checkbox, SIGNAL(clicked(bool)), ui.id_1_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_1_checkbox, SIGNAL(clicked(bool)), ui.id_1_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_1_checkbox, SIGNAL(clicked(bool)), ui.id_1_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_1_checkbox, SIGNAL(clicked(bool)), ui.id_1_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_2_checkbox, SIGNAL(clicked(bool)), ui.id_2_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_2_checkbox, SIGNAL(clicked(bool)), ui.id_2_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_2_checkbox, SIGNAL(clicked(bool)), ui.id_2_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_2_checkbox, SIGNAL(clicked(bool)), ui.id_2_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_2_checkbox, SIGNAL(clicked(bool)), ui.id_2_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_2_checkbox, SIGNAL(clicked(bool)), ui.id_2_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_2_checkbox, SIGNAL(clicked(bool)), ui.id_2_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_3_checkbox, SIGNAL(clicked(bool)), ui.id_3_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_3_checkbox, SIGNAL(clicked(bool)), ui.id_3_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_3_checkbox, SIGNAL(clicked(bool)), ui.id_3_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_3_checkbox, SIGNAL(clicked(bool)), ui.id_3_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_3_checkbox, SIGNAL(clicked(bool)), ui.id_3_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_3_checkbox, SIGNAL(clicked(bool)), ui.id_3_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_3_checkbox, SIGNAL(clicked(bool)), ui.id_3_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_4_checkbox, SIGNAL(clicked(bool)), ui.id_4_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_4_checkbox, SIGNAL(clicked(bool)), ui.id_4_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_4_checkbox, SIGNAL(clicked(bool)), ui.id_4_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_4_checkbox, SIGNAL(clicked(bool)), ui.id_4_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_4_checkbox, SIGNAL(clicked(bool)), ui.id_4_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_4_checkbox, SIGNAL(clicked(bool)), ui.id_4_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_4_checkbox, SIGNAL(clicked(bool)), ui.id_4_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_5_checkbox, SIGNAL(clicked(bool)), ui.id_5_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_5_checkbox, SIGNAL(clicked(bool)), ui.id_5_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_5_checkbox, SIGNAL(clicked(bool)), ui.id_5_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_5_checkbox, SIGNAL(clicked(bool)), ui.id_5_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_5_checkbox, SIGNAL(clicked(bool)), ui.id_5_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_5_checkbox, SIGNAL(clicked(bool)), ui.id_5_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_5_checkbox, SIGNAL(clicked(bool)), ui.id_5_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_6_checkbox, SIGNAL(clicked(bool)), ui.id_6_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_6_checkbox, SIGNAL(clicked(bool)), ui.id_6_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_6_checkbox, SIGNAL(clicked(bool)), ui.id_6_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_6_checkbox, SIGNAL(clicked(bool)), ui.id_6_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_6_checkbox, SIGNAL(clicked(bool)), ui.id_6_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_6_checkbox, SIGNAL(clicked(bool)), ui.id_6_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_6_checkbox, SIGNAL(clicked(bool)), ui.id_6_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_7_checkbox, SIGNAL(clicked(bool)), ui.id_7_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_7_checkbox, SIGNAL(clicked(bool)), ui.id_7_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_7_checkbox, SIGNAL(clicked(bool)), ui.id_7_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_7_checkbox, SIGNAL(clicked(bool)), ui.id_7_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_7_checkbox, SIGNAL(clicked(bool)), ui.id_7_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_7_checkbox, SIGNAL(clicked(bool)), ui.id_7_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_7_checkbox, SIGNAL(clicked(bool)), ui.id_7_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_8_checkbox, SIGNAL(clicked(bool)), ui.id_8_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_8_checkbox, SIGNAL(clicked(bool)), ui.id_8_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_8_checkbox, SIGNAL(clicked(bool)), ui.id_8_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_8_checkbox, SIGNAL(clicked(bool)), ui.id_8_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_8_checkbox, SIGNAL(clicked(bool)), ui.id_8_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_8_checkbox, SIGNAL(clicked(bool)), ui.id_8_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_8_checkbox, SIGNAL(clicked(bool)), ui.id_8_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_9_checkbox, SIGNAL(clicked(bool)), ui.id_9_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_9_checkbox, SIGNAL(clicked(bool)), ui.id_9_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_9_checkbox, SIGNAL(clicked(bool)), ui.id_9_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_9_checkbox, SIGNAL(clicked(bool)), ui.id_9_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_9_checkbox, SIGNAL(clicked(bool)), ui.id_9_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_9_checkbox, SIGNAL(clicked(bool)), ui.id_9_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_9_checkbox, SIGNAL(clicked(bool)), ui.id_9_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_10_checkbox, SIGNAL(clicked(bool)), ui.id_10_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_10_checkbox, SIGNAL(clicked(bool)), ui.id_10_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_10_checkbox, SIGNAL(clicked(bool)), ui.id_10_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_10_checkbox, SIGNAL(clicked(bool)), ui.id_10_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_10_checkbox, SIGNAL(clicked(bool)), ui.id_10_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_10_checkbox, SIGNAL(clicked(bool)), ui.id_10_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_10_checkbox, SIGNAL(clicked(bool)), ui.id_10_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_11_checkbox, SIGNAL(clicked(bool)), ui.id_11_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_11_checkbox, SIGNAL(clicked(bool)), ui.id_11_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_11_checkbox, SIGNAL(clicked(bool)), ui.id_11_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_11_checkbox, SIGNAL(clicked(bool)), ui.id_11_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_11_checkbox, SIGNAL(clicked(bool)), ui.id_11_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_11_checkbox, SIGNAL(clicked(bool)), ui.id_11_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_11_checkbox, SIGNAL(clicked(bool)), ui.id_11_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_12_checkbox, SIGNAL(clicked(bool)), ui.id_12_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_12_checkbox, SIGNAL(clicked(bool)), ui.id_12_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_12_checkbox, SIGNAL(clicked(bool)), ui.id_12_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_12_checkbox, SIGNAL(clicked(bool)), ui.id_12_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_12_checkbox, SIGNAL(clicked(bool)), ui.id_12_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_12_checkbox, SIGNAL(clicked(bool)), ui.id_12_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_12_checkbox, SIGNAL(clicked(bool)), ui.id_12_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_13_checkbox, SIGNAL(clicked(bool)), ui.id_13_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_13_checkbox, SIGNAL(clicked(bool)), ui.id_13_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_13_checkbox, SIGNAL(clicked(bool)), ui.id_13_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_13_checkbox, SIGNAL(clicked(bool)), ui.id_13_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_13_checkbox, SIGNAL(clicked(bool)), ui.id_13_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_13_checkbox, SIGNAL(clicked(bool)), ui.id_13_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_13_checkbox, SIGNAL(clicked(bool)), ui.id_13_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_14_checkbox, SIGNAL(clicked(bool)), ui.id_14_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_14_checkbox, SIGNAL(clicked(bool)), ui.id_14_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_14_checkbox, SIGNAL(clicked(bool)), ui.id_14_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_14_checkbox, SIGNAL(clicked(bool)), ui.id_14_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_14_checkbox, SIGNAL(clicked(bool)), ui.id_14_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_14_checkbox, SIGNAL(clicked(bool)), ui.id_14_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_14_checkbox, SIGNAL(clicked(bool)), ui.id_14_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_15_checkbox, SIGNAL(clicked(bool)), ui.id_15_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_15_checkbox, SIGNAL(clicked(bool)), ui.id_15_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_15_checkbox, SIGNAL(clicked(bool)), ui.id_15_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_15_checkbox, SIGNAL(clicked(bool)), ui.id_15_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_15_checkbox, SIGNAL(clicked(bool)), ui.id_15_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_15_checkbox, SIGNAL(clicked(bool)), ui.id_15_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_15_checkbox, SIGNAL(clicked(bool)), ui.id_15_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_16_checkbox, SIGNAL(clicked(bool)), ui.id_16_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_16_checkbox, SIGNAL(clicked(bool)), ui.id_16_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_16_checkbox, SIGNAL(clicked(bool)), ui.id_16_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_16_checkbox, SIGNAL(clicked(bool)), ui.id_16_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_16_checkbox, SIGNAL(clicked(bool)), ui.id_16_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_16_checkbox, SIGNAL(clicked(bool)), ui.id_16_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_16_checkbox, SIGNAL(clicked(bool)), ui.id_16_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_17_checkbox, SIGNAL(clicked(bool)), ui.id_17_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_17_checkbox, SIGNAL(clicked(bool)), ui.id_17_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_17_checkbox, SIGNAL(clicked(bool)), ui.id_17_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_17_checkbox, SIGNAL(clicked(bool)), ui.id_17_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_17_checkbox, SIGNAL(clicked(bool)), ui.id_17_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_17_checkbox, SIGNAL(clicked(bool)), ui.id_17_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_17_checkbox, SIGNAL(clicked(bool)), ui.id_17_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_18_checkbox, SIGNAL(clicked(bool)), ui.id_18_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_18_checkbox, SIGNAL(clicked(bool)), ui.id_18_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_18_checkbox, SIGNAL(clicked(bool)), ui.id_18_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_18_checkbox, SIGNAL(clicked(bool)), ui.id_18_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_18_checkbox, SIGNAL(clicked(bool)), ui.id_18_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_18_checkbox, SIGNAL(clicked(bool)), ui.id_18_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_18_checkbox, SIGNAL(clicked(bool)), ui.id_18_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_19_checkbox, SIGNAL(clicked(bool)), ui.id_19_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_19_checkbox, SIGNAL(clicked(bool)), ui.id_19_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_19_checkbox, SIGNAL(clicked(bool)), ui.id_19_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_19_checkbox, SIGNAL(clicked(bool)), ui.id_19_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_19_checkbox, SIGNAL(clicked(bool)), ui.id_19_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_19_checkbox, SIGNAL(clicked(bool)), ui.id_19_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_19_checkbox, SIGNAL(clicked(bool)), ui.id_19_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_20_checkbox, SIGNAL(clicked(bool)), ui.id_20_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_20_checkbox, SIGNAL(clicked(bool)), ui.id_20_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_20_checkbox, SIGNAL(clicked(bool)), ui.id_20_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_20_checkbox, SIGNAL(clicked(bool)), ui.id_20_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_20_checkbox, SIGNAL(clicked(bool)), ui.id_20_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_20_checkbox, SIGNAL(clicked(bool)), ui.id_20_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_20_checkbox, SIGNAL(clicked(bool)), ui.id_20_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_21_checkbox, SIGNAL(clicked(bool)), ui.id_21_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_21_checkbox, SIGNAL(clicked(bool)), ui.id_21_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_21_checkbox, SIGNAL(clicked(bool)), ui.id_21_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_21_checkbox, SIGNAL(clicked(bool)), ui.id_21_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_21_checkbox, SIGNAL(clicked(bool)), ui.id_21_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_21_checkbox, SIGNAL(clicked(bool)), ui.id_21_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_21_checkbox, SIGNAL(clicked(bool)), ui.id_21_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_22_checkbox, SIGNAL(clicked(bool)), ui.id_22_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_22_checkbox, SIGNAL(clicked(bool)), ui.id_22_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_22_checkbox, SIGNAL(clicked(bool)), ui.id_22_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_22_checkbox, SIGNAL(clicked(bool)), ui.id_22_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_22_checkbox, SIGNAL(clicked(bool)), ui.id_22_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_22_checkbox, SIGNAL(clicked(bool)), ui.id_22_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_22_checkbox, SIGNAL(clicked(bool)), ui.id_22_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_23_checkbox, SIGNAL(clicked(bool)), ui.id_23_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_23_checkbox, SIGNAL(clicked(bool)), ui.id_23_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_23_checkbox, SIGNAL(clicked(bool)), ui.id_23_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_23_checkbox, SIGNAL(clicked(bool)), ui.id_23_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_23_checkbox, SIGNAL(clicked(bool)), ui.id_23_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_23_checkbox, SIGNAL(clicked(bool)), ui.id_23_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_23_checkbox, SIGNAL(clicked(bool)), ui.id_23_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_24_checkbox, SIGNAL(clicked(bool)), ui.id_24_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_24_checkbox, SIGNAL(clicked(bool)), ui.id_24_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_24_checkbox, SIGNAL(clicked(bool)), ui.id_24_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_24_checkbox, SIGNAL(clicked(bool)), ui.id_24_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_24_checkbox, SIGNAL(clicked(bool)), ui.id_24_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_24_checkbox, SIGNAL(clicked(bool)), ui.id_24_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_24_checkbox, SIGNAL(clicked(bool)), ui.id_24_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_25_checkbox, SIGNAL(clicked(bool)), ui.id_25_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_25_checkbox, SIGNAL(clicked(bool)), ui.id_25_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_25_checkbox, SIGNAL(clicked(bool)), ui.id_25_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_25_checkbox, SIGNAL(clicked(bool)), ui.id_25_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_25_checkbox, SIGNAL(clicked(bool)), ui.id_25_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_25_checkbox, SIGNAL(clicked(bool)), ui.id_25_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_25_checkbox, SIGNAL(clicked(bool)), ui.id_25_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_26_checkbox, SIGNAL(clicked(bool)), ui.id_26_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_26_checkbox, SIGNAL(clicked(bool)), ui.id_26_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_26_checkbox, SIGNAL(clicked(bool)), ui.id_26_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_26_checkbox, SIGNAL(clicked(bool)), ui.id_26_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_26_checkbox, SIGNAL(clicked(bool)), ui.id_26_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_26_checkbox, SIGNAL(clicked(bool)), ui.id_26_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_26_checkbox, SIGNAL(clicked(bool)), ui.id_26_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_27_checkbox, SIGNAL(clicked(bool)), ui.id_27_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_27_checkbox, SIGNAL(clicked(bool)), ui.id_27_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_27_checkbox, SIGNAL(clicked(bool)), ui.id_27_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_27_checkbox, SIGNAL(clicked(bool)), ui.id_27_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_27_checkbox, SIGNAL(clicked(bool)), ui.id_27_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_27_checkbox, SIGNAL(clicked(bool)), ui.id_27_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_27_checkbox, SIGNAL(clicked(bool)), ui.id_27_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_28_checkbox, SIGNAL(clicked(bool)), ui.id_28_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_28_checkbox, SIGNAL(clicked(bool)), ui.id_28_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_28_checkbox, SIGNAL(clicked(bool)), ui.id_28_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_28_checkbox, SIGNAL(clicked(bool)), ui.id_28_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_28_checkbox, SIGNAL(clicked(bool)), ui.id_28_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_28_checkbox, SIGNAL(clicked(bool)), ui.id_28_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_28_checkbox, SIGNAL(clicked(bool)), ui.id_28_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_29_checkbox, SIGNAL(clicked(bool)), ui.id_29_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_29_checkbox, SIGNAL(clicked(bool)), ui.id_29_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_29_checkbox, SIGNAL(clicked(bool)), ui.id_29_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_29_checkbox, SIGNAL(clicked(bool)), ui.id_29_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_29_checkbox, SIGNAL(clicked(bool)), ui.id_29_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_29_checkbox, SIGNAL(clicked(bool)), ui.id_29_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_29_checkbox, SIGNAL(clicked(bool)), ui.id_29_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_30_checkbox, SIGNAL(clicked(bool)), ui.id_30_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_30_checkbox, SIGNAL(clicked(bool)), ui.id_30_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_30_checkbox, SIGNAL(clicked(bool)), ui.id_30_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_30_checkbox, SIGNAL(clicked(bool)), ui.id_30_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_30_checkbox, SIGNAL(clicked(bool)), ui.id_30_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_30_checkbox, SIGNAL(clicked(bool)), ui.id_30_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_30_checkbox, SIGNAL(clicked(bool)), ui.id_30_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_31_checkbox, SIGNAL(clicked(bool)), ui.id_31_goal_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_31_checkbox, SIGNAL(clicked(bool)), ui.id_31_offset_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_31_checkbox, SIGNAL(clicked(bool)), ui.id_31_modval_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_31_checkbox, SIGNAL(clicked(bool)), ui.id_31_present_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_31_checkbox, SIGNAL(clicked(bool)), ui.id_31_p_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_31_checkbox, SIGNAL(clicked(bool)), ui.id_31_i_spinbox, SLOT(setEnabled(bool)));
    QObject::connect(ui.id_31_checkbox, SIGNAL(clicked(bool)), ui.id_31_d_spinbox, SLOT(setEnabled(bool)));

    QObject::connect(ui.id_1_goal_spinbox, SIGNAL(valueChanged(double))     , this, SLOT(id_1_valueChanged()) );
    QObject::connect(ui.id_1_offset_spinbox, SIGNAL(valueChanged(double))   , this, SLOT(id_1_valueChanged()) );
    QObject::connect(ui.id_1_p_spinbox, SIGNAL(valueChanged(int))           , this, SLOT(id_1_valueChanged()) );
    QObject::connect(ui.id_1_i_spinbox, SIGNAL(valueChanged(int))           , this, SLOT(id_1_valueChanged()) );
    QObject::connect(ui.id_1_d_spinbox, SIGNAL(valueChanged(int))           , this, SLOT(id_1_valueChanged()) );

    QObject::connect(ui.id_2_goal_spinbox, SIGNAL(valueChanged(double))     , this, SLOT(id_2_valueChanged()) );
    QObject::connect(ui.id_2_offset_spinbox, SIGNAL(valueChanged(double))   , this, SLOT(id_2_valueChanged()) );
    QObject::connect(ui.id_2_p_spinbox, SIGNAL(valueChanged(int))           , this, SLOT(id_2_valueChanged()) );
    QObject::connect(ui.id_2_i_spinbox, SIGNAL(valueChanged(int))           , this, SLOT(id_2_valueChanged()) );
    QObject::connect(ui.id_2_d_spinbox, SIGNAL(valueChanged(int))           , this, SLOT(id_2_valueChanged()) );

    QObject::connect(ui.id_3_goal_spinbox, SIGNAL(valueChanged(double))		, this, SLOT(id_3_valueChanged()) );
    QObject::connect(ui.id_3_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_3_valueChanged()) );
    QObject::connect(ui.id_3_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_3_valueChanged()) );
    QObject::connect(ui.id_3_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_3_valueChanged()) );
    QObject::connect(ui.id_3_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_3_valueChanged()) );

    QObject::connect(ui.id_4_goal_spinbox, SIGNAL(valueChanged(double))		, this, SLOT(id_4_valueChanged()) );
    QObject::connect(ui.id_4_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_4_valueChanged()) );
    QObject::connect(ui.id_4_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_4_valueChanged()) );
    QObject::connect(ui.id_4_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_4_valueChanged()) );
    QObject::connect(ui.id_4_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_4_valueChanged()) );

    QObject::connect(ui.id_5_goal_spinbox, SIGNAL(valueChanged(double))		, this, SLOT(id_5_valueChanged()) );
    QObject::connect(ui.id_5_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_5_valueChanged()) );
    QObject::connect(ui.id_5_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_5_valueChanged()) );
    QObject::connect(ui.id_5_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_5_valueChanged()) );
    QObject::connect(ui.id_5_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_5_valueChanged()) );

    QObject::connect(ui.id_6_goal_spinbox, SIGNAL(valueChanged(double))		, this, SLOT(id_6_valueChanged()) );
    QObject::connect(ui.id_6_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_6_valueChanged()) );
    QObject::connect(ui.id_6_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_6_valueChanged()) );
    QObject::connect(ui.id_6_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_6_valueChanged()) );
    QObject::connect(ui.id_6_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_6_valueChanged()) );

    QObject::connect(ui.id_7_goal_spinbox, SIGNAL(valueChanged(double))		, this, SLOT(id_7_valueChanged()) );
    QObject::connect(ui.id_7_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_7_valueChanged()) );
    QObject::connect(ui.id_7_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_7_valueChanged()) );
    QObject::connect(ui.id_7_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_7_valueChanged()) );
    QObject::connect(ui.id_7_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_7_valueChanged()) );

    QObject::connect(ui.id_8_goal_spinbox, SIGNAL(valueChanged(double))		, this, SLOT(id_8_valueChanged()) );
    QObject::connect(ui.id_8_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_8_valueChanged()) );
    QObject::connect(ui.id_8_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_8_valueChanged()) );
    QObject::connect(ui.id_8_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_8_valueChanged()) );
    QObject::connect(ui.id_8_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_8_valueChanged()) );

    QObject::connect(ui.id_9_goal_spinbox, SIGNAL(valueChanged(double))		, this, SLOT(id_9_valueChanged()) );
    QObject::connect(ui.id_9_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_9_valueChanged()) );
    QObject::connect(ui.id_9_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_9_valueChanged()) );
    QObject::connect(ui.id_9_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_9_valueChanged()) );
    QObject::connect(ui.id_9_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_9_valueChanged()) );

    QObject::connect(ui.id_10_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_10_valueChanged()) );
    QObject::connect(ui.id_10_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_10_valueChanged()) );
    QObject::connect(ui.id_10_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_10_valueChanged()) );
    QObject::connect(ui.id_10_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_10_valueChanged()) );
    QObject::connect(ui.id_10_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_10_valueChanged()) );

    QObject::connect(ui.id_11_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_11_valueChanged()) );
    QObject::connect(ui.id_11_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_11_valueChanged()) );
    QObject::connect(ui.id_11_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_11_valueChanged()) );
    QObject::connect(ui.id_11_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_11_valueChanged()) );
    QObject::connect(ui.id_11_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_11_valueChanged()) );

    QObject::connect(ui.id_12_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_12_valueChanged()) );
    QObject::connect(ui.id_12_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_12_valueChanged()) );
    QObject::connect(ui.id_12_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_12_valueChanged()) );
    QObject::connect(ui.id_12_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_12_valueChanged()) );
    QObject::connect(ui.id_12_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_12_valueChanged()) );

    QObject::connect(ui.id_13_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_13_valueChanged()) );
    QObject::connect(ui.id_13_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_13_valueChanged()) );
    QObject::connect(ui.id_13_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_13_valueChanged()) );
    QObject::connect(ui.id_13_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_13_valueChanged()) );
    QObject::connect(ui.id_13_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_13_valueChanged()) );

    QObject::connect(ui.id_14_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_14_valueChanged()) );
    QObject::connect(ui.id_14_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_14_valueChanged()) );
    QObject::connect(ui.id_14_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_14_valueChanged()) );
    QObject::connect(ui.id_14_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_14_valueChanged()) );
    QObject::connect(ui.id_14_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_14_valueChanged()) );

    QObject::connect(ui.id_15_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_15_valueChanged()) );
    QObject::connect(ui.id_15_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_15_valueChanged()) );
    QObject::connect(ui.id_15_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_15_valueChanged()) );
    QObject::connect(ui.id_15_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_15_valueChanged()) );
    QObject::connect(ui.id_15_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_15_valueChanged()) );

    QObject::connect(ui.id_16_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_16_valueChanged()) );
    QObject::connect(ui.id_16_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_16_valueChanged()) );
    QObject::connect(ui.id_16_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_16_valueChanged()) );
    QObject::connect(ui.id_16_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_16_valueChanged()) );
    QObject::connect(ui.id_16_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_16_valueChanged()) );

    QObject::connect(ui.id_17_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_17_valueChanged()) );
    QObject::connect(ui.id_17_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_17_valueChanged()) );
    QObject::connect(ui.id_17_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_17_valueChanged()) );
    QObject::connect(ui.id_17_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_17_valueChanged()) );
    QObject::connect(ui.id_17_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_17_valueChanged()) );

    QObject::connect(ui.id_18_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_18_valueChanged()) );
    QObject::connect(ui.id_18_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_18_valueChanged()) );
    QObject::connect(ui.id_18_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_18_valueChanged()) );
    QObject::connect(ui.id_18_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_18_valueChanged()) );
    QObject::connect(ui.id_18_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_18_valueChanged()) );

    QObject::connect(ui.id_19_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_19_valueChanged()) );
    QObject::connect(ui.id_19_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_19_valueChanged()) );
    QObject::connect(ui.id_19_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_19_valueChanged()) );
    QObject::connect(ui.id_19_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_19_valueChanged()) );
    QObject::connect(ui.id_19_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_19_valueChanged()) );

    QObject::connect(ui.id_20_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_20_valueChanged()) );
    QObject::connect(ui.id_20_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_20_valueChanged()) );
    QObject::connect(ui.id_20_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_20_valueChanged()) );
    QObject::connect(ui.id_20_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_20_valueChanged()) );
    QObject::connect(ui.id_20_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_20_valueChanged()) );

    QObject::connect(ui.id_21_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_21_valueChanged()) );
    QObject::connect(ui.id_21_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_21_valueChanged()) );
    QObject::connect(ui.id_21_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_21_valueChanged()) );
    QObject::connect(ui.id_21_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_21_valueChanged()) );
    QObject::connect(ui.id_21_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_21_valueChanged()) );

    QObject::connect(ui.id_22_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_22_valueChanged()) );
    QObject::connect(ui.id_22_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_22_valueChanged()) );
    QObject::connect(ui.id_22_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_22_valueChanged()) );
    QObject::connect(ui.id_22_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_22_valueChanged()) );
    QObject::connect(ui.id_22_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_22_valueChanged()) );

    QObject::connect(ui.id_23_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_23_valueChanged()) );
    QObject::connect(ui.id_23_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_23_valueChanged()) );
    QObject::connect(ui.id_23_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_23_valueChanged()) );
    QObject::connect(ui.id_23_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_23_valueChanged()) );
    QObject::connect(ui.id_23_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_23_valueChanged()) );

    QObject::connect(ui.id_24_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_24_valueChanged()) );
    QObject::connect(ui.id_24_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_24_valueChanged()) );
    QObject::connect(ui.id_24_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_24_valueChanged()) );
    QObject::connect(ui.id_24_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_24_valueChanged()) );
    QObject::connect(ui.id_24_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_24_valueChanged()) );

    QObject::connect(ui.id_25_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_25_valueChanged()) );
    QObject::connect(ui.id_25_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_25_valueChanged()) );
    QObject::connect(ui.id_25_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_25_valueChanged()) );
    QObject::connect(ui.id_25_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_25_valueChanged()) );
    QObject::connect(ui.id_25_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_25_valueChanged()) );

    QObject::connect(ui.id_26_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_26_valueChanged()) );
    QObject::connect(ui.id_26_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_26_valueChanged()) );
    QObject::connect(ui.id_26_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_26_valueChanged()) );
    QObject::connect(ui.id_26_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_26_valueChanged()) );
    QObject::connect(ui.id_26_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_26_valueChanged()) );

    QObject::connect(ui.id_27_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_27_valueChanged()) );
    QObject::connect(ui.id_27_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_27_valueChanged()) );
    QObject::connect(ui.id_27_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_27_valueChanged()) );
    QObject::connect(ui.id_27_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_27_valueChanged()) );
    QObject::connect(ui.id_27_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_27_valueChanged()) );

    QObject::connect(ui.id_28_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_28_valueChanged()) );
    QObject::connect(ui.id_28_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_28_valueChanged()) );
    QObject::connect(ui.id_28_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_28_valueChanged()) );
    QObject::connect(ui.id_28_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_28_valueChanged()) );
    QObject::connect(ui.id_28_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_28_valueChanged()) );

    QObject::connect(ui.id_29_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_29_valueChanged()) );
    QObject::connect(ui.id_29_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_29_valueChanged()) );
    QObject::connect(ui.id_29_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_29_valueChanged()) );
    QObject::connect(ui.id_29_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_29_valueChanged()) );
    QObject::connect(ui.id_29_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_29_valueChanged()) );

    QObject::connect(ui.id_30_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_30_valueChanged()) );
    QObject::connect(ui.id_30_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_30_valueChanged()) );
    QObject::connect(ui.id_30_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_30_valueChanged()) );
    QObject::connect(ui.id_30_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_30_valueChanged()) );
    QObject::connect(ui.id_30_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_30_valueChanged()) );

    QObject::connect(ui.id_31_goal_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_31_valueChanged()) );
    QObject::connect(ui.id_31_offset_spinbox, SIGNAL(valueChanged(double))	, this, SLOT(id_31_valueChanged()) );
    QObject::connect(ui.id_31_p_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_31_valueChanged()) );
    QObject::connect(ui.id_31_i_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_31_valueChanged()) );
    QObject::connect(ui.id_31_d_spinbox, SIGNAL(valueChanged(int))			, this, SLOT(id_31_valueChanged()) );


    /****************************
    ** Connect
    ****************************/

    qRegisterMetaType< thormang3_offset_tuner_msgs::JointOffsetPositionData >("thormang3_offset_tuner_msgs::JointOffsetPositionData");
    QObject::connect(&qnode , SIGNAL(update_present_joint_offset_data(thormang3_offset_tuner_msgs::JointOffsetPositionData)), this, SLOT(update_joint_offset_data_spinbox(thormang3_offset_tuner_msgs::JointOffsetPositionData)));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
                     SLOT(updateLoggingView()));

    /****************************
    ** Connect
    ****************************/

    /*********************
    ** Auto Start
    **********************/
    qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::on_save_button_clicked( bool check )
{
    std_msgs::String _msg;
    _msg.data = "save";

    qnode.send_command_msg( _msg );
}

void MainWindow::on_inipose_button_clicked( bool checck )
{
    std_msgs::String _msg;
    _msg.data = "ini_pose";

    qnode.send_command_msg( _msg );
}

void MainWindow::on_refresh_button_clicked( bool check )
{
    qnode.getPresentJointOffsetData();
}

void MainWindow::on_right_arm_all_torque_on_button_clicked(bool check)
{
    all_torque_on = true;
    QList<QAbstractButton *> _torque_buttons = ui.right_arm_torque_buttongroup->buttons();
    for(int ix = 0; ix < _torque_buttons.size(); ix++)
    {
        if( _torque_buttons[ix]->isChecked() == false )
        _torque_buttons[ix]->click();
    }

    qnode.getPresentJointOffsetData();
    all_torque_on = false;
}

void MainWindow::on_right_arm_all_torque_off_button_clicked(bool check)
{
    QList<QAbstractButton *> _torque_buttons = ui.right_arm_torque_buttongroup->buttons();
    for(int ix = 0; ix < _torque_buttons.size(); ix++)
    {
        if( _torque_buttons[ix]->isChecked() == true )
        _torque_buttons[ix]->click();
    }
}

void MainWindow::on_left_arm_all_torque_on_button_clicked(bool check)
{
    all_torque_on = true;
    QList<QAbstractButton *> _torque_buttons = ui.left_arm_torque_buttongroup->buttons();
    for(int ix = 0; ix < _torque_buttons.size(); ix++)
    {
        if( _torque_buttons[ix]->isChecked() == false )
        _torque_buttons[ix]->click();
    }

    qnode.getPresentJointOffsetData();
    all_torque_on = false;
}

void MainWindow::on_left_arm_all_torque_off_button_clicked(bool check)
{
    QList<QAbstractButton *> _torque_buttons = ui.left_arm_torque_buttongroup->buttons();
    for(int ix = 0; ix < _torque_buttons.size(); ix++)
    {
        if( _torque_buttons[ix]->isChecked() == true )
        _torque_buttons[ix]->click();
    }
}

void MainWindow::on_leg_all_torque_on_button_clicked(bool check)
{
    all_torque_on = true;
    QList<QAbstractButton *> _torque_buttons = ui.leg_torque_buttongroup->buttons();
    for(int ix = 0; ix < _torque_buttons.size(); ix++)
    {
        if( _torque_buttons[ix]->isChecked() == false )
        _torque_buttons[ix]->click();
    }

    qnode.getPresentJointOffsetData();
    all_torque_on = false;
}

void MainWindow::on_leg_all_torque_off_button_clicked(bool check)
{
    QList<QAbstractButton *> _torque_buttons = ui.leg_torque_buttongroup->buttons();
    for(int ix = 0; ix < _torque_buttons.size(); ix++)
    {
        if( _torque_buttons[ix]->isChecked() == true )
        _torque_buttons[ix]->click();
    }
}

void MainWindow::on_body_all_torque_on_button_clicked(bool check)
{
    all_torque_on = true;
    QList<QAbstractButton *> _torque_buttons = ui.body_torque_buttongroup->buttons();
    for(int ix = 0; ix < _torque_buttons.size(); ix++)
    {
        if( _torque_buttons[ix]->isChecked() == false )
        _torque_buttons[ix]->click();
    }

    qnode.getPresentJointOffsetData();
    all_torque_on = false;
}

void MainWindow::on_body_all_torque_off_button_clicked(bool check)
{
    QList<QAbstractButton *> _torque_buttons = ui.body_torque_buttongroup->buttons();
    for(int ix = 0; ix < _torque_buttons.size(); ix++)
    {
        if( _torque_buttons[ix]->isChecked() == true )
        _torque_buttons[ix]->click();
    }
}

void MainWindow::on_id_1_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_1_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_1_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_2_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_2_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_2_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_3_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_3_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_3_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_4_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_4_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_4_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_5_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_5_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_5_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_6_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_6_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_6_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_7_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_7_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_7_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_8_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_8_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_8_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_9_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_9_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_9_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_10_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_10_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_10_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_11_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_11_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_11_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_12_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_12_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_12_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_13_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_13_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_13_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_14_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_14_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_14_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_15_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_15_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_15_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_16_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_16_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_16_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_17_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_17_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_17_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_18_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_18_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_18_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_19_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_19_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_19_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_20_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_20_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_20_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_21_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_21_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_21_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_22_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_22_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_22_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_23_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_23_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_23_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_24_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_24_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_24_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_25_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_25_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_25_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_26_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_26_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_26_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_27_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_27_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_27_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_28_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_28_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_28_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_29_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_29_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_29_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_30_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_30_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_30_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::on_id_31_checkbox_clicked( bool check )
{
    thormang3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    thormang3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = ui.id_31_checkbox->text().toStdString();
    _msg.torque_enable = ui.id_31_checkbox->isChecked();

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::id_1_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_arm_sh_p1";
    _msg.goal_value = ui.id_1_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_1_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_1_p_spinbox->value();
    _msg.i_gain = ui.id_1_i_spinbox->value();
    _msg.d_gain = ui.id_1_d_spinbox->value();

    ui.id_1_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_2_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_arm_sh_p1";
    _msg.goal_value = ui.id_2_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_2_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_2_p_spinbox->value();
    _msg.i_gain = ui.id_2_i_spinbox->value();
    _msg.d_gain = ui.id_2_d_spinbox->value();

    ui.id_2_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_3_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_arm_sh_r";
    _msg.goal_value = ui.id_3_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_3_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_3_p_spinbox->value();
    _msg.i_gain = ui.id_3_i_spinbox->value();
    _msg.d_gain = ui.id_3_d_spinbox->value();

    ui.id_3_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_4_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_arm_sh_r";
    _msg.goal_value = ui.id_4_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_4_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_4_p_spinbox->value();
    _msg.i_gain = ui.id_4_i_spinbox->value();
    _msg.d_gain = ui.id_4_d_spinbox->value();

    ui.id_4_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_5_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_arm_sh_p2";
    _msg.goal_value = ui.id_5_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_5_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_5_p_spinbox->value();
    _msg.i_gain = ui.id_5_i_spinbox->value();
    _msg.d_gain = ui.id_5_d_spinbox->value();

    ui.id_5_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_6_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_arm_sh_p2";
    _msg.goal_value = ui.id_6_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_6_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_6_p_spinbox->value();
    _msg.i_gain = ui.id_6_i_spinbox->value();
    _msg.d_gain = ui.id_6_d_spinbox->value();

    ui.id_6_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_7_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_arm_el_y";
    _msg.goal_value = ui.id_7_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_7_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_7_p_spinbox->value();
    _msg.i_gain = ui.id_7_i_spinbox->value();
    _msg.d_gain = ui.id_7_d_spinbox->value();

    ui.id_7_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_8_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_arm_el_y";
    _msg.goal_value = ui.id_8_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_8_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_8_p_spinbox->value();
    _msg.i_gain = ui.id_8_i_spinbox->value();
    _msg.d_gain = ui.id_8_d_spinbox->value();

    ui.id_8_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_9_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_arm_wr_r";
    _msg.goal_value = ui.id_9_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_9_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_9_p_spinbox->value();
    _msg.i_gain = ui.id_9_i_spinbox->value();
    _msg.d_gain = ui.id_9_d_spinbox->value();

    ui.id_9_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_10_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_arm_wr_r";
    _msg.goal_value = ui.id_10_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_10_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_10_p_spinbox->value();
    _msg.i_gain = ui.id_10_i_spinbox->value();
    _msg.d_gain = ui.id_10_d_spinbox->value();

    ui.id_10_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_11_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_arm_wr_y";
    _msg.goal_value = ui.id_11_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_11_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_11_p_spinbox->value();
    _msg.i_gain = ui.id_11_i_spinbox->value();
    _msg.d_gain = ui.id_11_d_spinbox->value();

    ui.id_11_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_12_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_arm_wr_y";
    _msg.goal_value = ui.id_12_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_12_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_12_p_spinbox->value();
    _msg.i_gain = ui.id_12_i_spinbox->value();
    _msg.d_gain = ui.id_12_d_spinbox->value();

    ui.id_12_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_13_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_arm_wr_p";
    _msg.goal_value = ui.id_13_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_13_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_13_p_spinbox->value();
    _msg.i_gain = ui.id_13_i_spinbox->value();
    _msg.d_gain = ui.id_13_d_spinbox->value();

    ui.id_13_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_14_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_arm_wr_p";
    _msg.goal_value = ui.id_14_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_14_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_14_p_spinbox->value();
    _msg.i_gain = ui.id_14_i_spinbox->value();
    _msg.d_gain = ui.id_14_d_spinbox->value();

    ui.id_14_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_15_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_leg_hip_y";
    _msg.goal_value = ui.id_15_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_15_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_15_p_spinbox->value();
    _msg.i_gain = ui.id_15_i_spinbox->value();
    _msg.d_gain = ui.id_15_d_spinbox->value();

    ui.id_15_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_16_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_leg_hip_y";
    _msg.goal_value = ui.id_16_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_16_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_16_p_spinbox->value();
    _msg.i_gain = ui.id_16_i_spinbox->value();
    _msg.d_gain = ui.id_16_d_spinbox->value();

    ui.id_16_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_17_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_leg_hip_r";
    _msg.goal_value = ui.id_17_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_17_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_17_p_spinbox->value();
    _msg.i_gain = ui.id_17_i_spinbox->value();
    _msg.d_gain = ui.id_17_d_spinbox->value();

    ui.id_17_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_18_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_leg_hip_r";
    _msg.goal_value = ui.id_18_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_18_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_18_p_spinbox->value();
    _msg.i_gain = ui.id_18_i_spinbox->value();
    _msg.d_gain = ui.id_18_d_spinbox->value();

    ui.id_18_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_19_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_leg_hip_p";
    _msg.goal_value = ui.id_19_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_19_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_19_p_spinbox->value();
    _msg.i_gain = ui.id_19_i_spinbox->value();
    _msg.d_gain = ui.id_19_d_spinbox->value();

    ui.id_19_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_20_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_leg_hip_p";
    _msg.goal_value = ui.id_20_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_20_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_20_p_spinbox->value();
    _msg.i_gain = ui.id_20_i_spinbox->value();
    _msg.d_gain = ui.id_20_d_spinbox->value();

    ui.id_20_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_21_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_leg_kn_p";
    _msg.goal_value = ui.id_21_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_21_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_21_p_spinbox->value();
    _msg.i_gain = ui.id_21_i_spinbox->value();
    _msg.d_gain = ui.id_21_d_spinbox->value();

    ui.id_21_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_22_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_leg_kn_p";
    _msg.goal_value = ui.id_22_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_22_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_22_p_spinbox->value();
    _msg.i_gain = ui.id_22_i_spinbox->value();
    _msg.d_gain = ui.id_22_d_spinbox->value();

    ui.id_22_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_23_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_leg_an_p";
    _msg.goal_value = ui.id_23_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_23_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_23_p_spinbox->value();
    _msg.i_gain = ui.id_23_i_spinbox->value();
    _msg.d_gain = ui.id_23_d_spinbox->value();

    ui.id_23_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_24_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_leg_an_p";
    _msg.goal_value = ui.id_24_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_24_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_24_p_spinbox->value();
    _msg.i_gain = ui.id_24_i_spinbox->value();
    _msg.d_gain = ui.id_24_d_spinbox->value();

    ui.id_24_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_25_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_leg_an_r";
    _msg.goal_value = ui.id_25_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_25_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_25_p_spinbox->value();
    _msg.i_gain = ui.id_25_i_spinbox->value();
    _msg.d_gain = ui.id_25_d_spinbox->value();

    ui.id_25_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_26_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_leg_an_r";
    _msg.goal_value = ui.id_26_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_26_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_26_p_spinbox->value();
    _msg.i_gain = ui.id_26_i_spinbox->value();
    _msg.d_gain = ui.id_26_d_spinbox->value();

    ui.id_26_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_27_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "torso_y";
    _msg.goal_value = ui.id_27_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_27_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_27_p_spinbox->value();
    _msg.i_gain = ui.id_27_i_spinbox->value();
    _msg.d_gain = ui.id_27_d_spinbox->value();

    ui.id_27_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_28_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "head_y";
    _msg.goal_value = ui.id_28_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_28_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_28_p_spinbox->value();
    _msg.i_gain = ui.id_28_i_spinbox->value();
    _msg.d_gain = ui.id_28_d_spinbox->value();

    ui.id_28_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_29_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "head_p";
    _msg.goal_value = ui.id_29_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_29_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_29_p_spinbox->value();
    _msg.i_gain = ui.id_29_i_spinbox->value();
    _msg.d_gain = ui.id_29_d_spinbox->value();

    ui.id_29_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_30_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "l_arm_grip";
    _msg.goal_value = ui.id_30_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_30_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_30_p_spinbox->value();
    _msg.i_gain = ui.id_30_i_spinbox->value();
    _msg.d_gain = ui.id_30_d_spinbox->value();

    ui.id_30_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::id_31_valueChanged( )
{
    thormang3_offset_tuner_msgs::JointOffsetData _msg;

    _msg.joint_name = "r_arm_grip";
    _msg.goal_value = ui.id_31_goal_spinbox->value() * M_PI / 180.0;
    _msg.offset_value = ui.id_31_offset_spinbox->value() * M_PI / 180.0;
    _msg.p_gain = ui.id_31_p_spinbox->value();
    _msg.i_gain = ui.id_31_i_spinbox->value();
    _msg.d_gain = ui.id_31_d_spinbox->value();

    ui.id_31_modval_spinbox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::update_joint_offset_data_spinbox( thormang3_offset_tuner_msgs::JointOffsetPositionData msg )
{
    std::vector<double> _double_value;
    _double_value.push_back( msg.goal_value * 180.0 / M_PI );
    _double_value.push_back( msg.offset_value * 180.0 / M_PI );
    _double_value.push_back( msg.present_value * 180.0 / M_PI );

    std::vector<int> _int_value;
    _int_value.push_back( msg.p_gain );
    _int_value.push_back( msg.i_gain );
    _int_value.push_back( msg.d_gain );

    if ( msg.joint_name == "r_arm_sh_p1")
    {
        for ( int ix = 0; ix < id_1_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_1_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_1_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_1_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "l_arm_sh_p1")
    {
        for ( int ix = 0; ix < id_2_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_2_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_2_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_2_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "r_arm_sh_r")
    {
        for ( int ix = 0; ix < id_3_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_3_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_3_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_3_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }    
    else if ( msg.joint_name == "l_arm_sh_r")
    {
        for ( int ix = 0; ix < id_4_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_4_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_4_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_4_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "r_arm_sh_p2")
    {
        for ( int ix = 0; ix < id_5_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_5_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_5_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_5_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "l_arm_sh_p2")
    {
        for ( int ix = 0; ix < id_6_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_6_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_6_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_6_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "r_arm_el_y")
    {
        for ( int ix = 0; ix < id_7_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_7_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_7_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_7_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "l_arm_el_y")
    {
        for ( int ix = 0; ix < id_8_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_8_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_8_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_8_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "r_arm_wr_r")
    {
        for ( int ix = 0; ix < id_9_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_9_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_9_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_9_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "l_arm_wr_r")
    {
        for ( int ix = 0; ix < id_10_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_10_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_10_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_10_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "r_arm_wr_y")
    {
        for ( int ix = 0; ix < id_11_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_11_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_11_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_11_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "l_arm_wr_y")
    {
        for ( int ix = 0; ix < id_12_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_12_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_12_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_12_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "r_arm_wr_p")
    {
        for ( int ix = 0; ix < id_13_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_13_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_13_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_13_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "l_arm_wr_p")
    {
        for ( int ix = 0; ix < id_14_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_14_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_14_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_14_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "r_leg_hip_y")
    {
        for ( int ix = 0; ix < id_15_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_15_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_15_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_15_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "l_leg_hip_y")
    {
        for ( int ix = 0; ix < id_16_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_16_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_16_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_16_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "r_leg_hip_r")
    {
        for ( int ix = 0; ix < id_17_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_17_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_17_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_17_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "l_leg_hip_r")
    {
        for ( int ix = 0; ix < id_18_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_18_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_18_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_18_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "r_leg_hip_p")
    {
        for ( int ix = 0; ix < id_19_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_19_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_19_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_19_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "l_leg_hip_p")
    {
        for ( int ix = 0; ix < id_20_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_20_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_20_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_20_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "r_leg_kn_p")
    {
        for ( int ix = 0; ix < id_21_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_21_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_21_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_21_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "l_leg_kn_p")
    {
        for ( int ix = 0; ix < id_22_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_22_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_22_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_22_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "r_leg_an_p")
    {
        for ( int ix = 0; ix < id_23_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_23_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_23_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_23_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "l_leg_an_p")
    {
        for ( int ix = 0; ix < id_24_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_24_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_24_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_24_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "r_leg_an_r")
    {
        for ( int ix = 0; ix < id_25_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_25_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_25_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_25_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "l_leg_an_r")
    {
        for ( int ix = 0; ix < id_26_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_26_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_26_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_26_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "torso_y")
    {
        for ( int ix = 0; ix < id_27_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_27_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_27_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_27_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "head_y")
    {
        for ( int ix = 0; ix < id_28_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_28_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_28_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_28_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "head_p")
    {
        for ( int ix = 0; ix < id_29_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_29_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_29_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_29_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "l_arm_grip")
    {
        for ( int ix = 0; ix < id_30_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_30_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_30_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_30_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }
    else if ( msg.joint_name == "r_arm_grip")
    {
        for ( int ix = 0; ix < id_31_double_spinbox.size(); ix++ )
            ((QDoubleSpinBox *)id_31_double_spinbox[ ix ])->setValue( _double_value[ ix ] );

        for ( int ix = 0; ix < id_31_int_spinbox.size(); ix++ )
            ((QSpinBox *)id_31_int_spinbox[ ix ])->setValue( _int_value[ ix ] );
    }

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


void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace thormang3_offset_tuner_client

