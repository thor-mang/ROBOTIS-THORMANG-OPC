/**
 * @file /include/thormang3_offset_tuner_client/main_window.hpp
 *
 * @brief Qt based gui for thormang3_offset_tuner_client.
 *
 * @date November 2010
 **/
#ifndef thormang3_offset_tuner_client_MAIN_WINDOW_H
#define thormang3_offset_tuner_client_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QList>
#include <QSpinBox>
#include <QtGui/QMainWindow>

#include <math.h>

#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace thormang3_offset_tuner_client {

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

	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();

    void on_save_button_clicked( bool check );
    void on_refresh_button_clicked( bool check );
    void on_inipose_button_clicked( bool checck );

    void id_1_valueChanged( );
    void id_2_valueChanged( );
    void id_3_valueChanged( );
    void id_4_valueChanged( );
    void id_5_valueChanged( );
    void id_6_valueChanged( );
    void id_7_valueChanged( );
    void id_8_valueChanged( );
    void id_9_valueChanged( );
    void id_10_valueChanged( );
    void id_11_valueChanged( );
    void id_12_valueChanged( );
    void id_13_valueChanged( );
    void id_14_valueChanged( );
    void id_15_valueChanged( );
    void id_16_valueChanged( );
    void id_17_valueChanged( );
    void id_18_valueChanged( );
    void id_19_valueChanged( );
    void id_20_valueChanged( );
    void id_21_valueChanged( );
    void id_22_valueChanged( );
    void id_23_valueChanged( );
    void id_24_valueChanged( );
    void id_25_valueChanged( );
    void id_26_valueChanged( );
    void id_27_valueChanged( );
    void id_28_valueChanged( );
    void id_29_valueChanged( );
    void id_30_valueChanged( );
    void id_31_valueChanged( );

    void on_id_1_checkbox_clicked( bool check );
    void on_id_2_checkbox_clicked( bool check );
    void on_id_3_checkbox_clicked( bool check );
    void on_id_4_checkbox_clicked( bool check );
    void on_id_5_checkbox_clicked( bool check );
    void on_id_6_checkbox_clicked( bool check );
    void on_id_7_checkbox_clicked( bool check );
    void on_id_8_checkbox_clicked( bool check );
    void on_id_9_checkbox_clicked( bool check );
    void on_id_10_checkbox_clicked( bool check );
    void on_id_11_checkbox_clicked( bool check );
    void on_id_12_checkbox_clicked( bool check );
    void on_id_13_checkbox_clicked( bool check );
    void on_id_14_checkbox_clicked( bool check );
    void on_id_15_checkbox_clicked( bool check );
    void on_id_16_checkbox_clicked( bool check );
    void on_id_17_checkbox_clicked( bool check );
    void on_id_18_checkbox_clicked( bool check );
    void on_id_19_checkbox_clicked( bool check );
    void on_id_20_checkbox_clicked( bool check );
    void on_id_21_checkbox_clicked( bool check );
    void on_id_22_checkbox_clicked( bool check );
    void on_id_23_checkbox_clicked( bool check );
    void on_id_24_checkbox_clicked( bool check );
    void on_id_25_checkbox_clicked( bool check );
    void on_id_26_checkbox_clicked( bool check );
    void on_id_27_checkbox_clicked( bool check );
    void on_id_28_checkbox_clicked( bool check );
    void on_id_29_checkbox_clicked( bool check );
    void on_id_30_checkbox_clicked( bool check );
    void on_id_31_checkbox_clicked( bool check );

    void on_right_arm_all_torque_on_button_clicked( bool check );
    void on_right_arm_all_torque_off_button_clicked( bool check );

    void on_left_arm_all_torque_on_button_clicked( bool check );
    void on_left_arm_all_torque_off_button_clicked( bool check );

    void on_leg_all_torque_on_button_clicked( bool check );
    void on_leg_all_torque_off_button_clicked( bool check );

    void on_body_all_torque_on_button_clicked( bool check );
    void on_body_all_torque_off_button_clicked( bool check );

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    void update_joint_offset_data_spinbox( thormang3_offset_tuner_msgs::JointOffsetPositionData msg );

private:
	Ui::MainWindowDesign ui;
	QNode qnode;

    bool all_torque_on;

    QList<QAbstractSpinBox *> id_1_double_spinbox;
    QList<QAbstractSpinBox *> id_1_int_spinbox;

    QList<QAbstractSpinBox *> id_2_double_spinbox;
    QList<QAbstractSpinBox *> id_2_int_spinbox;

    QList<QAbstractSpinBox *> id_3_double_spinbox;
    QList<QAbstractSpinBox *> id_3_int_spinbox;

    QList<QAbstractSpinBox *> id_4_double_spinbox;
    QList<QAbstractSpinBox *> id_4_int_spinbox;

    QList<QAbstractSpinBox *> id_5_double_spinbox;
    QList<QAbstractSpinBox *> id_5_int_spinbox;

    QList<QAbstractSpinBox *> id_6_double_spinbox;
    QList<QAbstractSpinBox *> id_6_int_spinbox;

    QList<QAbstractSpinBox *> id_7_double_spinbox;
    QList<QAbstractSpinBox *> id_7_int_spinbox;

    QList<QAbstractSpinBox *> id_8_double_spinbox;
    QList<QAbstractSpinBox *> id_8_int_spinbox;

    QList<QAbstractSpinBox *> id_9_double_spinbox;
    QList<QAbstractSpinBox *> id_9_int_spinbox;

    QList<QAbstractSpinBox *> id_10_double_spinbox;
    QList<QAbstractSpinBox *> id_10_int_spinbox;

    QList<QAbstractSpinBox *> id_11_double_spinbox;
    QList<QAbstractSpinBox *> id_11_int_spinbox;

    QList<QAbstractSpinBox *> id_12_double_spinbox;
    QList<QAbstractSpinBox *> id_12_int_spinbox;

    QList<QAbstractSpinBox *> id_13_double_spinbox;
    QList<QAbstractSpinBox *> id_13_int_spinbox;

    QList<QAbstractSpinBox *> id_14_double_spinbox;
    QList<QAbstractSpinBox *> id_14_int_spinbox;

    QList<QAbstractSpinBox *> id_15_double_spinbox;
    QList<QAbstractSpinBox *> id_15_int_spinbox;

    QList<QAbstractSpinBox *> id_16_double_spinbox;
    QList<QAbstractSpinBox *> id_16_int_spinbox;

    QList<QAbstractSpinBox *> id_17_double_spinbox;
    QList<QAbstractSpinBox *> id_17_int_spinbox;

    QList<QAbstractSpinBox *> id_18_double_spinbox;
    QList<QAbstractSpinBox *> id_18_int_spinbox;

    QList<QAbstractSpinBox *> id_19_double_spinbox;
    QList<QAbstractSpinBox *> id_19_int_spinbox;

    QList<QAbstractSpinBox *> id_20_double_spinbox;
    QList<QAbstractSpinBox *> id_20_int_spinbox;

    QList<QAbstractSpinBox *> id_21_double_spinbox;
    QList<QAbstractSpinBox *> id_21_int_spinbox;

    QList<QAbstractSpinBox *> id_22_double_spinbox;
    QList<QAbstractSpinBox *> id_22_int_spinbox;

    QList<QAbstractSpinBox *> id_23_double_spinbox;
    QList<QAbstractSpinBox *> id_23_int_spinbox;

    QList<QAbstractSpinBox *> id_24_double_spinbox;
    QList<QAbstractSpinBox *> id_24_int_spinbox;

    QList<QAbstractSpinBox *> id_25_double_spinbox;
    QList<QAbstractSpinBox *> id_25_int_spinbox;

    QList<QAbstractSpinBox *> id_26_double_spinbox;
    QList<QAbstractSpinBox *> id_26_int_spinbox;

    QList<QAbstractSpinBox *> id_27_double_spinbox;
    QList<QAbstractSpinBox *> id_27_int_spinbox;

    QList<QAbstractSpinBox *> id_28_double_spinbox;
    QList<QAbstractSpinBox *> id_28_int_spinbox;

    QList<QAbstractSpinBox *> id_29_double_spinbox;
    QList<QAbstractSpinBox *> id_29_int_spinbox;

    QList<QAbstractSpinBox *> id_30_double_spinbox;
    QList<QAbstractSpinBox *> id_30_int_spinbox;

    QList<QAbstractSpinBox *> id_31_double_spinbox;
    QList<QAbstractSpinBox *> id_31_int_spinbox;

};

}  // namespace thormang3_offset_tuner_client

#endif // thormang3_offset_tuner_client_MAIN_WINDOW_H
