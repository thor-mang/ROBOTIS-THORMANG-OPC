/**
 * @file /include/thormang3_offset_tuner_client/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef thormang3_offset_tuner_client_QNODE_HPP_
#define thormang3_offset_tuner_client_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <std_msgs/String.h>

#include "thormang3_offset_tuner_msgs/JointOffsetData.h"
#include "thormang3_offset_tuner_msgs/JointOffsetPositionData.h"
#include "thormang3_offset_tuner_msgs/JointTorqueOnOff.h"
#include "thormang3_offset_tuner_msgs/JointTorqueOnOffArray.h"

#include "thormang3_offset_tuner_msgs/GetPresentJointOffsetData.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace thormang3_offset_tuner_client {

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

    void send_torque_enable_msg( thormang3_offset_tuner_msgs::JointTorqueOnOffArray msg );
    void send_joint_offset_data_msg( thormang3_offset_tuner_msgs::JointOffsetData msg );
    void send_command_msg( std_msgs::String msg );

public Q_SLOTS:
    void getPresentJointOffsetData();

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

    void update_present_joint_offset_data( thormang3_offset_tuner_msgs::JointOffsetPositionData msg );

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    ros::Publisher joint_offset_data_pub;
    ros::Publisher torque_enable_pub;
    ros::Publisher command_pub;

    ros::ServiceClient get_present_joint_offset_data_client;

};

}  // namespace thormang3_offset_tuner_client

#endif /* thormang3_offset_tuner_client_QNODE_HPP_ */
