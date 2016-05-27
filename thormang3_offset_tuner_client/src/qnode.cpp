/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/thormang3_offset_tuner_client/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace thormang3_offset_tuner_client {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"thormang3_offset_tuner_client");

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

    // Add your ros communications here
    joint_offset_data_pub = n.advertise<thormang3_offset_tuner_msgs::JointOffsetData>("robotis/offset_tuner/joint_offset_data", 0);
    torque_enable_pub = n.advertise<thormang3_offset_tuner_msgs::JointTorqueOnOffArray>("robotis/offset_tuner/torque_enable", 0);
    command_pub = n.advertise<std_msgs::String>("robotis/offset_tuner/command", 0);

    get_present_joint_offset_data_client = n.serviceClient<thormang3_offset_tuner_msgs::GetPresentJointOffsetData>("robotis/offset_tuner/get_present_joint_offset_data");

	start();
	return true;
}

void QNode::run() {

    ros::Rate loop_rate(125);

	while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();

	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::send_torque_enable_msg( thormang3_offset_tuner_msgs::JointTorqueOnOffArray msg )
{
    torque_enable_pub.publish( msg );

    log( Info , "Joint Torque On/Off" );
}

void QNode::send_joint_offset_data_msg( thormang3_offset_tuner_msgs::JointOffsetData msg )
{
    joint_offset_data_pub.publish( msg );

    log( Info , "Send Joint Offset Data" );
}

void QNode::send_command_msg( std_msgs::String msg )
{
    command_pub.publish( msg );

    std::stringstream log_msg;
    log_msg << "Send Command : " << msg.data ;

    log( Info , log_msg.str() );
}

void QNode::getPresentJointOffsetData()
{
    thormang3_offset_tuner_msgs::GetPresentJointOffsetData _get_present_joint_offset_data;

    //request


    //response
    if ( get_present_joint_offset_data_client.call( _get_present_joint_offset_data ) )
    {
        for ( int id = 0; id < _get_present_joint_offset_data.response.present_data_array.size(); id++ )
        {
            thormang3_offset_tuner_msgs::JointOffsetPositionData _temp = _get_present_joint_offset_data.response.present_data_array[ id ];

            Q_EMIT update_present_joint_offset_data( _temp );
        }
    }
    else
        log(Error, "Fail to get joint offset data");

}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace thormang3_offset_tuner_client
