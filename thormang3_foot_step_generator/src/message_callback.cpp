/*
 * message_callback.cpp
 *
 *  Created on: 2016. 2. 20.
 *      Author: HJSONG
 */

#include "thormang3_foot_step_generator/message_callback.h"


//ros::ServiceClient	remove_exisitng_step_data_client;
ros::ServiceClient	get_ref_step_data_client;
ros::ServiceClient	add_step_data_array_client;
//ros::ServiceClient	walking_start_client;
ros::ServiceClient	is_running_client;

ros::ServiceClient	set_balance_param_client;

ros::Subscriber		walking_module_status_msg_sub;

ros::Subscriber     walking_command_sub;
ros::Subscriber		balance_command_sub;
ros::Subscriber		footsteps_2d_sub;


ROBOTIS::FootStepGenerator thormang3_foot_stp_generator;

thormang3_walking_module_msgs::AddStepDataArray     add_step_data_array_srv;

void Initialize(void)
{
	ros::NodeHandle _nh;

    get_ref_step_data_client		= _nh.serviceClient<thormang3_walking_module_msgs::GetReferenceStepData>("/robotis/walking/get_reference_step_data");
    add_step_data_array_client		= _nh.serviceClient<thormang3_walking_module_msgs::AddStepDataArray>("/robotis/walking/add_step_data");
    set_balance_param_client		= _nh.serviceClient<thormang3_walking_module_msgs::SetBalanceParam>("/robotis/walking/set_balance_param");
	is_running_client				= _nh.serviceClient<thormang3_walking_module_msgs::IsRunning>("/robotis/walking/is_running");

	walking_module_status_msg_sub	= _nh.subscribe("/robotis/status", 10, WalkingModuleStatusMSGCallback);

	walking_command_sub			= _nh.subscribe("/robotis/thormang3_foot_step_generator/walking_command", 0, WalkingCommandCallback);
	//balance_command_sub			= _nh.subscribe("/robotis/thormang3_foot_step_generator/balance_command", 0, BalanceCommandCallback);
	footsteps_2d_sub			= _nh.subscribe("/robotis/thormang3_foot_step_generator/footsteps_2d",    0, Step2DArrayCallback);

}

void WalkingModuleStatusMSGCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg)
{
	if(msg->type == msg->STATUS_ERROR)
		ROS_ERROR_STREAM("[Robot] : " << msg->status_msg);
	else if(msg->type == msg->STATUS_INFO)
		ROS_INFO_STREAM("[Robot] : " << msg->status_msg);
	else if(msg->type == msg->STATUS_WARN)
		ROS_WARN_STREAM("[Robot] : " << msg->status_msg);
	else if(msg->type == msg->STATUS_UNKNOWN)
		ROS_ERROR_STREAM("[Robot] : " << msg->status_msg);
	else
		ROS_ERROR_STREAM("[Robot] : " << msg->status_msg);
}

void WalkingCommandCallback(const thormang3_foot_step_generator::FootStepCommand::ConstPtr &msg)
{
	ROS_INFO("[Demo]  : Walking Command");
	ROS_INFO_STREAM("  command        : " << msg->command );
	ROS_INFO_STREAM("  step_num       : " << msg->step_num );
	ROS_INFO_STREAM("  step_length    : " << msg->step_length);
	ROS_INFO_STREAM("  side_ste_lengh : " << msg->side_step_length );
	ROS_INFO_STREAM("  step_angle_rad : " << msg->step_angle_rad );

	if((msg->step_num == 0) && ( (msg->command != "left kick") && (msg->command != "right kick") ))
		return;

    thormang3_walking_module_msgs::GetReferenceStepData	_get_ref_stp_data_srv;
    thormang3_walking_module_msgs::StepData				_ref_step_data;
    thormang3_walking_module_msgs::AddStepDataArray		_add_stp_data_srv;

	//set walking parameter
	thormang3_foot_stp_generator.fb_step_length_m_ = msg->step_length;
	thormang3_foot_stp_generator.rl_step_length_m_ = msg->side_step_length;
	thormang3_foot_stp_generator.rotate_step_angle_rad_ = msg->step_angle_rad;
	thormang3_foot_stp_generator.num_of_step_ = 2*(msg->step_num) + 2;


	//get reference step data
	if(get_ref_step_data_client.call(_get_ref_stp_data_srv) == false) {
		ROS_ERROR("Failed to get reference step data");
		return;
	}

	_ref_step_data = _get_ref_stp_data_srv.response.reference_step_data;


	//calc step data
	if(msg->command == "forward") {
		thormang3_foot_stp_generator.getStepData( &_add_stp_data_srv.request.step_data_array, _ref_step_data, ForwardWalking);
	}
	else if(msg->command == "backward") {
		thormang3_foot_stp_generator.getStepData( &_add_stp_data_srv.request.step_data_array, _ref_step_data, BackwardWalking);
	}
	else if(msg->command == "turn left") {
		thormang3_foot_stp_generator.getStepData( &_add_stp_data_srv.request.step_data_array, _ref_step_data, PlusRotatingWalking);
	}
	else if(msg->command == "turn right") {
		thormang3_foot_stp_generator.getStepData( &_add_stp_data_srv.request.step_data_array, _ref_step_data, MinusRotatingWalking);
	}
	else if(msg->command == "right") {
		thormang3_foot_stp_generator.getStepData( &_add_stp_data_srv.request.step_data_array, _ref_step_data, RightwardWalking);
	}
	else if(msg->command == "left") {
		thormang3_foot_stp_generator.getStepData( &_add_stp_data_srv.request.step_data_array, _ref_step_data, LeftwardWalking);
	}
	else if(msg->command == "right kick") {
		if(isRunning()== true)
			return;
		thormang3_foot_stp_generator.calcRightKickStep(&_add_stp_data_srv.request.step_data_array, _ref_step_data);
	}
	else if(msg->command == "left kick") {
		if(isRunning()== true)
			return;
		thormang3_foot_stp_generator.calcLeftKickStep(&_add_stp_data_srv.request.step_data_array, _ref_step_data);
	}
	else {
		ROS_ERROR("[Demo]  : Invalid Command");
		return;
	}

	//set add step data srv fot auto start and remove existing step data
	_add_stp_data_srv.request.auto_start = true;
	_add_stp_data_srv.request.remove_existing_step_data = true;

	//add step data
	if(add_step_data_array_client.call(_add_stp_data_srv) == true) {
		int _result = _add_stp_data_srv.response.result;
		if(_result== STEP_DATA_ERR::NO_ERROR)
			ROS_INFO("[Demo]  : Succeed to add step data array");
		else {
			ROS_ERROR("[Demo]  : Failed to add step data array");

			if(_result & STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE");
			if(_result & STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA");
			if(_result & STEP_DATA_ERR::PROBLEM_IN_TIME_DATA)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_TIME_DATA");
			if(_result & STEP_DATA_ERR::ROBOT_IS_WALKING_NOW)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");

			return;
		}
	}
	else {
		ROS_ERROR("[Demo]  : Failed to add step data array ");
		return;
	}

}


void Step2DArrayCallback(const thormang3_foot_step_generator::Step2DArray::ConstPtr& msg)
{
    thormang3_walking_module_msgs::GetReferenceStepData	_get_ref_stp_data_srv;
    thormang3_walking_module_msgs::StepData				_ref_step_data;
    thormang3_walking_module_msgs::AddStepDataArray		_add_stp_data_srv;
    thormang3_walking_module_msgs::IsRunning			_is_running_srv;


    if(is_running_client.call(_is_running_srv) == false) {
    	ROS_ERROR("[Demo]  : Failed to Walking Status");
    	return;
    }
    else {
    	if(_is_running_srv.response.is_running == true)
    	{
    		ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");
    		return;
    	}
    }


	//get reference step data
	if(get_ref_step_data_client.call(_get_ref_stp_data_srv) == false) {
		ROS_ERROR("[Demo]  : Failed to get reference step data");
		return;
	}

	_ref_step_data = _get_ref_stp_data_srv.response.reference_step_data;


	thormang3_foot_stp_generator.getStepDataFromStepData2DArray(&_add_stp_data_srv.request.step_data_array, _ref_step_data, msg);

	//set add step data srv fot auto start and remove existing step data
	_add_stp_data_srv.request.auto_start = true;
	_add_stp_data_srv.request.remove_existing_step_data = true;

	//add step data
	if(add_step_data_array_client.call(_add_stp_data_srv) == true) {
		int _result = _add_stp_data_srv.response.result;
		if(_result== STEP_DATA_ERR::NO_ERROR)
			ROS_INFO("[Demo]  : Succeed to add step data array");
		else {
			ROS_ERROR("[Demo]  : Failed to add step data array");

			if(_result & STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE");
			if(_result & STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA");
			if(_result & STEP_DATA_ERR::PROBLEM_IN_TIME_DATA)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_TIME_DATA");
			if(_result & STEP_DATA_ERR::ROBOT_IS_WALKING_NOW)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");

			return;
		}
	}
	else {
		ROS_ERROR("[Demo]  : Failed to add step data array ");
		return;
	}
}

bool isRunning(void)
{
	thormang3_walking_module_msgs::IsRunning is_running_srv;
    if(is_running_client.call(is_running_srv) == false) {
        ROS_ERROR("[Demo]  : Failed to Walking Status");
        return true;
    }
    else {
        if(is_running_srv.response.is_running == true)
        {
            ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");
            return true;
        }
    }

    return false;
}


