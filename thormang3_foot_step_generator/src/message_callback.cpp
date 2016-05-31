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




//
//
//void BalanceCommandCallback(const std_msgs::Bool::ConstPtr &msg)
//{
//    thormang3_walking_module_msgs::SetBalanceParam _set_balance_param_srv;
//
//	if(LoadBalanceParam(_set_balance_param_srv) == false)
//	{
//		return;
//	}
//
//	_set_balance_param_srv.request.updating_duration = 2.0; //1sec
//	if(msg->data == false)
//	{
//		_set_balance_param_srv.request.balance_param.hip_roll_swap_angle_rad = 0;
//		_set_balance_param_srv.request.balance_param.gyro_gain               = 0;
//		_set_balance_param_srv.request.balance_param.foot_roll_angle_gain    = 0;
//		_set_balance_param_srv.request.balance_param.foot_pitch_angle_gain   = 0;
//		_set_balance_param_srv.request.balance_param.foot_x_force_gain       = 0;
//		_set_balance_param_srv.request.balance_param.foot_y_force_gain       = 0;
//		_set_balance_param_srv.request.balance_param.foot_z_force_gain       = 0;
//		_set_balance_param_srv.request.balance_param.foot_roll_torque_gain   = 0;
//		_set_balance_param_srv.request.balance_param.foot_pitch_torque_gain  = 0;
//	}
//
//	if(set_balance_param_client.call(_set_balance_param_srv) == true)
//	{
//		int _result = _set_balance_param_srv.response.result;
//		if( _result == BALANCE_PARAM_ERR::NO_ERROR) {
//			ROS_INFO("[Demo]  : Succeed to set balance param");
//			ROS_INFO("[Demo]  : Please wait 2 sec for turning on balance");
//		}
//		else {
//			if(_result & BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE)
//				ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE");
//			if(_result & BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED)
//				ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED");
//			if(_result & BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE)
//				ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE");
//		}
//	}
//	else
//		ROS_ERROR("[Demo]  : Failed to set balance param ");
//}
//
//bool LoadBalanceParam(thormang3_walking_module_msgs::SetBalanceParam& _set_param)
//{
//    ros::NodeHandle _ros_node;
//    std::string _balance_yaml_path = "";
//    _balance_yaml_path = ros::package::getPath("thormang3_foot_step_generator") + "/data/balance_param.yaml";
//
//    YAML::Node _doc;
//	try	{
//		// load yaml
//		_doc = YAML::LoadFile(_balance_yaml_path.c_str());
//	}
//	catch(const std::exception& e) {
//		ROS_ERROR("Failed to load balance param yaml file.");
//		return false;
//	}
//
//	double cob_x_offset_m					= _doc["cob_x_offset_m"].as<double>();
//	double cob_y_offset_m					= _doc["cob_y_offset_m"].as<double>();
//	double hip_roll_swap_angle_rad			= _doc["hip_roll_swap_angle_rad"].as<double>();
//	double gyro_gain						= _doc["gyro_gain"].as<double>();
//	double foot_roll_angle_gain				= _doc["foot_roll_angle_gain"].as<double>();
//	double foot_pitch_angle_gain			= _doc["foot_pitch_angle_gain"].as<double>();
//	double foot_x_force_gain				= _doc["foot_x_force_gain"].as<double>();
//	double foot_y_force_gain				= _doc["foot_y_force_gain"].as<double>();
//	double foot_z_force_gain				= _doc["foot_z_force_gain"].as<double>();
//	double foot_roll_torque_gain			= _doc["foot_roll_torque_gain"].as<double>();
//	double foot_pitch_torque_gain			= _doc["foot_pitch_torque_gain"].as<double>();
//	double foot_roll_angle_time_constant	= _doc["foot_roll_angle_time_constant"].as<double>();
//	double foot_pitch_angle_time_constant	= _doc["foot_pitch_angle_time_constant"].as<double>();
//	double foot_x_force_time_constant		= _doc["foot_x_force_time_constant"].as<double>();
//	double foot_y_force_time_constant		= _doc["foot_y_force_time_constant"].as<double>();
//	double foot_z_force_time_constant		= _doc["foot_z_force_time_constant"].as<double>();
//	double foot_roll_torque_time_constant	= _doc["foot_roll_torque_time_constant"].as<double>();
//	double foot_pitch_torque_time_constant	= _doc["foot_pitch_torque_time_constant"].as<double>();
//
////	std::cout << "cob_x_offset_m                  : " << cob_x_offset_m					    <<std::endl;
////	std::cout << "cob_y_offset_m                  : " << cob_y_offset_m					    <<std::endl;
////	std::cout << "hip_roll_swap_angle_rad         : " << hip_roll_swap_angle_rad			<<std::endl;
////	std::cout << "gyro_gain                       : " << gyro_gain						    <<std::endl;
////	std::cout << "foot_roll_angle_gain            : " << foot_roll_angle_gain				<<std::endl;
////	std::cout << "foot_pitch_angle_gain           : " << foot_pitch_angle_gain			    <<std::endl;
////	std::cout << "foot_x_force_gain               : " << foot_x_force_gain				    <<std::endl;
////	std::cout << "foot_y_force_gain               : " << foot_y_force_gain				    <<std::endl;
////	std::cout << "foot_z_force_gain               : " << foot_z_force_gain				    <<std::endl;
////	std::cout << "foot_roll_torque_gain           : " << foot_roll_torque_gain			    <<std::endl;
////	std::cout << "foot_pitch_torque_gain          : " << foot_pitch_torque_gain			    <<std::endl;
////	std::cout << "foot_roll_angle_time_constant   : " << foot_roll_angle_time_constant	    <<std::endl;
////	std::cout << "foot_pitch_angle_time_constant  : " << foot_pitch_angle_time_constant	    <<std::endl;
////	std::cout << "foot_x_force_time_constant      : " << foot_x_force_time_constant		    <<std::endl;
////	std::cout << "foot_y_force_time_constant      : " << foot_y_force_time_constant		    <<std::endl;
////	std::cout << "foot_z_force_time_constant      : " << foot_z_force_time_constant		    <<std::endl;
////	std::cout << "foot_roll_torque_time_constant  : " << foot_roll_torque_time_constant	    <<std::endl;
////	std::cout << "foot_pitch_torque_time_constant : " << foot_pitch_torque_time_constant	<<std::endl;
//
//	_set_param.request.balance_param.cob_x_offset_m                  = cob_x_offset_m;
//	_set_param.request.balance_param.cob_y_offset_m                  = cob_y_offset_m;
//	_set_param.request.balance_param.hip_roll_swap_angle_rad         = hip_roll_swap_angle_rad;
//	_set_param.request.balance_param.gyro_gain                       = gyro_gain;
//	_set_param.request.balance_param.foot_roll_angle_gain            = foot_roll_angle_gain;
//	_set_param.request.balance_param.foot_pitch_angle_gain           = foot_pitch_angle_gain;
//	_set_param.request.balance_param.foot_x_force_gain               = foot_x_force_gain;
//	_set_param.request.balance_param.foot_y_force_gain               = foot_y_force_gain;
//	_set_param.request.balance_param.foot_z_force_gain               = foot_z_force_gain;
//	_set_param.request.balance_param.foot_roll_torque_gain           = foot_roll_torque_gain;
//	_set_param.request.balance_param.foot_pitch_torque_gain          = foot_pitch_torque_gain;
//	_set_param.request.balance_param.foot_roll_angle_time_constant   = foot_roll_angle_time_constant;
//	_set_param.request.balance_param.foot_pitch_angle_time_constant  = foot_pitch_angle_time_constant;
//	_set_param.request.balance_param.foot_x_force_time_constant      = foot_x_force_time_constant;
//	_set_param.request.balance_param.foot_y_force_time_constant      = foot_y_force_time_constant;
//	_set_param.request.balance_param.foot_z_force_time_constant      = foot_z_force_time_constant;
//	_set_param.request.balance_param.foot_roll_torque_time_constant  = foot_roll_torque_time_constant;
//	_set_param.request.balance_param.foot_pitch_torque_time_constant = foot_pitch_torque_time_constant;
//
//    return true;
//}
//
