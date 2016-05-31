/*
 * FootStepGenerator.cpp
 *
 *  Created on: 2014. 7. 7.
 *      Author: hjsong
 */

#include <cmath>
#include "thormang3_foot_step_generator/FootStepGenerator.h"


using namespace ROBOTIS;

#define RAD2DEG  (M_PI/180.0)

double sign(double n)
{
	if(n < 0)
		return -1;
	else if(n > 0)
		return 1;
	else
		return 0;
}

FootStepGenerator::FootStepGenerator()
{
	num_of_step_				= 2*2 + 2;
	fb_step_length_m_		= 0.1;
	rl_step_length_m_		= 0.07;
	rotate_step_angle_rad_	= 10.0*RAD2DEG;

	step_time_sec_ = 1.0;
	start_end_time_sec_ = 2.0;
	dsp_ratio_ = 0.2;

	foot_z_swap_m_ = 0.1;
	body_z_swap_m_ = 0.01;

	default_y_feet_offset_m_ = 0.186;

	previous_step_type_ = Stop;

	step_data_array_.clear();
}


FootStepGenerator::~FootStepGenerator()
{    }



void FootStepGenerator::getStepData(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* _step_data_array, const thormang3_walking_module_msgs::StepData& ref_step_data, int desired_step_type)
{
	_step_data_array->clear();
	step_data_array_.clear();

	if(calcStep(ref_step_data, previous_step_type_, desired_step_type)) {
		previous_step_type_ = desired_step_type;
		for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++) {
			_step_data_array->push_back(step_data_array_[stp_idx]);
		}
	}
	else {
		return;
	}
}



void FootStepGenerator::getStepDataFromStepData2DArray(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* _step_data_array,
							const thormang3_walking_module_msgs::StepData& ref_step_data,
							const thormang3_foot_step_generator::Step2DArray::ConstPtr& request_step_2d)
{
	_step_data_array->clear();

	thormang3_walking_module_msgs::StepData _stp_data;

	_stp_data = ref_step_data;
	_stp_data.time_data.abs_step_time += start_end_time_sec_;
	_stp_data.time_data.dsp_ratio = dsp_ratio_;
	_stp_data.time_data.walking_state = WalkingStateFlag::InWalkingStarting;
	_stp_data.time_data.start_time_delay_ratio_x     = 0.0;
	_stp_data.time_data.start_time_delay_ratio_y     = 0.0;
	_stp_data.time_data.start_time_delay_ratio_z     = 0.0;
	_stp_data.time_data.start_time_delay_ratio_roll  = 0.0;
	_stp_data.time_data.start_time_delay_ratio_pitch = 0.0;
	_stp_data.time_data.start_time_delay_ratio_yaw   = 0.0;
	_stp_data.time_data.finish_time_advance_ratio_x     = 0.0;
	_stp_data.time_data.finish_time_advance_ratio_y     = 0.0;
	_stp_data.time_data.finish_time_advance_ratio_z     = 0.0;
	_stp_data.time_data.finish_time_advance_ratio_roll  = 0.0;
	_stp_data.time_data.finish_time_advance_ratio_pitch = 0.0;
	_stp_data.time_data.finish_time_advance_ratio_yaw   = 0.0;

	_stp_data.position_data.moving_foot = MovingFootFlag::NFootMove;
	_stp_data.position_data.foot_z_swap = 0;
	_stp_data.position_data.body_z_swap = 0;

	_step_data_array->push_back(_stp_data);

	std::cout	<< _stp_data.time_data.abs_step_time << " "
				<< _stp_data.position_data.right_foot_pose.x <<" "
				<< _stp_data.position_data.right_foot_pose.y <<" "
				<< _stp_data.position_data.right_foot_pose.z <<" "
				<< _stp_data.position_data.right_foot_pose.roll <<" "
				<< _stp_data.position_data.right_foot_pose.pitch <<" "
				<< _stp_data.position_data.right_foot_pose.yaw <<"     "
				<< _stp_data.position_data.left_foot_pose.x <<" "
				<< _stp_data.position_data.left_foot_pose.y <<" "
				<< _stp_data.position_data.left_foot_pose.z <<" "
				<< _stp_data.position_data.left_foot_pose.roll <<" "
				<< _stp_data.position_data.left_foot_pose.pitch <<" "
				<< _stp_data.position_data.left_foot_pose.yaw <<"     "
				<< std::endl;


	for(unsigned int stp_idx = 0; stp_idx < request_step_2d->footsteps_2d.size(); stp_idx++) {

		_stp_data.time_data.abs_step_time += step_time_sec_;
		_stp_data.time_data.walking_state = WalkingStateFlag::InWalking;

		if(request_step_2d->footsteps_2d[stp_idx].moving_foot == thormang3_foot_step_generator::Step2D::LFootMove)
		{
			_stp_data.position_data.moving_foot = MovingFootFlag::LFootMove;
			_stp_data.position_data.body_z_swap = body_z_swap_m_;
			_stp_data.position_data.foot_z_swap = foot_z_swap_m_;
			_stp_data.position_data.left_foot_pose.x   = request_step_2d->footsteps_2d[stp_idx].step2d.x;
			_stp_data.position_data.left_foot_pose.y   = request_step_2d->footsteps_2d[stp_idx].step2d.y;
			_stp_data.position_data.left_foot_pose.yaw = request_step_2d->footsteps_2d[stp_idx].step2d.theta;

		}
		else if(request_step_2d->footsteps_2d[stp_idx].moving_foot == thormang3_foot_step_generator::Step2D::RFootMove)
		{
			_stp_data.position_data.moving_foot = MovingFootFlag::RFootMove;
			_stp_data.position_data.body_z_swap = body_z_swap_m_;
			_stp_data.position_data.foot_z_swap = foot_z_swap_m_;
			_stp_data.position_data.right_foot_pose.x   = request_step_2d->footsteps_2d[stp_idx].step2d.x;
			_stp_data.position_data.right_foot_pose.y   = request_step_2d->footsteps_2d[stp_idx].step2d.y;
			_stp_data.position_data.right_foot_pose.yaw = request_step_2d->footsteps_2d[stp_idx].step2d.theta;
		}
		else {
			ROS_ERROR("Invalid Step2D");
			_step_data_array->clear();
			return;
		}

		std::cout	<< _stp_data.time_data.abs_step_time << " "
					<< _stp_data.position_data.right_foot_pose.x <<" "
					<< _stp_data.position_data.right_foot_pose.y <<" "
					<< _stp_data.position_data.right_foot_pose.z <<" "
					<< _stp_data.position_data.right_foot_pose.roll <<" "
					<< _stp_data.position_data.right_foot_pose.pitch <<" "
					<< _stp_data.position_data.right_foot_pose.yaw <<"     "
					<< _stp_data.position_data.left_foot_pose.x <<" "
					<< _stp_data.position_data.left_foot_pose.y <<" "
					<< _stp_data.position_data.left_foot_pose.z <<" "
					<< _stp_data.position_data.left_foot_pose.roll <<" "
					<< _stp_data.position_data.left_foot_pose.pitch <<" "
					<< _stp_data.position_data.left_foot_pose.yaw <<"     "
					<< std::endl;

		if(fabs(_stp_data.position_data.right_foot_pose.yaw - _stp_data.position_data.left_foot_pose.yaw) > M_PI) {
			_stp_data.position_data.body_pose.yaw = 0.5*(_stp_data.position_data.right_foot_pose.yaw + _stp_data.position_data.left_foot_pose.yaw)
						- sign(0.5*(_stp_data.position_data.right_foot_pose.yaw - _stp_data.position_data.left_foot_pose.yaw))*M_PI;
		}
		else
			_stp_data.position_data.body_pose.yaw = 0.5*(_stp_data.position_data.right_foot_pose.yaw
				+ _stp_data.position_data.left_foot_pose.yaw);

		_step_data_array->push_back(_stp_data);
	}

	std::cout	<< _stp_data.time_data.abs_step_time << " "
				<< _stp_data.position_data.right_foot_pose.x <<" "
				<< _stp_data.position_data.right_foot_pose.y <<" "
				<< _stp_data.position_data.right_foot_pose.z <<" "
				<< _stp_data.position_data.right_foot_pose.roll <<" "
				<< _stp_data.position_data.right_foot_pose.pitch <<" "
				<< _stp_data.position_data.right_foot_pose.yaw <<"     "
				<< _stp_data.position_data.left_foot_pose.x <<" "
				<< _stp_data.position_data.left_foot_pose.y <<" "
				<< _stp_data.position_data.left_foot_pose.z <<" "
				<< _stp_data.position_data.left_foot_pose.roll <<" "
				<< _stp_data.position_data.left_foot_pose.pitch <<" "
				<< _stp_data.position_data.left_foot_pose.yaw <<"     "
				<< std::endl;

	_stp_data.time_data.abs_step_time += start_end_time_sec_;
	_stp_data.time_data.dsp_ratio = dsp_ratio_;
	_stp_data.time_data.walking_state = WalkingStateFlag::InWalkingEnding;
	_stp_data.time_data.start_time_delay_ratio_x     = 0.0;
	_stp_data.time_data.start_time_delay_ratio_y     = 0.0;
	_stp_data.time_data.start_time_delay_ratio_z     = 0.0;
	_stp_data.time_data.start_time_delay_ratio_roll  = 0.0;
	_stp_data.time_data.start_time_delay_ratio_pitch = 0.0;
	_stp_data.time_data.start_time_delay_ratio_yaw   = 0.0;
	_stp_data.time_data.finish_time_advance_ratio_x     = 0.0;
	_stp_data.time_data.finish_time_advance_ratio_y     = 0.0;
	_stp_data.time_data.finish_time_advance_ratio_z     = 0.0;
	_stp_data.time_data.finish_time_advance_ratio_roll  = 0.0;
	_stp_data.time_data.finish_time_advance_ratio_pitch = 0.0;
	_stp_data.time_data.finish_time_advance_ratio_yaw   = 0.0;

	_stp_data.position_data.moving_foot = MovingFootFlag::NFootMove;
	_stp_data.position_data.foot_z_swap = 0;
	_stp_data.position_data.body_z_swap = 0;

	_step_data_array->push_back(_stp_data);

}

//
bool FootStepGenerator::calcStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int previous_step_type,  int desired_step_type)
{
	int direction = 0;
    thormang3_walking_module_msgs::StepData stp_data[2];

    thormang3_walking_module_msgs::PoseXYZRPY poseLtoRF, poseLtoLF;

	poseLtoRF = ref_step_data.position_data.right_foot_pose;
	poseLtoLF = ref_step_data.position_data.left_foot_pose;

	if((desired_step_type == ForwardWalking) || (desired_step_type == LeftwardWalking) || (desired_step_type == PlusRotatingWalking))
		direction = 1;
	else if((desired_step_type == BackwardWalking ) || (desired_step_type == RightwardWalking) || (desired_step_type == MinusRotatingWalking))
		direction = -1;
	else if(desired_step_type == Stop)
		direction = 0;
	else {
		return false;
	}

	stp_data[0] = ref_step_data;
	stp_data[0].position_data.torso_yaw_angle_rad = 0.0*M_PI;

//	stp_data[0].position_data.right_foot_pose = poseLtoRF;
//	stp_data[0].position_data.left_foot_pose = poseLtoLF;
	stp_data[0].time_data.start_time_delay_ratio_x     = 0.0;
	stp_data[0].time_data.start_time_delay_ratio_y     = 0.0;
	stp_data[0].time_data.start_time_delay_ratio_z     = 0.0;
	stp_data[0].time_data.start_time_delay_ratio_roll  = 0.0;
	stp_data[0].time_data.start_time_delay_ratio_pitch = 0.0;
	stp_data[0].time_data.start_time_delay_ratio_yaw   = 0.0;
	stp_data[0].time_data.finish_time_advance_ratio_x     = 0.0;
	stp_data[0].time_data.finish_time_advance_ratio_y     = 0.0;
	stp_data[0].time_data.finish_time_advance_ratio_z     = 0.0;
	stp_data[0].time_data.finish_time_advance_ratio_roll  = 0.0;
	stp_data[0].time_data.finish_time_advance_ratio_pitch = 0.0;
	stp_data[0].time_data.finish_time_advance_ratio_yaw   = 0.0;


	if(stp_data[0].time_data.walking_state != WalkingStateFlag::InWalking)
	{
		if(desired_step_type == ForwardWalking || desired_step_type == BackwardWalking )
			calcFBStep(stp_data[0], direction);
		else if(desired_step_type == RightwardWalking || desired_step_type == LeftwardWalking )
			calcRLStep(stp_data[0], direction);
		else if(desired_step_type == PlusRotatingWalking || desired_step_type == MinusRotatingWalking )
			calcRoStep(stp_data[0], direction);
		else if(desired_step_type == Stop)
			calcStopStep(stp_data[0], direction);
		else {
			return false;
		}
	}
	else {
		if(desired_step_type != previous_step_type)
		{
			stp_data[0].time_data.walking_state = WalkingStateFlag::InWalking;
			if(previous_step_type == ForwardWalking || previous_step_type == BackwardWalking)
			{
				if(fabs(poseLtoRF.x - poseLtoLF.x) >= 0.001) {
					stp_data[0].time_data.abs_step_time += step_time_sec_;

					if(ref_step_data.position_data.moving_foot == MovingFootFlag::LFootMove) {
						stp_data[0].position_data.moving_foot = MovingFootFlag::RFootMove;
						stp_data[0].position_data.right_foot_pose.x = stp_data[0].position_data.left_foot_pose.x;
					}
					else {
						stp_data[0].position_data.moving_foot = MovingFootFlag::LFootMove;
						stp_data[0].position_data.left_foot_pose.x = stp_data[0].position_data.right_foot_pose.x;
					}
					//mwalking_module_msgs::StepDataList.push_back(stp_data[0]);
				}
			}
			else if(previous_step_type == LeftwardWalking || previous_step_type == RightwardWalking)
			{
				if(fabs(poseLtoRF.y - poseLtoLF.y) >= default_y_feet_offset_m_ - 0.001) {
					stp_data[0].time_data.abs_step_time += step_time_sec_;

					if(ref_step_data.position_data.moving_foot == MovingFootFlag::LFootMove) {
						stp_data[0].position_data.moving_foot = MovingFootFlag::RFootMove;
						stp_data[0].position_data.right_foot_pose.y = stp_data[0].position_data.left_foot_pose.y - default_y_feet_offset_m_;
					}
					else {
						stp_data[0].position_data.moving_foot = MovingFootFlag::LFootMove;
						stp_data[0].position_data.left_foot_pose.y = stp_data[0].position_data.right_foot_pose.y + default_y_feet_offset_m_;
					}
					//mwalking_module_msgs::StepDataList.push_back(stp_data[0]);
				}
			}
			else if(previous_step_type == PlusRotatingWalking || previous_step_type == MinusRotatingWalking)
			{
				if(fabs(poseLtoRF.yaw - poseLtoLF.yaw) > 0.0000001) {
					stp_data[0].time_data.abs_step_time += step_time_sec_;
					if(ref_step_data.position_data.moving_foot == MovingFootFlag::LFootMove) {
						stp_data[0].position_data.moving_foot = MovingFootFlag::RFootMove;
						stp_data[0].position_data.right_foot_pose.x   = stp_data[0].position_data.left_foot_pose.x;
						stp_data[0].position_data.right_foot_pose.y   = stp_data[0].position_data.left_foot_pose.y - default_y_feet_offset_m_;
						stp_data[0].position_data.right_foot_pose.yaw = stp_data[0].position_data.left_foot_pose.yaw;
					}
					else {
						stp_data[0].position_data.moving_foot = MovingFootFlag::LFootMove;
						stp_data[0].position_data.left_foot_pose.x   = stp_data[0].position_data.right_foot_pose.x;
						stp_data[0].position_data.left_foot_pose.y   = stp_data[0].position_data.right_foot_pose.y + default_y_feet_offset_m_;
						stp_data[0].position_data.left_foot_pose.yaw = stp_data[0].position_data.right_foot_pose.yaw;
					}
					//mwalking_module_msgs::StepDataList.push_back(stp_data[0]);
				}
			}
			else if(previous_step_type == Stop) {

				///// Add New Code
				if(fabs(poseLtoRF.yaw - poseLtoLF.yaw) > 0.0000001 || fabs(poseLtoRF.y - poseLtoLF.y) >= default_y_feet_offset_m_ - 0.001 || fabs(poseLtoRF.x - poseLtoLF.x) >= 0.001) {
					stp_data[0].time_data.abs_step_time += step_time_sec_;
					if(ref_step_data.position_data.moving_foot == MovingFootFlag::LFootMove) {
						stp_data[0].position_data.moving_foot = MovingFootFlag::RFootMove;
						stp_data[0].position_data.right_foot_pose.x   = stp_data[0].position_data.left_foot_pose.x;
						stp_data[0].position_data.right_foot_pose.y   = stp_data[0].position_data.left_foot_pose.y - default_y_feet_offset_m_;
						stp_data[0].position_data.right_foot_pose.yaw = stp_data[0].position_data.left_foot_pose.yaw;
					}
					else {
						stp_data[0].position_data.moving_foot = MovingFootFlag::LFootMove;
						stp_data[0].position_data.left_foot_pose.x   = stp_data[0].position_data.right_foot_pose.x;
						stp_data[0].position_data.left_foot_pose.y   = stp_data[0].position_data.right_foot_pose.y + default_y_feet_offset_m_;
						stp_data[0].position_data.left_foot_pose.yaw = stp_data[0].position_data.right_foot_pose.yaw;
					}
					//mwalking_module_msgs::StepDataList.push_back(stp_data[0]);
				}
				///// Add New Code
			}
			else {
				return false;
			}


			stp_data[1] = stp_data[0];
			if(desired_step_type == ForwardWalking || desired_step_type == BackwardWalking || desired_step_type == Stop) {

			}
			else if(desired_step_type == LeftwardWalking || desired_step_type == PlusRotatingWalking)
			{
				if(stp_data[0].position_data.moving_foot == MovingFootFlag::LFootMove) {
					stp_data[1].time_data.abs_step_time += step_time_sec_;
					stp_data[1].position_data.moving_foot = MovingFootFlag::RFootMove;
					//mwalking_module_msgs::StepDataList.push_back(stp_data[1]);
				}
			}
			else if(desired_step_type == RightwardWalking || desired_step_type == MinusRotatingWalking)
			{
				if(stp_data[0].position_data.moving_foot == MovingFootFlag::RFootMove) {
					stp_data[1].time_data.abs_step_time += step_time_sec_;
					stp_data[1].position_data.moving_foot = MovingFootFlag::LFootMove;
					//mwalking_module_msgs::StepDataList.push_back(stp_data[1]);
				}
			}
			else{
				return false;
			}

			if(desired_step_type == ForwardWalking || desired_step_type == BackwardWalking )
				calcFBStep(stp_data[1], direction);
			else if(desired_step_type == RightwardWalking || desired_step_type == LeftwardWalking )
				calcRLStep(stp_data[1], direction);
			else if(desired_step_type == PlusRotatingWalking || desired_step_type == MinusRotatingWalking )
				calcRoStep(stp_data[1], direction);
			else if(desired_step_type == Stop) {
				calcStopStep(stp_data[1], direction);
			}
			else {
				return false;
			}
		}
		else
		{
			if(desired_step_type == ForwardWalking || desired_step_type == BackwardWalking )
				calcFBStep(stp_data[0], direction);
			else if(desired_step_type == RightwardWalking || desired_step_type == LeftwardWalking )
				calcRLStep(stp_data[0], direction);
			else if(desired_step_type == PlusRotatingWalking || desired_step_type == MinusRotatingWalking )
				calcRoStep(stp_data[0], direction);
			else if(desired_step_type == Stop)
				calcStopStep(stp_data[0], direction);
			else {
				return false;
			}
		}
	}


	for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++) {
		if(fabs(step_data_array_[stp_idx].position_data.right_foot_pose.yaw - step_data_array_[stp_idx].position_data.left_foot_pose.yaw) > M_PI) {
			step_data_array_[stp_idx].position_data.body_pose.yaw = 0.5*(step_data_array_[stp_idx].position_data.right_foot_pose.yaw + step_data_array_[stp_idx].position_data.left_foot_pose.yaw)
						- sign(0.5*(step_data_array_[stp_idx].position_data.right_foot_pose.yaw - step_data_array_[stp_idx].position_data.left_foot_pose.yaw))*M_PI;
		}
		else
			step_data_array_[stp_idx].position_data.body_pose.yaw = 0.5*(step_data_array_[stp_idx].position_data.right_foot_pose.yaw
				+ step_data_array_[stp_idx].position_data.left_foot_pose.yaw);
	}

	return true;
}


void FootStepGenerator::calcFBStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction)
{
    thormang3_walking_module_msgs::StepData stp_data[num_of_step_];
	stp_data[0] = ref_step_data;

	if(ref_step_data.time_data.walking_state == WalkingStateFlag::InWalking) {
		stp_data[0].time_data.abs_step_time += step_time_sec_;
		stp_data[0].time_data.dsp_ratio = dsp_ratio_;
		stp_data[0].position_data.body_z_swap = body_z_swap_m_;
		stp_data[0].position_data.foot_z_swap = foot_z_swap_m_;
		if(stp_data[0].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[0].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[0].position_data.right_foot_pose.x = stp_data[0].position_data.left_foot_pose.x + (double)direction*fb_step_length_m_;
		}
		else {
			stp_data[0].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[0].position_data.left_foot_pose.x = stp_data[0].position_data.right_foot_pose.x + (double)direction*fb_step_length_m_;
		}

		for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++) {
			stp_data[stp_idx] = stp_data[stp_idx-1];
			stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
			if(stp_data[stp_idx].position_data.moving_foot == MovingFootFlag::LFootMove) {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::RFootMove;
				stp_data[stp_idx].position_data.right_foot_pose.x = stp_data[stp_idx].position_data.left_foot_pose.x + (double)direction*fb_step_length_m_;
			}
			else {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::LFootMove;
				stp_data[stp_idx].position_data.left_foot_pose.x = stp_data[stp_idx].position_data.right_foot_pose.x + (double)direction*fb_step_length_m_;
			}
		}

		stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
		stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
		if(stp_data[num_of_step_-2].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[num_of_step_-2].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[num_of_step_-2].position_data.right_foot_pose.x = stp_data[num_of_step_-2].position_data.left_foot_pose.x;
		}
		else {
			stp_data[num_of_step_-2].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[num_of_step_-2].position_data.left_foot_pose.x = stp_data[num_of_step_-2].position_data.right_foot_pose.x;
		}

		stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
		stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
		stp_data[num_of_step_-1].time_data.walking_state = WalkingStateFlag::InWalkingEnding;
		stp_data[num_of_step_-1].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[num_of_step_-1].position_data.body_z_swap = 0;
	}
	else {
		stp_data[0].time_data.walking_state = WalkingStateFlag::InWalkingStarting;
		stp_data[0].time_data.abs_step_time += start_end_time_sec_;
		stp_data[0].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[0].position_data.body_z_swap = 0;

		for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++) {
			stp_data[stp_idx] = stp_data[stp_idx-1];
			stp_data[stp_idx].time_data.walking_state = WalkingStateFlag::InWalking;
			stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
			stp_data[stp_idx].time_data.dsp_ratio = dsp_ratio_;
			stp_data[stp_idx].position_data.body_z_swap = body_z_swap_m_;
			stp_data[stp_idx].position_data.foot_z_swap = foot_z_swap_m_;

			if(stp_data[stp_idx].position_data.moving_foot == MovingFootFlag::LFootMove) {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::RFootMove;
				stp_data[stp_idx].position_data.right_foot_pose.x = stp_data[stp_idx].position_data.left_foot_pose.x + (double)direction*fb_step_length_m_;
			}
			else {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::LFootMove;
				stp_data[stp_idx].position_data.left_foot_pose.x = stp_data[stp_idx].position_data.right_foot_pose.x + (double)direction*fb_step_length_m_;
			}
		}

		stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
		stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
		if(stp_data[num_of_step_-2].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[num_of_step_-2].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[num_of_step_-2].position_data.right_foot_pose.x = stp_data[num_of_step_-2].position_data.left_foot_pose.x;
		}
		else {
			stp_data[num_of_step_-2].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[num_of_step_-2].position_data.left_foot_pose.x = stp_data[num_of_step_-2].position_data.right_foot_pose.x;
		}

		stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
		stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
		stp_data[num_of_step_-1].time_data.walking_state = WalkingStateFlag::InWalkingEnding;
		stp_data[num_of_step_-1].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[num_of_step_-1].position_data.body_z_swap = 0;
	}

	for(int stp_idx = 0; stp_idx < num_of_step_; stp_idx++) {
		step_data_array_.push_back(stp_data[stp_idx]);
	}
}

void FootStepGenerator::calcRLStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction)
{
    thormang3_walking_module_msgs::StepData stp_data[num_of_step_];
	stp_data[0] = ref_step_data;

	if(ref_step_data.time_data.walking_state == WalkingStateFlag::InWalking) {
		stp_data[0].time_data.abs_step_time += step_time_sec_;
		stp_data[0].time_data.dsp_ratio = dsp_ratio_;
		stp_data[0].position_data.body_z_swap = body_z_swap_m_;
		stp_data[0].position_data.foot_z_swap = foot_z_swap_m_;
		if(stp_data[0].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[0].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[0].position_data.right_foot_pose.y = stp_data[0].position_data.right_foot_pose.y + (double)direction*rl_step_length_m_;
		}
		else {
			stp_data[0].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[0].position_data.left_foot_pose.y = stp_data[0].position_data.left_foot_pose.y + (double)direction*rl_step_length_m_;
		}

		for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++) {
			stp_data[stp_idx] = stp_data[stp_idx-1];
			stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
			if(stp_data[stp_idx].position_data.moving_foot == MovingFootFlag::LFootMove) {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::RFootMove;
				stp_data[stp_idx].position_data.right_foot_pose.y = stp_data[stp_idx].position_data.right_foot_pose.y + (double)direction*rl_step_length_m_;
			}
			else {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::LFootMove;
				stp_data[stp_idx].position_data.left_foot_pose.y = stp_data[stp_idx].position_data.left_foot_pose.y + (double)direction*rl_step_length_m_;
			}
		}

		stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
		stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
		if(stp_data[num_of_step_-2].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[num_of_step_-2].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[num_of_step_-2].position_data.right_foot_pose.y = stp_data[num_of_step_-2].position_data.left_foot_pose.y - default_y_feet_offset_m_;
		}
		else {
			stp_data[num_of_step_-2].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[num_of_step_-2].position_data.left_foot_pose.y = stp_data[num_of_step_-2].position_data.right_foot_pose.y + default_y_feet_offset_m_;
		}

		stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
		stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
		stp_data[num_of_step_-1].time_data.walking_state = WalkingStateFlag::InWalkingEnding;
		stp_data[num_of_step_-1].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[num_of_step_-1].position_data.body_z_swap = 0;
	}
	else {
		stp_data[0].time_data.walking_state = WalkingStateFlag::InWalkingStarting;
		stp_data[0].time_data.abs_step_time += start_end_time_sec_;
		stp_data[0].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[0].position_data.body_z_swap = 0;


		stp_data[1] = stp_data[0];
		stp_data[1].time_data.walking_state = WalkingStateFlag::InWalking;
		stp_data[1].time_data.abs_step_time += step_time_sec_;
		stp_data[1].time_data.dsp_ratio = dsp_ratio_;
		stp_data[1].position_data.body_z_swap = body_z_swap_m_;
		stp_data[1].position_data.foot_z_swap = foot_z_swap_m_;

		if(direction < 0) {
			stp_data[1].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[1].position_data.right_foot_pose.y = stp_data[1].position_data.right_foot_pose.y + (double)direction*rl_step_length_m_;
		}
		else {
			stp_data[1].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[1].position_data.left_foot_pose.y = stp_data[1].position_data.left_foot_pose.y + (double)direction*rl_step_length_m_;
		}

		for(int stp_idx = 2; stp_idx < num_of_step_-2; stp_idx++) {
			stp_data[stp_idx] = stp_data[stp_idx-1];
			stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
			if(stp_data[stp_idx].position_data.moving_foot == MovingFootFlag::LFootMove) {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::RFootMove;
				stp_data[stp_idx].position_data.right_foot_pose.y = stp_data[stp_idx].position_data.right_foot_pose.y + (double)direction*rl_step_length_m_;
			}
			else {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::LFootMove;
				stp_data[stp_idx].position_data.left_foot_pose.y = stp_data[stp_idx].position_data.left_foot_pose.y + (double)direction*rl_step_length_m_;
			}
		}

		stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
		stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
		if(stp_data[num_of_step_-2].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[num_of_step_-2].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[num_of_step_-2].position_data.right_foot_pose.y = stp_data[num_of_step_-2].position_data.left_foot_pose.y - default_y_feet_offset_m_;
		}
		else {
			stp_data[num_of_step_-2].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[num_of_step_-2].position_data.left_foot_pose.y = stp_data[num_of_step_-2].position_data.right_foot_pose.y + default_y_feet_offset_m_;
		}

		stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
		stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
		stp_data[num_of_step_-1].time_data.walking_state = WalkingStateFlag::InWalkingEnding;
		stp_data[num_of_step_-1].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[num_of_step_-1].position_data.body_z_swap = 0;

	}

	for(int stp_idx = 0; stp_idx < num_of_step_; stp_idx++) {
		step_data_array_.push_back(stp_data[stp_idx]);
	}
}

void FootStepGenerator::calcRoStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction)
{
    thormang3_walking_module_msgs::StepData stp_data[num_of_step_];
	stp_data[0] = ref_step_data;
	if(ref_step_data.time_data.walking_state == WalkingStateFlag::InWalking) {
		stp_data[0].time_data.walking_state = WalkingStateFlag::InWalking;
		stp_data[0].time_data.abs_step_time += step_time_sec_;
		stp_data[0].time_data.dsp_ratio = dsp_ratio_;
		stp_data[0].position_data.body_z_swap = body_z_swap_m_;
		stp_data[0].position_data.foot_z_swap = foot_z_swap_m_;

		if(stp_data[0].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[0].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[0].position_data.right_foot_pose.yaw = stp_data[0].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;
			stp_data[0].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(stp_data[0].position_data.right_foot_pose.yaw);
			stp_data[0].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(stp_data[0].position_data.right_foot_pose.yaw);
		}
		else {
			stp_data[0].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[0].position_data.left_foot_pose.yaw = stp_data[0].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;
			stp_data[0].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(stp_data[0].position_data.left_foot_pose.yaw);
			stp_data[0].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(stp_data[0].position_data.left_foot_pose.yaw);
		}


		for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++) {
			stp_data[stp_idx] = stp_data[stp_idx-1];
			stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
			if(stp_data[stp_idx].position_data.moving_foot == MovingFootFlag::LFootMove) {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::RFootMove;
				stp_data[stp_idx].position_data.right_foot_pose.yaw = stp_data[stp_idx].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;
				stp_data[stp_idx].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(stp_data[stp_idx].position_data.right_foot_pose.yaw);
				stp_data[stp_idx].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(stp_data[stp_idx].position_data.right_foot_pose.yaw);
			}
			else {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::LFootMove;
				stp_data[stp_idx].position_data.left_foot_pose.yaw = stp_data[stp_idx].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;
				stp_data[stp_idx].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(stp_data[stp_idx].position_data.left_foot_pose.yaw);
				stp_data[stp_idx].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(stp_data[stp_idx].position_data.left_foot_pose.yaw);

			}

			if(fabs(stp_data[stp_idx].position_data.right_foot_pose.yaw) > M_PI)
				stp_data[stp_idx].position_data.right_foot_pose.yaw -= 2.0*M_PI*sign(stp_data[stp_idx].position_data.right_foot_pose.yaw);
			if(fabs(stp_data[stp_idx].position_data.left_foot_pose.yaw) > M_PI)
				stp_data[stp_idx].position_data.left_foot_pose.yaw -= 2.0*M_PI*sign(stp_data[stp_idx].position_data.left_foot_pose.yaw);
		}

		stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
		stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
		if(stp_data[num_of_step_-2].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[num_of_step_-2].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[num_of_step_-2].position_data.right_foot_pose.yaw = stp_data[num_of_step_-2].position_data.left_foot_pose.yaw;
			stp_data[num_of_step_-2].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw);
			stp_data[num_of_step_-2].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw);

		}
		else {
			stp_data[num_of_step_-2].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[num_of_step_-2].position_data.left_foot_pose.yaw = stp_data[num_of_step_-2].position_data.right_foot_pose.yaw;
			stp_data[num_of_step_-2].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(stp_data[num_of_step_-2].position_data.right_foot_pose.yaw);
			stp_data[num_of_step_-2].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(stp_data[num_of_step_-2].position_data.right_foot_pose.yaw);
		}

		stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
		stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
		stp_data[num_of_step_-1].time_data.walking_state = WalkingStateFlag::InWalkingEnding;
		stp_data[num_of_step_-1].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[num_of_step_-1].position_data.body_z_swap = 0;
	}
	else {
		stp_data[0].time_data.walking_state = WalkingStateFlag::InWalkingStarting;
		stp_data[0].time_data.abs_step_time += start_end_time_sec_;
		stp_data[0].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[0].position_data.body_z_swap = 0;
		stp_data[0].position_data.foot_z_swap = 0;


		stp_data[1] = stp_data[0];
		stp_data[1].time_data.walking_state = WalkingStateFlag::InWalking;
		stp_data[1].time_data.abs_step_time += step_time_sec_;
		stp_data[1].time_data.dsp_ratio = dsp_ratio_;
		stp_data[1].position_data.body_z_swap = body_z_swap_m_;
		stp_data[1].position_data.foot_z_swap = foot_z_swap_m_;

		if(direction < 0) {
			stp_data[1].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[1].position_data.right_foot_pose.yaw  = stp_data[1].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;
			stp_data[1].position_data.right_foot_pose.x =  0.5*default_y_feet_offset_m_*sin(stp_data[1].position_data.left_foot_pose.yaw);
			stp_data[1].position_data.right_foot_pose.y = -0.5*default_y_feet_offset_m_*cos(stp_data[1].position_data.left_foot_pose.yaw);
		}
		else {
			stp_data[1].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[1].position_data.left_foot_pose.yaw  = stp_data[1].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;
			stp_data[1].position_data.left_foot_pose.x =  -0.5*default_y_feet_offset_m_*sin(stp_data[1].position_data.left_foot_pose.yaw);
			stp_data[1].position_data.left_foot_pose.y =   0.5*default_y_feet_offset_m_*cos(stp_data[1].position_data.left_foot_pose.yaw);
		}

		for(int stp_idx = 2; stp_idx < num_of_step_-2; stp_idx++) {
			stp_data[stp_idx] = stp_data[stp_idx-1];
			stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
			if(stp_data[stp_idx].position_data.moving_foot == MovingFootFlag::LFootMove) {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::RFootMove;
				stp_data[stp_idx].position_data.right_foot_pose.yaw = stp_data[stp_idx].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;
				stp_data[stp_idx].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(stp_data[stp_idx].position_data.right_foot_pose.yaw);
				stp_data[stp_idx].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(stp_data[stp_idx].position_data.right_foot_pose.yaw);
			}
			else {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::LFootMove;
				stp_data[stp_idx].position_data.left_foot_pose.yaw = stp_data[stp_idx].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;
				stp_data[stp_idx].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(stp_data[stp_idx].position_data.left_foot_pose.yaw);
				stp_data[stp_idx].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(stp_data[stp_idx].position_data.left_foot_pose.yaw);

			}

			if(fabs(stp_data[stp_idx].position_data.right_foot_pose.yaw) > M_PI)
				stp_data[stp_idx].position_data.right_foot_pose.yaw -= 2.0*M_PI*sign(stp_data[stp_idx].position_data.right_foot_pose.yaw);
			if(fabs(stp_data[stp_idx].position_data.left_foot_pose.yaw) > M_PI)
				stp_data[stp_idx].position_data.left_foot_pose.yaw -= 2.0*M_PI*sign(stp_data[stp_idx].position_data.left_foot_pose.yaw);
		}

		stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
		stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
		if(stp_data[num_of_step_-2].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[num_of_step_-2].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[num_of_step_-2].position_data.right_foot_pose.yaw  = stp_data[num_of_step_-2].position_data.left_foot_pose.yaw;
			stp_data[num_of_step_-2].position_data.right_foot_pose.x =  0.5*default_y_feet_offset_m_*sin(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw);
			stp_data[num_of_step_-2].position_data.right_foot_pose.y = -0.5*default_y_feet_offset_m_*cos(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw);

		}
		else {
			stp_data[num_of_step_-2].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[num_of_step_-2].position_data.left_foot_pose.yaw  = stp_data[num_of_step_-2].position_data.right_foot_pose.yaw;
			stp_data[num_of_step_-2].position_data.left_foot_pose.x =  -0.5*default_y_feet_offset_m_*sin(stp_data[num_of_step_-2].position_data.right_foot_pose.yaw);
			stp_data[num_of_step_-2].position_data.left_foot_pose.y =   0.5*default_y_feet_offset_m_*cos(stp_data[num_of_step_-2].position_data.right_foot_pose.yaw);
		}

		stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
		stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
		stp_data[num_of_step_-1].time_data.walking_state = WalkingStateFlag::InWalkingEnding;
		stp_data[num_of_step_-1].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[num_of_step_-1].position_data.body_z_swap = 0;

	}

	for(int stp_idx = 0; stp_idx < num_of_step_; stp_idx++) {
		step_data_array_.push_back(stp_data[stp_idx]);
	}
}


void FootStepGenerator::calcStopStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction)
{
    thormang3_walking_module_msgs::StepData stp_data;
	stp_data = ref_step_data;
	stp_data.time_data.walking_state = WalkingStateFlag::InWalkingEnding;
	stp_data.time_data.abs_step_time += start_end_time_sec_;
	stp_data.position_data.body_z_swap = 0;
	stp_data.position_data.moving_foot = MovingFootFlag::NFootMove;

	step_data_array_.push_back(stp_data);
}


void FootStepGenerator::calcRightKickStep(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
		const thormang3_walking_module_msgs::StepData& ref_step_data)
{
	thormang3_walking_module_msgs::StepData step_data_msg;
	//meter
	double kick_height = 0.08;
	double kick_far	   = 0.18;
	double kick_pitch  = 15.0*M_PI/180.0;

	//sec
	double kick_time   = 0.8;

	step_data_msg = ref_step_data;

	step_data_array->clear();
	step_data_array_.clear();

	//Start 1 Step Data
	step_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::InWalkingStarting;
	step_data_msg.time_data.abs_step_time += kick_time*1.8;
	step_data_msg.time_data.dsp_ratio = 1.0;

	step_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::NFootMove;
	step_data_msg.position_data.foot_z_swap = 0;
	step_data_msg.position_data.body_z_swap = 0;
	step_data_array_.push_back(step_data_msg);


	//StepData 2 move back Left Foot
	step_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::InWalking;
	step_data_msg.time_data.abs_step_time += kick_time*1.0;
	step_data_msg.time_data.dsp_ratio = 0.0;

	step_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::RFootMove;
	step_data_msg.position_data.right_foot_pose.x = -0.8*kick_far;
	step_data_msg.position_data.right_foot_pose.z += kick_height;
	step_data_msg.position_data.right_foot_pose.pitch = kick_pitch;
	step_data_msg.position_data.foot_z_swap = 0.05;
	step_data_array_.push_back(step_data_msg);


	//StepData 3 kick
	step_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::InWalking;
	step_data_msg.time_data.abs_step_time += kick_time*1.2;
	step_data_msg.time_data.dsp_ratio = 0.0;

	step_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::RFootMove;
	step_data_msg.position_data.right_foot_pose.x = 1.5*kick_far;
	step_data_msg.position_data.right_foot_pose.pitch = -kick_pitch;
	step_data_msg.position_data.foot_z_swap = 0.0;
	step_data_array_.push_back(step_data_msg);


	//StepData 4 move back
	step_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::InWalking;
	step_data_msg.time_data.abs_step_time += kick_time*1.2;
	step_data_msg.time_data.dsp_ratio = 0.0;

	step_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::RFootMove;
	step_data_msg.position_data.right_foot_pose.x = 0;
	step_data_msg.position_data.right_foot_pose.z -= kick_height;
	step_data_msg.position_data.right_foot_pose.pitch = 0;
	step_data_msg.position_data.foot_z_swap = 0.05;
	step_data_array_.push_back(step_data_msg);


	//StepData 5 End
	step_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::InWalkingEnding;
	step_data_msg.time_data.abs_step_time += kick_time*1.8;
	step_data_msg.time_data.dsp_ratio = 0.0;

	step_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::NFootMove;
	step_data_array_.push_back(step_data_msg);

	for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++) {
		step_data_array->push_back(step_data_array_[stp_idx]);
	}

}

void FootStepGenerator::calcLeftKickStep(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
		const thormang3_walking_module_msgs::StepData& ref_step_data)
{
	thormang3_walking_module_msgs::StepData step_data_msg;
	//meter
	double kick_height = 0.08;
	double kick_far	   = 0.18;
	double kick_pitch  = 15.0*M_PI/180.0;

	//sec
	double kick_time   = 0.8;

	step_data_msg = ref_step_data;

	step_data_array->clear();
	step_data_array_.clear();

	//Start 1 Step Data
	step_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::InWalkingStarting;
	step_data_msg.time_data.abs_step_time += kick_time*1.8;
	step_data_msg.time_data.dsp_ratio = 1.0;

	step_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::NFootMove;
	step_data_msg.position_data.foot_z_swap = 0;
	step_data_msg.position_data.body_z_swap = 0;
	step_data_array_.push_back(step_data_msg);


	//StepData 2 move back Left Foot
	step_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::InWalking;
	step_data_msg.time_data.abs_step_time += kick_time*1.0;
	step_data_msg.time_data.dsp_ratio = 0.0;

	step_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::LFootMove;
	step_data_msg.position_data.left_foot_pose.x = -0.8*kick_far;
	step_data_msg.position_data.left_foot_pose.z += kick_height;
	step_data_msg.position_data.left_foot_pose.pitch = kick_pitch;
	step_data_msg.position_data.foot_z_swap = 0.05;
	step_data_array_.push_back(step_data_msg);


	//StepData 3 kick
	step_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::InWalking;
	step_data_msg.time_data.abs_step_time += kick_time*1.2;
	step_data_msg.time_data.dsp_ratio = 0.0;

	step_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::LFootMove;
	step_data_msg.position_data.left_foot_pose.x = 1.5*kick_far;
	step_data_msg.position_data.left_foot_pose.pitch = -kick_pitch;
	step_data_msg.position_data.foot_z_swap = 0.0;
	step_data_array_.push_back(step_data_msg);


	//StepData 4 move back
	step_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::InWalking;
	step_data_msg.time_data.abs_step_time += kick_time*1.2;
	step_data_msg.time_data.dsp_ratio = 0.0;

	step_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::LFootMove;
	step_data_msg.position_data.left_foot_pose.x = 0;
	step_data_msg.position_data.left_foot_pose.z -= kick_height;
	step_data_msg.position_data.left_foot_pose.pitch = 0;
	step_data_msg.position_data.foot_z_swap = 0.05;
	step_data_array_.push_back(step_data_msg);


	//StepData 5 End
	step_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::InWalkingEnding;
	step_data_msg.time_data.abs_step_time += kick_time*1.8;
	step_data_msg.time_data.dsp_ratio = 0.0;

	step_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::NFootMove;
	step_data_array_.push_back(step_data_msg);

	for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++) {
		step_data_array->push_back(step_data_array_[stp_idx]);
	}
}

