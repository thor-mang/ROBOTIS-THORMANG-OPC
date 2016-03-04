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
	num_of_step				= 2*2 + 2;
	fb_step_length_m		= 0.1;
	rl_step_length_m		= 0.07;
	rotate_step_angle_rad	= 10.0*RAD2DEG;

	step_time_sec = 1.0;
	start_end_time_sec = 2.0;
	dsp_ratio = 0.2;

	foot_z_swap_m = 0.1;
	body_z_swap_m = 0.01;

	default_y_feet_offset_m = 0.186;

	mPreviousStepType = Stop;

	step_data_array_.clear();
}


FootStepGenerator::~FootStepGenerator()
{    }



void FootStepGenerator::GetStepData(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* _step_data_list, const thormang3_walking_module_msgs::StepData& ref_step_data, int desired_step_type)
{
	_step_data_list->clear();
	step_data_array_.clear();

	if(CalcStep(ref_step_data, mPreviousStepType, desired_step_type)) {
		mPreviousStepType = desired_step_type;
		for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++) {
			_step_data_list->push_back(step_data_array_[stp_idx]);
//			std::cout << step_data_array_[stp_idx].time_data.abs_step_time << " "
//					<< step_data_array_[stp_idx].position_data.right_foot_pose.x <<" "
//					<< step_data_array_[stp_idx].position_data.right_foot_pose.y <<" "
//					<< step_data_array_[stp_idx].position_data.right_foot_pose.z <<" "
//					<< step_data_array_[stp_idx].position_data.right_foot_pose.roll <<" "
//					<< step_data_array_[stp_idx].position_data.right_foot_pose.pitch <<" "
//					<< step_data_array_[stp_idx].position_data.right_foot_pose.yaw <<"     "
//					<< step_data_array_[stp_idx].position_data.left_foot_pose.x <<" "
//					<< step_data_array_[stp_idx].position_data.left_foot_pose.y <<" "
//					<< step_data_array_[stp_idx].position_data.left_foot_pose.z <<" "
//					<< step_data_array_[stp_idx].position_data.left_foot_pose.roll <<" "
//					<< step_data_array_[stp_idx].position_data.left_foot_pose.pitch <<" "
//					<< step_data_array_[stp_idx].position_data.left_foot_pose.yaw <<"     "
//					<< std::endl;
		}
	}
	else {
		return;
	}
}

//
bool FootStepGenerator::CalcStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int previous_step_type,  int desired_step_type)
{
	int direction = 0;
    thormang3_walking_module_msgs::StepData stp_data[2];
	//walking_module_msgs::StepPositionData poseGtoRF, poseGtoLF;
    thormang3_walking_module_msgs::PoseXYZRPY poseLtoRF, poseLtoLF;
//	matd matGtoRF, matGtoLF;
//	matd matLtoRF, matLtoLF;
//	matd matGlobal2Local, matLocal2Global;
//	matd matLFtoLocal, matRFtoLocal;
//	poseGtoRF = ref_step_data.position_data.right_foot_pose;
//	poseGtoLF = ref_step_data.position_data.left_foot_pose;
//
//	matGtoRF = GetTransformMatrix(poseGtoRF.x, poseGtoRF.y, poseGtoRF.z, poseGtoRF.roll, poseGtoRF.pitch, poseGtoRF.yaw);
//	matGtoLF = GetTransformMatrix(poseGtoLF.x, poseGtoLF.y, poseGtoLF.z, poseGtoLF.roll, poseGtoLF.pitch, poseGtoLF.yaw);
//
//	matLFtoLocal = GetTranslationMatrix(0, -0.5*g_y_feet_offset, 0);
//	matRFtoLocal = GetTranslationMatrix(0,  0.5*g_y_feet_offset, 0);
//	if(ref_step_data.position_data.moving_foot == RFootMove) {
//		matGlobal2Local = matGtoRF*matRFtoLocal;
//		matLocal2Global = GetTransformMatrixInverse(matGlobal2Local);
//	}
//	else {
//		matGlobal2Local = matGtoLF*matLFtoLocal;
//		matLocal2Global = GetTransformMatrixInverse(matGlobal2Local);
//	}
//
//	matLtoRF = matLocal2Global*matGtoRF;
//	matLtoLF = matLocal2Global*matGtoLF;
//	poseLtoRF = GetPose3DfromTransformMatrix(matLtoRF);
//	poseLtoLF = GetPose3DfromTransformMatrix(matLtoLF);
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
			CalcFBStep(stp_data[0], direction);
		else if(desired_step_type == RightwardWalking || desired_step_type == LeftwardWalking )
			CalcRLStep(stp_data[0], direction);
		else if(desired_step_type == PlusRotatingWalking || desired_step_type == MinusRotatingWalking )
			CalcRoStep(stp_data[0], direction);
		else if(desired_step_type == Stop)
			CalcStopStep(stp_data[0], direction);
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
					stp_data[0].time_data.abs_step_time += step_time_sec;

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
				if(fabs(poseLtoRF.y - poseLtoLF.y) >= default_y_feet_offset_m - 0.001) {
					stp_data[0].time_data.abs_step_time += step_time_sec;

					if(ref_step_data.position_data.moving_foot == MovingFootFlag::LFootMove) {
						stp_data[0].position_data.moving_foot = MovingFootFlag::RFootMove;
						stp_data[0].position_data.right_foot_pose.y = stp_data[0].position_data.left_foot_pose.y - default_y_feet_offset_m;
					}
					else {
						stp_data[0].position_data.moving_foot = MovingFootFlag::LFootMove;
						stp_data[0].position_data.left_foot_pose.y = stp_data[0].position_data.right_foot_pose.y + default_y_feet_offset_m;
					}
					//mwalking_module_msgs::StepDataList.push_back(stp_data[0]);
				}
			}
			else if(previous_step_type == PlusRotatingWalking || previous_step_type == MinusRotatingWalking)
			{
				if(fabs(poseLtoRF.yaw - poseLtoLF.yaw) > 0.0000001) {
					stp_data[0].time_data.abs_step_time += step_time_sec;
					if(ref_step_data.position_data.moving_foot == MovingFootFlag::LFootMove) {
						stp_data[0].position_data.moving_foot = MovingFootFlag::RFootMove;
						stp_data[0].position_data.right_foot_pose.x   = stp_data[0].position_data.left_foot_pose.x;
						stp_data[0].position_data.right_foot_pose.y   = stp_data[0].position_data.left_foot_pose.y - default_y_feet_offset_m;
						stp_data[0].position_data.right_foot_pose.yaw = stp_data[0].position_data.left_foot_pose.yaw;
					}
					else {
						stp_data[0].position_data.moving_foot = MovingFootFlag::LFootMove;
						stp_data[0].position_data.left_foot_pose.x   = stp_data[0].position_data.right_foot_pose.x;
						stp_data[0].position_data.left_foot_pose.y   = stp_data[0].position_data.right_foot_pose.y + default_y_feet_offset_m;
						stp_data[0].position_data.left_foot_pose.yaw = stp_data[0].position_data.right_foot_pose.yaw;
					}
					//mwalking_module_msgs::StepDataList.push_back(stp_data[0]);
				}
			}
			else if(previous_step_type == Stop) {

				///// Add New Code
				if(fabs(poseLtoRF.yaw - poseLtoLF.yaw) > 0.0000001 || fabs(poseLtoRF.y - poseLtoLF.y) >= default_y_feet_offset_m - 0.001 || fabs(poseLtoRF.x - poseLtoLF.x) >= 0.001) {
					stp_data[0].time_data.abs_step_time += step_time_sec;
					if(ref_step_data.position_data.moving_foot == MovingFootFlag::LFootMove) {
						stp_data[0].position_data.moving_foot = MovingFootFlag::RFootMove;
						stp_data[0].position_data.right_foot_pose.x   = stp_data[0].position_data.left_foot_pose.x;
						stp_data[0].position_data.right_foot_pose.y   = stp_data[0].position_data.left_foot_pose.y - default_y_feet_offset_m;
						stp_data[0].position_data.right_foot_pose.yaw = stp_data[0].position_data.left_foot_pose.yaw;
					}
					else {
						stp_data[0].position_data.moving_foot = MovingFootFlag::LFootMove;
						stp_data[0].position_data.left_foot_pose.x   = stp_data[0].position_data.right_foot_pose.x;
						stp_data[0].position_data.left_foot_pose.y   = stp_data[0].position_data.right_foot_pose.y + default_y_feet_offset_m;
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
					stp_data[1].time_data.abs_step_time += step_time_sec;
					stp_data[1].position_data.moving_foot = MovingFootFlag::RFootMove;
					//mwalking_module_msgs::StepDataList.push_back(stp_data[1]);
				}
			}
			else if(desired_step_type == RightwardWalking || desired_step_type == MinusRotatingWalking)
			{
				if(stp_data[0].position_data.moving_foot == MovingFootFlag::RFootMove) {
					stp_data[1].time_data.abs_step_time += step_time_sec;
					stp_data[1].position_data.moving_foot = MovingFootFlag::LFootMove;
					//mwalking_module_msgs::StepDataList.push_back(stp_data[1]);
				}
			}
			else{
				return false;
			}

			if(desired_step_type == ForwardWalking || desired_step_type == BackwardWalking )
				CalcFBStep(stp_data[1], direction);
			else if(desired_step_type == RightwardWalking || desired_step_type == LeftwardWalking )
				CalcRLStep(stp_data[1], direction);
			else if(desired_step_type == PlusRotatingWalking || desired_step_type == MinusRotatingWalking )
				CalcRoStep(stp_data[1], direction);
			else if(desired_step_type == Stop) {
				CalcStopStep(stp_data[1], direction);
			}
			else {
				return false;
			}
		}
		else
		{
			if(desired_step_type == ForwardWalking || desired_step_type == BackwardWalking )
				CalcFBStep(stp_data[0], direction);
			else if(desired_step_type == RightwardWalking || desired_step_type == LeftwardWalking )
				CalcRLStep(stp_data[0], direction);
			else if(desired_step_type == PlusRotatingWalking || desired_step_type == MinusRotatingWalking )
				CalcRoStep(stp_data[0], direction);
			else if(desired_step_type == Stop)
				CalcStopStep(stp_data[0], direction);
			else {
				return false;
			}
		}
	}


	for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++) {
//		mwalking_module_msgs::StepDataList[stp_idx].position_data.right_foot_pose = GetPose3DfromTransformMatrix(matGlobal2Local*GetTransformMatrix(mwalking_module_msgs::StepDataList[stp_idx].position_data.right_foot_pose.x,
//				mwalking_module_msgs::StepDataList[stp_idx].position_data.right_foot_pose.y,
//				mwalking_module_msgs::StepDataList[stp_idx].position_data.right_foot_pose.z,
//				mwalking_module_msgs::StepDataList[stp_idx].position_data.right_foot_pose.roll,
//				mwalking_module_msgs::StepDataList[stp_idx].position_data.right_foot_pose.pitch,
//				mwalking_module_msgs::StepDataList[stp_idx].position_data.right_foot_pose.yaw));

//		mwalking_module_msgs::StepDataList[stp_idx].position_data.left_foot_pose = GetPose3DfromTransformMatrix(matGlobal2Local*GetTransformMatrix(mwalking_module_msgs::StepDataList[stp_idx].position_data.left_foot_pose.x,
//				mwalking_module_msgs::StepDataList[stp_idx].position_data.left_foot_pose.y,
//				mwalking_module_msgs::StepDataList[stp_idx].position_data.left_foot_pose.z,
//				mwalking_module_msgs::StepDataList[stp_idx].position_data.left_foot_pose.roll,
//				mwalking_module_msgs::StepDataList[stp_idx].position_data.left_foot_pose.pitch,
//				mwalking_module_msgs::StepDataList[stp_idx].position_data.left_foot_pose.yaw));

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


void FootStepGenerator::CalcFBStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction)
{
    thormang3_walking_module_msgs::StepData stp_data[num_of_step];
	stp_data[0] = ref_step_data;

	if(ref_step_data.time_data.walking_state == WalkingStateFlag::InWalking) {
		stp_data[0].time_data.abs_step_time += step_time_sec;
		stp_data[0].time_data.dsp_ratio = dsp_ratio;
		stp_data[0].position_data.body_z_swap = body_z_swap_m;
		stp_data[0].position_data.foot_z_swap = foot_z_swap_m;
		if(stp_data[0].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[0].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[0].position_data.right_foot_pose.x = stp_data[0].position_data.left_foot_pose.x + (double)direction*fb_step_length_m;
		}
		else {
			stp_data[0].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[0].position_data.left_foot_pose.x = stp_data[0].position_data.right_foot_pose.x + (double)direction*fb_step_length_m;
		}

		for(int stp_idx = 1; stp_idx < num_of_step-2; stp_idx++) {
			stp_data[stp_idx] = stp_data[stp_idx-1];
			stp_data[stp_idx].time_data.abs_step_time += step_time_sec;
			if(stp_data[stp_idx].position_data.moving_foot == MovingFootFlag::LFootMove) {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::RFootMove;
				stp_data[stp_idx].position_data.right_foot_pose.x = stp_data[stp_idx].position_data.left_foot_pose.x + (double)direction*fb_step_length_m;
			}
			else {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::LFootMove;
				stp_data[stp_idx].position_data.left_foot_pose.x = stp_data[stp_idx].position_data.right_foot_pose.x + (double)direction*fb_step_length_m;
			}
		}

		stp_data[num_of_step-2] = stp_data[num_of_step-3];
		stp_data[num_of_step-2].time_data.abs_step_time += step_time_sec;
		if(stp_data[num_of_step-2].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[num_of_step-2].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[num_of_step-2].position_data.right_foot_pose.x = stp_data[num_of_step-2].position_data.left_foot_pose.x;
		}
		else {
			stp_data[num_of_step-2].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[num_of_step-2].position_data.left_foot_pose.x = stp_data[num_of_step-2].position_data.right_foot_pose.x;
		}

		stp_data[num_of_step-1] = stp_data[num_of_step-2];
		stp_data[num_of_step-1].time_data.abs_step_time += start_end_time_sec;
		stp_data[num_of_step-1].time_data.walking_state = WalkingStateFlag::InWalkingEnding;
		stp_data[num_of_step-1].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[num_of_step-1].position_data.body_z_swap = 0;
	}
	else {
		stp_data[0].time_data.walking_state = WalkingStateFlag::InWalkingStarting;
		stp_data[0].time_data.abs_step_time += start_end_time_sec;
		stp_data[0].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[0].position_data.body_z_swap = 0;

		for(int stp_idx = 1; stp_idx < num_of_step-2; stp_idx++) {
			stp_data[stp_idx] = stp_data[stp_idx-1];
			stp_data[stp_idx].time_data.walking_state = WalkingStateFlag::InWalking;
			stp_data[stp_idx].time_data.abs_step_time += step_time_sec;
			stp_data[stp_idx].time_data.dsp_ratio = dsp_ratio;
			stp_data[stp_idx].position_data.body_z_swap = body_z_swap_m;
			stp_data[stp_idx].position_data.foot_z_swap = foot_z_swap_m;

			if(stp_data[stp_idx].position_data.moving_foot == MovingFootFlag::LFootMove) {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::RFootMove;
				stp_data[stp_idx].position_data.right_foot_pose.x = stp_data[stp_idx].position_data.left_foot_pose.x + (double)direction*fb_step_length_m;
			}
			else {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::LFootMove;
				stp_data[stp_idx].position_data.left_foot_pose.x = stp_data[stp_idx].position_data.right_foot_pose.x + (double)direction*fb_step_length_m;
			}
		}

		stp_data[num_of_step-2] = stp_data[num_of_step-3];
		stp_data[num_of_step-2].time_data.abs_step_time += step_time_sec;
		if(stp_data[num_of_step-2].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[num_of_step-2].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[num_of_step-2].position_data.right_foot_pose.x = stp_data[num_of_step-2].position_data.left_foot_pose.x;
		}
		else {
			stp_data[num_of_step-2].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[num_of_step-2].position_data.left_foot_pose.x = stp_data[num_of_step-2].position_data.right_foot_pose.x;
		}

		stp_data[num_of_step-1] = stp_data[num_of_step-2];
		stp_data[num_of_step-1].time_data.abs_step_time += start_end_time_sec;
		stp_data[num_of_step-1].time_data.walking_state = WalkingStateFlag::InWalkingEnding;
		stp_data[num_of_step-1].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[num_of_step-1].position_data.body_z_swap = 0;
	}

	for(int stp_idx = 0; stp_idx < num_of_step; stp_idx++) {
		step_data_array_.push_back(stp_data[stp_idx]);
	}
}

void FootStepGenerator::CalcRLStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction)
{
    thormang3_walking_module_msgs::StepData stp_data[num_of_step];
	stp_data[0] = ref_step_data;

	if(ref_step_data.time_data.walking_state == WalkingStateFlag::InWalking) {
		stp_data[0].time_data.abs_step_time += step_time_sec;
		stp_data[0].time_data.dsp_ratio = dsp_ratio;
		stp_data[0].position_data.body_z_swap = body_z_swap_m;
		stp_data[0].position_data.foot_z_swap = foot_z_swap_m;
		if(stp_data[0].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[0].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[0].position_data.right_foot_pose.y = stp_data[0].position_data.right_foot_pose.y + (double)direction*rl_step_length_m;
		}
		else {
			stp_data[0].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[0].position_data.left_foot_pose.y = stp_data[0].position_data.left_foot_pose.y + (double)direction*rl_step_length_m;
		}

		for(int stp_idx = 1; stp_idx < num_of_step-2; stp_idx++) {
			stp_data[stp_idx] = stp_data[stp_idx-1];
			stp_data[stp_idx].time_data.abs_step_time += step_time_sec;
			if(stp_data[stp_idx].position_data.moving_foot == MovingFootFlag::LFootMove) {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::RFootMove;
				stp_data[stp_idx].position_data.right_foot_pose.y = stp_data[stp_idx].position_data.right_foot_pose.y + (double)direction*rl_step_length_m;
			}
			else {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::LFootMove;
				stp_data[stp_idx].position_data.left_foot_pose.y = stp_data[stp_idx].position_data.left_foot_pose.y + (double)direction*rl_step_length_m;
			}
		}

		stp_data[num_of_step-2] = stp_data[num_of_step-3];
		stp_data[num_of_step-2].time_data.abs_step_time += step_time_sec;
		if(stp_data[num_of_step-2].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[num_of_step-2].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[num_of_step-2].position_data.right_foot_pose.y = stp_data[num_of_step-2].position_data.left_foot_pose.y - default_y_feet_offset_m;
		}
		else {
			stp_data[num_of_step-2].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[num_of_step-2].position_data.left_foot_pose.y = stp_data[num_of_step-2].position_data.right_foot_pose.y + default_y_feet_offset_m;
		}

		stp_data[num_of_step-1] = stp_data[num_of_step-2];
		stp_data[num_of_step-1].time_data.abs_step_time += start_end_time_sec;
		stp_data[num_of_step-1].time_data.walking_state = WalkingStateFlag::InWalkingEnding;
		stp_data[num_of_step-1].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[num_of_step-1].position_data.body_z_swap = 0;
	}
	else {
		stp_data[0].time_data.walking_state = WalkingStateFlag::InWalkingStarting;
		stp_data[0].time_data.abs_step_time += start_end_time_sec;
		stp_data[0].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[0].position_data.body_z_swap = 0;


		stp_data[1] = stp_data[0];
		stp_data[1].time_data.walking_state = WalkingStateFlag::InWalking;
		stp_data[1].time_data.abs_step_time += step_time_sec;
		stp_data[1].time_data.dsp_ratio = dsp_ratio;
		stp_data[1].position_data.body_z_swap = body_z_swap_m;
		stp_data[1].position_data.foot_z_swap = foot_z_swap_m;

		if(direction < 0) {
			stp_data[1].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[1].position_data.right_foot_pose.y = stp_data[1].position_data.right_foot_pose.y + (double)direction*rl_step_length_m;
		}
		else {
			stp_data[1].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[1].position_data.left_foot_pose.y = stp_data[1].position_data.left_foot_pose.y + (double)direction*rl_step_length_m;
		}

		for(int stp_idx = 2; stp_idx < num_of_step-2; stp_idx++) {
			stp_data[stp_idx] = stp_data[stp_idx-1];
			stp_data[stp_idx].time_data.abs_step_time += step_time_sec;
			if(stp_data[stp_idx].position_data.moving_foot == MovingFootFlag::LFootMove) {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::RFootMove;
				stp_data[stp_idx].position_data.right_foot_pose.y = stp_data[stp_idx].position_data.right_foot_pose.y + (double)direction*rl_step_length_m;
			}
			else {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::LFootMove;
				stp_data[stp_idx].position_data.left_foot_pose.y = stp_data[stp_idx].position_data.left_foot_pose.y + (double)direction*rl_step_length_m;
			}
		}

		stp_data[num_of_step-2] = stp_data[num_of_step-3];
		stp_data[num_of_step-2].time_data.abs_step_time += step_time_sec;
		if(stp_data[num_of_step-2].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[num_of_step-2].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[num_of_step-2].position_data.right_foot_pose.y = stp_data[num_of_step-2].position_data.left_foot_pose.y - default_y_feet_offset_m;
		}
		else {
			stp_data[num_of_step-2].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[num_of_step-2].position_data.left_foot_pose.y = stp_data[num_of_step-2].position_data.right_foot_pose.y + default_y_feet_offset_m;
		}

		stp_data[num_of_step-1] = stp_data[num_of_step-2];
		stp_data[num_of_step-1].time_data.abs_step_time += start_end_time_sec;
		stp_data[num_of_step-1].time_data.walking_state = WalkingStateFlag::InWalkingEnding;
		stp_data[num_of_step-1].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[num_of_step-1].position_data.body_z_swap = 0;

	}

	for(int stp_idx = 0; stp_idx < num_of_step; stp_idx++) {
		step_data_array_.push_back(stp_data[stp_idx]);
	}
}

void FootStepGenerator::CalcRoStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction)
{
    thormang3_walking_module_msgs::StepData stp_data[num_of_step];
	stp_data[0] = ref_step_data;
	if(ref_step_data.time_data.walking_state == WalkingStateFlag::InWalking) {
		stp_data[0].time_data.walking_state = WalkingStateFlag::InWalking;
		stp_data[0].time_data.abs_step_time += step_time_sec;
		stp_data[0].time_data.dsp_ratio = dsp_ratio;
		stp_data[0].position_data.body_z_swap = body_z_swap_m;
		stp_data[0].position_data.foot_z_swap = foot_z_swap_m;

		if(stp_data[0].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[0].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[0].position_data.right_foot_pose.yaw = stp_data[0].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad;
			stp_data[0].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m*sin(stp_data[0].position_data.right_foot_pose.yaw);
			stp_data[0].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m*cos(stp_data[0].position_data.right_foot_pose.yaw);
		}
		else {
			stp_data[0].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[0].position_data.left_foot_pose.yaw = stp_data[0].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad;
			stp_data[0].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m*sin(stp_data[0].position_data.left_foot_pose.yaw);
			stp_data[0].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m*cos(stp_data[0].position_data.left_foot_pose.yaw);
		}


		for(int stp_idx = 1; stp_idx < num_of_step-2; stp_idx++) {
			stp_data[stp_idx] = stp_data[stp_idx-1];
			stp_data[stp_idx].time_data.abs_step_time += step_time_sec;
			if(stp_data[stp_idx].position_data.moving_foot == MovingFootFlag::LFootMove) {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::RFootMove;
				stp_data[stp_idx].position_data.right_foot_pose.yaw = stp_data[stp_idx].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad;
				stp_data[stp_idx].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m*sin(stp_data[stp_idx].position_data.right_foot_pose.yaw);
				stp_data[stp_idx].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m*cos(stp_data[stp_idx].position_data.right_foot_pose.yaw);
			}
			else {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::LFootMove;
				stp_data[stp_idx].position_data.left_foot_pose.yaw = stp_data[stp_idx].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad;
				stp_data[stp_idx].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m*sin(stp_data[stp_idx].position_data.left_foot_pose.yaw);
				stp_data[stp_idx].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m*cos(stp_data[stp_idx].position_data.left_foot_pose.yaw);

			}
		}

		stp_data[num_of_step-2] = stp_data[num_of_step-3];
		stp_data[num_of_step-2].time_data.abs_step_time += step_time_sec;
		if(stp_data[num_of_step-2].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[num_of_step-2].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[num_of_step-2].position_data.right_foot_pose.yaw = stp_data[num_of_step-2].position_data.left_foot_pose.yaw;
			stp_data[num_of_step-2].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m*sin(stp_data[num_of_step-2].position_data.left_foot_pose.yaw);
			stp_data[num_of_step-2].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m*cos(stp_data[num_of_step-2].position_data.left_foot_pose.yaw);

		}
		else {
			stp_data[num_of_step-2].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[num_of_step-2].position_data.left_foot_pose.yaw = stp_data[num_of_step-2].position_data.right_foot_pose.yaw;
			stp_data[num_of_step-2].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m*sin(stp_data[num_of_step-2].position_data.right_foot_pose.yaw);
			stp_data[num_of_step-2].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m*cos(stp_data[num_of_step-2].position_data.right_foot_pose.yaw);
		}

		stp_data[num_of_step-1] = stp_data[num_of_step-2];
		stp_data[num_of_step-1].time_data.abs_step_time += start_end_time_sec;
		stp_data[num_of_step-1].time_data.walking_state = WalkingStateFlag::InWalkingEnding;
		stp_data[num_of_step-1].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[num_of_step-1].position_data.body_z_swap = 0;
	}
	else {
		stp_data[0].time_data.walking_state = WalkingStateFlag::InWalkingStarting;
		stp_data[0].time_data.abs_step_time += start_end_time_sec;
		stp_data[0].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[0].position_data.body_z_swap = 0;
		stp_data[0].position_data.foot_z_swap = 0;


		stp_data[1] = stp_data[0];
		stp_data[1].time_data.walking_state = WalkingStateFlag::InWalking;
		stp_data[1].time_data.abs_step_time += step_time_sec;
		stp_data[1].time_data.dsp_ratio = dsp_ratio;
		stp_data[1].position_data.body_z_swap = body_z_swap_m;
		stp_data[1].position_data.foot_z_swap = foot_z_swap_m;

		if(direction < 0) {
			stp_data[1].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[1].position_data.right_foot_pose.yaw  = stp_data[1].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad;
			stp_data[1].position_data.right_foot_pose.x =  0.5*default_y_feet_offset_m*sin(stp_data[1].position_data.left_foot_pose.yaw);
			stp_data[1].position_data.right_foot_pose.y = -0.5*default_y_feet_offset_m*cos(stp_data[1].position_data.left_foot_pose.yaw);
		}
		else {
			stp_data[1].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[1].position_data.left_foot_pose.yaw  = stp_data[1].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad;
			stp_data[1].position_data.left_foot_pose.x =  -0.5*default_y_feet_offset_m*sin(stp_data[1].position_data.left_foot_pose.yaw);
			stp_data[1].position_data.left_foot_pose.y =   0.5*default_y_feet_offset_m*cos(stp_data[1].position_data.left_foot_pose.yaw);
		}

		for(int stp_idx = 2; stp_idx < num_of_step-2; stp_idx++) {
			stp_data[stp_idx] = stp_data[stp_idx-1];
			stp_data[stp_idx].time_data.abs_step_time += step_time_sec;
			if(stp_data[stp_idx].position_data.moving_foot == MovingFootFlag::LFootMove) {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::RFootMove;
				stp_data[stp_idx].position_data.right_foot_pose.yaw = stp_data[stp_idx].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad;
				stp_data[stp_idx].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m*sin(stp_data[stp_idx].position_data.right_foot_pose.yaw);
				stp_data[stp_idx].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m*cos(stp_data[stp_idx].position_data.right_foot_pose.yaw);
			}
			else {
				stp_data[stp_idx].position_data.moving_foot = MovingFootFlag::LFootMove;
				stp_data[stp_idx].position_data.left_foot_pose.yaw = stp_data[stp_idx].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad;
				stp_data[stp_idx].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m*sin(stp_data[stp_idx].position_data.left_foot_pose.yaw);
				stp_data[stp_idx].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m*cos(stp_data[stp_idx].position_data.left_foot_pose.yaw);

			}
		}

		stp_data[num_of_step-2] = stp_data[num_of_step-3];
		stp_data[num_of_step-2].time_data.abs_step_time += step_time_sec;
		if(stp_data[num_of_step-2].position_data.moving_foot == MovingFootFlag::LFootMove) {
			stp_data[num_of_step-2].position_data.moving_foot = MovingFootFlag::RFootMove;
			stp_data[num_of_step-2].position_data.right_foot_pose.yaw  = stp_data[num_of_step-2].position_data.left_foot_pose.yaw;
			stp_data[num_of_step-2].position_data.right_foot_pose.x =  0.5*default_y_feet_offset_m*sin(stp_data[num_of_step-2].position_data.left_foot_pose.yaw);
			stp_data[num_of_step-2].position_data.right_foot_pose.y = -0.5*default_y_feet_offset_m*cos(stp_data[num_of_step-2].position_data.left_foot_pose.yaw);

		}
		else {
			stp_data[num_of_step-2].position_data.moving_foot = MovingFootFlag::LFootMove;
			stp_data[num_of_step-2].position_data.left_foot_pose.yaw  = stp_data[num_of_step-2].position_data.right_foot_pose.yaw;
			stp_data[num_of_step-2].position_data.left_foot_pose.x =  -0.5*default_y_feet_offset_m*sin(stp_data[num_of_step-2].position_data.right_foot_pose.yaw);
			stp_data[num_of_step-2].position_data.left_foot_pose.y =   0.5*default_y_feet_offset_m*cos(stp_data[num_of_step-2].position_data.right_foot_pose.yaw);
		}

		stp_data[num_of_step-1] = stp_data[num_of_step-2];
		stp_data[num_of_step-1].time_data.abs_step_time += start_end_time_sec;
		stp_data[num_of_step-1].time_data.walking_state = WalkingStateFlag::InWalkingEnding;
		stp_data[num_of_step-1].position_data.moving_foot = MovingFootFlag::NFootMove;
		stp_data[num_of_step-1].position_data.body_z_swap = 0;

	}

	for(int stp_idx = 0; stp_idx < num_of_step; stp_idx++) {
		step_data_array_.push_back(stp_data[stp_idx]);
	}
}



void FootStepGenerator::CalcStopStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction)
{
    thormang3_walking_module_msgs::StepData stp_data;
	stp_data = ref_step_data;
	stp_data.time_data.walking_state = WalkingStateFlag::InWalkingEnding;
	stp_data.time_data.abs_step_time += start_end_time_sec;
	stp_data.position_data.body_z_swap = 0;
	stp_data.position_data.moving_foot = MovingFootFlag::NFootMove;

	step_data_array_.push_back(stp_data);
}
