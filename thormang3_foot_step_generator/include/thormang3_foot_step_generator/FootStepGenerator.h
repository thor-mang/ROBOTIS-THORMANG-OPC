/*
 * RealTimeFootStepGenerator.h
 *
 *  Created on: 2014. 7. 7.
 *      Author: hjsong
 */

#ifndef REALTIMEFOOTSTEPGENERATOR_H_
#define REALTIMEFOOTSTEPGENERATOR_H_


#include "thormang3_walking_module_msgs/AddStepDataArray.h"
#include "thormang3_foot_step_generator/WalkingModuleCommon.h"

#define Stop					0
#define ForwardWalking			1
#define BackwardWalking			2
#define RightwardWalking		3
#define LeftwardWalking			4
#define PlusRotatingWalking		5
#define MinusRotatingWalking	6


namespace ROBOTIS
{
	class FootStepGenerator
	{
	private:
		int mPreviousStepType;
        thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type step_data_array_;

	public:
		int    num_of_step;
		double fb_step_length_m;
		double rl_step_length_m;
		double rotate_step_angle_rad;

		double step_time_sec;
		double start_end_time_sec;
		double dsp_ratio;

		double foot_z_swap_m;
		double body_z_swap_m;

		double default_y_feet_offset_m;

		//walking_module_msgs::StepData mRefStepData;

        bool CalcStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int previous_step_type,  int desired_step_type);
        void CalcFBStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction);
        void CalcRLStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction);
        void CalcRoStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction);
        void CalcStopStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction);

		FootStepGenerator();
		~FootStepGenerator();

        void SetReferenceStepData(const thormang3_walking_module_msgs::StepData& ref_stp_data);
        void GetStepData(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* _step_data_array, const thormang3_walking_module_msgs::StepData& ref_step_data, int desired_step_type);
	};


}



#endif /* REALTIMEFOOTSTEPGENERATOR_H_ */
