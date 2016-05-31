/*
 * RealTimeFootStepGenerator.h
 *
 *  Created on: 2014. 7. 7.
 *      Author: hjsong
 */

#ifndef REALTIMEFOOTSTEPGENERATOR_H_
#define REALTIMEFOOTSTEPGENERATOR_H_

#include <ros/ros.h>
#include "thormang3_walking_module_msgs/AddStepDataArray.h"
#include "thormang3_foot_step_generator/WalkingModuleCommon.h"
#include "thormang3_foot_step_generator/Step2DArray.h"

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
		int previous_step_type_;
        thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type step_data_array_;

	public:
		int    num_of_step_;
		double fb_step_length_m_;
		double rl_step_length_m_;
		double rotate_step_angle_rad_;

		double step_time_sec_;
		double start_end_time_sec_;
		double dsp_ratio_;

		double foot_z_swap_m_;
		double body_z_swap_m_;

		double default_y_feet_offset_m_;


        bool calcStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int previous_step_type,  int desired_step_type);
        void calcFBStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction);
        void calcRLStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction);
        void calcRoStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction);
        void calcStopStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction);

        void calcRightKickStep(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
				const thormang3_walking_module_msgs::StepData& ref_step_data);
        void calcLeftKickStep(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
				const thormang3_walking_module_msgs::StepData& ref_step_data);

		FootStepGenerator();
		~FootStepGenerator();

        void getStepData(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
        				const thormang3_walking_module_msgs::StepData& ref_step_data,
						int desired_step_type);

        void getStepDataFromStepData2DArray(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
        							const thormang3_walking_module_msgs::StepData& ref_step_data,
									const thormang3_foot_step_generator::Step2DArray::ConstPtr& request_step_2d);
	};


}



#endif /* REALTIMEFOOTSTEPGENERATOR_H_ */
