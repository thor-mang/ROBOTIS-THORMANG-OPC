/*
 * message_callback.h
 *
 *  Created on: 2016. 2. 20.
 *      Author: HJSONG
 */

#ifndef SRC_ROBOTIS_THORMANG_OPC_THOMANG3_FOOT_STEP_GENERATOR_INCLUDE_THOMAMG3_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_
#define SRC_ROBOTIS_THORMANG_OPC_THOMANG3_FOOT_STEP_GENERATOR_INCLUDE_THOMAMG3_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_


#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <yaml-cpp/yaml.h>

#include "thormang3_foot_step_generator/FootStepCommand.h"
#include "thormang3_walking_module_msgs/RobotPose.h"
#include "thormang3_walking_module_msgs/GetReferenceStepData.h"
#include "thormang3_walking_module_msgs/AddStepDataArray.h"
#include "thormang3_walking_module_msgs/WalkingStart.h"
#include "thormang3_walking_module_msgs/SetBalanceParam.h"
#include "thormang3_walking_module_msgs/IsRunning.h"
#include "thormang3_walking_module_msgs/RemoveExistingStepData.h"

#include "WalkingModuleCommon.h"
#include "FootStepGenerator.h"


void Initialize();

void WalkingModuleStatusMSGCallback(const std_msgs::String::ConstPtr& msg);

void WalkingCommandCallback(const thormang3_foot_step_generator::FootStepCommand::ConstPtr &msg);

bool LoadBalanceParam(thormang3_walking_module_msgs::SetBalanceParam& _set_param);
void BalanceCommandCallback(const std_msgs::Bool::ConstPtr &msg);

#endif /* SRC_ROBOTIS_THORMANG_OPC_THOMANG3_FOOT_STEP_GENERATOR_INCLUDE_THOMAMG3_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_ */
