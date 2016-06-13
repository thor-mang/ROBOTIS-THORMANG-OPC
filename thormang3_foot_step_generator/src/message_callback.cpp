/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * message_callback.cpp
 *
 *  Created on: 2016. 6. 10.
 *      Author: Jay Song
 */

#include "thormang3_foot_step_generator/message_callback.h"

ros::ServiceClient   get_ref_step_data_client;
ros::ServiceClient   add_step_data_array_client;

ros::ServiceClient   is_running_client;

ros::ServiceClient  set_balance_param_client;

ros::Subscriber     walking_module_status_msg_sub;

ros::Subscriber     walking_command_sub;
ros::Subscriber     balance_command_sub;
ros::Subscriber     footsteps_2d_sub;


thormang3::FootStepGenerator thormang3_foot_stp_generator;

thormang3_walking_module_msgs::AddStepDataArray     add_step_data_array_srv;


void initialize(void)
{
  ros::NodeHandle nh;

  get_ref_step_data_client      = nh.serviceClient<thormang3_walking_module_msgs::GetReferenceStepData>("/robotis/walking/get_reference_step_data");
  add_step_data_array_client    = nh.serviceClient<thormang3_walking_module_msgs::AddStepDataArray>("/robotis/walking/add_step_data");
  set_balance_param_client      = nh.serviceClient<thormang3_walking_module_msgs::SetBalanceParam>("/robotis/walking/set_balance_param");
  is_running_client             = nh.serviceClient<thormang3_walking_module_msgs::IsRunning>("/robotis/walking/is_running");

  walking_module_status_msg_sub = nh.subscribe("/robotis/status", 10, walkingModuleStatusMSGCallback);

  walking_command_sub           = nh.subscribe("/robotis/thormang3_foot_step_generator/walking_command", 0, walkingCommandCallback);
  footsteps_2d_sub              = nh.subscribe("/robotis/thormang3_foot_step_generator/footsteps_2d",    0, step2DArrayCallback);

}

void walkingModuleStatusMSGCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg)
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

void walkingCommandCallback(const thormang3_foot_step_generator::FootStepCommand::ConstPtr &msg)
{
  ROS_INFO("[Demo]  : Walking Command");
  ROS_INFO_STREAM("  command        : " << msg->command );
  ROS_INFO_STREAM("  step_num       : " << msg->step_num );
  ROS_INFO_STREAM("  step_length    : " << msg->step_length);
  ROS_INFO_STREAM("  side_ste_lengh : " << msg->side_step_length );
  ROS_INFO_STREAM("  step_angle_rad : " << msg->step_angle_rad );

  if((msg->step_num == 0) && ( (msg->command != "left kick") && (msg->command != "right kick") ))
    return;

  thormang3_walking_module_msgs::GetReferenceStepData    get_ref_stp_data_srv;
  thormang3_walking_module_msgs::StepData                ref_step_data;
  thormang3_walking_module_msgs::AddStepDataArray        add_stp_data_srv;

  //set walking parameter
  thormang3_foot_stp_generator.fb_step_length_m_ = msg->step_length;
  thormang3_foot_stp_generator.rl_step_length_m_ = msg->side_step_length;
  thormang3_foot_stp_generator.rotate_step_angle_rad_ = msg->step_angle_rad;
  thormang3_foot_stp_generator.num_of_step_ = 2*(msg->step_num) + 2;


  //get reference step data
  if(get_ref_step_data_client.call(get_ref_stp_data_srv) == false)
  {
    ROS_ERROR("Failed to get reference step data");
    return;
  }

  ref_step_data = get_ref_stp_data_srv.response.reference_step_data;


  //calc step data
  if(msg->command == "forward")
  {
    thormang3_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, FORWARD_WALKING);
  }
  else if(msg->command == "backward")
  {
    thormang3_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, BACKWARD_WALKING);
  }
  else if(msg->command == "turn left")
  {
    thormang3_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, LEFT_ROTATING_WALKING);
  }
  else if(msg->command == "turn right")
  {
    thormang3_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, RIGHT_ROTATING_WALKING);
  }
  else if(msg->command == "right")
  {
    thormang3_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, RIGHTWARD_WALKING);
  }
  else if(msg->command == "left")
  {
    thormang3_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, LEFTWARD_WALKING);
  }
  else if(msg->command == "right kick")
  {
    if(isRunning() == true)
      return;

    thormang3_foot_stp_generator.calcRightKickStep( &add_stp_data_srv.request.step_data_array, ref_step_data);
  }
  else if(msg->command == "left kick")
  {
    if(isRunning() == true)
      return;

    thormang3_foot_stp_generator.calcLeftKickStep( &add_stp_data_srv.request.step_data_array, ref_step_data);
  }
  else if(msg->command == "stop")
  {
    thormang3_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, STOP_WALKING);
  }
  else
  {
    ROS_ERROR("[Demo]  : Invalid Command");
    return;
  }

  //set add step data srv for auto start and remove existing step data
  add_stp_data_srv.request.auto_start = true;
  add_stp_data_srv.request.remove_existing_step_data = true;

  //add step data
  if(add_step_data_array_client.call(add_stp_data_srv) == true)
  {
    int _result = add_stp_data_srv.response.result;
    if(_result== thormang3_walking_module_msgs::AddStepDataArray::Response::NO_ERROR)
    {
      ROS_INFO("[Demo]  : Succeed to add step data array");
    }
    else
    {
      ROS_ERROR("[Demo]  : Failed to add step data array");

      if(_result & thormang3_walking_module_msgs::AddStepDataArray::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE");
      if(_result & thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA");
      if(_result & thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_TIME_DATA");
      if(_result & thormang3_walking_module_msgs::AddStepDataArray::Response::ROBOT_IS_WALKING_NOW)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");

      return;
    }
  }
  else
  {
    ROS_ERROR("[Demo]  : Failed to add step data array ");
    return;
  }

}


void step2DArrayCallback(const thormang3_foot_step_generator::Step2DArray::ConstPtr& msg)
{
  thormang3_walking_module_msgs::GetReferenceStepData get_ref_stp_data_srv;
  thormang3_walking_module_msgs::StepData             ref_step_data;
  thormang3_walking_module_msgs::AddStepDataArray     add_stp_data_srv;
  thormang3_walking_module_msgs::IsRunning            is_running_srv;

  if(is_running_client.call(is_running_srv) == false)
  {
    ROS_ERROR("[Demo]  : Failed to Walking Status");
    return;
  }
  else
  {
    if(is_running_srv.response.is_running == true)
    {
      ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");
      return;
    }
  }


  //get reference step data
  if(get_ref_step_data_client.call(get_ref_stp_data_srv) == false)
  {
    ROS_ERROR("[Demo]  : Failed to get reference step data");
    return;
  }

  ref_step_data = get_ref_stp_data_srv.response.reference_step_data;


  thormang3_foot_stp_generator.getStepDataFromStepData2DArray(&add_stp_data_srv.request.step_data_array, ref_step_data, msg);

  //set add step data srv fot auto start and remove existing step data
  add_stp_data_srv.request.auto_start = true;
  add_stp_data_srv.request.remove_existing_step_data = true;

  //add step data
  if(add_step_data_array_client.call(add_stp_data_srv) == true)
  {
    int add_stp_data_srv_result = add_stp_data_srv.response.result;
    if(add_stp_data_srv_result== thormang3_walking_module_msgs::AddStepDataArray::Response::NO_ERROR)
      ROS_INFO("[Demo]  : Succeed to add step data array");
    else {
      ROS_ERROR("[Demo]  : Failed to add step data array");

      if(add_stp_data_srv_result & thormang3_walking_module_msgs::AddStepDataArray::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE");
      if(add_stp_data_srv_result & thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA");
      if(add_stp_data_srv_result & thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_TIME_DATA");
      if(add_stp_data_srv_result & thormang3_walking_module_msgs::AddStepDataArray::Response::ROBOT_IS_WALKING_NOW)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");

      return;
    }
  }
  else
  {
    ROS_ERROR("[Demo]  : Failed to add step data array ");
    return;
  }
}

bool isRunning(void)
{
  thormang3_walking_module_msgs::IsRunning is_running_srv;
  if(is_running_client.call(is_running_srv) == false)
  {
    ROS_ERROR("[Demo]  : Failed to Walking Status");
    return true;
  }
  else
  {
    if(is_running_srv.response.is_running == true)
    {
      ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");
      return true;
    }
  }
  return false;
}


