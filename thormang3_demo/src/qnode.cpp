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


/* Author: Kayman Jung */


/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/thormang3_demo/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace thor3_control {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNodeThor3::QNodeThor3(int argc, char** argv )
  : init_argc_(argc)
  , init_argv_(argv)
  , marker_name_("THORMANG3_demo_marker")
{
  // code to DEBUG
  DEBUG = false;

  if(argc >= 2)
  {
    std::string debug_code(argv[1]);
    if(debug_code == "debug")
      DEBUG = true;
    else
      DEBUG = false;
  }
}

QNodeThor3::~QNodeThor3() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNodeThor3::init()
{
  ros::init(init_argc_, init_argv_, "thormang3_demo");

  if ( ! ros::master::check() ) {
    return false;
  }

  ros::start(); // explicitly needed since our nodehandle is going out of scope.

  ros::NodeHandle nh;

  // Add your ros communications here.
  move_lidar_pub_     = nh.advertise<std_msgs::String>("/robotis/head_control/move_lidar", 0);
  module_control_pub_  = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
  module_control_preset_pub_ = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  init_pose_pub_      = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  init_ft_pub_        = nh.advertise<std_msgs::String>("/robotis/feet_ft/ft_calib_command", 0);
  set_head_joint_angle_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states", 0);

  init_ft_foot_sub_ = nh.subscribe("/robotis/feet_ft/both_ft_value", 10, &QNodeThor3::initFTFootCallback, this);

  status_msg_sub_ = nh.subscribe("/robotis/status", 10, &QNodeThor3::statusMsgCallback, this);
  current_module_control_sub_ = nh.subscribe("/robotis/present_joint_ctrl_modules", 10, &QNodeThor3::refreshCurrentJointControlCallback, this);
  current_joint_states_sub_ = nh.subscribe("/robotis/present_joint_states", 10, &QNodeThor3::updateHeadJointStatesCallback, this);

  get_module_control_client_ = nh.serviceClient<robotis_controller_msgs::GetJointModule>("/robotis/get_present_joint_ctrl_modules");

  humanoidFootStepClient = nh.serviceClient<humanoid_nav_msgs::PlanFootsteps>("plan_footsteps");
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/robotis/demo/foot_step_marker", 0);
  pose_sub_ = nh.subscribe("/robotis/demo/pose", 10, &QNodeThor3::poseCallback, this);

  // demo
  clicked_point_sub_ = nh.subscribe("clicked_point", 0, &QNodeThor3::pointStampedCallback, this);
  interactive_marker_server_.reset( new interactive_markers::InteractiveMarkerServer("THORMANG_Pose","",false) );

  // Manipulation
  kenematics_pose_sub = nh.subscribe("/thormang3_demo/ik_target_pose", 10, &QNodeThor3::getKinematicsPoseCallback, this);

  send_ini_pose_msg_pub = nh.advertise<std_msgs::String>("/robotis/manipulation/ini_pose_msg", 0);
  send_des_joint_msg_pub = nh.advertise<thormang3_manipulation_module_msgs::JointPose>("/robotis/manipulation/joint_pose_msg", 0);
  send_ik_msg_pub = nh.advertise<thormang3_manipulation_module_msgs::KinematicsPose>("/robotis/manipulation/kinematics_pose_msg", 0);

  get_joint_pose_client = nh.serviceClient<thormang3_manipulation_module_msgs::GetJointPose>("/robotis/manipulation/get_joint_pose");
  get_kinematics_pose_client = nh.serviceClient<thormang3_manipulation_module_msgs::GetKinematicsPose>("/robotis/manipulation/get_kinematics_pose");

  // Walking
  set_balance_param_client_   = nh.serviceClient<thormang3_walking_module_msgs::SetBalanceParam>("/robotis/walking/set_balance_param");
  set_walking_command_pub = nh.advertise<thormang3_foot_step_generator::FootStepCommand>("/robotis/thormang3_foot_step_generator/walking_command", 0);
  set_walking_footsteps_pub = nh.advertise<thormang3_foot_step_generator::Step2DArray>("/robotis/thormang3_foot_step_generator/footsteps_2d", 0);
  set_walking_balance_pub = nh.advertise<std_msgs::Bool>("/robotis/thormang3_foot_step_generator/balance_command", 0);

  // Action
  motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/demo/motion_index", 0);

  // Config
  std::string default_path = ros::package::getPath("thormang3_demo") +"/config/demo_config.yaml";
  std::string path = nh.param<std::string>("demo_config", default_path);
  parseJointNameFromYaml(path);

  std::string motion_path = ros::package::getPath("thormang3_demo") +"/config/motion.yaml";
  parseMotionMapFromYaml(motion_path);

  // start time
  start_time_ = ros::Time::now();

  // start qthread
  start();

  return true;
}

void QNodeThor3::run()
{
  ros::Rate loop_rate(1);

  while ( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  interactive_marker_server_.reset();

  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNodeThor3::parseJointNameFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load id_joint table yaml.");
    return;
  }

  // parse id_joint table
  YAML::Node id_sub_node = doc["id_joint"];
  for(YAML::iterator it = id_sub_node.begin() ; it != id_sub_node.end() ; ++it)
  {
    int id;
    std::string joint_name;

    id = it->first.as<int>();
    joint_name = it->second.as<std::string>();

    id_joint_table_[id] = joint_name;
    joint_id_table_[joint_name] = id;

    if(DEBUG) std::cout << "ID : " << id << " - " << joint_name << std::endl;
  }

  // parse module
  std::vector< std::string > modules = doc["module_list"].as< std::vector<std::string> >();

  int module_index = 0;
  for(std::vector<std::string>::iterator it = modules.begin() ; it != modules.end() ; ++it)
  {
    std::string module_name = *it ;

    index_mode_table_[module_index]    = module_name;
    mode_index_table_[module_name]     = module_index++;

    using_mode_table_[module_name]     = false;
  }


  // parse module_joint preset
  YAML::Node sub_node = doc["module_button"];
  for(YAML::iterator it = sub_node.begin() ; it != sub_node.end() ; ++it)
  {
    int key;
    std::string module_name;

    key = it->first.as<int>();
    module_name = it->second.as<std::string>();

    module_table_[key] = module_name;
    if(DEBUG) std::cout << "Preset : " << module_name << std::endl;
  }
}

void QNodeThor3::parseMotionMapFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load motion yaml.");
    return;
  }

  // parse motion_table
  YAML::Node motion_sub_node = doc["motion"];
  for(YAML::iterator it = motion_sub_node.begin() ; it != motion_sub_node.end() ; ++it)
  {
    int motion_index;
    std::string motion_name;

    motion_index = it->first.as<int>();
    motion_name = it->second.as<std::string>();

    motion_table_[motion_index] = motion_name;

    if(DEBUG) std::cout << "Motion Index : " << motion_index << " - " << motion_name << std::endl;
  }
}

// joint id -> joint name
bool QNodeThor3::getJointNameFromID(const int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator iter;

  iter = id_joint_table_.find(id);
  if(iter == id_joint_table_.end()) return false;

  joint_name = iter->second;
  return true;
}

// joint name -> joint id
bool QNodeThor3::getIDFromJointName(const std::string &joint_name, int &id)
{
  std::map<std::string, int>::iterator iter;

  iter = joint_id_table_.find(joint_name);
  if(iter == joint_id_table_.end()) return false;

  id = iter->second;
  return true;
}

// map index -> joint id & joint name
bool QNodeThor3::getIDJointNameFromIndex(const int &index, int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator iter;
  int count = 0;
  for(iter = id_joint_table_.begin(); iter != id_joint_table_.end(); ++iter, count++)
  {
    if(index == count)
    {
      id = iter->first;
      joint_name = iter->second;
      return true;
    }
  }
  return false;
}

// mode(module) index -> mode(module) name
std::string QNodeThor3::getModuleName(const int &index)
{
  std::string mode = "";
  std::map<int, std::string>::iterator iter = index_mode_table_.find(index);

  if(iter != index_mode_table_.end())
    mode = iter->second;

  return mode;
}

// mode(module) name -> mode(module) index
int QNodeThor3::getModuleIndex(const std::string &mode_name)
{
  int mode_index = -1;
  std::map<std::string, int>::iterator iter = mode_index_table_.find(mode_name);

  if(iter != mode_index_table_.end())
    mode_index = iter->second;

  return mode_index;
}

// number of mode(module)s
int QNodeThor3::getModuleTableSize()
{
  return index_mode_table_.size();
}

// number of joints
int QNodeThor3::getJointTableSize()
{
  return id_joint_table_.size();
}

void QNodeThor3::clearUsingModule()
{
  for(std::map<std::string, bool>::iterator iter = using_mode_table_.begin(); iter != using_mode_table_.end(); ++iter)
    iter->second = false;
}

bool QNodeThor3::isUsingModule(std::string module_name)
{
  std::map<std::string, bool>::iterator iter = using_mode_table_.find(module_name);

  if(iter == using_mode_table_.end()) return false;

  return iter->second;
}

void QNodeThor3::setCurrentControlUI(int mode)
{
  current_control_ui_ = mode;

  ROS_INFO("Current UI : %d", mode);
}

// move ini pose : base module
void QNodeThor3::moveInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub_.publish(init_msg);

  log(Info, "Go to robot initial pose.");
}

void QNodeThor3::initFTCommand(std::string command)
{
  std_msgs::String ft_msg;
  ft_msg.data = command;

  init_ft_pub_.publish(ft_msg);

}

// move head to assemble 3d lidar(pointcloud)
void QNodeThor3::assembleLidar()
{
  std_msgs::String lidar_msg;
  lidar_msg.data = "start";

  move_lidar_pub_.publish(lidar_msg);
  log(Info, "Publish move_lidar topic");
}

// enable mode(module)
void QNodeThor3::enableControlModule(const std::string &mode)
{
  std_msgs::String msg;
  msg.data = mode;

  module_control_preset_pub_.publish(msg);

  std::stringstream ss;
  ss << "Set Mode : " << mode;
  log(Info, ss.str());
}

// get current mode(module) of joints
void QNodeThor3::getJointControlModule()
{
  robotis_controller_msgs::GetJointModule get_joint;
  std::map<std::string, int> service_map;

  // get_joint.request
  std::map<int, std::string>::iterator iter;
  int index = 0;
  for(iter = id_joint_table_.begin(); iter != id_joint_table_.end(); ++iter, index++)
  {
    get_joint.request.joint_name.push_back(iter->second);
    service_map[iter->second] = index;
  }

  if(get_module_control_client_.call(get_joint))
  {
    // get_joint.response
    std::vector<int> modules;
    modules.resize(getJointTableSize());

    // clear current using modules
    clearUsingModule();

    for(int ix = 0; ix < get_joint.response.joint_name.size(); ix++)
    {
      std::string joint_name = get_joint.response.joint_name[ix];
      std::string module_name = get_joint.response.module_name[ix];

      std::map<std::string, int>::iterator service_iter = service_map.find(joint_name);
      if(service_iter == service_map.end())
        continue;

      index = service_iter->second;

      service_iter = mode_index_table_.find(module_name);
      if(service_iter == mode_index_table_.end())
        continue;

      // ROS_INFO_STREAM("joint[" << ix << "] : " << service_iter->second);
      modules.at(index) = service_iter->second;

      std::map<std::string, bool>::iterator module_iter = using_mode_table_.find(module_name);
      if(module_iter != using_mode_table_.end())
        module_iter->second = true;
    }

    // update ui
    Q_EMIT updatePresentJointControlModules(modules);
    log(Info, "Get current Mode");
  }
  else
    log(Error, "Fail to get current joint control module.");
}

void QNodeThor3::refreshCurrentJointControlCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg)
{
  ROS_INFO("set current joint module");
  int index = 0;

  std::vector<int> modules;
  modules.resize(getJointTableSize());

  std::map<std::string, int> joint_module;

  // clear current using modules
  clearUsingModule();

  for(int ix = 0; ix < msg->joint_name.size(); ix++)
  {
    std::string joint_name = msg->joint_name[ix];
    std::string module_name = msg->module_name[ix];

    joint_module[joint_name] = getModuleIndex(module_name);

    std::map<std::string, bool>::iterator module_iter = using_mode_table_.find(module_name);
    if(module_iter != using_mode_table_.end())
      module_iter->second = true;
  }

  for(int ix = 0; ix < getJointTableSize(); ix++)
  {
    int id = 0;
    std::string joint_name= "";

    if(getIDJointNameFromIndex(ix, id, joint_name) == false) continue;

    std::map<std::string, int>::iterator module_iter = joint_module.find(joint_name);
    if(module_iter == joint_module.end()) continue;

    // ROS_INFO_STREAM("joint[" << ix << "] : " << module_iter->second);
    modules.at(ix) = module_iter->second;
  }

  // update ui
  Q_EMIT updatePresentJointControlModules(modules);

  log(Info, "Applied Mode", "Manager");
}

void QNodeThor3::updateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  double head_pan, head_tilt;
  int get = 0;

  for(int ix = 0; ix < msg->name.size(); ix++)
  {
    if(msg->name[ix] == "head_y")
    {
      head_pan = - msg->position[ix];
      get += 1;
    }
    else if(msg->name[ix] == "head_p")
    {
      head_tilt = - msg->position[ix];
      get += 1;
    }

    if(get == 2) break;
  }

  if(get > 0)
    Q_EMIT updateHeadJointsAngle(head_pan, head_tilt);
}

void QNodeThor3::initFTFootCallback(const thormang3_feet_ft_module_msgs::BothWrench::ConstPtr &msg)
{
  std::stringstream ss;
  ss << "Type : " << msg->name << std::endl;
  ss << " - Right - " << std::endl << msg->right << std::endl;
  ss << " - Left - " << std::endl << msg->left;

  log( Info , ss.str() );
}

void QNodeThor3::setHeadJoint(double pan, double tilt)
{
  sensor_msgs::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_y");
  head_angle_msg.name.push_back("head_p");

  head_angle_msg.position.push_back(- pan);
  head_angle_msg.position.push_back(- tilt);

  set_head_joint_angle_pub_.publish(head_angle_msg);
}

// Manipulation
void QNodeThor3::sendInitPoseMsg( std_msgs::String msg )
{
  send_ini_pose_msg_pub.publish( msg );

  log( Info , "Send Ini. Pose" );
}

void QNodeThor3::sendDestJointMsg( thormang3_manipulation_module_msgs::JointPose msg )
{
  send_des_joint_msg_pub.publish( msg );

  log( Info , "Set Des. Joint Vale" );

  std::stringstream log_msg;

  log_msg << " \n "
          << "joint name : "
          << msg.name << " \n "
          << "joint value : "
          << msg.value * 180.0 / M_PI << " \n ";

  log( Info , log_msg.str() );
}

void QNodeThor3::sendIkMsg( thormang3_manipulation_module_msgs::KinematicsPose msg )
{
  send_ik_msg_pub.publish( msg );

  log( Info , "Solve Inverse Kinematics" );

  log( Info , "Set Des. End Effector's Pose : " );

  std::stringstream log_msgs;

  log_msgs << " \n "
           << "group name : "
           << msg.name << " \n "
           << "des. pos. x : "
           << msg.pose.position.x << " \n "
           << "des. pos. y : "
           << msg.pose.position.y << " \n "
           << "des. pos. z : "
           << msg.pose.position.z << " \n "
           << "des. ori. x : "
           << msg.pose.orientation.x << " \n "
           << "des. ori. y : "
           << msg.pose.orientation.y << " \n "
           << "des. ori. z : "
           << msg.pose.orientation.z << " \n "
           << "des. ori. w : "
           << msg.pose.orientation.w << " \n ";

  log( Info , log_msgs.str() );
}

void QNodeThor3::getJointPose( std::string joint_name )
{
  thormang3_manipulation_module_msgs::GetJointPose get_joint_pose;

  // requeset
  get_joint_pose.request.joint_name = joint_name;

  log( Info , "Get Curr. Joint Value" );

  std::stringstream log_msg;

  log_msg << " \n "
          << "joint name : "
          << joint_name << " \n " ;

  log( Info , log_msg.str() );

  //response
  if( get_joint_pose_client.call( get_joint_pose ) )
  {
    double joint_value = get_joint_pose.response.joint_value;

    log( Info , "Joint Curr. Value" );

    std::stringstream log_msg;

    log_msg << " \n "
            << "curr. value : "
            << joint_value << " \n ";

    log( Info , log_msg.str() );

    Q_EMIT updateCurrJoint( joint_value );
  }
  else
    log(Error, "fail to get joint pose.");
}

void QNodeThor3::getKinematicsPose (std::string group_name )
{
  thormang3_manipulation_module_msgs::GetKinematicsPose get_kinematics_pose;

  //request
  get_kinematics_pose.request.group_name = group_name;

  log( Info , "Solve Forward Kinematics" );

  log( Info , "Get Curr. End Effector's Pose" );

  std::stringstream log_msg;

  log_msg << " \n "
          << "group name : "
          << group_name << " \n " ;

  log( Info , log_msg.str() );

  //response
  if ( get_kinematics_pose_client.call( get_kinematics_pose ) )
  {
    double pos_x = get_kinematics_pose.response.group_pose.position.x;
    double pos_y = get_kinematics_pose.response.group_pose.position.y;
    double pos_z = get_kinematics_pose.response.group_pose.position.z;

    double ori_x = get_kinematics_pose.response.group_pose.orientation.x;
    double ori_y = get_kinematics_pose.response.group_pose.orientation.y;
    double ori_z = get_kinematics_pose.response.group_pose.orientation.z;
    double ori_w = get_kinematics_pose.response.group_pose.orientation.w;

    log( Info , "End Effector Curr. Pose : " );

    std::stringstream log_msg;

    log_msg << " \n "
            << "curr. pos. x : "
            << pos_x << " \n "
            << "curr. pos. y : "
            << pos_y << " \n "
            << "curr. pos. z : "
            << pos_z << " \n "
            << "curr. ori. w : "
            << ori_w << " \n "
            << "curr. ori. x : "
            << ori_x << " \n "
            << "curr. ori. y : "
            << ori_y << " \n "
            << "curr. ori. z : "
            << ori_z << " \n ";

    log( Info , log_msg.str() );

    Q_EMIT updateCurrPos( pos_x , pos_y , pos_z );
    Q_EMIT updateCurrOri( ori_x , ori_y , ori_z , ori_w );
  }
  else
    log(Error, "fail to get kinematics pose.");
}

void QNodeThor3::getKinematicsPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  double z_offset = 0.801;
  Q_EMIT updateCurrPos( msg->position.x , msg->position.y , msg->position.z + z_offset);
  Q_EMIT updateCurrOri( msg->orientation.x , msg->orientation.y , msg->orientation.z , msg->orientation.w );
}

// Walking
void QNodeThor3::setWalkingCommand(thormang3_foot_step_generator::FootStepCommand msg)
{
  set_walking_command_pub.publish(msg);

  std::stringstream ss;
  ss << "Set Walking Command : " << msg.command << std::endl;
  ss << "- Number of Step : " << msg.step_num << std::endl;
  ss << "- Step Length : " << msg.step_length << std::endl;
  ss << "- Side Step Length : " << msg.side_step_length << std::endl;
  ss << "- Rotation Angle : " << msg.step_angle_rad << std::endl;

  log(Info, ss.str());
}

void QNodeThor3::setWalkingBalance(bool on_command)
{
  //  std_msgs::Bool msg;
  //  msg.data = on_command;

  //  set_walking_balance_pub.publish(msg);

  //  std::stringstream ss;
  //  ss << "Set Walking Balance : " << (on_command ? "True" : "False");

  //  log(Info, ss.str());

  if(on_command == true) turnOnBalance();
  else turnOffBalance();
}

void QNodeThor3::setWalkingBalanceParam(const double &gyro_gain, const double &ft_gain_ratio, const double &imu_time_const, const double &ft_time_const)
{
  // todo : make the walking balance param msg and fill it, sent to the mpc of thormange3
  if(loadBalanceParameterFromYaml() == false)
  {
    return;
  }

  set_balance_param_srv_.request.updating_duration        = 2.0;

  set_balance_param_srv_.request.balance_param.gyro_gain               = gyro_gain;
  set_balance_param_srv_.request.balance_param.foot_x_force_gain      *= ft_gain_ratio;
  set_balance_param_srv_.request.balance_param.foot_y_force_gain      *= ft_gain_ratio;
  set_balance_param_srv_.request.balance_param.foot_z_force_gain      *= ft_gain_ratio;
  set_balance_param_srv_.request.balance_param.foot_roll_torque_gain  *= ft_gain_ratio;
  set_balance_param_srv_.request.balance_param.foot_pitch_torque_gain *= ft_gain_ratio;

  set_balance_param_srv_.request.balance_param.foot_roll_angle_time_constant  = imu_time_const;
  set_balance_param_srv_.request.balance_param.foot_pitch_angle_time_constant = imu_time_const;

  set_balance_param_srv_.request.balance_param.foot_x_force_time_constant      = ft_time_const;
  set_balance_param_srv_.request.balance_param.foot_y_force_time_constant      = ft_time_const;
  set_balance_param_srv_.request.balance_param.foot_z_force_time_constant      = ft_time_const;
  set_balance_param_srv_.request.balance_param.foot_roll_torque_time_constant  = ft_time_const;
  set_balance_param_srv_.request.balance_param.foot_pitch_torque_time_constant = ft_time_const;

  setBalanceParameter();
}

void QNodeThor3::setWalkingFootsteps()
{
  if(preview_foot_steps_.size() != preview_foot_types_.size())
  {
    log(Error, "Footsteps are corrupted.");
    return;
  }
  else if(preview_foot_steps_.size() == 0)
  {
    log(Warn, "No Footsteps");
    return;
  }

  thormang3_foot_step_generator::Step2DArray footsteps;

  for(int ix = 0; ix < preview_foot_steps_.size(); ix++)
  {
    thormang3_foot_step_generator::Step2D step;

    int type = preview_foot_types_[ix];
    if(type == humanoid_nav_msgs::StepTarget::right) step.moving_foot = thormang3_foot_step_generator::Step2D::RFootMove;
    else if(type == humanoid_nav_msgs::StepTarget::left) step.moving_foot = thormang3_foot_step_generator::Step2D::LFootMove;
    else step.moving_foot = thormang3_foot_step_generator::Step2D::NFootMove;

    step.step2d = preview_foot_steps_[ix];

    footsteps.footsteps_2d.push_back(step);
  }

  set_walking_footsteps_pub.publish(footsteps);
  log(Info, "Set command to walk using footsteps");

  clearFootsteps();
}

void QNodeThor3::clearFootsteps()
{
  visualizePreviewFootsteps(true);

  preview_foot_steps_.clear();
  preview_foot_types_.clear();
}

void QNodeThor3::makeFootstepUsingPlanner()
{
  makeFootstepUsingPlanner(pose_from_ui_);
}

void QNodeThor3::makeFootstepUsingPlanner(geometry_msgs::Pose foot_target)
{
  //foot step service
  humanoid_nav_msgs::PlanFootsteps get_step;

  geometry_msgs::Pose2D start;
  geometry_msgs::Pose2D goal;
  goal.x = foot_target.position.x;
  goal.y = foot_target.position.y;

  Eigen::Quaterniond goal_orientation;
  tf::quaternionMsgToEigen(foot_target.orientation, goal_orientation);

  Eigen::Vector3d forward, f_x(1, 0, 0);
  forward = goal_orientation.toRotationMatrix() * f_x;
  double theta = forward.y() > 0 ? acos(forward.x()) : - acos(forward.x());
  goal.theta = theta;

  get_step.request.start = start;
  get_step.request.goal = goal;

  std::stringstream call_msg;
  call_msg << "Start [" << start.x << ", " << start.y << " | " << start.theta << "]"
           << " , Goal [" << goal.x << ", " << goal.y << " | " << goal.theta << "]";
  log(Info, call_msg.str());

  // clear visualization
  visualizePreviewFootsteps(true);

  // init foot steps
  preview_foot_steps_.clear();
  preview_foot_types_.clear();

  if(humanoidFootStepClient.call(get_step))
  {
    if(get_step.response.result)
    {
      for(int ix = 0; ix < get_step.response.footsteps.size(); ix++)
      {
        // foot step log
        std::stringstream msg;
        int foot_type = get_step.response.footsteps[ix].leg;
        std::string foot = foot_type == humanoid_nav_msgs::StepTarget::right ? "right" : "left";
        geometry_msgs::Pose2D foot_pose = get_step.response.footsteps[ix].pose;

        // log footsteps
        msg << "Foot Step #" << ix + 1 << " [ " << foot << "] - ["
            << foot_pose.x << ", " << foot_pose.y << " | " << (foot_pose.theta * 180 / M_PI) << "]";
        log(Info, msg.str());

        preview_foot_steps_.push_back(foot_pose);
        preview_foot_types_.push_back(foot_type);
      }

      // visualize foot steps
      visualizePreviewFootsteps(false);
    }
    else
    {
      log(Info, "fail to get foot step from planner");
      return;
    }
  }
  else
  {
    log(Error, "cannot call service");
    return;
  }

  return;
}

void QNodeThor3::visualizePreviewFootsteps(bool clear)
{
  if(clear && preview_foot_steps_.size() == 0) return;

  visualization_msgs::MarkerArray marker_array;
  ros::Time now = ros::Time::now();
  visualization_msgs::Marker rviz_marker;

  rviz_marker.header.frame_id = "pelvis_link";
  rviz_marker.header.stamp = now;
  rviz_marker.ns = "foot_step_marker";

  rviz_marker.id = 1;
  rviz_marker.type = visualization_msgs::Marker::CUBE;
  rviz_marker.action = !clear ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

  rviz_marker.scale.x = 0.216;
  rviz_marker.scale.y = 0.144;
  rviz_marker.scale.z = 0.01;

  double alpha = 0.7;
  double height = -0.723;

  for(int ix = preview_foot_types_.size(); ix > 0; ix--)
  {
    // foot step marker array
    rviz_marker.id += 10;

    if(!clear)
    {
      Eigen::Vector3d marker_position(preview_foot_steps_[ix - 1].x, preview_foot_steps_[ix - 1].y, height);
      Eigen::Vector3d marker_position_offset;

      Eigen::Vector3d toward(1, 0, 0), direction(cos(preview_foot_steps_[ix - 1].theta), sin(preview_foot_steps_[ix - 1].theta), 0);
      Eigen::Quaterniond marker_orientation(Eigen::Quaterniond::FromTwoVectors(toward, direction));

      if(DEBUG)
      {
        std::stringstream msg;
        msg << "Foot Step #" << ix << " [ " << preview_foot_types_[ix - 1] << "] - ["
            << rviz_marker.pose.position.x << ", " << rviz_marker.pose.position.y << "]";
        log(Info, msg.str());
      }
      alpha *= 0.9;

      // set foot step color
      if(preview_foot_types_[ix - 1] == humanoid_nav_msgs::StepTarget::left)             // left
      {
        rviz_marker.color.r = 0.0;
        rviz_marker.color.g = 0.0;
        rviz_marker.color.b = 1.0;
        rviz_marker.color.a = alpha + 0.3;

        Eigen::Vector3d offset_y(0, 0.015, 0);
        marker_position_offset = marker_orientation.toRotationMatrix() * offset_y;

      }
      else if(preview_foot_types_[ix - 1] == humanoid_nav_msgs::StepTarget::right)       //right
      {
        rviz_marker.color.r = 1.0;
        rviz_marker.color.g = 0.0;
        rviz_marker.color.b = 0.0;
        rviz_marker.color.a = alpha + 0.3;

        Eigen::Vector3d offset_y(0, -0.015, 0);
        marker_position_offset = marker_orientation.toRotationMatrix() * offset_y;
      }

      marker_position = marker_position_offset + marker_position;

      tf::pointEigenToMsg(marker_position, rviz_marker.pose.position);
      tf::quaternionEigenToMsg(marker_orientation, rviz_marker.pose.orientation);

      // apply foot x offset
    }

    marker_array.markers.push_back(rviz_marker);
  }

  // publish foot step marker array
  if(clear == false) log(Info, "Visualize Preview Footstep Marker Array");
  else log(Info, "Clear Visualize Preview Footstep Marker Array");

  marker_pub_.publish(marker_array);
}


void QNodeThor3::setBalanceParameter()
{
  if(set_balance_param_client_.call(set_balance_param_srv_) == true)
  {
    int _result = set_balance_param_srv_.response.result;
    if( _result == thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR)
    {
      ROS_INFO("[Demo]  : Succeed to set balance param");
      ROS_INFO("[Demo]  : Please wait 2 sec for turning on balance");
    }
    else
    {
      if(_result & thormang3_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE");
      if(_result & thormang3_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED)
        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED");
      if(_result & thormang3_walking_module_msgs::SetBalanceParam::Response::TIME_CONST_IS_ZERO_OR_NEGATIVE)
        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE");
    }
  }
  else
    ROS_ERROR("[Demo]  : Failed to set balance param ");
}

bool QNodeThor3::loadBalanceParameterFromYaml()
{
  std::string balance_yaml_path = "";
  balance_yaml_path = ros::package::getPath("thormang3_demo") + "/config/balance_param.yaml";

  YAML::Node _doc;
  try
  {
    // load yaml
    _doc = YAML::LoadFile(balance_yaml_path.c_str());
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Failed to load balance param yaml file.");
    return false;
  }

  double cob_x_offset_m                   = _doc["cob_x_offset_m"].as<double>();
  double cob_y_offset_m                   = _doc["cob_y_offset_m"].as<double>();
  double hip_roll_swap_angle_rad          = _doc["hip_roll_swap_angle_rad"].as<double>();
  double gyro_gain                        = _doc["gyro_gain"].as<double>();
  double foot_roll_angle_gain             = _doc["foot_roll_angle_gain"].as<double>();
  double foot_pitch_angle_gain            = _doc["foot_pitch_angle_gain"].as<double>();
  double foot_x_force_gain                = _doc["foot_x_force_gain"].as<double>();
  double foot_y_force_gain                = _doc["foot_y_force_gain"].as<double>();
  double foot_z_force_gain                = _doc["foot_z_force_gain"].as<double>();
  double foot_roll_torque_gain            = _doc["foot_roll_torque_gain"].as<double>();
  double foot_pitch_torque_gain           = _doc["foot_pitch_torque_gain"].as<double>();
  double foot_roll_angle_time_constant    = _doc["foot_roll_angle_time_constant"].as<double>();
  double foot_pitch_angle_time_constant   = _doc["foot_pitch_angle_time_constant"].as<double>();
  double foot_x_force_time_constant       = _doc["foot_x_force_time_constant"].as<double>();
  double foot_y_force_time_constant       = _doc["foot_y_force_time_constant"].as<double>();
  double foot_z_force_time_constant       = _doc["foot_z_force_time_constant"].as<double>();
  double foot_roll_torque_time_constant   = _doc["foot_roll_torque_time_constant"].as<double>();
  double foot_pitch_torque_time_constant  = _doc["foot_pitch_torque_time_constant"].as<double>();

  set_balance_param_srv_.request.updating_duration                             = 2.0;
  set_balance_param_srv_.request.balance_param.cob_x_offset_m                  = cob_x_offset_m;
  set_balance_param_srv_.request.balance_param.cob_y_offset_m                  = cob_y_offset_m;
  set_balance_param_srv_.request.balance_param.hip_roll_swap_angle_rad         = hip_roll_swap_angle_rad;
  set_balance_param_srv_.request.balance_param.gyro_gain                       = gyro_gain;
  set_balance_param_srv_.request.balance_param.foot_roll_angle_gain            = foot_roll_angle_gain;
  set_balance_param_srv_.request.balance_param.foot_pitch_angle_gain           = foot_pitch_angle_gain;
  set_balance_param_srv_.request.balance_param.foot_x_force_gain               = foot_x_force_gain;
  set_balance_param_srv_.request.balance_param.foot_y_force_gain               = foot_y_force_gain;
  set_balance_param_srv_.request.balance_param.foot_z_force_gain               = foot_z_force_gain;
  set_balance_param_srv_.request.balance_param.foot_roll_torque_gain           = foot_roll_torque_gain;
  set_balance_param_srv_.request.balance_param.foot_pitch_torque_gain          = foot_pitch_torque_gain;
  set_balance_param_srv_.request.balance_param.foot_roll_angle_time_constant   = foot_roll_angle_time_constant;
  set_balance_param_srv_.request.balance_param.foot_pitch_angle_time_constant  = foot_pitch_angle_time_constant;
  set_balance_param_srv_.request.balance_param.foot_x_force_time_constant      = foot_x_force_time_constant;
  set_balance_param_srv_.request.balance_param.foot_y_force_time_constant      = foot_y_force_time_constant;
  set_balance_param_srv_.request.balance_param.foot_z_force_time_constant      = foot_z_force_time_constant;
  set_balance_param_srv_.request.balance_param.foot_roll_torque_time_constant  = foot_roll_torque_time_constant;
  set_balance_param_srv_.request.balance_param.foot_pitch_torque_time_constant = foot_pitch_torque_time_constant;

  return true;
}

void QNodeThor3::turnOnBalance()
{
  if(loadBalanceParameterFromYaml() == false)
  {
    return;
  }

  setBalanceParameter();

  log(Info, "Turn On Walking Balance");
}

void QNodeThor3::turnOffBalance()
{
  if(loadBalanceParameterFromYaml() == false)
  {
    return;
  }

  set_balance_param_srv_.request.updating_duration                             = 2.0;
  set_balance_param_srv_.request.balance_param.gyro_gain                       = 0.0;
  set_balance_param_srv_.request.balance_param.foot_roll_angle_gain            = 0.0;
  set_balance_param_srv_.request.balance_param.foot_pitch_angle_gain           = 0.0;
  set_balance_param_srv_.request.balance_param.foot_x_force_gain               = 0.0;
  set_balance_param_srv_.request.balance_param.foot_y_force_gain               = 0.0;
  set_balance_param_srv_.request.balance_param.foot_z_force_gain               = 0.0;
  set_balance_param_srv_.request.balance_param.foot_roll_torque_gain           = 0.0;
  set_balance_param_srv_.request.balance_param.foot_pitch_torque_gain          = 0.0;

  setBalanceParameter();

  log(Info, "Turn Off Walking Balance");
}


// Motion
void QNodeThor3::playMotion(int motion_index)
{
  if(motion_table_.find(motion_index) == motion_table_.end())
  {
    log(Error, "Motion index is not valid.");
    return;
  }

  std::stringstream ss;
  switch(motion_index)
  {
    case -2:
      ss << "Break Motion";
      break;

    case -1:
      ss << "STOP Motion";
      break;

    default:
      std::string motion_name = motion_table_[motion_index];
      ss << "Play Motion : [" << motion_index << "] " << motion_name;
  }

  // publish motion index
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_.publish(motion_msg);

  log(Info, ss.str());
}

void QNodeThor3::poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  switch(current_control_ui_)
  {
    case WALKING_UI:
    {
      pose_from_ui_ = *msg;
      Q_EMIT havePoseToMakeFootstep();
      log(Info, "Get Pose For Step");
      break;
    }

    case MANIPULATION_UI:
    {
      double z_offset = 0.801;
      Q_EMIT updateCurrPos( msg->position.x , msg->position.y , msg->position.z + z_offset);
      Q_EMIT updateCurrOri( msg->orientation.x , msg->orientation.y , msg->orientation.z , msg->orientation.w );
      log(Info, "Get Pose For IK");
      break;
    }

    default:
      break;
  }
}

// demo
void QNodeThor3::pointStampedCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  ROS_INFO("clicked_point");

  frame_id_ = msg->header.frame_id;

  // update point ui
  Q_EMIT updateDemoPoint(msg->point);
}

void QNodeThor3::interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // event
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      current_pose_ = feedback->pose;

      // update pose ui
      Q_EMIT updateDemoPose(feedback->pose);

      break;
    }
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      break;
  }

  interactive_marker_server_->applyChanges();
}

void QNodeThor3::makeInteractiveMarker(const geometry_msgs::Pose &pose)
{
  if(frame_id_ == "")
  {
    ROS_ERROR("No frame id!!!");
    // return;

    frame_id_ = "world";
  }

  ROS_INFO_STREAM("Make Interactive Marker! - " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z
                  << " [" <<  pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << " | " << pose.orientation.w << "]");

  interactive_marker_server_->clear();

  visualization_msgs::InteractiveMarker _interactive_marker;
  _interactive_marker.pose = pose;    // set pose


  // Visualize Interactive Marker
  _interactive_marker.header.frame_id = frame_id_;
  _interactive_marker.scale = 0.3;

  _interactive_marker.name = marker_name_; //"pose_marker";
  _interactive_marker.description = "3D Pose Control";

  // ----- center marker
  visualization_msgs::InteractiveMarkerControl _interactive_marker_control;

  _interactive_marker_control.always_visible = true;

  visualization_msgs::Marker _marker;

  _marker.type = visualization_msgs::Marker::CUBE;

  // center cube
  _marker.scale.x = 0.03;
  _marker.scale.y = 0.03;
  _marker.scale.z = 0.03;

  _marker.color.r = 1.0;
  _marker.color.g = 0.5;
  _marker.color.b = 0.5;
  _marker.color.a = 1.0;

  _interactive_marker_control.markers.push_back(_marker);

  // axis x
  _marker.pose.position.x = 0.05;
  _marker.pose.position.y = 0.0;
  _marker.pose.position.z = 0.0;

  _marker.scale.x = 0.1;
  _marker.scale.y = 0.01;
  _marker.scale.z = 0.01;

  _marker.color.r = 1.0;
  _marker.color.g = 0.0;
  _marker.color.b = 0.0;
  _marker.color.a = 1.0;

  _interactive_marker_control.markers.push_back(_marker);

  // axis y
  _marker.pose.position.x = 0.0;
  _marker.pose.position.y = 0.05;
  _marker.pose.position.z = 0.0;

  _marker.scale.x = 0.01;
  _marker.scale.y = 0.1;
  _marker.scale.z = 0.01;

  _marker.color.r = 0.0;
  _marker.color.g = 1.0;
  _marker.color.b = 0.0;
  _marker.color.a = 1.0;

  _interactive_marker_control.markers.push_back(_marker);

  // axis z
  _marker.pose.position.x = 0.0;
  _marker.pose.position.y = 0.0;
  _marker.pose.position.z = 0.05;

  _marker.scale.x = 0.01;
  _marker.scale.y = 0.01;
  _marker.scale.z = 0.1;

  _marker.color.r = 0.0;
  _marker.color.g = 0.0;
  _marker.color.b = 1.0;
  _marker.color.a = 1.0;

  _interactive_marker_control.markers.push_back(_marker);

  _interactive_marker.controls.push_back(_interactive_marker_control);

  // ----- controller
  visualization_msgs::InteractiveMarkerControl _interactive_control;
  _interactive_marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;

  // move and rotate along axis x : default
  _interactive_control.orientation.x = 1;
  _interactive_control.orientation.y = 0;
  _interactive_control.orientation.z = 0;
  _interactive_control.orientation.w = 1;
  _interactive_control.name = "rotate";
  _interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  _interactive_marker.controls.push_back(_interactive_control);
  _interactive_control.name = "move";
  _interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  _interactive_marker.controls.push_back(_interactive_control);

  // move and rotate along axis y
  _interactive_control.orientation.x = 0;
  _interactive_control.orientation.y = 1;
  _interactive_control.orientation.z = 0;
  _interactive_control.orientation.w = 1;
  _interactive_control.name = "rotate";
  _interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  _interactive_marker.controls.push_back(_interactive_control);
  _interactive_control.name = "move";
  _interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  _interactive_marker.controls.push_back(_interactive_control);

  // move and rotate along axis z
  _interactive_control.orientation.x = 0;
  _interactive_control.orientation.y = 0;
  _interactive_control.orientation.z = 1;
  _interactive_control.orientation.w = 1;
  _interactive_control.name = "rotate";
  _interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  _interactive_marker.controls.push_back(_interactive_control);
  _interactive_control.name = "move";
  _interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  _interactive_marker.controls.push_back(_interactive_control);

  interactive_marker_server_->insert(_interactive_marker);
  interactive_marker_server_->setCallback(_interactive_marker.name, boost::bind(&QNodeThor3::interactiveMarkerFeedback, this, _1));

  interactive_marker_server_->applyChanges();
}

void QNodeThor3::updateInteractiveMarker(const geometry_msgs::Pose &pose)
{
  ROS_INFO("Update Interactive Marker Pose");

  visualization_msgs::InteractiveMarker _interactive_marker;
  if(!(interactive_marker_server_->get(marker_name_, _interactive_marker)))
  {
    ROS_ERROR("No Interactive marker to set pose");
    return;
  }

  interactive_marker_server_->setPose(_interactive_marker.name, pose);
  interactive_marker_server_->applyChanges();
}

void QNodeThor3::getInteractiveMarkerPose()
{
  ROS_INFO("Get Interactive Marker Pose");

  visualization_msgs::InteractiveMarker _interactive_marker;
  if(!(interactive_marker_server_->get(marker_name_, _interactive_marker)))
  {
    ROS_ERROR("No Interactive marker to get pose");
    return;
  }

  // update pose ui
  Q_EMIT updateDemoPose(_interactive_marker.pose);

  clearInteractiveMarker();
}

void QNodeThor3::clearInteractiveMarker()
{
  ROS_INFO("Clear Interactive Marker");

  // clear and apply
  interactive_marker_server_->clear();
  interactive_marker_server_->applyChanges();
}

void QNodeThor3::manipulationDemo(const int &index)
{
  switch(index)
  {
    case 2:
    {
      enableControlModule("head control");

      usleep(10 * 1000);

      // scan
      assembleLidar();
    }

    default:
      break;

  }
}

void QNodeThor3::kickDemo(const std::string &kick_foot)
{
  if(kick_foot == "right kick")
  {
    if(loadBalanceParameterFromYaml() == false)
        return;

    double old_hip_swap = set_balance_param_srv_.request.balance_param.hip_roll_swap_angle_rad;
    set_balance_param_srv_.request.balance_param.hip_roll_swap_angle_rad = 0;
    set_balance_param_srv_.request.balance_param.cob_x_offset_m -= 0.03;
    set_balance_param_srv_.request.balance_param.cob_y_offset_m += 0.02;
    setBalanceParameter();

    thormang3_foot_step_generator::FootStepCommand msg;
    msg.command = kick_foot;
    setWalkingCommand(msg);

    usleep(7.2 * 1000 * 1000);

    set_balance_param_srv_.request.balance_param.hip_roll_swap_angle_rad = old_hip_swap;
    set_balance_param_srv_.request.balance_param.cob_x_offset_m += 0.03;
    set_balance_param_srv_.request.balance_param.cob_y_offset_m -= 0.02;
    setBalanceParameter();
    usleep(2 * 1000*1000);
  }
  else if(kick_foot == "left kick")
  {
    if(loadBalanceParameterFromYaml() == false)
        return;

    double old_hip_swap = set_balance_param_srv_.request.balance_param.hip_roll_swap_angle_rad;
    set_balance_param_srv_.request.balance_param.cob_x_offset_m -= 0.03;
    set_balance_param_srv_.request.balance_param.cob_y_offset_m -= 0.02;
    setBalanceParameter();

    thormang3_foot_step_generator::FootStepCommand msg;
    msg.command = kick_foot;
    setWalkingCommand(msg);

    usleep(7.2 * 1000 * 1000);

    set_balance_param_srv_.request.balance_param.hip_roll_swap_angle_rad = old_hip_swap;
    set_balance_param_srv_.request.balance_param.cob_x_offset_m += 0.03;
    set_balance_param_srv_.request.balance_param.cob_y_offset_m += 0.02;
    setBalanceParameter();
    usleep(2 * 1000*1000);
  }
}

// LOG
void QNodeThor3::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  log((LogLevel) msg->type, msg->status_msg, msg->module_name);
}

void QNodeThor3::log( const LogLevel &level, const std::string &msg, std::string sender)
{
  logging_model_.insertRows(logging_model_.rowCount(),1);
  std::stringstream logging_model_msg;

  ros::Duration duration_time = ros::Time::now() - start_time_;
  int current_time = duration_time.sec;
  int min_time = 0, sec_time = 0;
  min_time = (int)(current_time / 60);
  sec_time = (int)(current_time % 60);

  std::stringstream min_str, sec_str;
  if(min_time < 10) min_str << "0";
  if(sec_time < 10) sec_str << "0";
  min_str << min_time;
  sec_str << sec_time;

  std::stringstream stream_sender;
  stream_sender << "[" << sender << "] ";

  switch ( level ) {
    case(Debug) :
    {
      ROS_DEBUG_STREAM(msg);
      logging_model_msg << "[DEBUG] [" << min_str.str() << ":" << sec_str.str() << "]: " << stream_sender.str() << msg;
      break;
    }
    case(Info) :
    {
      ROS_INFO_STREAM(msg);
      logging_model_msg << "[INFO] [" << min_str.str() << ":" << sec_str.str() << "]: " << stream_sender.str() << msg;
      break;
    }
    case(Warn) :
    {
      ROS_WARN_STREAM(msg);
      logging_model_msg << "[WARN] [" << min_str.str() << ":" << sec_str.str() << "]: " << stream_sender.str() <<msg;
      break;
    }
    case(Error) :
    {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "<ERROR> [" << min_str.str() << ":" << sec_str.str() << "]: " << stream_sender.str() <<msg;
      break;
    }
    case(Fatal) :
    {
      ROS_FATAL_STREAM(msg);
      logging_model_msg << "[FATAL] [" << min_str.str() << ":" << sec_str.str() << "]: " << stream_sender.str() <<msg;
      break;
    }
  }

  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNodeThor3::clearLog()
{
  if(logging_model_.rowCount() == 0) return;

  logging_model_.removeRows(0, logging_model_.rowCount());
}

}  // namespace thor3_control
