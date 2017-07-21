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
 ** Ifdefs
 *****************************************************************************/

#ifndef thormang3_demo_QNODE_HPP_
#define thormang3_demo_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN

#include <string>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include <QThread>
#include <QStringListModel>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <eigen_conversions/eigen_msg.h>

#include "humanoid_nav_msgs/PlanFootsteps.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/GetJointModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "thormang3_feet_ft_module_msgs/BothWrench.h"

#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
#include "thormang3_manipulation_module_msgs/GetJointPose.h"
#include "thormang3_manipulation_module_msgs/GetKinematicsPose.h"

#include "thormang3_walking_module_msgs/SetBalanceParam.h"
#include "thormang3_walking_module_msgs/SetJointFeedBackGain.h"

#include "thormang3_foot_step_generator/FootStepCommand.h"
#include "thormang3_foot_step_generator/Step2DArray.h"

#endif // Q_MOC_RUN
/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace thormang3_demo
{

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNodeThor3 : public QThread
{
Q_OBJECT

 public:
  enum LogLevel
  {
    Debug = 0,
    Info = 1,
    Warn = 2,
    Error = 3,
    Fatal = 4
  };

  //  enum ModuleIndex
  //  {
  //    Control_None = 0,
  //    Control_Walking = 1,
  //    Control_Manipulation = 2,
  //    Control_Head = 3,
  //  };

  QNodeThor3(int argc, char** argv);
  virtual ~QNodeThor3();

  bool init();
  void run();
  QStringListModel* loggingModel()
  {
    return &logging_model_;
  }
  void log(const LogLevel& level, const std::string& msg, std::string sender = "Demo");
  void clearLog();
  void assembleLidar();
  void enableControlModule(const std::string& mode);
  bool getJointNameFromID(const int& id, std::string& joint_name);
  bool getIDFromJointName(const std::string& joint_name, int& id);
  bool getIDJointNameFromIndex(const int& index, int& id, std::string& joint_name);
  std::string getModuleName(const int& index);
  int getModuleIndex(const std::string& mode_name);
  int getModuleTableSize();
  int getJointTableSize();
  void clearUsingModule();
  bool isUsingModule(const std::string& module_name);
  void moveInitPose();
  void initFTCommand(std::string command);

  // Head control
  void setHeadJoint(double pan, double tilt);

  // Manipulation
  void sendInitPoseMsg(std_msgs::String msg);
  void sendDestJointMsg(thormang3_manipulation_module_msgs::JointPose msg);
  void sendIkMsg(thormang3_manipulation_module_msgs::KinematicsPose msg);
  void sendGripperPosition(sensor_msgs::JointState msg);

  // Walking
  void setWalkingCommand(thormang3_foot_step_generator::FootStepCommand msg);
  void setWalkingBalance(bool on_command);
  void setWalkingBalanceParam(const double& gyro_gain, const double& ft_gain_ratio, const double& imu_time_const,
                              const double& ft_time_const);
  bool setFeedBackGain(); 

  void setWalkingFootsteps();
  void clearFootsteps();
  void makeFootstepUsingPlanner();
  void makeFootstepUsingPlanner(const geometry_msgs::Pose& target_foot_pose);
  void visualizePreviewFootsteps(bool clear);

  // motion
  void playMotion(int motion_index, bool to_action_script = true);

  // demo
  void makeInteractiveMarker(const geometry_msgs::Pose& marker_pose);
  void updateInteractiveMarker(const geometry_msgs::Pose& pose);
  void getInteractiveMarkerPose();
  void clearInteractiveMarker();
  void manipulationDemo(const int& index);
  void kickDemo(const std::string& kick_foot);

  std::map<int, std::string> module_table_;
  std::map<int, std::string> motion_table_;

  std::string package_name_;

 public Q_SLOTS:
  void getJointControlModule();
  void getJointPose(std::string joint_name);
  void getKinematicsPose(std::string group_name);
  void getKinematicsPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void setCurrentControlUI(int mode);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void updatePresentJointControlModules(std::vector<int> mode);

  // Manipulation
  void updateCurrJoint(double value);
  void updateCurrPos(double x, double y, double z);
  void updateCurrOri(double x, double y, double z, double w);

  // Head control
  void updateHeadJointsAngle(double pan, double tilt);

  // Interactive marker
  void updateDemoPoint(const geometry_msgs::Point point);
  void updateDemoPose(const geometry_msgs::Pose pose);

 private:
  enum Control_Index
  {
    MODE_UI = 0,
    WALKING_UI = 1,
    MANIPULATION_UI = 2,
    HEAD_CONTROL_UI = 3,
    MOTION_UI = 4,
    DEMO_UI = 5,
  };

  static constexpr double DEGREE2RADIAN = M_PI / 180.0;
  static constexpr double RADIAN2DEGREE = 180.0 / M_PI;

  void parseJointNameFromYaml(const std::string& path);
  void parseMotionMapFromYaml(const std::string& path);
  void refreshCurrentJointControlCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr& msg);
  void
  updateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void initFTFootCallback(const thormang3_feet_ft_module_msgs::BothWrench::ConstPtr& msg);
  void
  statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);
  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void pointStampedCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  void setBalanceParameter();
  bool loadBalanceParameterFromYaml();
  void turnOnBalance();
  void turnOffBalance();
  bool loadFeedbackGainFromYaml();

  int init_argc_;
  char** init_argv_;
  bool debug_print_;
  int current_control_ui_;

  // demo : interactive marker
  ros::Subscriber rviz_clicked_point_sub_;
  std::string frame_id_;
  std::string marker_name_;
  geometry_msgs::Pose pose_from_ui_;
  geometry_msgs::Pose current_pose_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  thormang3_walking_module_msgs::SetBalanceParam set_balance_param_srv_;
  thormang3_walking_module_msgs::SetJointFeedBackGain set_joint_feedback_gain_srv_;

  ros::Publisher init_pose_pub_;
  ros::Publisher init_ft_pub_;
  ros::Publisher module_control_pub_;
  ros::Publisher module_control_preset_pub_;
  ros::Subscriber status_msg_sub_;
  ros::Subscriber init_ft_foot_sub_;
  ros::Subscriber both_ft_foot_sub_;
  ros::Subscriber current_module_control_sub_;
  ros::ServiceClient get_module_control_client_;

  ros::Publisher marker_pub_;
  ros::Subscriber pose_sub_;

  // Head
  ros::Publisher move_lidar_pub_;
  ros::Publisher set_head_joint_angle_pub_;
  ros::Subscriber current_joint_states_sub_;

  // Manipulation
  ros::Publisher send_ini_pose_msg_pub_;
  ros::Publisher send_des_joint_msg_pub_;
  ros::Publisher send_ik_msg_pub_;
  ros::Subscriber kenematics_pose_sub_;
  ros::ServiceClient get_joint_pose_client_;
  ros::ServiceClient get_kinematics_pose_client_;

  ros::Publisher send_gripper_pub_;

  // Walking
  ros::ServiceClient humanoid_footstep_client_;
  ros::ServiceClient set_balance_param_client_;
  ros::ServiceClient set_joint_feedback_gain_client_;
  ros::Publisher set_walking_command_pub_;
  ros::Publisher set_walking_footsteps_pub_;
  ros::Publisher set_walking_balance_pub_;

  std::vector<geometry_msgs::Pose2D> preview_foot_steps_;
  std::vector<int> preview_foot_types_;

  // Action
  ros::Publisher motion_index_pub_;
  ros::Publisher motion_page_pub_;

  ros::Time start_time_;
  QStringListModel logging_model_;
  std::map<int, std::string> id_joint_table_;
  std::map<std::string, int> joint_id_table_;
  std::map<int, std::string> index_mode_table_;
  std::map<std::string, int> mode_index_table_;
  std::map<std::string, bool> using_mode_table_;
};

}  // namespace thormang3_demo

#endif /* thormang3_demo_QNODE_HPP_ */
