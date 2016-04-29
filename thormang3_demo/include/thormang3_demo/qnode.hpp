/**
 * @file /include/thormang3_demo/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef thormang3_demo_QNODE_HPP_
#define thormang3_demo_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <sstream>

#include <QThread>
#include <QStringListModel>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/GetJointModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "thormang3_feet_ft_module_msgs/BothWrench.h"

// manipulation demo
#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/DemoPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
#include "thormang3_manipulation_module_msgs/GetJointPose.h"
#include "thormang3_manipulation_module_msgs/GetKinematicsPose.h"

// walking demo
#include "thormang3_foot_step_generator/FootStepCommand.h"
#include "thormang3_foot_step_generator/Step2DArray.h"
#include <std_msgs/Bool.h>
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/MarkerArray.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace thor3_control {

/*****************************************************************************
** Class
*****************************************************************************/

class QNodeThor3 : public QThread {
        Q_OBJECT
    public:
        QNodeThor3(int argc, char** argv );
        virtual ~QNodeThor3();
        bool init();
        void run();

        /*********************
        ** Logging
        **********************/
        enum LogLevel {
            Debug = 0,
            Info = 1,
            Warn = 2,
            Error = 3,
            Fatal = 4
        };

        enum CONTROL_INDEX
        {
            Control_None = 0,
            Control_Walking = 1,
            Control_Manipulation = 2,
            Control_Head = 3,
        };

        std::map< int, std::string > module_table;
        std::map< int, std::string> motion_table;

        QStringListModel* LoggingModel() { return &logging_model_; }
        void Log(const LogLevel &level, const std::string &msg, std::string sender="Demo");
        void ClearLog();
        void Assemble_lidar();
        void EnableControlModule(const std::string &mode);
        bool GetJointNameFromID(const int &id, std::string &joint_name);
        bool GetIDFromJointName(const std::string &joint_name, int &id);
        bool GetIDJointNameFromIndex(const int &index, int &id, std::string &joint_name);
        std::string GetModuleName(const int &index);
        int GetModuleIndex(const std::string &mode_name);
        int GetModuleTableSize();
        int GetJointTableSize();
        void ClearUsingModule();
        bool IsUsingModule(std::string module_name);
        void MoveInitPose();
        void InitFTCommand(std::string command);

        void SetHeadJoint(double pan, double tilt);

        // Manipulation
        void SendInitPoseMsg( std_msgs::String msg );
        void SendDestJointMsg( thormang3_manipulation_module_msgs::JointPose msg );
        void SendIkMsg( thormang3_manipulation_module_msgs::KinematicsPose msg );

        // Walking
        void SetWalkingCommand( thormang3_foot_step_generator::FootStepCommand msg);
        void SetWalkingBalance(bool on_command);
        void SetWalkingFootsteps();
        void ClearFootsteps();
        void MakeFootstepUsingPlanner();
        void MakeFootstepUsingPlanner(geometry_msgs::Pose foot_target);
        void VisualizePreviewFootsteps(bool clear);

    public Q_SLOTS:
        void GetJointControlModule();
        void GetJointPose( std::string joint_name );
        void GetKinematicsPose (std::string group_name );
        void GetKinematicsPoseCallback(const geometry_msgs::Pose::ConstPtr &msg);
        void SetCurrentControlUI(int mode);

    Q_SIGNALS:
        void LoggingUpdated();
        void RosShutdown();
        void UpdatePresentJointControlModules(std::vector<int> mode);

        // Manipulation
        void UpdateCurrJoint( double value );
        void UpdateCurrPos( double x , double y , double z );
        void UpdateCurrOri( double x , double y , double z , double w );

        void UpdateHeadJointsAngle(double pan, double tilt);

        // Walking
        void HavePoseToMakeFootstep();

    private:
        int init_argc;
        char** init_argv;
        bool DEBUG;
        int current_control_ui;
        enum
        {
            MODE_UI = 0,
            WALKING_UI = 1,
            MANIPULATION_UI = 2,
            HEAD_CONTROL_UI = 3,
            MOTION = 4,
        };
        geometry_msgs::Pose pose_from_ui_;

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
        ros::Publisher send_ini_pose_msg_pub;
        ros::Publisher send_des_joint_msg_pub;
        ros::Publisher send_ik_msg_pub;
        ros::Subscriber kenematics_pose_sub;
        ros::ServiceClient get_joint_pose_client;
        ros::ServiceClient get_kinematics_pose_client;

        // Walking
        ros::ServiceClient humanoidFootStepClient;
        ros::Publisher set_walking_command_pub;
        ros::Publisher set_walking_footsteps_pub;
        ros::Publisher set_walking_balance_pub;

        std::vector<geometry_msgs::Pose2D>  preview_foot_steps_;
        std::vector<int>                    preview_foot_types_;

        ros::Time start_time_;

        QStringListModel logging_model_;
        std::map<int, std::string> id_joint_table_;
        std::map<std::string, int> joint_id_table_;

        std::map<int, std::string> index_mode_table_;
        std::map<std::string, int> mode_index_table_;

        std::map<std::string, bool> using_mode_table_;


        void ParseJointNameFromYaml(const std::string &path);
        void RefreshCurrentJointControlCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg);
        void UpdateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
        void InitFTFootCallback(const thormang3_feet_ft_module_msgs::BothWrench::ConstPtr &msg);
        void StatusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);
        void PoseCallback(const geometry_msgs::Pose::ConstPtr &msg);
};

}  // namespace thormang3_demo

#endif /* thormang3_demo_QNODE_HPP_ */
