//#include <yaml-cpp/yaml.h>

#include "thormang3_test/thormang3_test.h"

namespace thormang3
{

THORMANG3::THORMANG3()
 : ros_node_()
{

}

THORMANG3::~THORMANG3()
{

}

void THORMANG3::initialize()
{
  walking_pub_ = ros_node_.advertise<thormang3_foot_step_generator::FootStepCommand>
      ("/robotis/thormang3_foot_step_generator/walking_command", 0);
  manipulation_pub_ = ros_node_.advertise<thormang3_manipulation_module_msgs::KinematicsPose>
      ("/robotis/manipulation/kinematics_pose_msg", 0);
}

void THORMANG3::sendWalkingCommand()
{
  thormang3_foot_step_generator::FootStepCommand msg;

  msg.command = "forward";
  msg.step_num = 2;
  msg.step_time = 1.0;
  msg.step_length = 0.0;
  msg.side_step_length = 0.0;
  msg.step_angle_rad = 0.0;

  walking_pub_.publish(msg);
}

void THORMANG3::sendManipulationCommand(int level)
{
  thormang3_manipulation_module_msgs::KinematicsPose msg;

  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;

  if (level == LeftForward)
  {
    msg.name = "left_arm";

    msg.pose.position.x = 0.3 + 0.15;
    msg.pose.position.y = 0.31;
    msg.pose.position.z = 0.799;
  }
  else if (level == LeftBackward)
  {
    msg.name = "left_arm";

    msg.pose.position.x = 0.3 - 0.15;
    msg.pose.position.y = 0.31;
    msg.pose.position.z = 0.799;
  }
  else if (level == RightForward)
  {
    msg.name = "right_arm";

    msg.pose.position.x = 0.3 + 0.15;
    msg.pose.position.y = -0.31;
    msg.pose.position.z = 0.799;
  }
  else if (level == RightBackward)
  {
    msg.name = "right_arm";

    msg.pose.position.x = 0.3 - 0.15;
    msg.pose.position.y = -0.31;
    msg.pose.position.z = 0.799;
  }

  manipulation_pub_.publish(msg);
}



} // namespace THORMANG3_test
