
// include
#include "thormang3_test/thormang3_test.h"

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "thormang3_test");

  //create ros wrapper object
  thormang3::THORMANG3 thormang3;
  thormang3.initialize();

  ros::NodeHandle ros_node;
  double test_tol = ros_node.param<double>("test_tol", 1.0);

  ros::Rate loop_rate(10);
  ros::Time update = ros::Time::now();

  int level = thormang3::THORMANG3::LeftForward;

  //node loop
  while ( ros::ok() )
  {
    ros::Time begin = ros::Time::now();
    ros::Duration time_duration;

    time_duration = begin - update;
    double update_time = time_duration.toSec();

    if (update_time > test_tol)
    {
      thormang3.sendWalkingCommand();
      ROS_INFO("Update Time : %f [sec]", update_time);

      thormang3.sendManipulationCommand(level);

      level = (level + 1) % thormang3::THORMANG3::Count;

      ROS_INFO("Manipulation Level : %d", level);

      update = ros::Time::now();
    }

    ros::spinOnce();

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  return 0;
}
