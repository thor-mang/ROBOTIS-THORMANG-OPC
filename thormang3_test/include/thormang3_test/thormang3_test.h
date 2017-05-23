#ifndef THORMANG3_TEST_H
#define THORMANG3_TEST_H

#include <stdio.h>
#include <iostream>

//std
#include <string>
#include <complex>

//ros dependencies
#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>

#include "std_msgs/String.h"

#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
#include "thormang3_foot_step_generator/FootStepCommand.h"

namespace thormang3
{

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

class THORMANG3
{

private:
    //ros node handle
    ros::NodeHandle ros_node_;

public:
    //constructor
    THORMANG3();
    //destructor
    ~THORMANG3();

    enum ManipulationLevel
    {
      LeftForward   = 0,
      RightForward  = 1,
      LeftBackward   = 2,
      RightBackward  = 3,
      Count         = 4
    };

    ros::Publisher walking_pub_;
    ros::Publisher manipulation_pub_;

    void initialize();
    void sendWalkingCommand();
    void sendManipulationCommand(int level);
};

}       // namespace

#endif // THORMANG3_TEST_H
