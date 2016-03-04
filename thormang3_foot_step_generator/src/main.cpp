/*
 * main.cpp
 *
 *  Created on: 2016. 2. 20.
 *      Author: HJSONG
 */

#include "thormang3_foot_step_generator/message_callback.h"



int main( int argc , char **argv )
{
    ros::init( argc , argv , "thormang3_foot_step_generator" );

    Initialize();

    ros::spin();
    return 0;
}
