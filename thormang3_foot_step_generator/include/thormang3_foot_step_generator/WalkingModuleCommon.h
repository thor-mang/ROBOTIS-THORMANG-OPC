/*
 * WalkingModuleError.h
 *
 *  Created on: 2016. 2. 18.
 *      Author: HJSONG
 */

#ifndef SRC_ROBOTIS_THORMANG_PPC_THORMANG3_WALKING_DEMO_INCLUDE_THORMANG3_WALKING_DEMO_WALKINGMODULECOMMON_H_
#define SRC_ROBOTIS_THORMANG_PPC_THORMANG3_WALKING_DEMO_INCLUDE_THORMANG3_WALKING_DEMO_WALKINGMODULECOMMON_H_

class WalkingStateFlag
{
public:
	static const int InWalkingStarting	= 0;
	static const int InWalking 			= 1;
	static const int InWalkingEnding	= 2;
};

class MovingFootFlag
{
public:
	static const int LFootMove = 1;
	static const int RFootMove = 2;
	static const int NFootMove = 3;
};


class STEP_DATA_ERR {
public:
	static const int NO_ERROR					= 0;
	static const int NOT_ENABLED_WALKING_MODULE	= 2;
	static const int PROBLEM_IN_POSITION_DATA	= 4;
	static const int PROBLEM_IN_TIME_DATA		= 8;
	static const int ROBOT_IS_WALKING_NOW 		= 1024;
};

class WALKING_START_ERR {
public:
	static const int NO_ERROR					= 0;
	static const int NOT_ENABLED_WALKING_MODULE	= 2;
	static const int NO_STEP_DATA				= 16;
	static const int ROBOT_IS_WALKING_NOW 		= 1024;
};

class BALANCE_PARAM_ERR {
public:
	static const int NO_ERROR 				    	= 0;
	static const int NOT_ENABLED_WALKING_MODULE		= 2;
	static const int PREV_REQUEST_IS_NOT_FINISHED	= 32;
	static const int TIME_CONST_IS_ZERO_OR_NEGATIVE	= 64;
};

class REMOVE_STEP_DATA_ERR {
public:
	static const int NO_ERROR				= 0;
	static const int ROBOT_IS_WALKING_NOW	= 1024;
};





#endif /* SRC_ROBOTIS_THORMANG_PPC_THORMANG3_WALKING_DEMO_INCLUDE_THORMANG3_WALKING_DEMO_WALKINGMODULECOMMON_H_ */
