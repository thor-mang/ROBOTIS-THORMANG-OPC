#2D StepData

geometry_msgs/Pose2D step2d   # step pose as relative offset to last leg


# which leg to use (left/right/no, see below)
uint8 moving_foot   

uint8 LFootMove=1   # left foot constant
uint8 RFootMove=2   # right foot constant
uint8 NFootMove=3   # no foot constant