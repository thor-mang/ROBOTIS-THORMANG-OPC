/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

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
    : init_argc(argc)
    , init_argv(argv)
{
    // code to DEBUG
    DEBUG = false;

    if(argc >= 2)
    {
        std::string _debug_code(argv[1]);
        if(_debug_code == "debug")
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
    ros::init(init_argc, init_argv, "thormang3_demo");

    if ( ! ros::master::check() ) {
        return false;
    }

    ros::start(); // explicitly needed since our nodehandle is going out of scope.

    ros::NodeHandle _nh;

    // Add your ros communications here.
    move_lidar_pub_     = _nh.advertise<std_msgs::String>("/robotis/head_control/move_lidar", 0);
    module_control_pub_  = _nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
    module_control_preset_pub_ = _nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
    init_pose_pub_      = _nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
    init_ft_pub_        = _nh.advertise<std_msgs::String>("/robotis/feet_ft/ft_calib_command", 0);
    set_head_joint_angle_pub_ = _nh.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states", 0);

    init_ft_foot_sub_ = _nh.subscribe("/robotis/feet_ft/both_ft_value", 10, &QNodeThor3::InitFTFootCallback, this);

    status_msg_sub_ = _nh.subscribe("/robotis/status", 10, &QNodeThor3::StatusMsgCallback, this);
    current_module_control_sub_ = _nh.subscribe("/robotis/present_joint_ctrl_modules", 10, &QNodeThor3::RefreshCurrentJointControlCallback, this);
    current_joint_states_sub_ = _nh.subscribe("/robotis/present_joint_states", 10, &QNodeThor3::UpdateHeadJointStatesCallback, this);

    get_module_control_client_ = _nh.serviceClient<robotis_controller_msgs::GetJointModule>("/robotis/get_present_joint_ctrl_modules");

    humanoidFootStepClient = _nh.serviceClient<humanoid_nav_msgs::PlanFootsteps>("plan_footsteps");
    marker_pub_ = _nh.advertise<visualization_msgs::MarkerArray>("/robotis/demo/foot_step_marker", 0);
    pose_sub_ = _nh.subscribe("/robotis/demo/pose", 10, &QNodeThor3::PoseCallback, this);

    // Manipulation
    kenematics_pose_sub = _nh.subscribe("/thormang3_demo/ik_target_pose", 10, &QNodeThor3::GetKinematicsPoseCallback, this);

    send_ini_pose_msg_pub = _nh.advertise<std_msgs::String>("/robotis/manipulation/ini_pose_msg", 0);
    send_des_joint_msg_pub = _nh.advertise<thormang3_manipulation_module_msgs::JointPose>("/robotis/manipulation/joint_pose_msg", 0);
    send_ik_msg_pub = _nh.advertise<thormang3_manipulation_module_msgs::KinematicsPose>("/robotis/manipulation/kinematics_pose_msg", 0);

    get_joint_pose_client = _nh.serviceClient<thormang3_manipulation_module_msgs::GetJointPose>("/robotis/manipulation/get_joint_pose");
    get_kinematics_pose_client = _nh.serviceClient<thormang3_manipulation_module_msgs::GetKinematicsPose>("/robotis/manipulation/get_kinematics_pose");

    // Walking
    set_walking_command_pub = _nh.advertise<thormang3_foot_step_generator::FootStepCommand>("/robotis/thormang3_foot_step_generator/walking_command", 0);
    set_walking_footsteps_pub = _nh.advertise<thormang3_foot_step_generator::Step2DArray>("/robotis/thormang3_foot_step_generator/footsteps_2d", 0);
    set_walking_balance_pub = _nh.advertise<std_msgs::Bool>("/robotis/thormang3_foot_step_generator/balance_command", 0);

    // Config
    std::string _default_path = ros::package::getPath("thormang3_demo") +"/config/demo_config.yaml";
    std::string _path = _nh.param<std::string>("demo_config", _default_path);
    ParseJointNameFromYaml(_path);

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

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT RosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNodeThor3::ParseJointNameFromYaml(const std::string &path)
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
    YAML::Node _id_sub_node = doc["id_joint"];
    for(YAML::iterator _it = _id_sub_node.begin() ; _it != _id_sub_node.end() ; ++_it)
    {
        int _id;
        std::string _joint_name;

        _id = _it->first.as<int>();
        _joint_name = _it->second.as<std::string>();

        id_joint_table_[_id] = _joint_name;
        joint_id_table_[_joint_name] = _id;

        if(DEBUG) std::cout << "ID : " << _id << " - " << _joint_name << std::endl;
    }

    // parse module
    std::vector< std::string > _modules = doc["module_list"].as< std::vector<std::string> >();

    int _module_index = 0;
    for(std::vector<std::string>::iterator _it = _modules.begin() ; _it != _modules.end() ; ++_it)
    {
        std::string _module_name = *_it ;

        index_mode_table_[_module_index]    = _module_name;
        mode_index_table_[_module_name]     = _module_index++;

        using_mode_table_[_module_name]     = false;
    }


    // parse module_joint preset
    YAML::Node _sub_node = doc["module_button"];
    for(YAML::iterator _it = _sub_node.begin() ; _it != _sub_node.end() ; ++_it)
    {
        int _key;
        std::string _module_name;

        _key = _it->first.as<int>();
        _module_name = _it->second.as<std::string>();

        module_table[_key] = _module_name;
        if(DEBUG) std::cout << "Preset : " << _module_name << std::endl;
    }
}

// joint id -> joint name
bool QNodeThor3::GetJointNameFromID(const int &id, std::string &joint_name)
{
    std::map<int, std::string>::iterator _iter;

    _iter = id_joint_table_.find(id);
    if(_iter == id_joint_table_.end()) return false;

    joint_name = _iter->second;
    return true;
}

// joint name -> joint id
bool QNodeThor3::GetIDFromJointName(const std::string &joint_name, int &id)
{
    std::map<std::string, int>::iterator _iter;

    _iter = joint_id_table_.find(joint_name);
    if(_iter == joint_id_table_.end()) return false;

    id = _iter->second;
    return true;
}

// map index -> joint id & joint name
bool QNodeThor3::GetIDJointNameFromIndex(const int &index, int &id, std::string &joint_name)
{
    std::map<int, std::string>::iterator _iter;
    int _count = 0;
    for(_iter = id_joint_table_.begin(); _iter != id_joint_table_.end(); ++_iter, _count++)
    {
        if(index == _count)
        {
            id = _iter->first;
            joint_name = _iter->second;
            return true;
        }
    }
    return false;
}

// mode(module) index -> mode(module) name
std::string QNodeThor3::GetModuleName(const int &index)
{
    std::string _mode = "";
    std::map<int, std::string>::iterator _iter = index_mode_table_.find(index);

    if(_iter != index_mode_table_.end())
        _mode = _iter->second;

    return _mode;
}

// mode(module) name -> mode(module) index
int QNodeThor3::GetModuleIndex(const std::string &mode_name)
{
    int _mode_index = -1;
    std::map<std::string, int>::iterator _iter = mode_index_table_.find(mode_name);

    if(_iter != mode_index_table_.end())
        _mode_index = _iter->second;

    return _mode_index;
}

// number of mode(module)s
int QNodeThor3::GetModuleTableSize()
{
    return index_mode_table_.size();
}

// number of joints
int QNodeThor3::GetJointTableSize()
{
    return id_joint_table_.size();
}

void QNodeThor3::ClearUsingModule()
{
    for(std::map<std::string, bool>::iterator _iter = using_mode_table_.begin(); _iter != using_mode_table_.end(); ++_iter)
        _iter->second = false;
}

bool QNodeThor3::IsUsingModule(std::string module_name)
{
    std::map<std::string, bool>::iterator _iter = using_mode_table_.find(module_name);

    if(_iter == using_mode_table_.end()) return false;

    return _iter->second;
}

void QNodeThor3::SetCurrentControlUI(int mode)
{
    current_control_ui = mode;

    ROS_INFO("Current UI : %d", mode);
}

// move ini pose : base module
void QNodeThor3::MoveInitPose()
{
    std_msgs::String _init_msg;
    _init_msg.data = "ini_pose";

    init_pose_pub_.publish(_init_msg);

    Log(Info, "Go to robot initial pose.");
}

void QNodeThor3::InitFTCommand(std::string command)
{
    std_msgs::String _ft_msg;
    _ft_msg.data = command;

    init_ft_pub_.publish(_ft_msg);

}

// move head to assemble 3d lidar(pointcloud)
void QNodeThor3::Assemble_lidar()
{
    std_msgs::String _lidar_msg;
    _lidar_msg.data = "start";

    move_lidar_pub_.publish(_lidar_msg);
    Log(Info, "Publish move_lidar topic");
}

// enable mode(module)
void QNodeThor3::EnableControlModule(const std::string &mode)
{
    std_msgs::String _msg;
    _msg.data = mode;

    module_control_preset_pub_.publish(_msg);

    std::stringstream _ss;
    _ss << "Set Mode : " << mode;
    Log(Info, _ss.str());
}

// get current mode(module) of joints
void QNodeThor3::GetJointControlModule()
{
    robotis_controller_msgs::GetJointModule _get_joint;
    std::map<std::string, int> _service_map;

    // _get_joint.request
    std::map<int, std::string>::iterator _iter;
    int _index = 0;
    for(_iter = id_joint_table_.begin(); _iter != id_joint_table_.end(); ++_iter, _index++)
    {
        _get_joint.request.joint_name.push_back(_iter->second);
        _service_map[_iter->second] = _index;
    }

    if(get_module_control_client_.call(_get_joint))
    {
        // _get_joint.response
        std::vector<int> _modules;
        _modules.resize(GetJointTableSize());

        // clear current using modules
        ClearUsingModule();

        for(int ix = 0; ix < _get_joint.response.joint_name.size(); ix++)
        {
            std::string _joint_name = _get_joint.response.joint_name[ix];
            std::string _module_name = _get_joint.response.module_name[ix];

            std::map<std::string, int>::iterator _service_iter = _service_map.find(_joint_name);
            if(_service_iter == _service_map.end())
                continue;

            _index = _service_iter->second;

            _service_iter = mode_index_table_.find(_module_name);
            if(_service_iter == mode_index_table_.end())
                continue;

            // ROS_INFO_STREAM("joint[" << ix << "] : " << _service_iter->second);
            _modules.at(_index) = _service_iter->second;

            std::map<std::string, bool>::iterator _module_iter = using_mode_table_.find(_module_name);
            if(_module_iter != using_mode_table_.end())
                _module_iter->second = true;
        }

        // update ui
        Q_EMIT UpdatePresentJointControlModules(_modules);
        Log(Info, "Get current Mode");
    }
    else
        Log(Error, "Fail to get current joint control module.");
}

void QNodeThor3::RefreshCurrentJointControlCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg)
{
    ROS_INFO("set current joint module");
    int _index = 0;

    std::vector<int> _modules;
    _modules.resize(GetJointTableSize());

    std::map<std::string, int> _joint_module;

    // clear current using modules
    ClearUsingModule();

    for(int ix = 0; ix < msg->joint_name.size(); ix++)
    {
        std::string _joint_name = msg->joint_name[ix];
        std::string _module_name = msg->module_name[ix];

        _joint_module[_joint_name] = GetModuleIndex(_module_name);

        std::map<std::string, bool>::iterator _module_iter = using_mode_table_.find(_module_name);
        if(_module_iter != using_mode_table_.end())
            _module_iter->second = true;
    }

    for(int ix = 0; ix < GetJointTableSize(); ix++)
    {
        int _id = 0;
        std::string _joint_name= "";

        if(GetIDJointNameFromIndex(ix, _id, _joint_name) == false) continue;

        std::map<std::string, int>::iterator _module_iter = _joint_module.find(_joint_name);
        if(_module_iter == _joint_module.end()) continue;

        // ROS_INFO_STREAM("joint[" << ix << "] : " << _module_iter->second);
        _modules.at(ix) = _module_iter->second;
    }

    // update ui
    Q_EMIT UpdatePresentJointControlModules(_modules);

    Log(Info, "Applied Mode", "Manager");
}

void QNodeThor3::UpdateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    double _head_pan, _head_tilt;
    int _get = 0;

    for(int ix = 0; ix < msg->name.size(); ix++)
    {
        if(msg->name[ix] == "head_y")
        {
            _head_pan = - msg->position[ix];
            _get += 1;
        }
        else if(msg->name[ix] == "head_p")
        {
            _head_tilt = - msg->position[ix];
            _get += 1;
        }

        if(_get == 2) break;
    }

    if(_get > 0)
        Q_EMIT UpdateHeadJointsAngle(_head_pan, _head_tilt);
}

void QNodeThor3::InitFTFootCallback(const thormang3_feet_ft_module_msgs::BothWrench::ConstPtr &msg)
{
    std::stringstream _ss;
    _ss << "Type : " << msg->name << std::endl;
    _ss << " - Right - " << std::endl << msg->right << std::endl;
    _ss << " - Left - " << std::endl << msg->left;

    Log( Info , _ss.str() );
}

void QNodeThor3::SetHeadJoint(double pan, double tilt)
{
    sensor_msgs::JointState _head_angle_msg;

    _head_angle_msg.name.push_back("head_y");
    _head_angle_msg.name.push_back("head_p");

    _head_angle_msg.position.push_back(- pan);
    _head_angle_msg.position.push_back(- tilt);

    set_head_joint_angle_pub_.publish(_head_angle_msg);
}

// Manipulation
void QNodeThor3::SendInitPoseMsg( std_msgs::String msg )
{
    send_ini_pose_msg_pub.publish( msg );

    Log( Info , "Send Ini. Pose" );
}

void QNodeThor3::SendDestJointMsg( thormang3_manipulation_module_msgs::JointPose msg )
{
    send_des_joint_msg_pub.publish( msg );

    Log( Info , "Set Des. Joint Vale" );

    std::stringstream log_msg;

    log_msg << " \n "
            << "joint name : "
            << msg.name << " \n "
            << "joint value : "
            << msg.value * 180.0 / M_PI << " \n ";

    Log( Info , log_msg.str() );
}

void QNodeThor3::SendIkMsg( thormang3_manipulation_module_msgs::KinematicsPose msg )
{
    send_ik_msg_pub.publish( msg );

    Log( Info , "Solve Inverse Kinematics" );

    Log( Info , "Set Des. End Effector's Pose : " );

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

    Log( Info , log_msgs.str() );
}

void QNodeThor3::GetJointPose( std::string joint_name )
{
    thormang3_manipulation_module_msgs::GetJointPose _get_joint_pose;

    // requeset
    _get_joint_pose.request.joint_name = joint_name;

    Log( Info , "Get Curr. Joint Value" );

    std::stringstream log_msg;

    log_msg << " \n "
            << "joint name : "
            << joint_name << " \n " ;

    Log( Info , log_msg.str() );

    //response
    if( get_joint_pose_client.call( _get_joint_pose ) )
    {
        double _joint_value = _get_joint_pose.response.joint_value;

        Log( Info , "Joint Curr. Value" );

        std::stringstream log_msg;

        log_msg << " \n "
                << "curr. value : "
                << _joint_value << " \n ";

        Log( Info , log_msg.str() );

        Q_EMIT UpdateCurrJoint( _joint_value );
    }
    else
        Log(Error, "fail to get joint pose.");
}

void QNodeThor3::GetKinematicsPose (std::string group_name )
{
    thormang3_manipulation_module_msgs::GetKinematicsPose _get_kinematics_pose;

    //request
    _get_kinematics_pose.request.group_name = group_name;

    Log( Info , "Solve Forward Kinematics" );

    Log( Info , "Get Curr. End Effector's Pose" );

    std::stringstream log_msg;

    log_msg << " \n "
            << "group name : "
            << group_name << " \n " ;

    Log( Info , log_msg.str() );

    //response
    if ( get_kinematics_pose_client.call( _get_kinematics_pose ) )
    {
        double _pos_x = _get_kinematics_pose.response.group_pose.position.x;
        double _pos_y = _get_kinematics_pose.response.group_pose.position.y;
        double _pos_z = _get_kinematics_pose.response.group_pose.position.z;

        double _ori_x = _get_kinematics_pose.response.group_pose.orientation.x;
        double _ori_y = _get_kinematics_pose.response.group_pose.orientation.y;
        double _ori_z = _get_kinematics_pose.response.group_pose.orientation.z;
        double _ori_w = _get_kinematics_pose.response.group_pose.orientation.w;

        Log( Info , "End Effector Curr. Pose : " );

        std::stringstream log_msg;

        log_msg << " \n "
                << "curr. pos. x : "
                << _pos_x << " \n "
                << "curr. pos. y : "
                << _pos_y << " \n "
                << "curr. pos. z : "
                << _pos_z << " \n "
                << "curr. ori. w : "
                << _ori_w << " \n "
                << "curr. ori. x : "
                << _ori_x << " \n "
                << "curr. ori. y : "
                << _ori_y << " \n "
                << "curr. ori. z : "
                << _ori_z << " \n ";

        Log( Info , log_msg.str() );

        Q_EMIT UpdateCurrPos( _pos_x , _pos_y , _pos_z );
        Q_EMIT UpdateCurrOri( _ori_x , _ori_y , _ori_z , _ori_w );
    }
    else
        Log(Error, "fail to get kinematics pose.");
}

void QNodeThor3::GetKinematicsPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    double _z_offset = 0.801;
    Q_EMIT UpdateCurrPos( msg->position.x , msg->position.y , msg->position.z + _z_offset);
    Q_EMIT UpdateCurrOri( msg->orientation.x , msg->orientation.y , msg->orientation.z , msg->orientation.w );
}

// Walking
void QNodeThor3::SetWalkingCommand(thormang3_foot_step_generator::FootStepCommand msg)
{
    set_walking_command_pub.publish(msg);

    std::stringstream _ss;
    _ss << "Set Walking Command : " << msg.command << std::endl;
    _ss << "- Number of Step : " << msg.step_num << std::endl;
    _ss << "- Step Length : " << msg.step_length << std::endl;
    _ss << "- Side Step Length : " << msg.side_step_length << std::endl;
    _ss << "- Rotation Angle : " << msg.step_angle_rad << std::endl;

    Log(Info, _ss.str());
}

void QNodeThor3::SetWalkingBalance(bool on_command)
{
    std_msgs::Bool _msg;
    _msg.data = on_command;

    set_walking_balance_pub.publish(_msg);

    std::stringstream _ss;
    _ss << "Set Walking Balance : " << (on_command ? "True" : "False");

    Log(Info, _ss.str());
}

void QNodeThor3::SetWalkingFootsteps()
{
    if(preview_foot_steps_.size() != preview_foot_types_.size())
    {
        Log(Error, "Footsteps are corrupted.");
        return;
    }
    else if(preview_foot_steps_.size() == 0)
    {
        Log(Warn, "No Footsteps");
        return;
    }

    thormang3_foot_step_generator::Step2DArray _footsteps;

    for(int ix = 0; ix < preview_foot_steps_.size(); ix++)
    {
        thormang3_foot_step_generator::Step2D _step;

        int _type = preview_foot_types_[ix];
        if(_type == humanoid_nav_msgs::StepTarget::right) _step.moving_foot = thormang3_foot_step_generator::Step2D::RFootMove;
        else if(_type == humanoid_nav_msgs::StepTarget::left) _step.moving_foot = thormang3_foot_step_generator::Step2D::LFootMove;
        else _step.moving_foot = thormang3_foot_step_generator::Step2D::NFootMove;

        _step.step2d = preview_foot_steps_[ix];

        _footsteps.footsteps_2d.push_back(_step);
    }

    set_walking_footsteps_pub.publish(_footsteps);
    Log(Info, "Set command to walk using footsteps");

    ClearFootsteps();
}

void QNodeThor3::ClearFootsteps()
{
    VisualizePreviewFootsteps(true);

    preview_foot_steps_.clear();
    preview_foot_types_.clear();
}

void QNodeThor3::MakeFootstepUsingPlanner()
{
    MakeFootstepUsingPlanner(pose_from_ui_);
}

void QNodeThor3::MakeFootstepUsingPlanner(geometry_msgs::Pose foot_target)
{
    //foot step service
    humanoid_nav_msgs::PlanFootsteps _get_step;

    geometry_msgs::Pose2D _start;
    geometry_msgs::Pose2D _goal;
    _goal.x = foot_target.position.x;
    _goal.y = foot_target.position.y;

    Eigen::Quaterniond _goal_orientation;
    tf::quaternionMsgToEigen(foot_target.orientation, _goal_orientation);

    Eigen::Vector3d _forward, _f_x(1, 0, 0);
    _forward = _goal_orientation.toRotationMatrix() * _f_x;
    double _theta = _forward.y() > 0 ? acos(_forward.x()) : - acos(_forward.x());
    _goal.theta = _theta;

    _get_step.request.start = _start;
    _get_step.request.goal = _goal;

    std::stringstream _call_msg;
    _call_msg << "Start [" << _start.x << ", " << _start.y << " | " << _start.theta << "]"
              << " , Goal [" << _goal.x << ", " << _goal.y << " | " << _goal.theta << "]";
    Log(Info, _call_msg.str());

    // clear visualization
    VisualizePreviewFootsteps(true);

    // init foot steps
    preview_foot_steps_.clear();
    preview_foot_types_.clear();

    if(humanoidFootStepClient.call(_get_step))
    {
        if(_get_step.response.result)
        {
            for(int ix = 0; ix < _get_step.response.footsteps.size(); ix++)
            {
                // foot step log
                std::stringstream _msg;
                int _foot_type = _get_step.response.footsteps[ix].leg;
                std::string _foot = _foot_type == humanoid_nav_msgs::StepTarget::right ? "right" : "left";
                geometry_msgs::Pose2D _foot_pose = _get_step.response.footsteps[ix].pose;

                // log footsteps
                _msg << "Foot Step #" << ix + 1 << " [ " << _foot << "] - ["
                     << _foot_pose.x << ", " << _foot_pose.y << " | " << (_foot_pose.theta * 180 / M_PI) << "]";
                Log(Info, _msg.str());

                preview_foot_steps_.push_back(_foot_pose);
                preview_foot_types_.push_back(_foot_type);
            }

            // visualize foot steps
            VisualizePreviewFootsteps(false);
        }
        else
        {
            Log(Info, "fail to get foot step from planner");
            return;
        }
    }
    else
    {
        Log(Error, "cannot call service");
        return;
    }

    return;
}

void QNodeThor3::VisualizePreviewFootsteps(bool clear)
{
    if(clear && preview_foot_steps_.size() == 0) return;

    visualization_msgs::MarkerArray _marker_array;
    ros::Time _now = ros::Time::now();
    visualization_msgs::Marker _rviz_marker;

    _rviz_marker.header.frame_id = "pelvis_link";
    _rviz_marker.header.stamp = _now;
    _rviz_marker.ns = "foot_step_marker";

    _rviz_marker.id = 1;
    _rviz_marker.type = visualization_msgs::Marker::CUBE;
    _rviz_marker.action = !clear ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

    _rviz_marker.scale.x = 0.216;
    _rviz_marker.scale.y = 0.144;
    _rviz_marker.scale.z = 0.01;

    double _alpha = 0.7;
    double _height = -0.723;

    for(int ix = preview_foot_types_.size(); ix > 0; ix--)
    {
        // foot step marker array
        _rviz_marker.id += 10;

        if(!clear)
        {
            Eigen::Vector3d _marker_position(preview_foot_steps_[ix - 1].x, preview_foot_steps_[ix - 1].y, _height);
            Eigen::Vector3d _marker_position_offset;

            Eigen::Vector3d _toward(1, 0, 0), _direction(cos(preview_foot_steps_[ix - 1].theta), sin(preview_foot_steps_[ix - 1].theta), 0);
            Eigen::Quaterniond _marker_orientation(Eigen::Quaterniond::FromTwoVectors(_toward, _direction));

            if(DEBUG)
            {
                std::stringstream _msg;
                _msg << "Foot Step #" << ix << " [ " << preview_foot_types_[ix - 1] << "] - ["
                     << _rviz_marker.pose.position.x << ", " << _rviz_marker.pose.position.y << "]";
                Log(Info, _msg.str());
            }
            _alpha *= 0.9;

            // set foot step color
            if(preview_foot_types_[ix - 1] == humanoid_nav_msgs::StepTarget::left)             // left
            {
                _rviz_marker.color.r = 0.0;
                _rviz_marker.color.g = 0.0;
                _rviz_marker.color.b = 1.0;
                _rviz_marker.color.a = _alpha + 0.3;

                Eigen::Vector3d _offset_y(0, 0.015, 0);
                _marker_position_offset = _marker_orientation.toRotationMatrix() * _offset_y;

            }
            else if(preview_foot_types_[ix - 1] == humanoid_nav_msgs::StepTarget::right)       //right
            {
                _rviz_marker.color.r = 1.0;
                _rviz_marker.color.g = 0.0;
                _rviz_marker.color.b = 0.0;
                _rviz_marker.color.a = _alpha + 0.3;

                Eigen::Vector3d _offset_y(0, -0.015, 0);
                _marker_position_offset = _marker_orientation.toRotationMatrix() * _offset_y;
            }

            _marker_position = _marker_position_offset + _marker_position;

            tf::pointEigenToMsg(_marker_position, _rviz_marker.pose.position);
            tf::quaternionEigenToMsg(_marker_orientation, _rviz_marker.pose.orientation);

            // apply foot x offset
        }

        _marker_array.markers.push_back(_rviz_marker);
    }

    // publish foot step marker array
    if(clear == false) Log(Info, "Visualize Preview Footstep Marker Array");
    else Log(Info, "Clear Visualize Preview Footstep Marker Array");

    marker_pub_.publish(_marker_array);
}

// Pose for Walking and Manipulation
void QNodeThor3::PoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    switch(current_control_ui)
    {
        case WALKING_UI:
        {
            pose_from_ui_ = *msg;
            Q_EMIT HavePoseToMakeFootstep();
            Log(Info, "Get Pose For Step");
            break;
        }

        case MANIPULATION_UI:
        {
            double _z_offset = 0.801;
            Q_EMIT UpdateCurrPos( msg->position.x , msg->position.y , msg->position.z + _z_offset);
            Q_EMIT UpdateCurrOri( msg->orientation.x , msg->orientation.y , msg->orientation.z , msg->orientation.w );
            Log(Info, "Get Pose For IK");
            break;
        }

        default:
            break;
    }
}

// LOG
void QNodeThor3::StatusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
    Log((LogLevel) msg->type, msg->status_msg, msg->module_name);
}

void QNodeThor3::Log( const LogLevel &level, const std::string &msg, std::string sender)
{
    logging_model_.insertRows(logging_model_.rowCount(),1);
    std::stringstream logging_model_msg;

    ros::Duration _duration_time = ros::Time::now() - start_time_;
    int _current_time = _duration_time.sec;
    int _min_time = 0, _sec_time = 0;
    _min_time = (int)(_current_time / 60);
    _sec_time = (int)(_current_time % 60);

    std::stringstream _min_str, _sec_str;
    if(_min_time < 10) _min_str << "0";
    if(_sec_time < 10) _sec_str << "0";
    _min_str << _min_time;
    _sec_str << _sec_time;

    std::stringstream _sender;
    _sender << "[" << sender << "] ";

    switch ( level ) {
        case(Debug) :
        {
            ROS_DEBUG_STREAM(msg);
            logging_model_msg << "[DEBUG] [" << _min_str.str() << ":" << _sec_str.str() << "]: " << _sender.str() << msg;
            break;
        }
        case(Info) :
        {
            ROS_INFO_STREAM(msg);
            logging_model_msg << "[INFO] [" << _min_str.str() << ":" << _sec_str.str() << "]: " << _sender.str() << msg;
            break;
        }
        case(Warn) :
        {
            ROS_WARN_STREAM(msg);
            logging_model_msg << "[WARN] [" << _min_str.str() << ":" << _sec_str.str() << "]: " << _sender.str() <<msg;
            break;
        }
        case(Error) :
        {
            ROS_ERROR_STREAM(msg);
            logging_model_msg << "<ERROR> [" << _min_str.str() << ":" << _sec_str.str() << "]: " << _sender.str() <<msg;
            break;
        }
        case(Fatal) :
        {
            ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [" << _min_str.str() << ":" << _sec_str.str() << "]: " << _sender.str() <<msg;
            break;
        }
    }

    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
    Q_EMIT LoggingUpdated(); // used to readjust the scrollbar
}

void QNodeThor3::ClearLog()
{
    if(logging_model_.rowCount() == 0) return;

    logging_model_.removeRows(0, logging_model_.rowCount());
}

}  // namespace thor3_control
