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

QNodeThor3::QNodeThor3(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv),
    DEBUG(false)
{
    //    index_mode_table_[0] = "none";
    //    index_mode_table_[1] = "walking_module";
    //    index_mode_table_[2] = "manipulation_module";
    //    index_mode_table_[3] = "head_control_module";
    //    index_mode_table_[4] = "wholebody_module";

    //    mode_index_table_["none"]                = 0;
    //    mode_index_table_["walking_module"]      = 1;
    //    mode_index_table_["manipulation_module"] = 2;
    //    mode_index_table_["head_control_module"] = 3;
    //    mode_index_table_["wholebody_module"]    = 4;
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
    module_control_pub_  = _nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_ctrl_module", 0);
    init_pose_pub_      = _nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
    init_ft_pub_        = _nh.advertise<std_msgs::String>("/robotis/base/ini_ft_command", 0);
    set_head_joint_angle_pub_ = _nh.advertise<sensor_msgs::JointState>("/robotis/head_control/joint_control", 0);

    init_ft_foot_sub_ = _nh.subscribe("/robotis/base/ini_ft_value", 10, &QNodeThor3::initFTFootCallback, this);

    current_module_control_sub_ = _nh.subscribe("/robotis/current_ctrl_module", 10, &QNodeThor3::refreshCurrentJointControlCallback, this);
    current_joint_states_sub_ = _nh.subscribe("/robotis/present_joint_states", 10, &QNodeThor3::updateHeadJointStatesCallback, this);

    get_module_control_client_ = _nh.serviceClient<robotis_controller_msgs::GetJointModule>("/robotis/get_ctrl_module");

    // Manipulation
    set_control_mode_msg_pub = _nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_ctrl_module", 0);
    kenematics_pose_sub = _nh.subscribe("/thormang3_demo/ik_target_pose", 10, &QNodeThor3::getKinematicsPoseCallback, this);

    send_ini_pose_msg_pub = _nh.advertise<std_msgs::String>("/robotis/manipulation/ini_pose_msg", 0);
    send_des_joint_msg_pub = _nh.advertise<thormang3_manipulation_module_msgs::JointPose>("/robotis/manipulation/des_joint_msg", 0);
    send_ik_msg_pub = _nh.advertise<thormang3_manipulation_module_msgs::KinematicsPose>("/robotis/manipulation/ik_msg", 0);
    send_pathplan_demo_pub = _nh.advertise<thormang3_manipulation_module_msgs::DemoPose>("/robotis/manipulation/demo_msg", 0);

    get_joint_pose_client = _nh.serviceClient<thormang3_manipulation_module_msgs::GetJointPose>("/robotis/manipulation/get_joint_pose");
    get_kinematics_pose_client = _nh.serviceClient<thormang3_manipulation_module_msgs::GetKinematicsPose>("/robotis/manipulation/get_kinematics_pose");

    // Walking
    set_walking_command_pub = _nh.advertise<thormang3_foot_step_generator::FootStepCommand>("/robotis/thormang3_foot_step_generator/walking_command", 0);
    set_walking_balance_pub = _nh.advertise<std_msgs::Bool>("/robotis/thormang3_foot_step_generator/balance_command", 0);

    std::string _default_path = ros::package::getPath("thormang3_demo") +"/data/id_joint_table.yaml";
    std::string _path = _nh.param<std::string>("id_joint_table", _default_path);
    parseJointNameFromYaml(_path);

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
    std::vector< std::string > _modules = doc["module"].as< std::vector<std::string> >();

    int _module_index = 0;
    for(std::vector<std::string>::iterator _it = _modules.begin() ; _it != _modules.end() ; ++_it)
    {
        std::string _module_name = *_it ;

        index_mode_table_[_module_index]    = _module_name;
        mode_index_table_[_module_name]     = _module_index++;

        using_mode_table_[_module_name]     = false;
    }

    // parse module_joint preset
    YAML::Node _sub_node = doc["preset"];
    int _preset_index = 0;
    for(YAML::iterator _it = _sub_node.begin() ; _it != _sub_node.end() ; ++_it)
    {
        std::string _key;
        std::vector<int> _value;

        _key = _it->first.as<std::string>();
        _value = _it->second.as< std::vector<int> >();

        preset_index[_preset_index++] = _key;   // for ordering of yaml
        preset_table[_key] = _value;
        if(DEBUG) std::cout << "Preset : " << _key << std::endl;
    }
}

// joint id -> joint name
bool QNodeThor3::getJointNameFromID(const int &id, std::string &joint_name)
{
    std::map<int, std::string>::iterator _iter;

    _iter = id_joint_table_.find(id);
    if(_iter == id_joint_table_.end()) return false;

    joint_name = _iter->second;
    return true;
}

// joint name -> joint id
bool QNodeThor3::getIDFromJointName(const std::string &joint_name, int &id)
{
    std::map<std::string, int>::iterator _iter;

    _iter = joint_id_table_.find(joint_name);
    if(_iter == joint_id_table_.end()) return false;

    id = _iter->second;
    return true;
}

// map index -> joint id & joint name
bool QNodeThor3::getIDJointNameFromIndex(const int &index, int &id, std::string &joint_name)
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
std::string QNodeThor3::getModeName(const int &index)
{
    std::string _mode = "";
    std::map<int, std::string>::iterator _iter = index_mode_table_.find(index);

    if(_iter != index_mode_table_.end())
        _mode = _iter->second;

    return _mode;
}

// mode(module) name -> mode(module) index
int QNodeThor3::getModeIndex(const std::string &mode_name)
{
    int _mode_index = 0;
    std::map<std::string, int>::iterator _iter = mode_index_table_.find(mode_name);

    if(_iter != mode_index_table_.end())
        _mode_index = _iter->second;

    return _mode_index;
}

// number of mode(module)s
int QNodeThor3::getModeSize()
{
    return index_mode_table_.size();
}

// number of joints
int QNodeThor3::getJointSize()
{
    return id_joint_table_.size();
}

void QNodeThor3::clearUsingModule()
{
    for(std::map<std::string, bool>::iterator _iter = using_mode_table_.begin(); _iter != using_mode_table_.end(); ++_iter)
        _iter->second = false;
}

bool QNodeThor3::isUsingModule(std::string module_name)
{
    std::map<std::string, bool>::iterator _iter = using_mode_table_.find(module_name);

    if(_iter == using_mode_table_.end()) return false;

    return _iter->second;
}

// move ini pose : wholedody module
void QNodeThor3::moveInitPose()
{
    std_msgs::String _init_msg;
    _init_msg.data = "ini_pose";

    init_pose_pub_.publish(_init_msg);
}

void QNodeThor3::initFTCommand(std::string command)
{
    std_msgs::String _ft_msg;
    _ft_msg.data = command;

    init_ft_pub_.publish(_ft_msg);
}

// move head to assemble 3d lidar(pointcloud)
void QNodeThor3::assemble_lidar()
{
    std_msgs::String _lidar_msg;
    _lidar_msg.data = "start";

    move_lidar_pub_.publish(_lidar_msg);
    log(Info, "publish move_lidar topic");
}

// set mode(module) to each joint
void QNodeThor3::setJointControlMode(const robotis_controller_msgs::JointCtrlModule &msg)
{
    module_control_pub_.publish(msg);
}

// get current mode(module) of joints
void QNodeThor3::getJointControlMode()
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
        _modules.resize(getJointSize());

        // clear current using modules
        clearUsingModule();

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
        Q_EMIT updateCurrentJointControlMode(_modules);
    }
    else
        log(Error, "fail to get current joint control module.");
}

void QNodeThor3::refreshCurrentJointControlCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg)
{
    ROS_INFO("set current joint module");
    int _index = 0;

    std::vector<int> _modules;
    _modules.resize(getJointSize());

    std::map<std::string, int> _joint_module;

    // clear current using modules
    clearUsingModule();

    for(int ix = 0; ix < msg->joint_name.size(); ix++)
    {
        std::string _joint_name = msg->joint_name[ix];
        std::string _module_name = msg->module_name[ix];

        _joint_module[_joint_name] = getModeIndex(_module_name);

        std::map<std::string, bool>::iterator _module_iter = using_mode_table_.find(_module_name);
        if(_module_iter != using_mode_table_.end())
            _module_iter->second = true;
    }

    for(int ix = 0; ix < getJointSize(); ix++)
    {
        int _id = 0;
        std::string _joint_name= "";

        if(getIDJointNameFromIndex(ix, _id, _joint_name) == false) continue;

        std::map<std::string, int>::iterator _module_iter = _joint_module.find(_joint_name);
        if(_module_iter == _joint_module.end()) continue;

        // ROS_INFO_STREAM("joint[" << ix << "] : " << _module_iter->second);
        _modules.at(ix) = _module_iter->second;                
    }

    // update ui
    Q_EMIT updateCurrentJointControlMode(_modules);
}

void QNodeThor3::updateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
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
        Q_EMIT updateHeadAngles(_head_pan, _head_tilt);
}

void QNodeThor3::initFTFootCallback(const thormang3_base_module_msgs::BothWrench::ConstPtr &msg)
{
    std::stringstream _ss;
    _ss << "Type : " << msg->name << std::endl;
    _ss << " - Right - " << std::endl << msg->right << std::endl;
    _ss << " - Left - " << std::endl << msg->left;

    log( Info , _ss.str() );
}

void QNodeThor3::calibrationFTFootCallback(const thormang3_base_module_msgs::CalibrationWrench::ConstPtr &msg) {}

void QNodeThor3::setHeadJoint(double pan, double tilt)
{
    sensor_msgs::JointState _head_angle_msg;

    _head_angle_msg.name.push_back("head_y");
    _head_angle_msg.name.push_back("head_p");

    _head_angle_msg.position.push_back(- pan);
    _head_angle_msg.position.push_back(- tilt);

    set_head_joint_angle_pub_.publish(_head_angle_msg);
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


void QNodeThor3::sendDemoMsg( thormang3_manipulation_module_msgs::DemoPose msg )
{
    send_pathplan_demo_pub.publish( msg );

    log( Info , "Path Plan demo" );

    log( Info , "Set Des. End Effector's Pose : " );

    std::stringstream log_msgs;

    log_msgs << " \n "
             << "group name : "
             << msg.name << " \n "
             << "demo name : "
             << msg.demo << " \n "
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
    thormang3_manipulation_module_msgs::GetJointPose _get_joint_pose;

    // requeset
    _get_joint_pose.request.joint_name = joint_name;

    log( Info , "Get Curr. Joint Value" );

    std::stringstream log_msg;

    log_msg << " \n "
            << "joint name : "
            << joint_name << " \n " ;

    log( Info , log_msg.str() );

    //response
    if( get_joint_pose_client.call( _get_joint_pose ) )
    {
        double _joint_value = _get_joint_pose.response.joint_value;

        log( Info , "Joint Curr. Value" );

        std::stringstream log_msg;

        log_msg << " \n "
                << "curr. value : "
                << _joint_value << " \n ";

        log( Info , log_msg.str() );

        Q_EMIT updateCurrJoint( _joint_value );
    }
    else
        log(Error, "fail to get joint pose.");
}

void QNodeThor3::getKinematicsPose (std::string group_name )
{
    thormang3_manipulation_module_msgs::GetKinematicsPose _get_kinematics_pose;

    //request
    _get_kinematics_pose.request.group_name = group_name;

    log( Info , "Solve Forward Kinematics" );

    log( Info , "Get Curr. End Effector's Pose" );

    std::stringstream log_msg;

    log_msg << " \n "
            << "group name : "
            << group_name << " \n " ;

    log( Info , log_msg.str() );

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

        log( Info , "End Effector Curr. Pose : " );

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

        log( Info , log_msg.str() );

        Q_EMIT updateCurrPos( _pos_x , _pos_y , _pos_z );
        Q_EMIT updateCurrOri( _ori_x , _ori_y , _ori_z , _ori_w );
    }
    else
        log(Error, "fail to get kinematics pose.");
}

void QNodeThor3::getKinematicsPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    double _z_offset = 0.801;
    Q_EMIT updateCurrPos( msg->position.x , msg->position.y , msg->position.z + _z_offset);
    Q_EMIT updateCurrOri( msg->orientation.x , msg->orientation.y , msg->orientation.z , msg->orientation.w );
}

// Walking
void QNodeThor3::setWalkingCommand(thormang3_foot_step_generator::FootStepCommand msg)
{
    set_walking_command_pub.publish(msg);

    std::stringstream _ss;
    _ss << "Set Walking Command : " << msg.command << std::endl;
    _ss << "- Number of Step : " << msg.step_num << std::endl;
    _ss << "- Step Length : " << msg.step_length << std::endl;
    _ss << "- Side Step Length : " << msg.side_step_length << std::endl;
    _ss << "- Rotation Angle : " << msg.step_angle_rad << std::endl;

    log(Info, _ss.str());
}

void QNodeThor3::setWalkingBalance(bool on_command)
{
    std_msgs::Bool _msg;
    _msg.data = on_command;

    set_walking_balance_pub.publish(_msg);

    std::stringstream _ss;
    _ss << "Set Walking Balance : " << (on_command ? "True" : "False");

    log(Info, _ss.str());
}


// LOG
void QNodeThor3::log( const LogLevel &level, const std::string &msg)
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

    switch ( level ) {
        case(Debug) : {
            ROS_DEBUG_STREAM(msg);
            logging_model_msg << "[DEBUG] [" << _min_str.str() << ":" << _sec_str.str() << "]: " << msg;
            break;
        }
        case(Info) : {
            ROS_INFO_STREAM(msg);
            logging_model_msg << "[INFO] [" << _min_str.str() << ":" << _sec_str.str() << "]: " << msg;
            break;
        }
        case(Warn) : {
            ROS_WARN_STREAM(msg);
            logging_model_msg << "[INFO] [" << _min_str.str() << ":" << _sec_str.str() << "]: " << msg;
            break;
        }
        case(Error) : {
            ROS_ERROR_STREAM(msg);
            logging_model_msg << "[ERROR] [" << _min_str.str() << ":" << _sec_str.str() << "]: " << msg;
            break;
        }
        case(Fatal) : {
            ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [" << _min_str.str() << ":" << _sec_str.str() << "]: " << msg;
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
