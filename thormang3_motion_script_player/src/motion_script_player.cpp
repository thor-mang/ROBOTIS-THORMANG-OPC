/*
 * thormang3_motion_script_player.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: jay
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>
#include "robotis_controller_msgs/StatusMsg.h"
#include "thormang3_action_module_msgs/IsRunning.h"

#define MOTION_PLAY_CMD_NAME  "play"
#define MP3_PLAY_CMD_NAME     "mp3"
#define WAIT_MOTION_CMD_NAME  "wait"
#define SLEEP_CMD_NAME        "sleep"



ros::Subscriber motion_script_num_sub;
ros::Publisher  action_page_num_pub;
ros::Publisher  sound_file_name_pub;
ros::ServiceClient is_running_client;

thormang3_action_module_msgs::IsRunning  is_running_srv;

boost::thread   *motion_script_play_thread;


std::string motion_script_file_path;

typedef struct
{
    std::string cmd_name;
    std::string cmd_arg_str;
    int         cmd_arg_int;
} motion_script_cmd;

std::vector<motion_script_cmd> motion_script_data;

std::string convertIntToString(int n)
{
    std::ostringstream ostr;
    ostr << n;
    return ostr.str();
}


int convertStringToInt(std::string str)
{
    return atoi(str.c_str());
}

bool IsActionRunning(void)
{
    if(is_running_client.call(is_running_srv) == false) {
        ROS_ERROR("Failed to get action status");
        return true;
    }
    else {
        if(is_running_srv.response.is_running == true)
        {
            return true;
        }
    }

    return false;
}


bool parseMotionScript(int motion_script_index)
{
    motion_script_data.clear();

    YAML::Node motion_script_file_doc;
    try
    {
        // load yaml
        motion_script_file_doc = YAML::LoadFile(motion_script_file_path.c_str());
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Failed to load motion script file.");
        return false;
    }


    //find motion script
    std::string script_index_key = "script" + convertIntToString(motion_script_index);
    YAML::Node motion_script_doc = motion_script_file_doc[script_index_key];
    if(motion_script_doc == NULL)
    {
        std::string status_msg = "Failed to find motion script #" + convertIntToString(motion_script_index);
        ROS_ERROR_STREAM(status_msg);
        return false;
    }


    int cmd_num = 1;
    while(true)
    {
        //check cmd exist
        std::string cmd_key = "cmd" + convertIntToString(cmd_num);
        YAML::Node motion_script_cmd_doc = motion_script_doc[cmd_key];
        if(motion_script_cmd_doc == NULL)
        {
            break;
        }

        //check validity of cmd_name
        motion_script_cmd temp_cmd;
        if(motion_script_cmd_doc["cmd_name"] == NULL)
        {
            std::string status_msg = cmd_key + " of " + "script" + convertIntToString(motion_script_index) + "is invalid.";
            ROS_ERROR_STREAM(status_msg);
            return false;
        }

        //check  validity of cmd_arg
        temp_cmd.cmd_name = motion_script_cmd_doc["cmd_name"].as<std::string>();
        if((temp_cmd.cmd_name != "wait") && (motion_script_cmd_doc["cmd_arg"] == NULL))
        {
            std::string status_msg = cmd_key + " of " + "script" + convertIntToString(motion_script_index) + "is invalid.";
            ROS_ERROR_STREAM(status_msg);
            return false;
        }


        if(temp_cmd.cmd_name == MOTION_PLAY_CMD_NAME)
        {
            temp_cmd.cmd_arg_int = convertStringToInt(motion_script_cmd_doc["cmd_arg"].as<std::string>());
        }
        else if(temp_cmd.cmd_name == MP3_PLAY_CMD_NAME)
        {
            temp_cmd.cmd_arg_str = motion_script_cmd_doc["cmd_arg"].as<std::string>();
        }
        else if(temp_cmd.cmd_name == WAIT_MOTION_CMD_NAME)
        {
            temp_cmd.cmd_arg_str = "";
            temp_cmd.cmd_arg_int = 0;
        }
        else if(temp_cmd.cmd_name == SLEEP_CMD_NAME)
        {
            temp_cmd.cmd_arg_int = convertStringToInt(motion_script_cmd_doc["cmd_arg"].as<std::string>());
        }
        else
        {
            std::string status_msg = cmd_key + " of " + "script" + convertIntToString(motion_script_index) + "is invalid.";
            ROS_ERROR_STREAM(status_msg);
            motion_script_data.clear();
            return false;
        }

        motion_script_data.push_back(temp_cmd);
        cmd_num++;
    }

    return true;
}

void motionScriptPlayThreadFunc(int motion_script_index)
{
    try
    {
        if(motion_script_index < 0)
        {
            std::string status_msg = "Invalid Motion Script Index";
            ROS_ERROR_STREAM(status_msg);
            return;
        }

        if(IsActionRunning() == true)
        {
            std::string status_msg = "Previous motion playing is not finished.";
            ROS_ERROR_STREAM(status_msg);
            return;
        }

        if(parseMotionScript(motion_script_index) == false)
            return;

        for(unsigned int motion_script_data_idx = 0; motion_script_data_idx < motion_script_data.size(); motion_script_data_idx++)
        {
            std::string cmd_name = motion_script_data[motion_script_data_idx].cmd_name;
            std_msgs::Int32   action_page_num_msg;
            std_msgs::String  sound_file_name_msg;

            if(cmd_name == MOTION_PLAY_CMD_NAME)
            {
                action_page_num_msg.data = motion_script_data[motion_script_data_idx].cmd_arg_int;
                action_page_num_pub.publish(action_page_num_msg);
            }
            else if(cmd_name == MP3_PLAY_CMD_NAME)
            {
                sound_file_name_msg.data = motion_script_data[motion_script_data_idx].cmd_arg_str;
                sound_file_name_pub.publish(sound_file_name_msg);
            }
            else if(cmd_name == WAIT_MOTION_CMD_NAME)
            {
                while(true)
                {
                    if(IsActionRunning() == false)
                        break;

                    boost::this_thread::sleep(boost::posix_time::milliseconds(32));
                }
            }
            else if(cmd_name == SLEEP_CMD_NAME)
            {
                boost::this_thread::sleep(boost::posix_time::milliseconds(motion_script_data[motion_script_data_idx].cmd_arg_int));
            }
            else
            {
                continue;
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }

    }
    catch (boost::thread_interrupted&)
    {
        ROS_INFO("Motion Script Thread is Interrupted");
        return;
    }
}

void motionScriptNumberCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if((msg->data == -1) || (msg->data == -2)) //Stop or Break
    {
        std_msgs::Int32   action_page_num_msg;
        action_page_num_msg.data = msg->data;
        action_page_num_pub.publish(action_page_num_msg);

        if((motion_script_play_thread != 0) && (motion_script_play_thread->get_thread_info()->done == false))
        {
            motion_script_play_thread->interrupt();
            motion_script_play_thread->join();
        }
    }
    else
    {
        if((motion_script_play_thread == 0) || (motion_script_play_thread->get_thread_info() == 0))
        {
            motion_script_play_thread = new boost::thread(motionScriptPlayThreadFunc, msg->data);
        }
        else if(motion_script_play_thread->get_thread_info()->done == true)
        {
            delete motion_script_play_thread;
            motion_script_play_thread = new boost::thread(motionScriptPlayThreadFunc, msg->data);
        }
        else
        {
            std::string status_msg = "Previous motion script is not finished.";
            ROS_ERROR_STREAM(status_msg);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "THORMANG3_MOTION_SCRIPT_PLAYER");
    ros::NodeHandle ros_node_handle;

    motion_script_play_thread = 0;

    motion_script_num_sub = ros_node_handle.subscribe("/robotis/demo/motion_index", 1, &motionScriptNumberCallback);
    action_page_num_pub   = ros_node_handle.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
    sound_file_name_pub   = ros_node_handle.advertise<std_msgs::String>("/play_sound_file", 0);
    is_running_client     = ros_node_handle.serviceClient<thormang3_action_module_msgs::IsRunning>("/robotis/action/is_running");

    //Setting motion script file path
    std::string temp_motion_script_file_path = ros::package::getPath("thormang3_motion_script_player") + "/script/motion_script.yaml";
    if(ros_node_handle.getParam("motion_script_file_path", motion_script_file_path) == false)
    {
        motion_script_file_path = temp_motion_script_file_path;
        ROS_WARN("Failed to get motion script_file_path.");
        ROS_WARN("The default motion script file path will be used.");
    }

    ROS_INFO("Start ThorMang3 Motion Script Player");

    ros::spin();
}








