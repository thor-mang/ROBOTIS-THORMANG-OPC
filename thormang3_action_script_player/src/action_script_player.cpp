/*
 * thormang3_action_script_player.cpp
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

#define ACTION_PLAY_CMD_NAME              "play"
#define MP3_PLAY_CMD_NAME                 "mp3"
#define WAIT_ACTION_PLAY_FINISH_CMD_NAME  "wait"
#define SLEEP_CMD_NAME                    "sleep"



ros::Subscriber    action_script_num_sub;
ros::Publisher     action_page_num_pub;
ros::Publisher     sound_file_name_pub;
ros::ServiceClient is_running_client;

thormang3_action_module_msgs::IsRunning  is_running_srv;

boost::thread   *action_script_play_thread;


std::string action_script_file_path;

typedef struct
{
    std::string cmd_name;
    std::string cmd_arg_str;
    int         cmd_arg_int;
} action_script_cmd;

std::vector<action_script_cmd> action_script_data;

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


bool parseActionScript(int action_script_index)
{
    action_script_data.clear();

    YAML::Node action_script_file_doc;
    try
    {
        // load yaml
        action_script_file_doc = YAML::LoadFile(action_script_file_path.c_str());
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Failed to load action script file.");
        return false;
    }


    //find action script
    std::string script_index_key = "script" + convertIntToString(action_script_index);
    YAML::Node  action_script_doc = action_script_file_doc[script_index_key];
    if(action_script_doc == NULL)
    {
        std::string status_msg = "Failed to find action script #" + convertIntToString(action_script_index);
        ROS_ERROR_STREAM(status_msg);
        return false;
    }

    int cmd_num = 1;
    std::string cmd_key = "";
    try
    {

    	while(true)
    	{
    		//check cmd exist
    		cmd_key = "cmd" + convertIntToString(cmd_num);
    		YAML::Node action_script_cmd_doc = action_script_doc[cmd_key];
    		if(action_script_cmd_doc == NULL)
    		{
    			break;
    		}

    		//check validity of cmd_name
    		action_script_cmd temp_cmd;
    		if(action_script_cmd_doc["cmd_name"] == NULL)
    		{
    			std::string status_msg = "cmd#" + convertIntToString(cmd_num) + " of " + "script#" + convertIntToString(action_script_index) + " is invalid.";
    			ROS_ERROR_STREAM(status_msg);
    			return false;
    		}

    		//check  validity of cmd_arg
    		temp_cmd.cmd_name = action_script_cmd_doc["cmd_name"].as<std::string>();
    		if((temp_cmd.cmd_name != "wait") && (action_script_cmd_doc["cmd_arg"] == NULL))
    		{
    			std::string status_msg = "cmd#" + convertIntToString(cmd_num) + " of " + "script#" + convertIntToString(action_script_index) + " is invalid.";
    			ROS_ERROR_STREAM(status_msg);
    			return false;
    		}

    		//get cmd_arg
    		if(temp_cmd.cmd_name == ACTION_PLAY_CMD_NAME)
    		{
    			temp_cmd.cmd_arg_int = action_script_cmd_doc["cmd_arg"].as<int>();
    		}
    		else if(temp_cmd.cmd_name == MP3_PLAY_CMD_NAME)
    		{
    			temp_cmd.cmd_arg_str = action_script_cmd_doc["cmd_arg"].as<std::string>();
    		}
    		else if(temp_cmd.cmd_name == WAIT_ACTION_PLAY_FINISH_CMD_NAME)
    		{
    			temp_cmd.cmd_arg_str = "";
    			temp_cmd.cmd_arg_int = 0;
    		}
    		else if(temp_cmd.cmd_name == SLEEP_CMD_NAME)
    		{
    			temp_cmd.cmd_arg_int = action_script_cmd_doc["cmd_arg"].as<int>();
    			if(temp_cmd.cmd_arg_int < 0 )
    			{
        			std::string status_msg = "cmd#" + convertIntToString(cmd_num) + " of " + "script#" + convertIntToString(action_script_index) + " is invalid.";
        			ROS_ERROR_STREAM(status_msg);
        			action_script_data.clear();
        			return false;
    			}
    		}
    		else
    		{
    			std::string status_msg = "cmd#" + convertIntToString(cmd_num) + " of " + "script#" + convertIntToString(action_script_index) + " is invalid.";
    			ROS_ERROR_STREAM(status_msg);
    			action_script_data.clear();
    			return false;
    		}

    		action_script_data.push_back(temp_cmd);
    		cmd_num++;
    	}
    }
    catch(const std::exception& e)
    {
		std::string status_msg = "cmd#" + convertIntToString(cmd_num) + " of " + "script#" + convertIntToString(action_script_index) + " is invalid.";
		ROS_ERROR_STREAM(status_msg);
		action_script_data.clear();
    	return false;
    }

    return true;
}

void actionScriptPlayThreadFunc(int action_script_index)
{
    try
    {
        if(action_script_index < 0)
        {
            std::string status_msg = "Invalid Action Script Index";
            ROS_ERROR_STREAM(status_msg);
            return;
        }

        if(IsActionRunning() == true)
        {
            std::string status_msg = "Previous action playing is not finished.";
            ROS_ERROR_STREAM(status_msg);
            return;
        }

        if(parseActionScript(action_script_index) == false)
            return;

        for(unsigned int action_script_data_idx = 0; action_script_data_idx < action_script_data.size(); action_script_data_idx++)
        {
            std::string cmd_name = action_script_data[action_script_data_idx].cmd_name;
            std_msgs::Int32   action_page_num_msg;
            std_msgs::String  sound_file_name_msg;

            if(cmd_name == ACTION_PLAY_CMD_NAME)
            {
                action_page_num_msg.data = action_script_data[action_script_data_idx].cmd_arg_int;
                action_page_num_pub.publish(action_page_num_msg);
            }
            else if(cmd_name == MP3_PLAY_CMD_NAME)
            {
                sound_file_name_msg.data = action_script_data[action_script_data_idx].cmd_arg_str;
                sound_file_name_pub.publish(sound_file_name_msg);
            }
            else if(cmd_name == WAIT_ACTION_PLAY_FINISH_CMD_NAME)
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
                boost::this_thread::sleep(boost::posix_time::milliseconds(action_script_data[action_script_data_idx].cmd_arg_int));
            }
            else
            {
            	boost::this_thread::interruption_point();
                continue;
            }
            boost::this_thread::interruption_point();
        }

    }
    catch (boost::thread_interrupted&)
    {
        ROS_INFO("Action Script Thread is Interrupted");
        return;
    }
}

void actionScriptNumberCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if((msg->data == -1) || (msg->data == -2)) //Stop or Break
    {
        std_msgs::Int32   action_page_num_msg;
        action_page_num_msg.data = msg->data;
        action_page_num_pub.publish(action_page_num_msg);

        if((action_script_play_thread != 0)	&& (action_script_play_thread->get_thread_info() != 0))
        {
        	if(action_script_play_thread->get_thread_info()->done == false)
        	{
        		action_script_play_thread->interrupt();
        		action_script_play_thread->join();
        	}
        }
    }
    else
    {
        if((action_script_play_thread == 0))
        {
            action_script_play_thread = new boost::thread(actionScriptPlayThreadFunc, msg->data);
        }
        else if(action_script_play_thread->get_thread_info() == 0)
        {
        	action_script_play_thread = new boost::thread(actionScriptPlayThreadFunc, msg->data);
        }
        else if(action_script_play_thread->get_thread_info()->done == true)
        {
            delete action_script_play_thread;
            action_script_play_thread = new boost::thread(actionScriptPlayThreadFunc, msg->data);
        }
        else
        {
            std::string status_msg = "Previous action script is not finished.";
            ROS_ERROR_STREAM(status_msg);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "THORMANG3_ACTION_SCRIPT_PLAYER");
    ros::NodeHandle ros_node_handle;

    action_script_play_thread = 0;

    action_script_num_sub = ros_node_handle.subscribe("/robotis/demo/action_index", 1, &actionScriptNumberCallback);
    action_page_num_pub   = ros_node_handle.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
    sound_file_name_pub   = ros_node_handle.advertise<std_msgs::String>("/play_sound_file", 0);
    is_running_client     = ros_node_handle.serviceClient<thormang3_action_module_msgs::IsRunning>("/robotis/action/is_running");

    //Setting action script file path
    std::string temp_action_script_file_path = ros::package::getPath("thormang3_action_script_player") + "/script/action_script.yaml";
    if(ros_node_handle.getParam("action_script_file_path", action_script_file_path) == false)
    {
        action_script_file_path = temp_action_script_file_path;
        ROS_WARN("Failed to get action script_file_path.");
        ROS_WARN("The default action script file path will be used.");
    }

    ROS_INFO("Start ThorMang3 Action Script Player");

    ros::spin();
}








