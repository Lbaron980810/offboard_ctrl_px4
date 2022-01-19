#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/PositionTarget.h>

using namespace std;
// bool debug = true;
bool debug = false;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// save waypoints as a vector
vector<mavros_msgs::PositionTarget> waypointList;
vector<double> traj_time;
bool path_updated = false;
void waypoint_cb(const std_msgs::Float64MultiArray msg){
    if (debug) ROS_WARN("DEBUG --- entry of waypoint_cb");
    if (debug) ROS_WARN("%d, %f", msg.data.size(), msg.data[0]);
    if (msg.data.size())
    {
        if (debug) ROS_WARN("DEBUG --- waypoint_cb msg size not 0");
        waypointList.clear();
        traj_time.clear();
        int timesize = (int) msg.data[0]; //　第一个是时间的尺寸
        if (debug) ROS_WARN("DEBUG --- waypoint_cb timesize");
        int dev_order = (msg.data.size() - timesize - 1) / (6 * timesize); // 控制到几阶导数
        int wpl_size = dev_order * 2 * timesize; // rows of Matrix waypoint list
        for (int i = 1; i < timesize + 1; i++) // 因为msg的第一个数据是timesize,所以要取第1个到第timesize个
        {
            traj_time.push_back(msg.data[i]);
        }
        int point_num = traj_time.size() + 1; 
        if (debug) ROS_WARN("DEBUG --- params of waypoint_cb: %d, %d, %d, %d", timesize, dev_order, wpl_size, point_num);
        for (int i = 0; i < point_num - 1; i++)
        {
            mavros_msgs::PositionTarget target;
            target.coordinate_frame = target.FRAME_LOCAL_NED;
            target.position.x = msg.data[timesize + 1 + i * dev_order * 2];
            target.position.y = msg.data[timesize + 1 + i * dev_order * 2 + wpl_size];
            target.position.z = msg.data[timesize + 1 + i * dev_order * 2 + wpl_size * 2];
            target.velocity.x = msg.data[timesize + 1 + i * dev_order * 2 + 1];
            target.velocity.y = msg.data[timesize + 1 + i * dev_order * 2 + 1 + wpl_size];
            target.velocity.z = msg.data[timesize + 1 + i * dev_order * 2 + 1 + wpl_size * 2];
            target.acceleration_or_force.x = msg.data[timesize + 1 + i * dev_order * 2 + 2 + wpl_size * 0];
            target.acceleration_or_force.y = msg.data[timesize + 1 + i * dev_order * 2 + 2 + wpl_size * 1];
            target.acceleration_or_force.z = msg.data[timesize + 1 + i * dev_order * 2 + 2 + wpl_size * 2];
            target.type_mask = target.IGNORE_YAW + target.IGNORE_YAW_RATE;
            waypointList.push_back(target);
            if (debug) {
                ROS_INFO("target pva: %f, %f, %f, %f, %f, %f, %f, %f, %f",
                        target.position.x, target.position.y, target.position.z,
                        target.velocity.x, target.velocity.y, target.velocity.z,
                        target.acceleration_or_force.x, target.acceleration_or_force.y, target.acceleration_or_force.z);
            }
        }
        mavros_msgs::PositionTarget lastpoint;
        lastpoint.coordinate_frame = lastpoint.FRAME_LOCAL_NED;
        lastpoint.position.x = msg.data[timesize + 1 + (point_num - 1) * dev_order * 2 + wpl_size * 0 - dev_order];
        lastpoint.position.y = msg.data[timesize + 1 + (point_num - 1) * dev_order * 2 + wpl_size * 1 - dev_order];
        lastpoint.position.z = msg.data[timesize + 1 + (point_num - 1) * dev_order * 2 + wpl_size * 2 - dev_order];
        lastpoint.velocity.x = msg.data[timesize + 1 + (point_num - 1) * dev_order * 2 + 1 + wpl_size * 0 - dev_order + 1];
        lastpoint.velocity.y = msg.data[timesize + 1 + (point_num - 1) * dev_order * 2 + 1 + wpl_size * 1 - dev_order + 1];
        lastpoint.velocity.z = msg.data[timesize + 1 + (point_num - 1) * dev_order * 2 + 1 + wpl_size * 2 - dev_order + 1];
        lastpoint.acceleration_or_force.x = msg.data[timesize + 1 + (point_num - 1) * dev_order * 2 + 2 + wpl_size * 0 - dev_order + 2];
        lastpoint.acceleration_or_force.y = msg.data[timesize + 1 + (point_num - 1) * dev_order * 2 + 2 + wpl_size * 1 - dev_order + 2];
        lastpoint.acceleration_or_force.z = msg.data[timesize + 1 + (point_num - 1) * dev_order * 2 + 2 + wpl_size * 2 - dev_order + 2];
        lastpoint.type_mask = lastpoint.IGNORE_YAW + lastpoint.IGNORE_YAW_RATE;
        waypointList.push_back(lastpoint);
        if (debug) {
            ROS_INFO("last pva: %f, %f, %f, %f, %f, %f, %f, %f, %f",
                    lastpoint.position.x, lastpoint.position.y, lastpoint.position.z,
                    lastpoint.velocity.x, lastpoint.velocity.y, lastpoint.velocity.z,
                    lastpoint.acceleration_or_force.x, lastpoint.acceleration_or_force.y, lastpoint.acceleration_or_force.z);

        }
        path_updated = true;
    }
   
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher traj_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
            

    // ros::Subscriber globalpose_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &globalpose_cb);
    ros::Subscriber waypoint_sub = nh.subscribe<std_msgs::Float64MultiArray>
            ("/trajectory_generator_node/waypoint_list_traj", 10, waypoint_cb);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // set vehicle start point
    geometry_msgs::PoseStamped pose_start;
    pose_start.pose.position.x = 0;
    pose_start.pose.position.y = 0;
    pose_start.pose.position.z = 2;

    // send a few setpoints before starting
    for(int i = 300; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose_start);
        ros::spinOnce();
        rate.sleep();
    }

    
    bool path_finished_flag = false;

    // Change mode and arm the vehicle
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && 
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                // ROS_ERROR("Not armed");
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose_start);
        // send new waypoint if the vehicle reached last one
        if (path_updated)
        {
            // iter = waypointList.begin();
            cout << "path updated" << endl;
            path_updated = false;
            // path_finished_flag = true;
        }

        if (!waypointList.empty()){
            for (int i = 0; i < waypointList.size(); i++)
            {
                mavros_msgs::PositionTarget pose_next = waypointList[i];
                traj_pub.publish(pose_next);
                float dura_of_traj = traj_time[i];
                ros::Duration(dura_of_traj).sleep();
                // 延时
            }
            waypointList.clear();
        }
        // cout << "current pose:" << current_pose.pose.position.x  << endl;
        // cout << "next pose:" << pose_next.pose.position.x << endl;
        // cout << "diff x" << current_pose.pose.position.x - pose_next.pose.position.x << endl;
        // cout << "diff y" << current_pose.pose.position.y - pose_next.pose.position.y << endl;
        // cout << "diff z" << current_pose.pose.position.z - pose_next.pose.position.z << endl;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

