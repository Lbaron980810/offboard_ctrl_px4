#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <gazebo_msgs/ModelStates.h>

using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// save waypoints as a vector
vector<geometry_msgs::PoseStamped> waypointList;
bool path_updated = false;
void waypoint_cb(const visualization_msgs::Marker msg){
    if (msg.ns == "demo_node/astar_path")
    {
        waypointList.clear();
        int pointNum = msg.points.size();
        geometry_msgs::PoseStamped point;
        for (int i = 0; i < pointNum; i++)
        {
            point.pose.position.x = msg.points[i].x;
            point.pose.position.y = msg.points[i].y;
            point.pose.position.z = msg.points[i].z;
            waypointList.push_back(point);
        }
    }
    path_updated = true;   
}
 
void voxbloxRrtPoly_cb(const visualization_msgs::MarkerArray msg){
    for (int i = 0; i < msg.markers.size(); i++)
    {
        visualization_msgs::Marker marker = msg.markers[i];
        if (marker.ns == "poly")
        {
            waypointList.clear();
            int pointNum = marker.points.size();
            geometry_msgs::PoseStamped point;
            for (int i = 0; i < pointNum; i++)
            {
                point.pose.position.x = marker.points[i].x;
                point.pose.position.y = marker.points[i].y;
                point.pose.position.z = marker.points[i].z;
                waypointList.push_back(point);
            }
        }
    }
    path_updated = true;
    ROS_INFO("voxbloxRRTPoly result received!");
    cout << waypointList.size() << endl;
    ROS_INFO("Points number: %d", int(waypointList.size()));
}


// save current position and pose
geometry_msgs::PoseStamped current_pose;
void localpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}
void globalpose_cb(const gazebo_msgs::ModelStatesConstPtr &msg){
	int modelCount = msg->name.size();
	for (int modelInd = 0; modelInd < modelCount; modelInd++)
	{
		if (msg->name[modelInd] == "omni_hex")
		{
			current_pose.pose = msg->pose[modelInd];
			break;
		}
	}
}
// check whether the vehicle has reached the target point
bool reach_target(geometry_msgs::PoseStamped target){
    float thresh = 0.5;
    if (abs(current_pose.pose.position.x-target.pose.position.x) <= thresh && 
        abs(current_pose.pose.position.y-target.pose.position.y) <= thresh && 
        abs(current_pose.pose.position.z-target.pose.position.z) <= thresh)
    {
        return true;
    }
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
            
//    ros::Subscriber localpose_sub = nh.subscribe<geometry_msgs::PoseStamped>
//            ("mavros/local_position/pose", 10, localpose_cb);
    ros::Subscriber globalpose_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &globalpose_cb);
    ros::Subscriber waypoint_sub = nh.subscribe<visualization_msgs::Marker>
            ("demo_node/grid_path_vis", 10, waypoint_cb);
    ros::Subscriber voxbloxRrtPoly_sub = nh.subscribe<visualization_msgs::MarkerArray>
            ("/voxblox_rrt_planner/path", 10, voxbloxRrtPoly_cb);

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

    // variables needed later
    geometry_msgs::PoseStamped pose_next = pose_start;
    std::vector<geometry_msgs::PoseStamped>::iterator iter;
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

        // send new waypoint if the vehicle reached last one
        if (path_updated)
        {
            iter = waypointList.begin();
            cout << "path updated" << endl;
            path_updated = false;
            path_finished_flag = true;
        }

        if (!waypointList.empty()){
            if (reach_target(pose_next) && iter != waypointList.end())
            {
                pose_next = *iter;
                iter++;
            }
            else if (reach_target(waypointList.back()) && iter == waypointList.end() && path_finished_flag)
            {
                cout << "path finished" << endl;
                path_finished_flag = false;
            }   
        }
            
        local_pos_pub.publish(pose_next);
        cout << "diff x" << current_pose.pose.position.x - pose_next.pose.position.x << endl;
        cout << "diff y" << current_pose.pose.position.y - pose_next.pose.position.y << endl;
        cout << "diff z" << current_pose.pose.position.z - pose_next.pose.position.z << endl;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

