#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <fstream>

#include <boost/shared_ptr.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h> 

#include <visualization_msgs/Marker.h>

#include <list>

using namespace std;

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

    // Connect to ROS
    ros::init(argc, argv, "simple_navigation_goals");
  
    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
  
    // Wait for the action server to come up so that we can begin processing goals.
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseGoal goal;
    
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    // 在 visualization_marker 上广播
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = "map";
    points_marker.header.stamp = ros::Time();
    
    // 设置 Marker 的 namespace 和 id。 
    // 相同的 namespace 和 id 相同时新 marker 会覆盖旧值
    points_marker.ns = "points";
    points_marker.id =0;

	// 动作：支持添加 ADD/删除 DELETE/删除全部 DELETEALL
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    
    points_marker.type = visualization_msgs::Marker::POINTS;
    
    // Marker 的比例，这里 1.0 为 1m
    points_marker.scale.x = 1.0;
    points_marker.scale.y = 1.0;
    points_marker.scale.z = 1.0;
    
    // 设置 Marker 颜色（此处为绿色
    points_marker.color.g = 1.0f;
    points_marker.color.a = 1.0;
    
    string line;
    ifstream pointsFile("/home/maary/points.txt");
    while (getline(pointsFile, line)) {
      stringstream ss(line);
      string x, y;
      getline(ss, x, ',');
      getline(ss, y, ',');
    	
    	// 生成并发布 Marker 在 rviz 中显示
    	geometry_msgs::Point p;
		p.x = std::stof(x);
  		p.y = std::stof(y);
	  	p.z = 1.0;

		  points_marker.points.push_back(p);
	
		  while (marker_pub.getNumSubscribers() < 1){
      		if (!ros::ok()){
        		return 0;
      		}
      		ROS_WARN_ONCE("Please create a subscriber to the marker");
    		sleep(1);
	  	}
	  	marker_pub.publish(points_marker);    	
    }
    
    
  return 0;
}
