#include <ros/ros.h>
#include <iostream>
#include "Controller.h"
#include <vector>
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
using namespace std;

double freq,L,V_DESIRED;
double v_max;
bool limit_v_and_kesi;
double initial_x,initial_y,initial_yaw,initial_v,initial_kesi;
double slow_LEVE1_DISTANCE,slow_LEVE2_DISTANCE,slow_LEVE1_V,slow_LEVE2_V,goal_tolerance_DISTANCE;
#define pi acos(-1)
#define T 1/freq 
 
vehicleState update_state(U control, vehicleState car) {  // this is useful
	car.v = control.v;
	car.kesi = control.kesi;
	car.x += car.v * cos(car.yaw) * T;
	car.y += car.v * sin(car.yaw) * T;
	car.yaw += car.v / L * tan(car.kesi) * T;
	return car;
}
 
class Path {
private:
	vector<waypoint> path;
public:

	void Add_new_point(waypoint& p)
	{
		path.push_back(p);
	}
 
	void Add_new_point(vector<waypoint>& p) 
	{
		path = p;
	}
 
	
	unsigned int Size()
	{
		return path.size();
	}
 

	waypoint Get_waypoint(int index)
	{
		waypoint p;
		p.ID = path[index].ID;
		p.x = path[index].x;
		p.y = path[index].y;
		p.yaw = path[index].yaw;
		return p;
	}

	vector<waypoint> Get_waypoints(){
		return path;
	}
 
 
	
	int Find_target_index(vehicleState state)  // this one is nice
	{
		double min = abs(sqrt(pow(state.x - path[0].x, 2) + pow(state.y - path[0].y, 2)));
		int index = 0;
		for (int i = 0; i < path.size(); i++)
		{
			double d = abs(sqrt(pow(state.x - path[i].x, 2) + pow(state.y - path[i].y, 2)));
			if (d < min)
			{
				min = d;
				index = i;
			}
		}
 
	
		if ((index + 1) < path.size())
		{
			double current_x = path[index].x; double current_y = path[index].y;
			double next_x = path[index + 1].x; double next_y = path[index + 1].y;
			double L_ = abs(sqrt(pow(next_x - current_x, 2) + pow(next_y - current_y, 2)));
			double L_1 = abs(sqrt(pow(state.x - next_x, 2) + pow(state.y - next_y, 2)));
			
			if (L_1 < L_)
			{
				index += 1;
			}
		}
		
		return index;
	}
 
};

// depending on your controller
std::vector<double> Q_set;
std::vector<double> R_set;
 
class Control_node {
private:
	//car
	vehicleState car;
	U control;
	
	int lastIndex;
	waypoint lastPoint;
	string action;//(tracking or reach goal!)
	
	//ROS
	ros::Subscriber path_sub;
	ros::Publisher vel_pub;
	ros::Publisher actual_state_pub;
	ros::Publisher visual_state_pub;
	geometry_msgs::Point visual_state_pose;
	visualization_msgs::Marker visual_state_trajectory;
	geometry_msgs::Pose2D actual_pose;
	geometry_msgs::Twist vel_msg;
	int temp;
    
	// depending on your controller
	double Q[3];
	double R[2];
	double kp_v, kd_v, kp_steer, kd_steer;

 
public:
    Controller* controller;
    Path* path;

	Control_node(ros::NodeHandle& nh)
	{
        		controller = new Controller();
        		path = new Path();
        
		//ROS:
		path_sub = nh.subscribe("path",10,&Control_node::addpointcallback,this);
		vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
		visual_state_pub = nh.advertise<visualization_msgs::Marker>("visualization_pose",10);
		actual_state_pub = nh.advertise<geometry_msgs::Pose2D>("Control_pose",10);

		nh.param<double>("/Control_node/kp_speed", kp_v, 1.0);
		nh.param<double>("/Control_node/kd_speed", kd_v, 1.0);
		nh.param<double>("/Control_node/kp_steer", kp_steer, 1.0);
		nh.param<double>("/Control_node/kd_steer", kd_steer, 1.0);
		action = "ready for tracking!!";

		//robot state initialize:
		car.x = initial_x;car.y = initial_y;car.yaw = initial_yaw;car.v = initial_v;car.kesi = initial_kesi;
	}
 
	~Control_node() {
        		delete(controller);
        		delete(path);
	}

	void addpointcallback(const nav_msgs::Path::ConstPtr& msg){
		vector<waypoint> waypoints;
		for(int i=0;i<msg->poses.size();i++){
			waypoint waypoint;
			//ROS_INFO("THE PATH[%d]'s ID is %d",i,msg->poses[i].header.seq);
			waypoint.ID = msg->poses[i].header.seq;
			waypoint.x = msg->poses[i].pose.position.x;
			waypoint.y = msg->poses[i].pose.position.y;
			
			double roll,pitch,yaw;
	    		tf::Quaternion quat;
	    		tf::quaternionMsgToTF(msg->poses[i].pose.orientation, quat);
	    		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			waypoint.yaw = yaw;
			waypoints.push_back(waypoint);
		}
		path->Add_new_point(waypoints);
		lastIndex = path->Size() - 1;
		lastPoint = path->Get_waypoint(lastIndex);
		if(action == "ready for tracking!!")
			action = "the car is tracking!!";	
	}
 
	double slow_judge(double distance) {
		if (distance>=slow_LEVE2_DISTANCE&&distance <= slow_LEVE1_DISTANCE) {
			return slow_LEVE1_V;
		}
		else if (distance>=goal_tolerance_DISTANCE&&distance < slow_LEVE2_DISTANCE) {
			return slow_LEVE2_V;
		}
		else if (distance < goal_tolerance_DISTANCE) {
			// action = "the car has reached the goal!";
			return 0.0;
		}
		else
		{
			return V_DESIRED;
		}
	}
 
	//The controller process

	void Controller_track(){
		if(action != "the car is tracking!!")
			return;
		//Search for path points
		int idx = path->Find_target_index(car);
		waypoint track_p = path->Get_waypoint(idx);
		double distance = sqrt(pow(car.x - track_p.x, 2) + pow(car.y - track_p.y, 2));
		std::cout << "car: " << car.x << " " << car.y << " des: " << track_p.x << " " << track_p.y << std::endl;
		double theta_ref = track_p.yaw;
		double des_v = slow_judge(distance);
		std::cout << "des_v" <<des_v << std::endl;
		//Construct the desired control amount
		// PID gogogo
		double theta = atan2(track_p.y-car.y,track_p.x-car.x);
		double e_long, e_latt, de_long, de_latt;
		std::cout << "theta_ref: " << theta_ref << std::endl;
		e_long = cos(theta_ref)*(car.x - track_p.x) + sin(theta_ref)*(car.y - track_p.y);
		e_latt = -sin(theta_ref)*(car.x - track_p.x) + cos(theta_ref)*(car.y-track_p.y);
		// derivative
		de_long = car.v * cos(car.yaw - track_p.yaw) - des_v;
		de_latt = car.v * sin(car.yaw - track_p.yaw);
		control.v = -kp_v * e_long - kd_v * de_long;
		control.kesi = -kp_steer * e_latt + kd_steer * de_latt;
		std::cout << "e_long: " << e_long << " de_long: " << de_long << " e_latt: " << e_latt << std::endl;
		std::cout << "v: " << control.v << " kesi " << control.kesi <<std::endl; 
		control = v_and_kesi_limit(control);
		//Use controller
		car = update_state(control, car);
		//topic release
		PUB();
		//Compare the state quantity with the expected value
		static int i = 0;
		static double ave_err_p = 0.0;
		static double ave_angle_err = 0.0;
		static double max_err_p = 0.0;
		static double max_err_angle = 0.0;
		double e_p = distance;
		double e_angle = abs(track_p.yaw - car.yaw);
		ave_err_p = (ave_err_p * i + e_p) / (i+1);
		ave_angle_err = (ave_angle_err * i + e_angle) / (i+1);
		i++;
		std::cout << "average position error: " << ave_err_p << " angle error:" << ave_angle_err <<std::endl;
		if(e_p > max_err_p)
			max_err_p = e_p;
		if(e_angle > max_err_angle)
			max_err_angle = e_angle;
		std::cout << "max position error: " << max_err_p << " angle error: " << e_angle  << std::endl;
	}
	

	void node_control() {
        ros::Rate loop_rate(freq);
		Marker_set();

		tf::TransformBroadcaster br;
		tf::Transform transform;
		tf::Quaternion q;

		while (ros::ok()) {
			transform.setOrigin(tf::Vector3(car.x, car.y, 0));
			q.setRPY(0, 0, car.yaw);
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "car"));

			ros::spinOnce();
			Controller_track();
			loop_rate.sleep();
		}
	}

	void PUB(){
		visual_state_pose.x = car.x; visual_state_pose.y = car.y;
		actual_pose.x = car.x; actual_pose.y = car.y; actual_pose.theta = car.yaw;
		vel_msg.linear.x = control.v; vel_msg.angular.z = control.v*tan(control.kesi)/L;
		visual_state_trajectory.points.push_back(visual_state_pose);
		visual_state_pub.publish(visual_state_trajectory);
		vel_pub.publish(vel_msg);
		actual_state_pub.publish(actual_pose);
	}

	void shutdown_controller(){
		if(action == "the car has reached the goal!"){
			temp+=1;
			if(temp ==50){
				ROS_WARN("shutdown the controller!");
				temp = 0;
				ros::shutdown();
			}
		}
	}

	void Marker_set(){
	
		visual_state_trajectory.header.frame_id = "map";
		visual_state_trajectory.header.stamp = ros::Time::now();
		visual_state_trajectory.action = visualization_msgs::Marker::ADD;
		visual_state_trajectory.ns = "Control";
	
		visual_state_trajectory.id = 0;
		visual_state_trajectory.type = visualization_msgs::Marker::POINTS;
		visual_state_trajectory.scale.x = 0.02;
		visual_state_trajectory.scale.y = 0.02;
		visual_state_trajectory.color.r = 1.0;
		visual_state_trajectory.color.a = 1.0;
	}

	U v_and_kesi_limit(U control_value){
		if(control_value.v>=v_max)
		{
			control_value.v = v_max;
			ROS_WARN("The calculated value may be inaccurate ");
		}
		else if(control_value.v<=-v_max){
			control_value.v = -v_max;
			ROS_WARN("The calculated value may be inaccurate ");
		}
			

		if(control_value.kesi>=pi/2)
		{
			control_value.kesi = pi/2;
			ROS_WARN("The calculated value may be inaccurate ");
		}
		else if(control_value.kesi<=-pi/2){
			control_value.kesi = -pi/2;
			ROS_WARN("The calculated value may be inaccurate ");
		}
		return control_value;
	}
};
 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Control_node");
	ros::NodeHandle n;
	ros::NodeHandle n_prv("~");

	n_prv.param<double>("freq",freq,20);
	n_prv.param<double>("L",L,0.2);
	n_prv.param<double>("V_DESIRED",V_DESIRED,0.5);
	n_prv.param<double>("v_max",v_max,1.0);
	n_prv.param<double>("initial_x",initial_x,0.0);
	n_prv.param<double>("initial_y",initial_y,2.0);
	n_prv.param<double>("initial_yaw",initial_yaw,0.0);
	n_prv.param<double>("initial_v",initial_v,0.0);
	n_prv.param<double>("initial_kesi",initial_kesi,0.1);
	n_prv.param<double>("slow_LEVE1_DISTANCE",slow_LEVE1_DISTANCE,5.0);
	n_prv.param<double>("slow_LEVE2_DISTANCE",slow_LEVE2_DISTANCE,2.0);
	n_prv.param<double>("goal_tolerance_DISTANCE",goal_tolerance_DISTANCE,0.1);
	n_prv.param<double>("slow_LEVE1_V",slow_LEVE1_V,0.35);
	n_prv.param<double>("slow_LEVE2_V",slow_LEVE2_V,0.15);
	n_prv.param<bool>("limit_v_and_kesi",limit_v_and_kesi,false);

	// depends on the controller desigh
	n_prv.param("Q_set",Q_set,Q_set);
	n_prv.param("R_set",R_set,R_set);

	Control_node* node = new Control_node(n);
	node->node_control();
	return (0);
}

 
