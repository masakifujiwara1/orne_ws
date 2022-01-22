#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "math.h"
#include <ros/ros.h>                        // ROSヘッダファイル
#include <std_srvs/SetBool.h>                // サービスヘッダファイル
#include <std_srvs/Trigger.h>                // サービスヘッダファイル
#include <std_msgs/Bool.h>
//#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#define PI 3.1415926

double wait_t;
std_srvs::Trigger::Request req;             
std_srvs::Trigger::Response resp;           
std_srvs::SetBool::Request req_ler_str;             
std_srvs::SetBool::Request req_ler_end;             
std_srvs::SetBool::Response resp_ler;           
geometry_msgs::PoseWithCovarianceStamped pose_msg;
ros::ServiceClient Start_Wp_Client; 
ros::ServiceClient StartClient;
ros::ServiceClient set_model_state_client;
ros::ServiceServer srv;
ros::Publisher initial_pose_pub; 
ros::Subscriber reset_sub;
ros::Publisher gazebo_pose_pub;

void initial_pose_set(float pose_x,float pose_y,float ori_z,float ori_w)//initial_pose set function
    
    {
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.pose.position.x = pose_x;
    pose_msg.pose.pose.position.y = pose_y;
    pose_msg.pose.covariance[0] = 0.25;
    pose_msg.pose.covariance[6 * 1 + 1] = 0.25;
    pose_msg.pose.covariance[6 * 5 + 5] = 0.06853891945200942;
    pose_msg.pose.pose.orientation.z = ori_z;
    pose_msg.pose.pose.orientation.w = ori_w;
    initial_pose_pub.publish(pose_msg);
    }
    
void gazebo_pose_set(std::string model_name, std::string reference_frame, geometry_msgs::Pose pose, geometry_msgs::Twist model_twist)

  {
  gazebo_msgs::SetModelState setmodelstate;

    // Model state msg
    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = model_name;
    modelstate.reference_frame = reference_frame;
    modelstate.pose = pose;
    modelstate.twist = model_twist;
    gazebo_pose_pub.publish(modelstate);

  }

void CallBack(const std_msgs::Int8 & msg)
 {
  geometry_msgs::Pose model_pose;
    model_pose.position.x = 0;
    model_pose.position.y = 0;
    model_pose.position.z = 0;
    model_pose.orientation.x = 0.0;
    model_pose.orientation.y = 0.0;
    model_pose.orientation.z = 0.0;
    model_pose.orientation.w = 0.0;

  geometry_msgs::Twist model_twist;
    model_twist.linear.x = 0.0;
    model_twist.linear.y = 0.0;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;

  initial_pose_set(-63.35,-105.17,0.96,0.279);
  gazebo_pose_set("mobile_base", "world", model_pose, model_twist);

 }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_initpose");
  ros::NodeHandle nh;
  //ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
  ros::Rate loop_rate(10);
  //define 2d estimate pose
  initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);//initial_pose publish
  gazebo_pose_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);//gazebo_pub
  Start_Wp_Client = nh.serviceClient<std_srvs::Trigger>("start_wp_nav");  //create start waypoint service  Client for waypoint nav
    //StartClient = nh.serviceClient<std_srvs::SetBool>("learn_out");  //create start learning service  Client for learning
    ROS_INFO("ready!");
  reset_sub = nh.subscribe("reset_pose", 10, &CallBack);//subscribe topic "swiching" from waypoint_nav

  Start_Wp_Client = nh.serviceClient<std_srvs::Trigger>("start_wp_nav");  //create start waypoint service  Client for waypoint nav
    //StartClient = nh.serviceClient<std_srvs::SetBool>("learn_out");  //create start learning service  Client for learning
  ROS_INFO("ready!");
  reset_sub = nh.subscribe("reset_pose", 1, &CallBack);//subscribe topic "swiching" from waypoint_nav
  ros::spin();
  return 0;
}
