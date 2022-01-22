#include <ros/ros.h>                        // ROSヘッダファイル
#include <std_srvs/SetBool.h>                // サービスヘッダファイル
#include <std_srvs/Trigger.h>                // サービスヘッダファイル
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32.h>
#include <array>

double wait_t;
double cov_diff;

using std::copy;
std_srvs::Trigger::Request req;             
std_srvs::Trigger::Response resp;         
std_srvs::SetBool::Request req_ler_str;             
std_srvs::SetBool::Request req_ler_end;             
std_srvs::SetBool::Response resp_ler;           
geometry_msgs::PoseWithCovarianceStamped pose_msg;
geometry_msgs::PoseWithCovarianceStamped amcl_pose_msg;
ros::ServiceClient Start_Wp_Client; 
ros::ServiceClient StartClient; 
ros::ServiceServer srv;
ros::Publisher initial_pose_pub; 
ros::Subscriber swiching_sub;
ros::Subscriber pose_sub;
ros::Subscriber alpha_sub;

void mode(float alpha_threshold)
    {
        if (alpha_threshold <= 0.3)
        {
            req_ler_str.data=true;
            bool  start_learning = StartClient.call(req_ler_str,resp_ler);
            ROS_INFO("learning_mode!");
      
        }

        else
        {
            req_ler_end.data=false;
            bool start_learning = StartClient.call(req_ler_end,resp_ler);
            bool  start_waypointnav = Start_Wp_Client.call(req,resp);
            ROS_INFO("waypoint_mode!");
        }

    }
        

void Alpha_Callback(std_msgs::Float32 alpha_msg)
    {
    //ROS_INFO("%f",&alpha_msg.data);
    mode(alpha_msg.data);
    }


int main(int argc, char **argv) {
    ros::init(argc, argv, "swiching_by_alpha");
    ros::NodeHandle nh;                
    
    Start_Wp_Client = nh.serviceClient<std_srvs::Trigger>("start_wp_nav");  
    StartClient = nh.serviceClient<std_srvs::SetBool>("learn_out");  
    ROS_INFO("ready!");
    alpha_sub= nh.subscribe("alpha", 10, &Alpha_Callback);
    ros::spin();                              
  return 0;
}   

