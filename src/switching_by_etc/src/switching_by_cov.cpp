#include <ros/ros.h>                        
#include <std_srvs/SetBool.h>                
#include <std_srvs/Trigger.h>                
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32.h>
#include <array>

double wait_t;
double cov_diff;
bool ch_flag=true;
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
boost::array<double , 36> old_pose;
boost::array<double , 36> current_pose;

void mode(double cov)
    {
        if (cov>=4.0)
        {
            req_ler_str.data=true;
            bool  start_learning = StartClient.call(req_ler_str,resp_ler); 
             ROS_INFO("learning_mode!");    
        }
        
        else 
            req_ler_end.data=false;
            bool start_learning = StartClient.call(req_ler_end,resp_ler); 
            //initial_pose_pub.publish(pose_msg);
            bool  start_waypointnav = Start_Wp_Client.call(req,resp); 
            ROS_INFO("waypoint_mode!");
    }

void Pose_Callback(geometry_msgs::PoseWithCovarianceStamped p)
    {
        if(ch_flag)
        {
            old_pose =  p.pose.covariance;
            ch_flag=false;
        }
        else
        {
            current_pose = p.pose.covariance;
            for (int i=0; i!=current_pose.size(); i++)
            {
                cov_diff = current_pose[i] - old_pose[i];
            }
            
        ROS_INFO("%f", &current_pose);
        mode(cov_diff);
        old_pose = current_pose;
        } 
    }

int main(int argc, char **argv) {
    ros::init(argc, argv, "swiching_by_time");        
    ros::NodeHandle nh;                    

    Start_Wp_Client = nh.serviceClient<std_srvs::Trigger>("start_wp_nav"); 
    StartClient = nh.serviceClient<std_srvs::SetBool>("learn_out"); 
    ROS_INFO("ready!");
    pose_sub = nh.subscribe("amcl_pose", 10, &Pose_Callback);
    ros::spin();
  return 0;
}   

