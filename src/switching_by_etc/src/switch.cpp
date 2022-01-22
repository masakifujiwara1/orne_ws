#include <ros/ros.h>                        // ROSヘッダファイル
#include <std_srvs/SetBool.h>                // サービスヘッダファイル
#include <std_srvs/Trigger.h>                // サービスヘッダファイル
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "yaml-cpp/yaml.h"
#include <array>
#include <vector>

#include <waypoint_manager_msgs/Waypoint.h>
#include <waypoint_manager_msgs/WaypointStamped.h>
#include <waypoint_manager_msgs/Waypoints.h>
#include <waypoint_manager_msgs/Route.h>

// #include <waypoint_server/waypoint_server.h>

double wait_t;
bool reach_flag,learning_flag;
// std::array<std::string>  white_list[],learning_list[];registed_2_1636869763428729064
/**std::vector<std::string>  white_list{"registed_2_1636869882306685745","","registed_2_1636870081841970239","registed_2_1636868839773613387","registed_2_1636869900782330398","registed_2_1636869763428729064","registed_2_1636868839773613387"},
                          learning_list{"registed_2_1636868922403569146","registed_2_1636869882306685745"};**/
std::vector<std::string>  white_list{"registed_3_1637657209290631669"},
			  learning_list{"registed_2_1636868922403569146","registed_2_1636869882306685745"};

// std_msgs::String white_list[],learning_list[];
ros::ServiceClient Start_Wp_Client; 
ros::ServiceClient StartClient; 
ros::ServiceServer srv;
std_srvs::SetBool::Request req_ler_str;             
std_srvs::SetBool::Response resp_ler;
std_srvs::SetBool::Request req_ler_end;   
ros::Publisher initial_pose_pub,white_pub,learning_pub,start_pub; 
ros::Subscriber swiching_sub,waypoint_sub,is_reached_sub;
//std_msgs::String id;

void way_callback(const waypoint_manager_msgs::Waypoint &msg){
  auto id = msg.identity;
   for (int i = 0; i < 5; i++)
  {
  
    if(id==white_list[i] &&reach_flag){
        std_msgs::Bool white_flag,start_flag;
        white_flag.data=true;
        start_flag.data=true;
        start_pub.publish(white_flag);
        white_pub.publish(start_flag);
        ROS_INFO("permit white!");
        }
   }

    for (int j = 0;  j< 2; j++)
    {
        if(id==learning_list[j]&&reach_flag){
        //learning_pub.publish(learning_flag);
          req_ler_str.data=true;
          bool  start_learning = StartClient.call(req_ler_str,resp_ler); //start learningmode
          ROS_INFO("permit learning!");
          req_ler_end.data=false;
          start_learning = StartClient.call(req_ler_end,resp_ler); //end learningmode
      }
    }
}
void reach_callback(const std_msgs::Bool &msg)
  {
    reach_flag = msg.data;
    ROS_INFO("reach!!");
  }


/* サーバー */
int main(int argc, char **argv) {
    ros::init(argc, argv, "mode_switch");
    ros::NodeHandle nh;

    ROS_INFO("ready!");
    white_pub=nh.advertise<std_msgs::Bool>("start_white",10);
    start_pub=nh.advertise<std_msgs::Bool>("start_detect",10);

    learning_pub=nh.advertise<std_msgs::Bool>("learning_out",10);
    StartClient = nh.serviceClient<std_srvs::SetBool>("learn_out");  //create start learning service  Client for learning
    waypoint_sub=nh.subscribe("/waypoint_manager/waypoint",1,&way_callback);
    is_reached_sub=nh.subscribe("/waypoint_manager/waypoint/is_reached",1,&reach_callback);
    ros::spin();                           
  return 0;
}   

