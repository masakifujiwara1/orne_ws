#include <ros/ros.h>                        // ROSヘッダファイル
#include <std_srvs/SetBool.h>                // サービスヘッダファイル
#include <std_srvs/Trigger.h>                // サービスヘッダファイル
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCoverianceStamped>

//servise callback
//flag == true
bool ch_flag =false;
double diff_goal_x,diff_goal_y;
double pose_x,pose_y,pose_z,ori_x,ori_y,ori_z,ori_w;
double diff_x,diff_y = 0;
ros::NodeHandle nh;                    // ノードハンドルの生成
ros::ServiceClient Start_Client = nh.serviceClient<std_srvs::Trigger>("start_wp_nav");  // クライアントの生成
ros::ServiceClient StartClient = nh.serviceClient<std_srvs::SetBool>("start_learning");  // クライアントの生成
ros::ServiceClient EndClient = nh.serviceClient<std_srvs::SetBool>("end_learning");  // クライアントの生成
std_srvs::Trigger::Request req;             // リクエストの生成
std_srvs::Trigger::Response resp;           // レスポンスの生成
std_srvs::SetBool::Request req_;             // リクエストの生成
std_srvs::SetBool::Response resp_;           // レスポンスの生成
geometry_msgs::PoseWithCoverianceStamped initPose;


//callbackの前にreq.dataで分ける
bool serviceCallBack(std_srvs::SetBool::Request &req,
std_srvs::SetBool::Response &resp_)
{
    ch_flag == true;
    /*switch(req.data)

    case 1:
        diff_goal_x=
        diff_goal_y=
        //waypose.pose.pose_x=; アクセス
        initPose.header.stamp = tmp_time;  //  時間
        initPose.header.frame_id = "map";  //  フレーム
        initPose.pose.pose.position.x = ;
        initPose.pose.pose.position.y = ;
        initPose.pose.pose.position.z = ;
        initPose.pose.pose.orientation.w = 1.0;
    case 2:
        diff_goal_x=
        diff_goal_y=
        //waypose.pose.pose_x=; アクセス
        initPose.header.stamp = tmp_time;  //  時間
        initPose.header.frame_id = "map";  //  フレーム
        initPose.pose.pose.position.x = ;
        initPose.pose.pose.position.y = ;
        initPose.pose.pose.position.z = ;
        initPose.pose.pose.orientation.w = 1.0;
    
    }*/
        bool  start_learning = StartClient.call(req,resp); //学習の開始
}

odom_callback(nav_msgs::Odometry &msg)
{
   double x = msg->pose.pose.position.x;
   double y = msg->pose.pose.position.y;
   diff_x = diff_x - x;
   diff_y = diff_y - y;

    if (diff_x=>diff_goal_x && diff_y=>diff_goal_y)
    {
        bool  end_learning = EndClient.call(req,resp); // endリクエストの送信
        pose_pub = nh.advertise<geometry_msgs::PoseWithCoverianceStamped>("initialpose", 1);
        pose_pub.publish(initialpose);
        bool  start_waypointnav = StartClient.call(req,resp_sta); // リクエストの送信
    }
}

/* サーバー */
int main(int argc, char **argv) {
  ros::init(argc, argv, "swiching_by_odom");        // ノードの初期化（ノード名を"srv_test"に設定）
  if (ch_flag==false)
  {
      ros::ServiceServer srv = nh.advertiseService("start_call", &serviceCallBack);  // サーバー生成とコールバック関数の登録
  }
  else if (ch_flag==true)
  {
      odom_sub_ = nh_.subscribe("/odom", 1,&odom_callback);//cmd_vel or icart_mini/cmd_vel
      ROS_INFO_STREAM("learning ready!");
  }
  
  ros::spin();                              // リクエストの無限待ち
  return true;
}   

