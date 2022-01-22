# obstacle_avoidance

### Build

以下のレポジトリをクローン  
https://github.com/open-rdc/obstacle_avoidance  
ブランチ　develop/refine  

https://github.com/open-rdc/turtlebot3  
ブランチ　3_cameras  

https://github.com/open-rdc/fulanghua_navigation  
ブランチ　long_period  

### Execution
roslaunch obstacle_avoidance navigation_based_learning_sim.launch  

rvizで自己位置を指定して，以下を入力  
rosservice call /start_wp_nav  

訓練からテストに移行．Trueにすると，訓練に移行  
rosservice call  /training False  

回帰モデルで４時間学習した結果  
https://youtu.be/aN8SloC03U8  

学習の様子  
![image](https://user-images.githubusercontent.com/5755200/91647763-52e72f80-ea99-11ea-85b6-aab8f08ad9b4.png)  

識別器をした場合の例  
https://youtu.be/vFZuFeirO5Q

# obstacle_avoidance

### Build

```
rosdep install obstacle_avoidance
catkin build obstacle_avoidance
```

### Execution

```
roslaunch obstacle_avoidance LiDAR_based_learning_sim.launch
```

### Checking Results

```
cd ~/.ros/data
```

![Screenshot 2020-04-12 11:54:06](https://user-images.githubusercontent.com/5755200/79059403-87a64600-7cb4-11ea-894c-1d5d825748a6.png)

### Navigation based
bring up gazebo and learning
```
roslaunch obstacle_avoidance navigation_based_learning_sim.launch
```

 /turtlebot3_navigation/launch/move_base.launchを変更
```
<arg name="cmd_vel_topic" default="/cmd_vel" /> --> <arg name="cmd_vel_topic" default="/nav_vel" />　
 
```

bring up navigation
```
roslaunch obstacle_avoidance turtlebot3_navigation.launch map_file:={/YOUR_PATH/maps/willowgarage.yaml} waypoints_file:={/YOUR_PATH/maps/willow_loop.yaml}
```

start waypoint navigation
```
rosservice call /start_wp_nav
```
