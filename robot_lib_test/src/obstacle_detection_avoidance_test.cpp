#include <ros/ros.h>
#include <gobot_msgs/robot_command.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_detection_avoidance_test");
  ros::NodeHandle n("~");

  robot_class::RobotCommand robot_lib;
  robot_lib.initialize();
  
  ROS_INFO("================== obstacle_detection_avoidance_test ==================");

  //@Obstacle Detection & Avoidance

  //get laser data
  sensor_msgs::LaserScan laser_data;
  robot_lib.getLaserData(laser_data);
  ROS_INFO("No. of laser beams: %zu", laser_data.ranges.size());

  //get plan path for given start pose and goal pose
  bool enablePlanPath = false;
  n.param("enable_plan_path", enablePlanPath, false);

  if (enablePlanPath)
  {
    geometry_msgs::PoseStamped start, goal;
    start.header.stamp = ros::Time::now();
    start.header.frame_id = "map";
    start.pose.orientation.w = 1.0;
    n.param("start_x", start.pose.position.x, 0.0);
    n.param("start_x", start.pose.position.y, 0.0);

    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.orientation.w = 1.0;
    n.param("goal_x", goal.pose.position.x, 8.0);
    n.param("goal_y", goal.pose.position.y, 2.0);

    std::vector<geometry_msgs::PoseStamped> make_plan;

    bool planSuccess = false;
    bool startFromCurrentPose = false;
    n.param("start_from_current_pose", startFromCurrentPose, false);
    if (startFromCurrentPose)
    {
        planSuccess = robot_lib.makePlanPath(goal,make_plan);
    }
    else
    {
        planSuccess = robot_lib.makePlanPath(start,goal,make_plan);
    }
    if (planSuccess)
    {
      ROS_INFO("No. of poses along planned path: %zu to (%f, %f)", make_plan.size(),goal.pose.position.x,goal.pose.position.y);
    }
    else
    {
      ROS_WARN("Can not find a path to (%f, %f)", goal.pose.position.x,goal.pose.position.y);
    }
  }

  //clear costmap 
  bool enableClearCostmap = false;
  n.param("enable_clear_costmap", enableClearCostmap, false);
  if (enableClearCostmap)
    robot_lib.clearCostMap();

  while(ros::ok()){
    //get planned path to goal pose if have
    std::vector<geometry_msgs::PoseStamped> planned_path;
    robot_lib.getPlanPath(planned_path);
    ROS_INFO("No. of poses along planned path: %zu", planned_path.size());
    
    //get four sonars data
    std::vector<int> sonar_data;
    robot_lib.getSonarData(sonar_data);
    if (sonar_data.size() == 4)
      ROS_INFO("Sonar Data 1: %d, 2: %d, 3: %d, 4: %d",sonar_data[0],sonar_data[1],sonar_data[2],sonar_data[3]);

    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  return 0;
}