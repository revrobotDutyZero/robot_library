#include <ros/ros.h>
#include <gobot_msgs/robot_command.h>

struct CommandEnabler
{
  CommandEnabler() :
    getPointCost(false),
    setTargetPoint(false),
    setTargetPath(false),
    play(false),
    stop(false),
    pause(false),
    startDock(false),
    stopDock(false),
    shutdown(false)
  {}

  bool getPointCost;
  bool setTargetPoint;
  bool setTargetPath;
  bool play; 
  bool stop;
  bool pause;
  bool startDock;
  bool stopDock;
  bool shutdown;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_route_test");
  ros::NodeHandle n("~");

  CommandEnabler enabler;

  robot_class::RobotCommand robot_lib;
  robot_lib.initialize();

  ROS_INFO("================== point_route_test ==================");

  n.param("enable_get_point_cost", enabler.getPointCost, false);
  n.param("enable_set_target_point", enabler.setTargetPoint, false);
  n.param("enable_set_target_path", enabler.setTargetPath, false);
  n.param("enable_play", enabler.play, false);
  n.param("enable_stop", enabler.stop, false);
  n.param("enable_pause", enabler.pause, false);
  n.param("enable_start_dock", enabler.startDock, false);
  n.param("enable_stop_dock", enabler.stopDock, false);
  n.param("enable_shutdown", enabler.shutdown, false);

  //@Target Points || Routes
  //getPointCost
  if (enabler.getPointCost)
  {
    double point_x, point_y;
    n.param("enable_get_point_cost_x", point_x, 0.0);
    n.param("enable_get_point_cost_y", point_y, 0.0);
    int point_cost = robot_lib.getPointCost(point_x,point_y);
    ROS_INFO("The cost value of point (%.2f,%.2f) is %d", point_x, point_y, point_cost);
  }
  
  
  //setTargetPoint
  if (enabler.setTargetPoint)
  {
    robot_class::Pose point1;
    n.param("enable_set_target_point_x", point1.x, 8.0);
    n.param("enable_set_target_point_y", point1.y, 1.0);
    n.param("enable_set_target_point_theta", point1.theta, 0.0);

    bool sentPoint = robot_lib.setTargetPoint(point1);
    if (sentPoint)
      ROS_INFO("Successfully send target point to robot.");
    else
      ROS_INFO("Failed to send target point to robot.");
  }
  

  //setTargetPath
  if (enabler.setTargetPath)
  {
    robot_class::Path path;
    std::vector<robot_class::Path> test_path;

    n.param("enable_set_target_path_x1", path.x, 8.0);
    n.param("enable_set_target_path_y1", path.y, 1.0);
    n.param("enable_set_target_path_theta1", path.theta, 90.0);
    n.param("enable_set_target_path_waiting1", path.waiting, 5);
    test_path.push_back(path);

    path.name = "P2";
    n.param("enable_set_target_path_x2", path.x, 2.0);
    n.param("enable_set_target_path_y2", path.y, 0.0);
    n.param("enable_set_target_path_theta2", path.theta, 0.0);
    n.param("enable_set_target_path_waiting2", path.waiting, 5);
    test_path.push_back(path);

    bool sentPath = robot_lib.setTargetPath(test_path,"test_path");
    if (sentPath)
      ROS_INFO("Successfully send target path to robot.");
    else
      ROS_INFO("Failed to send target path to robot.");
  }
  

  //@Control robot motion
  if (enabler.play)
    robot_lib.playTargetPath();

  if (enabler.pause)
    robot_lib.pauseRobot();

  if (enabler.stop)
    robot_lib.stopRobot();

  if (enabler.startDock)
    robot_lib.startDock();

  if (enabler.stopDock)
    robot_lib.stopDock();

  if (enabler.shutdown)
    robot_lib.shutDown();

  
  while(ros::ok()){
    //getCurrentGoal
    geometry_msgs::PoseStamped current_goal;
    robot_lib.getCurrentGoal(current_goal);
    ROS_INFO("Current goal: x-%f, y-%f, quaternion-(0.0,0.0,%f,%f)", current_goal.pose.position.x,current_goal.pose.position.y,
    current_goal.pose.orientation.z,current_goal.pose.orientation.w);

    //getGoalStatus
    int goal_status = robot_lib.getGoalStatus();
    ROS_INFO("Current goal status: %d",goal_status);

    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  return 0;
}