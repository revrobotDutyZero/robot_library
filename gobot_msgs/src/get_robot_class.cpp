#include <gobot_msgs/get_robot_class.h>

namespace robot_class {
    GetRobot::GetRobot(){};
    GetRobot::~GetRobot(){};

    void GetRobot::getStatus(int &status,std::string &text){
        ros::service::call("/gobot_status/get_gobot_status",get_gobot_status_);
        status = get_gobot_status_.response.status;
        text = get_gobot_status_.response.text;
    }

    void GetRobot::getStatus(int &status){
        ros::service::call("/gobot_status/get_gobot_status",get_gobot_status_);
        status = get_gobot_status_.response.status;
    }

    void GetRobot::getHome(double &x, double &y, double &qx, double &qy, double &qz, double &qw){
        gobot_msgs::GetPoseWithCovariance get_home;
        ros::service::call("/gobot_status/get_home", get_home);
        x  = get_home.response.covar_pose.pose.position.x;
        y  = get_home.response.covar_pose.pose.position.y;
        qx = get_home.response.covar_pose.pose.orientation.x;
        qy = get_home.response.covar_pose.pose.orientation.y;
        qz = get_home.response.covar_pose.pose.orientation.z;
        qw = get_home.response.covar_pose.pose.orientation.w;
    }

    geometry_msgs::PoseWithCovariance GetRobot::getHome() {
        gobot_msgs::GetPoseWithCovariance get_home;
        ros::service::call("/gobot_status/get_home", get_home);
        return get_home.response.covar_pose;
    }

    int GetRobot::getDock(){
        ros::service::call("/gobot_status/get_dock_status", get_dock_status_);
        return get_dock_status_.response.data;
    }

    int GetRobot::getLoop(){
        gobot_msgs::GetInt get_loop;
        ros::service::call("/gobot_status/get_loop",get_loop);
        return get_loop.response.data;
    }

    int GetRobot::getStage(){
        gobot_msgs::GetInt get_stage;
	    ros::service::call("/gobot_status/get_stage", get_stage);
        return get_stage.response.data;
    }

    std::string GetRobot::getName(){
        gobot_msgs::GetString get_name;
        ros::service::call("/gobot_status/get_name",get_name);
        return get_name.response.data;
    }

    std::string GetRobot::getSoftwareVersion() {
        gobot_msgs::GetString get_software_version;
        ros::service::call("/gobot_status/get_software_version", get_software_version);
        return get_software_version.response.data;
    }

    int GetRobot::getVolume(){
        gobot_msgs::GetInt get_volume;
        ros::service::call("/gobot_status/get_volume",get_volume);
        return get_volume.response.data;
    }
};

