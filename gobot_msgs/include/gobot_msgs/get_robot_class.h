#ifndef GET_STATUS
#define GET_STATUS

//ros headers
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>

//c++ headers
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <numeric> 

#include <gobot_msgs/robot_msgs.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <gobot_msgs/GetPose.h>
#include <gobot_msgs/GetPoseWithCovariance.h>

namespace robot_class {
    class GetRobot {
        public:
            GetRobot();
            ~GetRobot();
            
            void getStatus(int &status,std::string &text);
            void getStatus(int &status);
            void getHome(double &x, double &y, double &qx, double &qy, double &qz, double &qw);
            geometry_msgs::PoseWithCovariance getHome();

            int getDock();
            int getLoop();
            int getStage();
            int getVolume();

            std::string getName();
            std::string getSoftwareVersion();

        private:
            gobot_msgs::GetGobotStatus get_gobot_status_;
            gobot_msgs::GetInt get_dock_status_;

    };
};

#endif
