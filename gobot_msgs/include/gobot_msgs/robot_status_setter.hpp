#pragma once

namespace gobot
{
    class RobotStatusSetter {
    public:
        void statusCallback(const std_msgs::Int8::ConstPtr& msg);
        int robot_status;
    };
} // namespace gobot
