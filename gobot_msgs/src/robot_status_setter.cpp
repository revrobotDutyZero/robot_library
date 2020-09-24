#include "gobot_msgs/robot_status_setter.hpp"

namespace gobot
{
    
void RobotStatusSetter::statusCallback(const std_msgs::Int8::ConstPtr& msg){
    this->robot_status = msg->data;
    //enable led change when moving away from CS for playing path/point, tracking or scanning
    if(!led_flag_ && (robot_status_==5 || robot_status_==16 ||robot_status_==25)){
        led_flag_ = true;
    }
}

} // namespace gobot
