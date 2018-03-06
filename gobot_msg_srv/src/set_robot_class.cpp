#include <gobot_msg_srv/set_robot_class.h>


namespace robot_class {
    SetRobot::SetRobot(){};
    SetRobot::~SetRobot(){};

    //Do remember to initilize class after ros::init if setMotorSpeed is used
    void SetRobot::initialize(){
        ros::NodeHandle nh_;
        speed_pub_ = nh_.advertise<gobot_msg_srv::MotorSpeedMsg>("/gobot_motor/motor_speed", 1);
        led_pub_ = nh_.advertise<gobot_msg_srv::LedMsg>("/gobot_base/set_led", 1);
        sound_pub_ = nh_.advertise<gobot_msg_srv::SoundMsg>("/gobot_base/set_sound", 1);
    }

    bool SetRobot::setStatus(int status,std::string text){
        set_gobot_status_.request.status = status;
        set_gobot_status_.request.text = text;
        return ros::service::call("/gobot_status/set_gobot_status",set_gobot_status_);
    }

    bool SetRobot::setDock(int status){
        set_dock_status_.request.data = status;
        return ros::service::call("/gobot_status/set_dock_status",set_dock_status_);
    }

    bool SetRobot::setStage(const int stage){
        set_stage_.request.data = stage;
        return ros::service::call("/gobot_status/set_stage", set_stage_);
    }

    bool SetRobot::setLoop(const int loop){
        set_loop_.request.data = loop;
        return ros::service::call("/gobot_status/set_loop",set_loop_);
    }

    bool SetRobot::setWifi(std::string wifi_name,std::string wifi_password){
        gobot_msg_srv::SetStringArray set_wifi;
        set_wifi.request.data.push_back(wifi_name);
        set_wifi.request.data.push_back(wifi_password);
        return ros::service::call("/gobot_status/set_wifi",set_wifi); 
    }  

    bool SetRobot::setMute(const int mute){
        gobot_msg_srv::SetInt set_mute;
        set_mute.request.data = mute;
        return ros::service::call("/gobot_status/set_mute", set_mute);
    }

    bool SetRobot::setName(std::string robot_name){
        gobot_msg_srv::SetString set_name;
        set_name.request.data = robot_name;
        return ros::service::call("/gobot_status/set_name",set_name);
    }

    bool SetRobot::setBatteryLvl(std::string battery_lvl){
        gobot_msg_srv::SetString set_battery;
        set_battery.request.data = battery_lvl;
        return ros::service::call("/gobot_status/set_battery",set_battery);
    }

    bool SetRobot::setSpeed(std::string linear, std::string angular){
        gobot_msg_srv::SetStringArray set_speed;
        set_speed.request.data.push_back(linear);
        set_speed.request.data.push_back(angular);
        return ros::service::call("/gobot_status/set_speed",set_speed);
    }

    bool SetRobot::setHome(std::string pos_x,std::string pos_y,std::string ori_x,std::string ori_y,std::string ori_z,std::string ori_w){
        gobot_msg_srv::SetStringArray set_home;
        set_home.request.data.push_back(pos_x);
        set_home.request.data.push_back(pos_y);
        set_home.request.data.push_back(ori_x);
        set_home.request.data.push_back(ori_y);
        set_home.request.data.push_back(ori_z);
        set_home.request.data.push_back(ori_w);
        return ros::service::call("/gobot_status/set_home",set_home);
    }

    bool SetRobot::clearPath(){
        gobot_msg_srv::SetStringArray set_path;
        ros::service::call("/gobot_status/set_path", set_path);
        return true;
    }

    //Do remember to initilize class after ros::init if setMotorSpeed is used
    bool SetRobot::setMotorSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){ 
        motor_speed_.directionR = std::string(1, directionR);
        motor_speed_.velocityR = velocityR;
        motor_speed_.directionL = std::string(1, directionL);
        motor_speed_.velocityL = velocityL;
        speed_pub_.publish(motor_speed_);
        return true;
    }

    void SetRobot::setSound(int num,int time_on){
        gobot_msg_srv::SoundMsg soundCmd; 
        soundCmd.num = num;
        soundCmd.time_on = time_on;
        sound_pub_.publish(soundCmd);
    }
    
    void SetRobot::setBatteryLed(){
        gobot_msg_srv::LedMsg ledCmd; 
        ledCmd.mode = -1;
        led_pub_.publish(ledCmd);
    }

    void SetRobot::setLed(int mode, const std::vector<std::string> &color){
        gobot_msg_srv::LedMsg ledCmd; 
        ledCmd.mode = mode;
        for(int i = 0; i<color.size();i++)
            ledCmd.color.push_back(color[i]);
        led_pub_.publish(ledCmd);
    }

    std::string SetRobot::killList(bool simulation){
        return "rosnode kill /move_base"; 
    }

    void SetRobot::runNavi(bool simulation){
        setMotorSpeed('F', 0, 'F', 0);
        setLed(0,{"white"});
        ros::service::call("/gobot_motor/resetOdom",empty_srv);

        std::string cmd;
        ros::Duration(4.0).sleep();
        /// Relaunch gobot_navigation
       if(simulation)
            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_gazebo gazebo_slam.launch\"";
        else
            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch\"";
        system(cmd.c_str());
        ros::Duration(4.0).sleep();
    }

    void SetRobot::runScan(bool simulation){
        setMotorSpeed('F', 0, 'F', 0);
        setLed(0,{"white"});
        ros::service::call("/gobot_motor/resetOdom",empty_srv);
        
        std::string cmd;
        ros::Duration(4.0).sleep();
        /// Relaunch gobot_scan
        if(simulation)
            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_gazebo gazebo_scan.launch\"";
        else
            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_scan.launch\"";
        system(cmd.c_str());
        ros::Duration(4.0).sleep();
    }

    //this two functions only work with robot equipped with speaker and ekho & festival packages
    void SetRobot::speakEnglish(std::string str){
        tts_en_ = "echo \"" + str + "\" | festival --tts";
        system(tts_en_.c_str());
    }

    void SetRobot::speakChinese(std::string str){
        tts_ch_ = "ekho -s -40 \"" + str + "\"";
        system(tts_ch_.c_str());
    }
    //this two functions only work with robot equipped with speaker and ekho & festival packages
    
};

