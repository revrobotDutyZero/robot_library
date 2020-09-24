#include <gobot_msgs/set_robot_class.h>
#include <thread> 


namespace robot_class {
    /**
    * @BRIEF  Class constructor
    */
    SetRobot::SetRobot(){};

    /**
    * @BRIEF  Class destructor
    */
    SetRobot::~SetRobot(){};

    //Do remember to initilize class after ros::init if setMotorSpeed is used
    void SetRobot::initialize(){
        ros::NodeHandle nh_;
        speed_pub_ = nh_.advertise<gobot_msgs::MotorSpeedMsg>("/gobot_motor/motor_speed", 1);
        nav_pub_ = nh_.advertise<gobot_msgs::MotorSpeedMsg>("/nav_speed", 1);
        led_pub_ = nh_.advertise<gobot_msgs::LedMsg>("/gobot_base/set_led", 1);
        sound_pub_ = nh_.advertise<gobot_msgs::SoundMsg>("/gobot_base/set_sound", 1);
        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

        // // Create a subscriber for manually setting/resetting pose
        // setPoseSub_ = nh_.subscribe("set_pose",
        //                             1,
        //                             &RosFilter<T>::setPoseCallback,
        //                             this, ros::TransportHints().tcpNoDelay(false));


        set_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("set_pose", 1);

        this->status_sub = nh_.subscribe("/gobot_status/set_gobot_status", 1, &SetRobot::statusCallback, this);

    }


    double SetRobot::degreeToRad(double degree){
        return degree*PI_/180.0;
    }

    double SetRobot::radToDegree(double rad){
        return rad*180.0/PI_;
    }

    //yaw's unit must be degree (received from application)
    double SetRobot::appToRobotYaw(double yaw, std::string unit){
        if(unit.compare("deg")!=0){
            yaw = radToDegree(yaw);
        }
        return degreeToRad(-(yaw+90));
    }

    int SetRobot::robotToAppYaw(double yaw, std::string unit){
        if(unit.compare("deg")!=0){
            yaw = radToDegree(yaw);
        }
        int result = -yaw-90;
        return result;
    }

    int SetRobot::stopRobotMoving(){
        gobot_msgs::GetGobotStatus get_gobot_status;
        ros::service::call("/gobot_status/get_gobot_status",get_gobot_status);

        //if robot is navigation / auto docking / scanning, then return the status
        if(get_gobot_status.response.status==5){
            ros::service::call("/gobot_command/pause_path",empty_srv);
            return get_gobot_status.response.status;
        }
        else if(get_gobot_status.response.status==15){
            ros::service::call("/gobot_command/stop_dock",empty_srv);
            return get_gobot_status.response.status;
        }
        else if(get_gobot_status.response.status==16){
            ros::service::call("/gobot_command/stop_track",empty_srv);
            return get_gobot_status.response.status;
        }
        else if(get_gobot_status.response.status==25){
            ros::service::call("/gobot_command/stop_explore",empty_srv);
            return get_gobot_status.response.status;
        }
        else{
            //if robot is not moving, then return 0
            return 0;
        }
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

    bool SetRobot::setMode(const int mode){
        gobot_msgs::SetInt set_mode;
        set_mode.request.data = mode;
        return ros::service::call("/gobot_status/set_mode", set_mode);
    }

    bool SetRobot::setStage(const int stage){
        set_stage_.request.data = stage;
        return ros::service::call("/gobot_status/set_stage", set_stage_);
    }

    bool SetRobot::setLoop(const int loop){
        set_loop_.request.data = loop;
        return ros::service::call("/gobot_status/set_loop", set_loop_);
    }

    bool SetRobot::setWifi(std::string wifi_name,std::string wifi_password){
        gobot_msgs::SetStringArray set_wifi;
        set_wifi.request.data.push_back(wifi_name);
        set_wifi.request.data.push_back(wifi_password);
        return ros::service::call("/gobot_status/set_wifi",set_wifi); 
    }  

    bool SetRobot::setVolume(const int volume){
        gobot_msgs::SetInt set_volume;
        set_volume.request.data = volume;
        return ros::service::call("/gobot_status/set_volume", set_volume);
    }

    bool SetRobot::setName(std::string robot_name){
        gobot_msgs::SetString set_name;
        set_name.request.data = robot_name;
        return ros::service::call("/gobot_status/set_name",set_name);
    }

    bool SetRobot::setBatteryLvl(std::string battery_lvl){
        gobot_msgs::SetString set_battery;
        set_battery.request.data = battery_lvl;
        return ros::service::call("/gobot_status/set_battery",set_battery);
    }

    bool SetRobot::setSpeedLimit(std::string linear, std::string angular){
        gobot_msgs::SetStringArray set_speed;
        set_speed.request.data.push_back(linear);
        set_speed.request.data.push_back(angular);
        return ros::service::call("/gobot_status/set_speed",set_speed);
    }

    bool SetRobot::setHome(double pos_x, double pos_y, double ori_x, double ori_y, double ori_z, double ori_w) {
        gobot_msgs::SetPoseWithCovariance set_home;
        set_home.request.covar_pose.pose.position.x = pos_x;
        set_home.request.covar_pose.pose.position.y = pos_y;
        set_home.request.covar_pose.pose.orientation.x = ori_x;
        set_home.request.covar_pose.pose.orientation.y = ori_y;
        set_home.request.covar_pose.pose.orientation.z = ori_z;
        set_home.request.covar_pose.pose.orientation.w = ori_w;
        return ros::service::call("/gobot_status/set_home", set_home);
    }

    bool SetRobot::setHome(geometry_msgs::PoseWithCovariance new_home_pose) {
        gobot_msgs::SetPoseWithCovariance set_home;
        set_home.request.covar_pose = new_home_pose;
        return ros::service::call("/gobot_status/set_home", set_home);
    }

    bool SetRobot::clearPath(){
        gobot_msgs::SetStringArray set_path;
        ros::service::call("/gobot_status/set_path", set_path);
        return true;
    }

    //when activating bumpers/cliffs/manual control, robot will not move
    bool SetRobot::setNavSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){ 
        motor_speed_.directionR = std::string(1, directionR);
        motor_speed_.velocityR = velocityR>127 ? 127 : velocityR;
        motor_speed_.directionL = std::string(1, directionL);
        motor_speed_.velocityL = velocityL>127 ? 127 : velocityL;
        nav_pub_.publish(motor_speed_);
        return true;
    }

    //Do remember to initilize class after ros::init if setMotorSpeed is used
    //robot will move regardless of robot's sensors and conditions
    bool SetRobot::setMotorSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){ 
        //maximum of int8 is 127
        motor_speed_.directionR = std::string(1, directionR);
        motor_speed_.velocityR = velocityR>127 ? 127 : velocityR;
        motor_speed_.directionL = std::string(1, directionL);
        motor_speed_.velocityL = velocityL>127 ? 127 : velocityL;
        speed_pub_.publish(motor_speed_);
        return true;
    }

    void SetRobot::setSound(int num,int time_on){
        gobot_msgs::SoundMsg soundCmd; 
        soundCmd.num = num;
        soundCmd.time_on = time_on;
        sound_pub_.publish(soundCmd);
    }
    
    void SetRobot::setInitialpose(const double p_x, const double p_y, const double q_x, const double q_y, const double q_z, const double q_w){
        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.pose.pose.position.x = p_x;
        initial_pose.pose.pose.position.y = p_y;
        initial_pose.pose.pose.orientation.x = q_x;
        initial_pose.pose.pose.orientation.y = q_y;
        initial_pose.pose.pose.orientation.z = q_z;
        initial_pose.pose.pose.orientation.w = q_w;
        setInitialpose(initial_pose);
    }

    void SetRobot::setInitialpose(const double p_x, const double p_y, const double p_yaw){
        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.pose.pose.position.x = p_x;
        initial_pose.pose.pose.position.y = p_y;
        initial_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(p_yaw);
        setInitialpose(initial_pose);
    }
            
    void SetRobot::setInitialpose(geometry_msgs::PoseWithCovarianceStamped pose){
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        //x-x, y-y, yaw-yaw covariance
        pose.pose.covariance[0] = 0.01;
        pose.pose.covariance[7] = 0.01;
        pose.pose.covariance[35] = 0.01;
        if(pose.pose.pose.position.x == 0 && pose.pose.pose.position.y == 0 && pose.pose.pose.orientation.z == 0 && pose.pose.pose.orientation.w == 0){
            ROS_ERROR("(SET_ROBOT_CLASS) Assigned pose is invalid, set it position to map origin");
            pose.pose.pose.orientation.w = 1.0;
        }
        initial_pose_pub_.publish(pose);

        set_pose_pub_.publish(pose);

        //clear costmap after manually setting robot pose
        ros::service::call("/move_base/clear_costmaps",empty_srv);
    }

    void SetRobot::setInitialpose(geometry_msgs::PoseWithCovariance pose){
        geometry_msgs::PoseWithCovarianceStamped stamped_pose;
        stamped_pose.header.frame_id = "map";
        stamped_pose.header.stamp = ros::Time::now();
        //x-x, y-y, yaw-yaw covariance
        stamped_pose.pose = pose;
        if(stamped_pose.pose.pose.position.x == 0 && stamped_pose.pose.pose.position.y == 0 
           && stamped_pose.pose.pose.orientation.z == 0 && stamped_pose.pose.pose.orientation.w == 0) {
            ROS_ERROR("(SET_ROBOT_CLASS) Assigned pose is invalid, set its position to map origin");
            stamped_pose.pose.pose.orientation.w = 1.0;
        }
        initial_pose_pub_.publish(stamped_pose);

        set_pose_pub_.publish(stamped_pose);

        //clear costmap after manually setting robot pose
        ros::service::call("/move_base/clear_costmaps",empty_srv);
    }

    void SetRobot::setInitialpose(geometry_msgs::Pose pose) {
        geometry_msgs::PoseWithCovarianceStamped stamped_pose;
        stamped_pose.header.frame_id = "map";
        stamped_pose.header.stamp = ros::Time::now();
        //x-x, y-y, yaw-yaw covariance
        stamped_pose.pose.pose = pose;
        if(stamped_pose.pose.pose.position.x == 0 && stamped_pose.pose.pose.position.y == 0 
           && stamped_pose.pose.pose.orientation.z == 0 && stamped_pose.pose.pose.orientation.w == 0) {
            ROS_ERROR("(SET_ROBOT_CLASS) Assigned pose is invalid, set its position to map origin");
            stamped_pose.pose.pose.orientation.w = 1.0;
        }

        initial_pose_pub_.publish(stamped_pose);

        //clear costmap after manually setting robot pose
        ros::service::call("/move_base/clear_costmaps",empty_srv);
    }

    void SetRobot::setBatteryLed(){
        gobot_msgs::LedMsg ledCmd; 
        ledCmd.mode = -1;
        led_pub_.publish(ledCmd);
    }

    void SetRobot::setLed(int mode, const std::vector<std::string> &color){
        gobot_msgs::LedMsg ledCmd; 
        ledCmd.mode = mode;
        for(int i = 0; i<color.size();i++)
            ledCmd.color.push_back(color[i]);
        led_pub_.publish(ledCmd);
    }

    void SetRobot::setMagnet(bool status){
        gobot_msgs::SetBool magnet;
        magnet.request.data = status;
        ros::service::call("/gobot_base/set_magnet",magnet);
    }

    void SetRobot::setFootprint(std::string footprint){
        dynamic_reconfigure::Reconfigure config;
        dynamic_reconfigure::StrParameter param;
        param.name = "footprint";
        param.value = footprint;
        config.request.config.strs.push_back(param);
        ros::service::call("/move_base/global_costmap//set_parameters",config);
        ros::service::call("/move_base/local_costmap//set_parameters",config);

        ros::param::set("/amcl/footprint",footprint);
        ros::param::set("/amcl/robot_footprint",footprint);
        ros::param::set("/amcl/footprint_model/vertices",footprint);

        ros::param::set("/move_base/TebLocalPlannerROS/footprint",footprint);
        ros::param::set("/move_base/TebLocalPlannerROS/robot_footprint",footprint);
        ros::param::set("/move_base/TebLocalPlannerROS/footprint_model/vertices",footprint);

        ros::param::set("/move_base/global_costmap/footprint",footprint);
        ros::param::set("/move_base/global_costmap/robot_footprint",footprint);
        ros::param::set("/move_base/global_costmap/footprint_model/vertices",footprint);

        ros::param::set("/move_base/local_costmap/footprint",footprint);
        ros::param::set("/move_base/local_costmap/robot_footprint",footprint);
        ros::param::set("/move_base/local_costmap/footprint_model/vertices",footprint);
    }

    std::string SetRobot::killList(){
        return "rosnode kill /move_base &"; 
    }

    void SetRobot::reloadMap(){
        setMotorSpeed('F', 0, 'F', 0);
        setLed(0,{"white"});
        std::string cmd = "rosnode kill /map_server";
        system(cmd.c_str());
        ros::Duration(3.0).sleep();
        setBatteryLed();
        setStatus(-4,"RELOAD_MAP");
    }

    void SetRobot::reloadCostOverlay() {
        std::string cmd = "rosnode kill /cost_overlay_map_server";
        system(cmd.c_str());
        ros::Duration(3.0).sleep();
        // the cost overlay will respawn automatically by itself
    }

    void SetRobot::runNavi(bool simulation, bool reset_odom){
        setMotorSpeed('F', 0, 'F', 0);
        setLed(0,{"white"});
        if(reset_odom)
            ros::service::call("/gobot_motor/reset_odom",empty_srv);

        std::string cmd;
        ros::Duration(4.0).sleep();
        /// Relaunch gobot_navigation
       if(simulation)
            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_gazebo gazebo_slam.launch\"";
        else
            cmd = "roslaunch gobot_navigation gobot_navigation.launch >> ~/catkin_ws/src/gobot_robotics/robot_log/navigation_log.txt &";
        system(cmd.c_str());
        ros::Duration(6.0).sleep();
    }

    void SetRobot::runScan(bool simulation, bool reset_odom){
        setMotorSpeed('F', 0, 'F', 0);
        setLed(0,{"white"});
        if(reset_odom)
            ros::service::call("/gobot_motor/reset_odom",empty_srv);
        
        std::string cmd;
        ros::Duration(4.0).sleep();
        /// Relaunch gobot_scan
        if(simulation)
            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_gazebo gazebo_scan.launch\"";
        else
            cmd = "roslaunch gobot_navigation gobot_scan.launch >> ~/catkin_ws/src/gobot_robotics/robot_log/navigation_log.txt &";
        system(cmd.c_str());
        ros::Duration(6.0).sleep();
    }

    void SetRobot::speakEnglish(std::string str){
        gobot_msgs::GetInt get_volume;
        ros::service::call("/gobot_status/get_volume",get_volume);
        if(get_volume.response.data != 0){
            tts_en_ = "echo \"" + str + "\" | festival --tts &";
            system(tts_en_.c_str());
        }
    }

    void SetRobot::speakChinese(std::string str){
        gobot_msgs::GetInt get_volume;
        ros::service::call("/gobot_status/get_volume",get_volume);
        if(get_volume.response.data != 0){
            tts_ch_ = "ekho -s -30 \"" + str + "\" &";
            system(tts_ch_.c_str());
        }
    }


    void SetRobot::playSystemAudio(std::string name_mp3, int volume){

        std::string speech_language, system_audio_volume;
        int media_audio_volume, fade_down_volume;
        std::vector<std::string> sink_input_;
        std::string mp3_duration, audio_duration_cmd;

        ros::param::get("system_audio_volume", system_audio_volume);
        ros::param::get("media_audio_volume", media_audio_volume);
        ros::param::get("fade_down_volume", fade_down_volume);
        ros::param::get("speech_language", speech_language);

        voice_file_ = "play -v " + system_audio_volume + " " + speech_language + name_mp3 + " delay 0.3";    
        audio_duration_cmd = "soxi -D " + speech_language + name_mp3;

        changeVolume(media_audio_volume);

        ros::Duration(0.5).sleep();

        FILE *fduration = popen(audio_duration_cmd.c_str(), "r");
        char duration_buff[1024];
        while (!feof(fduration)) {
            if (fgets(duration_buff, 128, fduration) != NULL)
                mp3_duration += duration_buff;
                mp3_duration.pop_back();
        }
        fclose(fduration);

        std::cout<<"Audio duration = "<<mp3_duration<<std::endl;
        double mp3_duration_ = std::stoi(mp3_duration);

        std::thread soxPlay(system, voice_file_.c_str());
        soxPlay.detach();

        sink_input_ = playbackVolumeCheck();

        if(sink_input_.size() != 0){
            std::cout<<"######## Decreasing the volume behind the system audio"<<std::endl;
            for(size_t i =0; i < sink_input_.size(); i++){
                std::thread volume_down(&SetRobot::volumeDownFade, this, sink_input_[i], 100, fade_down_volume );
                volume_down.detach();
            }
        }

        std::cout<<" Delay for duration"<<std::endl;
        ros::Duration(mp3_duration_).sleep();
        std::cout<<" Finish delay"<<std::endl;

        if(sink_input_.size() != 0){
            std::cout<<"######## Icreasing the volume behind the system audio"<<std::endl;
            for(size_t i =0; i < sink_input_.size(); i++){
                std::thread volume_up(&SetRobot::volumeUpFade, this, sink_input_[i], fade_down_volume, 100 );
                volume_up.detach();
            }
        }
    }


    std::vector<std::string> SetRobot::playbackVolumeCheck(){

        std::string sys_sink_input, sink_input;;
        FILE *fp;
        
        ros::Duration(0.1).sleep(); 
        while(!sys_sink_input.size()){
            fp = popen("pactl list sink-inputs |grep -e \'Sink Input\' -e\'application.name = \"SoX\"\' |grep -B 1 \'SoX\' | grep \'Sink Input\' | cut -d \'#\' -f2", "r");
            char buf[1024];
            while (!feof(fp)) {
                if (fgets(buf, 1024, fp) != NULL){
                    sys_sink_input = buf;
                    sys_sink_input.pop_back();
                }
            }
            std::cout<<"Attempt to receive sink-input information"<<std::endl;
        }
        fclose(fp);

        std::cout<<"The system audio sink-input is = "<< sys_sink_input <<std::endl;

        std::string cmd = "pactl list sink-inputs |grep 'Sink Input'|grep -v '" + sys_sink_input +"'|cut -d '#' -f2";
        
        FILE *fpp = popen(cmd.c_str(), "r");
        char buff[1024];
        while (!feof(fpp)) {
            if (fgets(buff, 128, fpp) != NULL)
                sink_input += buff;
        }
        fclose(fpp);

        std::stringstream ss(sink_input);
        std::string item;
        std::vector<std::string> result;
        while (std::getline(ss, item, '\n')) {
            result.push_back(item);
        }
        std::cout << "All sink-input found : ";
        if(result.size() != 0){
            for (size_t i = 0; i < result.size(); i++)
                std::cout << result[i] << " | ";
                std::cout << std::endl;
        }

        return result;
    }

    void SetRobot::volumeDownFade(std::string sink_input, int high_volume, int low_volume){

        for(int i=high_volume; i>=low_volume; i-=5){
            std::string volume = std::to_string(i);
            std::string v_cmd = "pactl set-sink-input-volume " + sink_input + " " + volume + "%";
            system(v_cmd.c_str());
        }
    }

    void SetRobot::volumeUpFade(std::string sink_input, int low_volume, int high_volume){

        for(int i=low_volume; i<=high_volume; i+=5){
            std::string volume = std::to_string(i);
            std::string v_cmd = "pactl set-sink-input-volume " + sink_input + " " + volume + "%";
            system(v_cmd.c_str());
        }
    }


    void SetRobot::killAudio(){
        std::string cmd = "sudo kill $(ps aux | grep \"sudo play\" | grep \"mp3\" | tr -s ' ' | cut -d ' ' -f2) &";
        system(cmd.c_str());
    }

    void SetRobot::changeVolume(int volume){
        if(volume < 0)
            volume = 0;
	std::string cmd = "pactl -- set-sink-volume $(pactl list | grep -oP 'Sink #\\K([0-9]+)') " + std::to_string(volume) + "%";
        //std::string cmd = "amixer -D pulse set Master " + std::to_string(volume) + "%";
        system(cmd.c_str());
    }

    void SetRobot::statusCallback(const std_msgs::Int8::ConstPtr& msg){
        this->robot_status = msg->data;
    }

};

