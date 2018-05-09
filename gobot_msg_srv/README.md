# Message, Service, and Topic Summary

### Overview
This package summurizes all the <strong> customized </strong> messages, services and topics to build up the mobile robot system. <br> ROS built-in message(e.g. `std_msgs, geometry_msgs`) and service(e.g. `std_srvs`) are not involved, and please refer to http://wiki.ros.org/ for more deail.

### Customized msg
* BatteryMsg.msg
	* int16 BatteryStatus 
	* int16 BatteryVoltage
	* float64 Percentage
	* int16 ChargingCurrent
	* bool ChargingFlag
	* int16 Temperature
	* int16 RemainCapacity
	* int16 FullCapacity
* BumperMsg.msg
	* int16 bumper1
	* int16 bumper2
	* int16 bumper3
	* int16 bumper4
	* int16 bumper5
	* int16 bumper6
	* int16 bumper7
	* int16 bumper8
* CliffMsg.msg
	* int16 cliff1
	* int16 cliff2
	* int16 cliff3
	* int16 cliff4
* EncodersMsg.msg
	* int32 left_encoder
	* int32 right_encoder
* GyroMsg.msg
	* int16 gyrox
	* int16 gyroy
	* int16 gyroz
	* int16 accelx
	* int16 accely
	* int16 accelz
* IrMsg.msg
	* int16 rearSignal
	* int16 leftSignal
	* int16 rightSignal
* LedMsg.msg
	* int16 mode
	* string[] color
* MotorMsg.msg
	* string directionR
	* int8 velocityR
	* string directionL
	* int8 velocityL
* ProximityMsg.msg
	* int16 signal1
	* int16 signal2
* SonarMsg.msg
	* int16 distance1
	* int16 distance2
	* int16 distance3
	* int16 distance4
	* int16 distance5
	* int16 distance6
	* int16 distance7
* SoundMsg.msg
	* int16 num
	* int16 time_on
* WeightMsg.msg
	* float32 weight

### Customized srv
* GetEncoders.srv
	* Request:
	* Response:
		* int32 leftEncoder
		* int32 rightEncoder
* GetGobotStatus.srv
	* Request:
	* Response:
		* string text
		* int16 status
* GetInt.srv
	* Request:
	* Response:
		* int16 data
* GetIntArray.srv
	* Request:
	* Response:
		* int16[] data
* GetString.srv
	* Request:
	* Response:
		* string data
* GetStringArray.srv
	* Request:
	* Response:
		* string[] data
* IsCharging.srv
	* Request:
	* Response:
		* bool isCharging
* SendMap.srv
	* Request:
		* string ip
		* int8 who
	* Response:
* SendMessageToPc.srv
	* Request:
		* string msg
		* int16 port
	* Response:
* SetBool.srv
	* Request:
		* bool data
	* Response:
* SetFloatArray.srv
	* Request:
		* float32[] data
	* Response:
* SetGobotStatus.srv
	* Request:
		* string text
		* int16 status
	* Response:
* SetInt.srv
	* Request:
		* int16 data
	* Response:
* SetIntArray.srv
	* Request:
		* int16[] data
	* Response:
* SetString.srv
	* Request:
		* string data
	* Response:
* SetStringArray.srv
	* Request:
		* string data[]
	* Response:

### Customized topics
* /cmd_vel
	* Type: geometry_msgs/Twist
* /encoders
	* Type: gobot_msg_srv/EncodersMsg
* /initialpose
	* Type: geometry_msgs/PoseWithCovarianceStamped
* /joy_connection
	* Type: std_msgs/Int8
* /joy
	* Type: sensor_msgs/Joy
* /map
	* Type: nav_msgs/OccupancyGrid
* /map_metadata
	* Type: nav_msgs/MapMetaData
* /move_base/NavfnROS/plan
	* Type: nav_msgs/Path
* /move_base/cancel
	* Type: actionlib_msgs/GoalID
* /move_base/goal
	* Type: move_base_msgs/MoveBaseActionGoal
* /move_base/result
	* Type: move_base_msgs/MoveBaseActionResult
* /odom
	* Type: nav_msgs/Odometry
* /robot_pose
	* Type: geometry_msgs/Pose
* /scan
	* Type: sensor_msgs/LaserScan
* /gobot_base/battery_topic
	* Type: gobot_msg_srv::BatteryMsg
* /gobot_base/bumpers_raw_topic
	* Type: gobot_msg_srv::BumperMsg
* /gobot_base/bumpers_raw_topic
	* Type: gobot_msg_srv::BumperMsg
* /gobot_base/bumpers_topic
	* Type: gobot_msg_srv::BumperMsg
* /gobot_base/button_topic
	* Type: std_msgs::Int8
* /gobot_base/cliff_topic
	* Type: gobot_msg_srv::CliffMsg
* /gobot_base/gyro_topic
	* Type: gobot_msg_srv::GyroMsg
* /gobot_base/ir_topic
	* Type: gobot_msg_srv::IrMsg
* /gobot_base/proximity_topic
	* Type: gobot_msg_srv::ProximityMsg
* /gobot_base/set_led
	* Type: gobot_msg_srv::LedMsg
* /gobot_base/set_sound
	* Type: gobot_msg_srv::SoundMsg
* /gobot_base/sonar_topic
	* Type: gobot_msg_srv::SonarMsg
* /gobot_base/temperature_topic
	* Type: std_msgs::Float32
* /gobot_base/weight_topic
	* Type: gobot_msg_srv::WeightMsg
* /gobot_motor/motor_speed
	* Type: gobot_msg_srv/MotorSpeedMsg
* /gobot_status/gobot_status
	* Type: std_msgs/Int8
* /gobot_status/mute
	* Type: std_msgs/Int8 
* /gobot_status/update_information
	* Type: std_msgs/String
* /gobot_software/connected_servers
	* Type: gobot_msg_srv/StringArrayMsg
* /gobot_software/online_servers
	* Type: gobot_msg_srv/StringArrayMsg
* /gobot_software/server_disconnected
	* Type: std_msgs/String
* /gobot_pc/~
	* Type: sensor_msgs/PointCloud2 

### Customized services
* /gobot_base/set_charging
	* Type: gobot_msg_srv/SetBool
* /gobot_base/set_joy_speed
	* Type: gobot_msg_srv/SetFloatArray
* /gobot_base/shutdown_robot
	* Type: std_srvs/Empty
* /gobot_base/use_bumper
	* Type: gobot_msg_srv/SetBool
* /gobot_base/use_cliff
	* Type: gobot_msg_srv/SetBool
* /gobot_base/use_sonar
	* Type: gobot_msg_srv/SetBool
* /gobot_command/goDock
	* Type: std_srvs/Empty
* /gobot_command/lowBattery
	* Type: std_srvs/Empty
* /gobot_command/pause_path
	* Type: std_srvs/Empty
* /gobot_command/play_path
	* Type: std_srvs/Empty
* /gobot_command/play_point
	* Type: std_srvs/Empty
* /gobot_command/set_path
	* Type: gobot_msg_srv/SetStringArray
* /gobot_command/set_speed
	* Type: gobot_msg_srv/SetStringArray
* /gobot_command/start_explore
	* Type: std_srvs/Empty
* /gobot_command/stopGoDock
	* Type: std_srvs/Empty
* /gobot_command/stop_explore
	* Type: std_srvs/Empty
* /gobot_command/stop_path
	* Type: std_srvs/Empty
* /gobot_function/goDockAfterPath
	* Type: std_srvs/Empty
* /gobot_function/interrupt_delay
	* Type: std_srvs/Empty
* /gobot_function/pause_path
	* Type: std_srvs/Empty
* /gobot_function/play_path
	* Type: std_srvs/Empty
* /gobot_function/play_point
	* Type: std_srvs/Empty
* /gobot_function/startDocking
	* Type: std_srvs/Empty
* /gobot_function/startLoopPath
	* Type: std_srvs/Empty
* /gobot_function/stopDocking
	* Type: std_srvs/Empty
* /gobot_function/stopLoopPath
	* Type: std_srvs/Empty
* /gobot_function/stop_path
	* Type: std_srvs/Empty
* /gobot_function/update_path
	* Type: gobot_msg_srv/SetStringArray
* /gobot_motor/reset_encoders
	* Type: std_srvs/Empty
* /gobot_motor/reset_odom
	* Type: std_srvs/Empty
* /gobot_startup/motor_ready
	* Type: std_srvs/Empty
* /gobot_startup/network_ready
	* Type: std_srvs/Empty
* /gobot_startup/pose_ready
	* Type: std_srvs/Empty
* /gobot_startup/sensors_ready
	* Type: std_srvs/Empty
* /gobot_status/battery_percent
	* Type: gobot_msg_srv/GetInt
* /gobot_status/charging_status
	* Type: gobot_msg_srv/IsCharging
* /gobot_status/get_battery
	* Type: gobot_msg_srv/GetString
* /gobot_status/get_dock_status
	* Type: gobot_msg_srv/GetInt
* /gobot_status/get_gobot_status
	* Type: gobot_msg_srv/GetGobotStatus
* /gobot_status/get_home
	* Type: gobot_msg_srv/GetStringArray
* /gobot_status/get_loop
	* Type: gobot_msg_srv/GetInt
* /gobot_status/get_mute
	* Type: gobot_msg_srv/GetInt
* /gobot_status/get_name
	* Type: gobot_msg_srv/GetString
* /gobot_status/get_path
	* Type: gobot_msg_srv/GetString
* /gobot_status/get_speed
	* Type: gobot_msg_srv/GetInt
* /gobot_status/get_stage
	* Type: gobot_msg_srv/GetInt
* /gobot_status/get_update_status
	* Type: gobot_msg_srv/GetString
* /gobot_status/get_wifi
	* Type: gobot_msg_srv/GetStringArray
* /gobot_status/set_battery
	* Type: gobot_msg_srv/SetString
* /gobot_status/set_dock_status
	* Type: gobot_msg_srv/SetInt
* /gobot_status/set_gobot_status
	* Type: gobot_msg_srv/SetGobotStatus
* /gobot_status/set_home
	* Type: gobot_msg_srv/SetStringArray
* /gobot_status/set_loop
	* Type: gobot_msg_srv/SetInt
* /gobot_status/set_mute
	* Type: gobot_msg_srv/SetInt
* /gobot_status/set_name
	* Type: gobot_msg_srv/SetString
* /gobot_status/set_path
	* Type: gobot_msg_srv/SetStringArray
* /gobot_status/set_speed
	* Type: gobot_msg_srv/SetStringArray
* /gobot_status/set_stage
	* Type: gobot_msg_srv/SetInt
* /gobot_status/set_wifi
	* Type: gobot_msg_srv/SetStringArray
* /move_base/clear_costmaps
	* Type: std_srvs/Empty
* /move_base/make_plan
	* Type: nav_msgs/GetPlan
* /request_nomotion_update
	* Type: std_srvs/Empty	
* /static_map
	* Type: nav_msgs/SetMap
