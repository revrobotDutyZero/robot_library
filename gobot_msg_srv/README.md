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
	* `Request`:
	* `Response`:
		* int32 leftEncoder
		* int32 rightEncoder
* GetGobotStatus.srv
	* `Request`:
	* `Response`:
		* string text
		* int16 status
* GetInt.srv
	* `Request`:
	* `Response`:
		* int16 data
* GetIntArray.srv
	* `Request`:
	* `Response`:
		* int16[] data
* GetString.srv
	* `Request`:
	* `Response`:
		* string data
* GetStringArray.srv
	* `Request`:
	* `Response`:
		* string[] data
* IsCharging.srv
	* `Request`:
	* `Response`:
		* bool isCharging
* SendMap.srv
	* `Request`:
		* string ip
		* int8 who
	* `Response`:
* SendMessageToPc.srv
	* `Request`:
		* string msg
		* int16 port
	* `Response`:
* SetBool.srv
	* `Request`:
		* bool data
	* `Response`:
* SetFloatArray.srv
	* `Request`:
		* float32[] data
	* `Response`:
* SetGobotStatus.srv
	* `Request`:
		* string text
		* int16 status
	* `Response`:
* SetInt.srv
	* `Request`:
		* int16 data
	* `Response`:
* SetIntArray.srv
	* `Request`:
		* int16[] data
	* `Response`:
* SetString.srv
	* `Request`:
		* string data
	* `Response`:
* SetStringArray.srv
	* `Request`:
		* string data[]
	* `Response`:


### Customized topic
* /cmd_vel
	* Type: `geometry_msgs/Twist`
	* Brief: linear and angular velocities computed by global/local planners
* /encoders
	* Type: `gobot_msg_srv/EncodersMsg`
	* Brief: accumulated motor left and right encoders' values to compute robot translational and rotational distance
* /initialpose
	* Type: `geometry_msgs/PoseWithCovarianceStamped`
	* Brief: set robot initial pose on the map
* /joy_connection
	* Type: `std_msgs/Int8`
	* Brief: joystick connnection state
	* Detail: 
		* 1 - connected 
		* 0 - disconnected
* /joy
	* Type: `sensor_msgs/Joy`
	* Brief: joystick feedback when axes or buttons are pressed
* /map
	* Type: `nav_msgs/OccupancyGrid`
	* Brief: currently used map for navigation
* /map_metadata
	* Type: `nav_msgs/MapMetaData`
	* Brief: currently used map metadata
* /move_base/NavfnROS/plan
	* Type: `nav_msgs/Path`
	* Brief: the last plan computed and published every time the planner computes a new path from current pose to goal
* /move_base/cancel
	* Type: `actionlib_msgs/GoalID`
	* Brief: a request to cancel a specific goal
* /move_base/goal
	* Type: `move_base_msgs/MoveBaseActionGoal`
	* Brief: a goal for robot to pursue on the map
* /move_base/result
	* Type: `move_base_msgs/MoveBaseActionResult`
	* Brief: provide terminal status on the lastest goal that is sent to robot
* /odom
	* Type: `nav_msgs/Odometry`
	* Brief: odometry data to estimate change in position
* /robot_pose
	* Type: `geometry_msgs/Pose`
	* Brief: robot's current pose w.r.t the map frame
* /scan
	* Type: `sensor_msgs/LaserScan`
	* Brief: laser information
* /gobot_base/battery_topic
	* Type: `gobot_msg_srv::BatteryMsg`
	* Brief: battery information
* /gobot_base/bumpers_raw_topic
	* Type: `gobot_msg_srv::BumperMsg`
	* Brief: bumpers raw information
* /gobot_base/bumpers_collision_topic
	* Type: `gobot_msg_srv::BumperMsg`
	* Brief: bumpers information when collision
* /gobot_base/bumpers_topic
	* Type: `gobot_msg_srv::BumperMsg`
	* Brief: bumpers information after checking bumpers broken state
* /gobot_base/button_topic
	* Type: `std_msgs::Int8`
	* Brief: customized button information
* /gobot_base/cliff_topic
	* Type: `gobot_msg_srv::CliffMsg`
	* Brief: cliff information
* /gobot_base/gyro_topic
	* Type: `gobot_msg_srv::GyroMsg`
	* Brief: gyro information
* /gobot_base/ir_topic
	* Type: `gobot_msg_srv::IrMsg`
	* Brief: infrads information
* /gobot_base/proximity_topic
	* Type: `gobot_msg_srv::ProximityMsg`
	* Brief: proximity information
* /gobot_base/set_led
	* Type: `gobot_msg_srv::LedMsg`
	* Brief: set led colors that robot shows in each side
	* Detail:
		* mode -1 - show battery level led
		* mode  0 - show permanent led
		* mode  1 - show running led
		* color - colors that is going to show
* /gobot_base/set_sound
	* Type: `gobot_msg_srv::SoundMsg`
	* Brief: set buzzer sound times and duration
	* Detail:
		* num - how many times buzzer will beep 
		* time_on 1 - each beep last for 100ms
		* time_on 2 - each beep last for 400ms
		* time_on others - each beep last for 800ms
* /gobot_base/sonar_topic
	* Type: `gobot_msg_srv::SonarMsg`
	* Brief: sonar information
* /gobot_base/temperature_topic
	* Type: `std_msgs::Float32`
	* Brief: temperature information
* /gobot_base/weight_topic
	* Type: `gobot_msg_srv::WeightMsg`
	* Brief: weight information
* /gobot_motor/motor_speed
	* Type: `gobot_msg_srv/MotorSpeedMsg`
	* Brief: set motor left and right speeds 
* /gobot_status/gobot_status
	* Type: `std_msgs/Int8`
	* Brief: current robot state
	* Detail:
		* -1 - ROBOT_READY 
		*  0 - COMPLETE_PATH/ABORTED_PATH/COMPLETE_POINT
		*  1 - STOP_PATH
		*  4 - PAUSE_PATH
		*  5 - PLAY_PATH/WAITING/DELAY/PLAY_POINT
		* 11 - STOP_DOCKING/FAIL_DOKCING/COMPLETE_DOCKING
		* 15 - DOCKING
		* 20 - EXPLORATION
		* 21 - STOP_EXPLORING/COMPLETE_EXPLORING
		* 25 - EXPLORING
		* text - describe current robot state
* /gobot_status/mute
	* Type: `std_msgs/Int8`
	* Brief: mute/unmute buzzer sound
	* Detail: 
		* 1 - mute 
		* 0 - unmute
* /gobot_status/update_information
	* Type: `std_msgs/String`
	* Brief: information that robot will send to server side
* /gobot_software/connected_servers
	* Type: gobot_msg_srv/StringArrayMsg
	* Brief: `connected servers IP address`
* /gobot_software/online_servers
	* Type: `gobot_msg_srv/StringArrayMsg`
	* Brief: all available IP address in the local network
* /gobot_software/server_disconnected
	* Type: `std_msgs/String`
	* Brief: disconnect server by publishing its IP address
* /gobot_pc/~
	* Type: `sensor_msgs/PointCloud2`
	* Brief: pointcloud information for bumpers, sonars, and cliffs sensors


### Customized service
* /gobot_base/set_joy_speed
	* Type: `gobot_msg_srv/SetFloatArray`
	* Brief: set linear and angular velocities limitation in manual mode
* /gobot_base/shutdown_robot
	* Type: `std_srvs/Empty`
	* Brief: shutdown robot
* /gobot_base/use_bumper
	* Type: gobot_msg_srv/SetBool`
	* Brief: use/not use bumpers sensor data
* /gobot_bas`e/use_cliff
	* Type: `gobot_msg_srv/SetBool`
	* Brief: use/not use cliffs sensor data
* /gobot_base/use_sonar
	* Type: `gobot_msg_srv/SetBool`
	* Brief: use/not use sonars sensor data
* /gobot_command/goDock
	* Type: `std_srvs/Empty`
	* Brief: start auto docking process
* /gobot_command/lowBattery
	* Type: `std_srvs/Empty`
	* Brief: trigger low battery warn and robot will execute auto docking after completing current path
* /gobot_command/pause_path
	* Type: `std_srvs/Empty`
	* Brief: pause execution of assigned path
* /gobot_command/play_path
	* Type: `std_srvs/Empty`
	* Brief: start execution of assigned path
* /gobot_command/play_point
	* Type: `std_srvs/Empty`
	* Brief: start execution of assigned point
* /gobot_command/set_path
	* Type: `gobot_msg_srv/SetStringArray`
	* Brief: assign path to robot
* /gobot_command/set_speed
	* Type: `gobot_msg_srv/SetStringArray`
	* Brief: set linear and angular velocities limitation in auto mode
* /gobot_command/start_explore
	* Type: `std_srvs/Empty`
	* Brief: start auto explore the environment in scan mode
* /gobot_command/stopGoDock
	* Type: `std_srvs/Empty`
	* Brief: stop auto docking process
* /gobot_command/stop_explore
	* Type: `std_srvs/Empty`
	* Brief: stop auto explore the environment in scan mode
* /gobot_command/stop_path
	* Type: `std_srvs/Empty`
	* Brief: stop execution of current task (path, point or auto docking)
* /gobot_function/interrupt_delay
	* Type: `std_srvs/Empty`
	* Brief: interrupt human action/delay to continue current path
* /gobot_function/startLoopPath
	* Type: `std_srvs/Empty`
	* Brief: loop assigned path
* /gobot_function/stopLoopPath
	* Type: `std_srvs/Empty`
	* Brief: unloop assigned path
* /gobot_motor/reset_encoders
	* Type: `std_srvs/Empty`
	* Brief: reset encoders accumulated values to be 0
* /gobot_motor/reset_odom
	* Type: `std_srvs/Empty`
	* Brief: reset odometry data to be 0
* /gobot_startup/motor_ready
	* Type: `std_srvs/Empty`
	* Brief: return ture if motor is ready
* /gobot_startup/network_ready
	* Type: `std_srvs/Empty`
	* Brief: return ture if network is ready
* /gobot_startup/pose_ready
	* Type: `std_srvs/Empty`
	* Brief: return ture if initial pose is ready
* /gobot_startup/sensors_ready
	* Type: `std_srvs/Empty`
	* Brief: return ture if sensor is ready
* /gobot_status/battery_percent
	* Type: `gobot_msg_srv/GetInt`
	* Brief: get remaining battery percentage
* /gobot_status/charging_status
	* Type: `gobot_msg_srv/IsCharging`
	* Brief: get battery charging state
* /gobot_status/get_battery
	* Type: `gobot_msg_srv/GetString`
	* Brief: get battery percentage set for low battery warn
* /gobot_status/get_dock_status
	* Type: `gobot_msg_srv/GetInt`
	* Brief: get auto docking state
	* Detail:
		* -1 - FAILED TO GO TO CHARGING
		*  0 - NOT CHARGING
		*  1 - CHARING
		*  3 - GO TO CHARGING
* /gobot_status/get_gobot_status
	* Type: `gobot_msg_srv/GetGobotStatus`
	* Brief: get current robot state
	* Detail:
		* -1 - ROBOT_READY 
		*  0 - COMPLETE_PATH/ABORTED_PATH/COMPLETE_POINT
		*  1 - STOP_PATH
		*  4 - PAUSE_PATH
		*  5 - PLAY_PATH/WAITING/DELAY/PLAY_POINT
		* 11 - STOP_DOCKING/FAIL_DOKCING/COMPLETE_DOCKING
		* 15 - DOCKING
		* 20 - EXPLORATION
		* 21 - STOP_EXPLORING/COMPLETE_EXPLORING
		* 25 - EXPLORING
		* text - describe current robot state
* /gobot_status/get_home
	* Type: `gobot_msg_srv/GetStringArray`
	* Brief: get charging station pose on the map
* /gobot_status/get_loop
	* Type: `gobot_msg_srv/GetInt`
	* Brief: get loop state
	* Detail:
		* 0 - unloop
		* 1 - loop 
* /gobot_status/get_mute
	* Type: `gobot_msg_srv/GetInt`
	* Brief: get mute state
	* Detail:
		* 0 - unmute 
		* 1 - mute
* /gobot_status/get_name
	* Type: `gobot_msg_srv/GetString`
	* Detail: get robot name
* /gobot_status/get_path
	* Type: `gobot_msg_srv/GetString`
	* Brief: get robot assigned path
* /gobot_status/get_speed
	* Type: `gobot_msg_srv/GetInt`
	* Brief: get linear and angular velocities limitation in auto mode
* /gobot_status/get_stage
	* Type: `gobot_msg_srv/GetInt`
	* Brief: get robot stage for assigned path
* /gobot_status/get_update_status
	* Type: `gobot_msg_srv/GetString`
	* Brief: get information that robot is sending to server
* /gobot_status/get_wifi
	* Type: `gobot_msg_srv/GetStringArray`
	* Brief: get current connected wifi information
* /gobot_status/set_battery
	* Type: `gobot_msg_srv/SetString`
	* Brief: set low battery warn value
* /gobot_status/set_dock_status
	* Type: `gobot_msg_srv/SetInt`
	* Brief: set auto docking state
	* Detail: refer to /gobot_status/get_dock_status
* /gobot_status/set_gobot_status
	* Type: `gobot_msg_srv/SetGobotStatus`
	* Brief: set robot state
	* Detail: refer to /gobot_status/get_gobot_status
* /gobot_status/set_home
	* Type: `gobot_msg_srv/SetStringArray`
	* Brief: set robot charging station pose on the map
* /gobot_status/set_loop
	* Type: `gobot_msg_srv/SetInt`
	* Brief: set loop state
	* Detail: refer to /gobot_status/get_loop
* /gobot_status/set_mute
	* Type: `gobot_msg_srv/SetInt`
	* Brief: set mute state
	* Detail: /gobot_status/get_mute
* /gobot_status/set_name
	* Type: `gobot_msg_srv/SetString`
	* Brief: set robot name 
* /gobot_status/set_path
	* Type: `gobot_msg_srv/SetStringArray`
	* Brief: assigned path to robot
* /gobot_status/set_speed
	* Type: `gobot_msg_srv/SetStringArray`
	* Brief: set linear and angular velocities limitation in auto mode 
* /gobot_status/set_stage
	* Type: `gobot_msg_srv/SetInt`
	* Brief: set stage state for assigned path
* /gobot_status/set_wifi
	* Type: `gobot_msg_srv/SetStringArray`
	* Brief: set wifi for robot to connect
* /move_base/clear_costmaps
	* Type: `std_srvs/Empty`
	* Brief: clear costmap obstacles information
* /move_base/make_plan
	* Type: `nav_msgs/GetPlan`
	* Brief: call global planner to make plan for given two poses
* /request_nomotion_update
	* Type: `std_srvs/Empty`
	* Brief: request robot pose update
* /static_map
	* Type: `nav_msgs/SetMap`
	* Brief: retrieve current used map
	
<strong> Enjoy and have fun! </strong> <br> `Hope our effort on developing various kinds of robots will make a better world.`
