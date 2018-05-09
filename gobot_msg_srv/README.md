# MESSAGE, SERVICE, AND TOPIC SUMMARY

### Overview
This package summurizes all the <strong> customized </strong> messages, services and topics to build up the mobile robot system. 
For ROS built-in message (e.g. `std_msgs`, `geometry_msgs`), service (e.g. `std_srvs`) will not be involved.

### Customized Messages
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

### Customized Services

### Customized Topics


