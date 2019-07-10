Stop the scan command：
	rostopic pub /startOrStop std_msgs/Int32 "data: 1"

Stop the motor command（Stop the motor will stop scanning）:
	rostopic pub /startOrStop std_msgs/Int32 "data: 2"

Start scanning（Drive motor, and then open the scanning）：
	rostopic pub /startOrStop std_msgs/Int32 "data: 4"
