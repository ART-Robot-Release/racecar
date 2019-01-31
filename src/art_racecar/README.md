# art_racecar
# art-racecar V1.0

ROS racecar
************************安装************************
cd  ~/
git clone https://github.com/ART-Robot-Release/racecar
cd racecar
./install.sh
配置小车串口udev：
cd  ~/racecar/src/art_racecar/udev
sudo  bash art_init.sh
sudo reboot
**********************建立地图**********************
先安装电脑用户名和主机名配置主从机
a) 运行车
roslaunch art_racecar Run_car.launch
b) 3.3运行gmapping
roslaunch art_racecar Run_gmapping.launch
c) 3.4运行键盘控制
rosrun art_racecar racecar_teleop.py
d) 3.5.本地电脑打开rviz
本地电脑打开：
source  工作空间
source art_racecar/art_rviz.sh
roslaunch art_racecar rviz.launch
e) 3.6 建立地图
键盘控制建立地图,按键如下：
U	 I 	O
J 	K	 L
M	 , 	.
加减速为W，S.
f) 保存地图（地图直接保存在小车上）
在art_racecar文件夹下执行：bash save_map.sh 
地图保存在art_racecar/map/mymap.pgm
检查无误后，修改mymap.pgm替换为test.pgm

************************导航************************
a) SSH连接小车（Ubuntu系统为例）sz为小车用户名
ssh sz@192.168.5.101
b) 运行车
roslaunch art_racecar test.launch
c) 运行AMCL
roslaunch art_racecar amcl_nav.launch
d) .本地电脑打开rviz
本地电脑打开：
source  工作空间
source art_racecar/art_rviz.sh
roslaunch art_racecar rviz.launch
e) 4.5 开始导航 
在RVIZ中设定初始坐标，设定目标位置，开始导航
*********************软件接口***********************
1.启动底盘
	启动底盘需要启动rosserial_python节点。
	设置参考art_racecar/launch/Run_car.launch
2.发布地盘控制指令：
	通过发布Twist消息控制底盘。
	线速度：twist.linear.x,这里的线速度范围为500～2500（对应PWM脉冲为0.5ms～2.5ms）,1500为静止，1500-2500为正向速度，500-1500为反向速度。
	角速度：twist.angular.x，这里角度范围为0～180度,90度为中间值，90-180度左转，0-90度右转。
3.里程计数据：
	里程计采用激光雷达和IMU数据融合的里程计。
	需要先启动IMU节点和雷达节点，参考art_racecar/launch/Run_car.launch
	然后启动rf2o节点，用rf2o生成激光里程计，参考art_racecar/launch/includes/rf2o.launch.xml
	再启动robot_localization用EKF融合里程计信息，参考art_racecar/launch/Run_gmapping.launch
	


											
											# Steven Zhang
											# 2019.01.30
	
