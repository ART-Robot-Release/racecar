#!/usr/bin/env python
# -*- coding:utf-8 -*-

import serial
import math
import time
from array import *
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

d2r = (2 * math.pi) / 360.0
r2d = 360.0 / (2 * math.pi)
pack_len = 1812

class ls01c:
    def __init__(self):
        # default parameters on parameter server
        self.angle_min = rospy.get_param('~/angle_min', -math.pi)
        self.angle_max = rospy.get_param('~/angle_max', math.pi)
        serial_baudrate = rospy.get_param('~/baudrate', 230400)
        serial_port = rospy.get_param('~/port', '/dev/laser')
        ScanPub = rospy.Publisher('scan', LaserScan, queue_size=5)
        ScanPub2 = rospy.Publisher('scan2', LaserScan, queue_size=5)
        angle_range1 = np.arange(0, -math.pi, math.radians(-1))
        angle_range2 = np.arange(math.pi, 0, math.radians(-1))
        self.angle_range = np.append(angle_range1, angle_range2)
        rospy.loginfo("laser is connected through port:%s and baudrate is %d" %(serial_port, serial_baudrate))
        rospy.loginfo("min angle is :%f and max angle is %f" %(self.angle_min, self.angle_max))
        rospy.on_shutdown(self.shutdown)

        header = [0xA5]
        ender = [0xE1, 0xAA, 0xBB, 0xCC, 0xDD]
        cmd_control = [0x3A]
        cmd_start = [0x2C]
        cmd_scan_continue = [0x20]
        cmd_scan_once = [0x22]
        cmd_stop = [0x21]
        cmd_stop_rot = [0x25]
        self.cmdstr_start = array("B", header + cmd_start + ender).tostring()
        self.cmdstr_control = array("B", header + cmd_control + ender).tostring()
        self.cmdstr_scan_continue = array("B", header + cmd_scan_continue + ender).tostring()
        self.cmdstr_scan_once = array("B", header + cmd_scan_once + ender).tostring()
        self.cmdstr_stop = array("B", header + cmd_stop + ender).tostring()
        self.cmdstr_stop_rot = array("B", header + cmd_stop_rot + ender).tostring()

        try:
            self.device = serial.Serial(serial_port, serial_baudrate, timeout=0.05)
            print("port opened successfully")
        except:
            print("Port failed open.\n"
                  "Are you sure device is connected correctly?\n"
                  "Please press (ctrl+c) and restart it after checked!"
                  )
            return

        time.sleep(0.2)
        #self.device.write(self.cmdstr_control)
        #time.sleep(3)
        self.device.write(self.cmdstr_start)
        time.sleep(0.2)
        #self.device.write(self.cmdstr_stop)  ## make sure no data is sending
        #time.sleep(0.2)
        self.device.write(self.cmdstr_scan_continue)
        time.sleep(5)

        rospy.loginfo("lidar is ready!")
        print self.angle_min, self.angle_max
        cnt = 0
        try:
            while not rospy.is_shutdown():
                laser_distance, laser_angle = self.resolveData()
                rospy.loginfo_throttle(30, "return data length is %d --->> "
                                           "this message appear every 30s to check driver"
                                       % len(laser_distance))
                if (cnt% 2== 0):
                    self.ls_scan_pub(ScanPub, laser_distance, laser_angle)
                else:
                    self.ls_scan_pub(ScanPub2, laser_distance, laser_angle)
                cnt += 1
                rospy.sleep(0.01)
        except:
            pass
        rospy.logwarn("keyboard interrupt---> rosnode shutdown is requested!")
        self.LaserStop()

    def LaserStop(self):
        self.device.write(self.cmdstr_stop)
        time.sleep(0.2)
        self.device.write(self.cmdstr_stop_rot)
        time.sleep(0.2)
        print('Laser is stopped to send data and thread is stopped')

    def resolveData(self):
        buffer_len = self.device.inWaiting()
        if buffer_len >= (pack_len * 3):
            print("need to empty buffer")
            self.device.read(pack_len * (buffer_len / pack_len))
        data = self.device.read(pack_len)
        cnt = 0
        while len(data) != pack_len:
            cnt += 1
            data_memst = self.device.read(pack_len - len(data))
            data = data + data_memst
            print("timeout %d times to read data from laser" % cnt)
            if cnt >= 50:
                raise IOError
        # print "go"
        if (data[0] == '\xA5') and (data[6] == '\x81'):
            # print "found"
            print ord(data[1]) / 15.0
            data_final = data[7:pack_len]
            pass
        else:
            while 1:
                data_check = self.device.read(1)
                if data_check != '\xA5':
                    continue
                else:
                    dat2 = self.device.read(6)
                    if dat2[-1] != '\x81':
                        continue
                    else:
                        data_final = self.device.read(pack_len - 7)
                        print ord(data_final[-4]), ord(data_final[-1])
                        if (data_final[-4] == '\xAA') and (data_final[-1] == '\xDD'):
                            break
                        else:
                            continue
        laser_distance = [0] * 360
        laser_angle = [0] * 360
        for i in range(360):
            laser_angle[i] = -(ord(data_final[2 + i * 5]) + ord(data_final[2 + i * 5]) * 256) / 10.0 * ((2 * math.pi) / 360.0)
            laser_distance[i] = (ord(data_final[3 + i * 5]) + ord(data_final[4 + i * 5]) * 256) / 1000.0

        return laser_distance, laser_angle

    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the laser...")
        # self.dev.LaserStop()

    def ls_scan_pub(self, pub, laser_distance1, laser_angle1):
        # distance data is from -pi to pi
        laser_distance1 = np.array(laser_distance1)
        scan_msg = LaserScan()
        point_num = 360.0
        scan_msg.angle_max = self.angle_max  # LS-lidar rotates clockwise, which is disobey right-hand rules for coordinate system
        scan_msg.angle_min = self.angle_min
        scan_msg.header.frame_id = 'base_laser_link'
        scan_msg.angle_increment = 2 * math.pi / point_num
        scan_msg.range_min = 0.15
        scan_msg.range_max = 3.0
        scan_msg.header.stamp = rospy.Time.now()
        # print("begin to select")
        if (self.angle_min > 0) or (self.angle_max <= 0):
            range_part = laser_distance1[(self.angle_range >= self.angle_min) & (self.angle_range <= self.angle_max)]
            scan_msg.ranges = np.flipud(range_part)
        else:
            range_part1 = laser_distance1[(self.angle_range >= self.angle_min) & (self.angle_range <= 0)]
            range_part2 = laser_distance1[(self.angle_range <= self.angle_max) & (self.angle_range > 0)]
            p1 = np.flipud(range_part1)
            p2 = np.flipud(range_part2)
            scan_msg.ranges = np.append(p1, p2)
        scan_msg.intensities = [255] * len(laser_distance1)
        pub.publish(scan_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('LeiShen_Lidar')
        rospy.loginfo("Leishen node is initialized!")
        print("Initialize node %s with Node_URI %s under namespace: %s"
              % (rospy.get_name(), rospy.get_node_uri(), rospy.get_namespace()))
        if rospy.get_namespace() != '/':
            rospy.logwarn("This node is contained under namespace %s, be careful" %rospy.get_namespace())
        ls01c = ls01c()
        try:
            rospy.spin()
            # pass
        except:
            pass
        # rospy.logwarn("keyboard interrupt---> rosnode shutdown is requested!")

    except rospy.ROSInterruptException:
        rospy.logerr("Leishen node initialization failed!")

