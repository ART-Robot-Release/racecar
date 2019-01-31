#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import sys, select, termios, tty
import time

msg = """

CTRL-C to quit
"""
global run
global speed_add_once,turn_add_once,speed_mid,turn_mid,control_speed,control_turn,speed_max,turn_max,speed_min,turn_min
global last_speed_add_once,last_turn_add_once



def callback(data):
    twist = Twist()   
    car_twist = Twist() 
    global speed_add_once,turn_add_once
    global last_turn_add_once,last_speed_add_once
    if (data.buttons[6] == 1 or data.buttons[7] == 1):
        twist.linear.x = 0
        twist.angular.z = 0
        print("STOP!!")
    else:
        if(data.axes[5] != 0):
            if(last_speed_add_once == 0):
                speed_add_once = speed_add_once + data.axes[5] * 0.1
            last_speed_add_once = 1
        else:
            last_speed_add_once = 0
        if(data.axes[4] != 0):
            if(last_turn_add_once == 0):
                turn_add_once = turn_add_once + data.axes[4] * 0.1
            last_turn_add_once = 1
        else:
            last_turn_add_once = 0
        if(speed_add_once > 1):
            speed_add_once = 1
        elif(speed_add_once < -1):
            speed_add_once = -1
        if(turn_add_once > 1):
            turn_add_once = 1
        elif(turn_add_once < -1):
            turn_add_once = -1
        twist.linear.x = data.axes[3] * speed_add_once
        twist.angular.z = data.axes[2] * turn_add_once
        control_speed = (data.axes[3] * (speed_max - speed_min) * speed_add_once / 2 + speed_mid)  
        control_turn = (data.axes[2] * (turn_max - turn_min) * turn_add_once  / 2 + turn_mid) 
        #print('speed: %.2f, turn: %.2f'%(twist.linear.x,twist.angular.z))
        car_twist.linear.x = control_speed
        car_twist.angular.z = control_turn
        print('speed: %.2f, turn: %.2f'%(control_speed,control_turn))
    pub.publish(twist)
    pub_car.publish(car_twist)



def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)
    speed_add_once = 0.5
    turn_add_once = 0.5
    speed_max = 2500
    speed_min = 500
    speed_mid = (speed_max + speed_min)/2
    turn_max = 180
    turn_min = 0
    turn_mid = (turn_max + turn_min)/2

    control_speed = speed_mid
    control_turn = turn_mid

    global pub
    rospy.init_node('racecar_joy')
    pub = rospy.Publisher('~/cmd_vel', Twist, queue_size=5)
    pub_car = rospy.Publisher('~/car/cmd_vel', Twist, queue_size=5)
    try:
       
        while(1):
            
            key = getKey()
           
            rospy.Subscriber("joy", Joy, callback)
            
            rospy.spin()
            
            if (key == '\x03'):   #for ctrl + c exit
                break 
           

    except:
        print "error"

    finally:
    	print "finally"
        twist = Twist()
        car_twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        pub_car.publish(car_twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
