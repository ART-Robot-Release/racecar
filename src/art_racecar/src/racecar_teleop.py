#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty
import time

msg = """
Control Your racecar!
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

space key, k : force stop
w/x: shift the middle pos of throttle by +/- 5 pwm
a/d: shift the middle pos of steering by +/- 2 pwm
CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,-1),
        'm':(-1,1),
        'I':(1,0),
        'O':(1,-1),
        'J':(0,1),
        'L':(0,-1),
        'U':(1,1),
        '<':(-1,0),
        '>':(-1,-1),
        'M':(-1,1),
           }


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('racecar_teleop')
    pub = rospy.Publisher('~/car/cmd_vel', Twist, queue_size=5)

    speed_start_value = 70 
    turn_start_value = 50
    speed_mid = 1500
    turn_mid = 90
    speed_bias = 0
    turn_bias = 0
    speed_add_once = 5
    turn_add_once = 2
    control_speed = speed_mid
    control_turn = turn_mid
    speed_dir = 0
    last_speed_dir = 0
    last_turn_dir = 0
    last_control_speed = control_speed
    last_control_turn = control_turn

    run = 0

    try:
        while(1):
            key = getKey()
            twist = Twist()
            if key in moveBindings.keys():
                run = 1

                #print "key =",key
                speed_dir = moveBindings[key][0]
                if(speed_dir!=0 and speed_dir + last_speed_dir == 0):#Reverse
                    control_speed = speed_mid                 
                    last_speed_dir = speed_dir
                    control_turn = turn_mid
                    print "Reverse"
                else:
                    if(speed_dir > 0):
                        control_speed = speed_dir * (speed_start_value + speed_bias) + speed_mid 
                    elif(speed_dir < 0):
                        control_speed = speed_dir * (speed_start_value + speed_bias) + speed_mid - 140
                    else:
                        control_speed = 1500

                    control_turn = moveBindings[key][1] * (turn_start_value + turn_bias)+ turn_mid 
                    last_speed_dir = speed_dir
                   
                    #print " "
                #print "speed_dir = ",speed_dir
                #print "control_speed = ",control_speed
                #print "control_turn = ",control_turn
                last_control_speed = control_speed
                last_control_turn = control_turn
            elif key == ' ' or key == 'k' :
                speed_dir = -speed_dir #for ESC Forward/Reverse with brake mode
                control_speed = last_control_speed * speed_dir
                control_turn = turn_mid
                run = 0
            elif key == 'w' :
                speed_bias += speed_add_once              
                if(speed_bias >= 450):  
                    speed_bias = 450
                else:
                    last_control_speed = last_control_speed+speed_add_once
                run = 0
            elif key == 's' :
                speed_bias -= speed_add_once
                last_control_speed = last_control_speed-speed_add_once
                if(speed_bias <= 0):  
                    speed_bias = 0
                run = 0
            elif key == 'a' :
                turn_bias += turn_add_once
                last_control_turn = last_control_turn+turn_add_once 
                if(turn_bias >= 80):  
                    turn_bias = 80
                run = 0
            elif key == 'd' :
                turn_bias -= turn_add_once
                last_control_turn = last_control_turn-turn_add_once 
                if(turn_bias <= 0):  
                    turn_bias = 0
                run = 0
            else:
                run = 0   
            print vels(control_speed,control_turn)     
            #print "speed_dir=",speed_dir,"last_speed_dir",last_speed_dir
            if(run == 1):                      
                twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
                #print "control_speed =",control_speed
                pub.publish(twist)
            else:              
                twist.linear.x = speed_mid; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn_mid
                pub.publish(twist)

            if (key == '\x03'):   #for ctrl + c exit
                break

    except:
        print "error"

    finally:
        twist = Twist()
        twist.linear.x = speed_mid; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn_mid
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
