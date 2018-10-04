#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import curses

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('wall_switcher', anonymous=True)
em_pub = rospy.Publisher('side', Int32, queue_size=1)
k_pub = rospy.Publisher('eStop',Bool,queue_size=10)
v_pub = rospy.Publisher('drive_velocity', Int32, queue_size=1)
stdscr.refresh()
VELOCITY = 0

SPEEDS = [-70, -7, 0, 12, 23]

msg = Int32()
msg.data = 1
v_msg = Int32(0)
em_pub.publish(msg)
v_pub.publish(v_msg) 
k_pub.publish(False)

key = ''
while key != ord('q'):
    key = stdscr.getch()
    stdscr.refresh()
    if key == curses.KEY_LEFT:
        msg = Int32()
        msg.data = -1
        em_pub.publish(msg)
        stdscr.addstr(5, 20, "LEFT ")
    elif key == curses.KEY_RIGHT:
        msg = Int32()
        msg.data = 1
        em_pub.publish(msg)
        stdscr.addstr(5, 20, "RIGHT")
    elif key == curses.KEY_DOWN:
        VELOCITY -= 10
        v_msg.data = VELOCITY
        v_pub.publish(v_msg)
    elif key == 49:
        VELOCITY = v_msg.data = SPEEDS[0]
       
        v_pub.publish(v_msg)
    elif key == 50:
        VELOCITY = v_msg.data = SPEEDS[1]
        v_pub.publish(v_msg)
    elif key == 51:
        VELOCITY = v_msg.data = SPEEDS[2]
        v_pub.publish(v_msg)
    elif key == 52:
        VELOCITY = v_msg.data = SPEEDS[3]
        v_pub.publish(v_msg)
    elif key == 53:
        VELOCITY = v_msg.data = SPEEDS[4]
        v_pub.publish(v_msg)
    elif key == 110:
        for x in range(10):
            VELOCITY -= 10
            v_msg.data = VELOCITY
            v_pub.publish(v_msg)




k_pub.publish(True)

curses.endwin()
