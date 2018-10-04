#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import curses
from std_msgs.msg import Int32
import time

'''
Brake and then go at 20 velocity
'''
def brakePump():
        global em_pub, activated
        for i in range(1):
            v_msg = Int32(-70)
            for x in range(10):
                v_msg.data -= 10
                v_pub.publish(v_msg)

            time.sleep(0.4)

            for x in range(19):
                if v_msg.data > 12:
                    v_msg.data = 12
                v_msg.data += 10
                v_pub.publish(v_msg)

            time.sleep(0.3)
            #em_pub.publish(True)
            print("BREAKS PUMPED")

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('kill_switch', anonymous=True)
em_pub = rospy.Publisher('eStop', Bool, queue_size=10)
v_pub = rospy.Publisher('drive_velocity', Int32, queue_size=1)

stdscr.refresh()
em_pub.publish(False)
key = ''
while key != ord('q'):
    key = stdscr.getch()
    stdscr.refresh()
    if key == curses.KEY_DC or key==107:
        v_msg = Int32(0)
        v_pub.publish(v_msg)
        #em_pub.publish(True)
        stdscr.addstr(5, 20, "Emergency STOP!!!!!")
        for x in range(10):
            v_msg.data -= 10
            v_pub.publish(v_msg)
        
        em_pub.publish(True)
    elif key == curses.KEY_HOME or key == 104:
        em_pub.publish(False)
        stdscr.addstr(5, 20, "Normal Operation :)")
    elif key == 98:
        brakePump()

#em_pub.publish(True)
curses.endwin()
