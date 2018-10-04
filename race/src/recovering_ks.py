#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import curses
from std_msgs.msg import Int32

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('kill_switch', anonymous=True)
em_pub = rospy.Publisher('eStop', Bool, queue_size=10)

stdscr.refresh()
em_pub.publish(False)
key = ''
while key != ord('q'):
    key = stdscr.getch()
    stdscr.refresh()
    if key == curses.KEY_DC:
        em_pub.publish(True)
        stdscr.addstr(5, 20, "Emergency STOP!!!!!")
    elif key == curses.KEY_HOME or key == 104:
        em_pub.publish(False)
        stdscr.addstr(5, 20, "Normal Operation :)")

em_pub.publish(True)
curses.endwin()
