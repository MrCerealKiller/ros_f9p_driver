#!/usr/bin/env python

import rospy
from ros_f9p_driver import ZEDF9P

if __name__ == "__main__":
    rospy.init_node('zed-f9p-gps')

    port = rospy.get_param('~port', '/dev/gps')
    baudrate = rospy.get_param('~baudrate', 460800)
    timeout = rospy.get_param('~timeout', 1)
    gnssType = rospy.get_param('~gnssType', 'GN')

    with ZEDF9P(port, baudrate, timeout, gnssType) as gps:
        try:
            gps.nmea_stream()
        except KeyboardInterrupt:
            gps.preempt()
