#!/usr/bin/env python

import rospy
import serial
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
from uwb.msg import UwbData

def serial_listener():
    rospy.init_node('UWB',anonymous=False)
    port_name = rospy.get_param('~serial_port','/dev/uwb_port')
    pub=rospy.Publisher('UwbData',UwbData,queue_size=10)
    ser = serial.Serial(
                port=port_name,
                baudrate=115200)
    while not rospy.is_shutdown():
        if ser.in_waiting:
            line=ser.readline().decode('utf-8').strip()
            data=line.split(',')
            if len(data) ==3:
                msg = UwbData()
                msg.header.stamp = rospy.Time.now()
                msg.Address = int(data[0])
                msg.range = float(data[1])
                msg.rxPower = float(data[2])
                pub.publish(msg)
         #       rospy.loginfo(msg)

if __name__ == '__main__':
    try:
        serial_listener()
    except rospy.ROSInterruptException:
        pass
