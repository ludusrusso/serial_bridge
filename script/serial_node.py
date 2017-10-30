#! /usr/bin/env python

'''
 Copyright (C) 2014 Politecnico di Torino

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this program; if not, write to the Free Software Foundation, Inc.,
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

 This software is developed within the PARLOMA project, which aims
 at developing a communication system for deablinf people (www.parloma.com)
 The PARLOMA project is developed with the Turin node of AsTech laboraroies
 network of Italian CINI (Consorzio Interuniversitario Nazionale di Informatica)

 Contributors:
     Ludovico O. Russo (ludovico.russo@polito.it)
'''

import rospy
from serial_bridge.msg import generic_serial

import serial
import sys

class SerialBridge():
    def __init__(self):
        # get parameters from ROS
        self._port = rospy.get_param('~port', '/dev/ttyACM0');
        self._baudrate= rospy.get_param('~baudrate', '9600');
        self._topic = rospy.get_param('serial_topic', '/serial_topic');

        self._serial_comm = serial.Serial(port=self._port, baudrate=self._baudrate)
        if (self._serial_comm.isOpen()):
            rospy.loginfo("Node connected to serial port %s ad baudrate %s"%(self._port,self._baudrate))
        else:
            rospy.logerr("Port %s not available"%(self._port,))
            return
        self._serial_comm.flushInput()
        self._serial_comm.flushOutput()

        # init topics
        rospy.Subscriber(self._topic, generic_serial, self._serial_callback)

        rospy.loginfo("Node Initialized")
        rospy.spin()

    def _serial_callback(self, serial_data):
        self._serial_comm.write(serial_data.msg)

if __name__ == '__main__':
    rospy.init_node('serial_bridge', anonymous=True)
    try:
        ne = SerialBridge()
    except rospy.ROSInterruptException: pass
