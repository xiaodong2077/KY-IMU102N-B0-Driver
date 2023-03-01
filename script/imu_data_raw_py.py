#!/usr/bin/python
# coding=utf-8

# Modifications Copyright 2023 Shadow.
# Editor: Shadow(8888888888@duck.com)
# Copyright 2020 Wechange Tech.
# Developer: FuZhi, Liu (liu.fuzhi@wechangetech.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from genpy import rostime
import rospy
import tf
import time
import sys
import math
import serial
import string
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import ctypes
import struct

import numpy as np
import scipy.signal as signal

from bondpy import bondpy


class queue:
    def __init__(self, capacity=1024*4):
        self.capacity = capacity
        self.size = 0
        self.front = 0
        self.rear = 0
        self.array = [0]*capacity

    def is_empty(self):
        return 0 == self.size

    def is_full(self):
        return self.size == self.capacity

    def enqueue(self, element):
        if self.is_full():
            raise Exception('queue is full')
        self.array[self.rear] = element
        self.size += 1
        self.rear = (self.rear + 1) % self.capacity

    def dequeue(self):
        if self.is_empty():
            raise Exception('queue is empty')
        self.size -= 1
        self.front = (self.front + 1) % self.capacity

    def get_front(self):
        return self.array[self.front]

    def get_front_second(self):
        return self.array[((self.front + 1) % self.capacity)]

    def get_queue_length(self):
        return (self.rear - self.front + self.capacity) % self.capacity

    def show_queue(self):
        for i in range(self.capacity):
            print (self.array[i],)
        print(' ')


class Sensor:
    def __init__(self):
        self.G = 9.80665
        self.degree2Rad = 3.1415926 / 180.0

        self.device_port = "/dev/ttyUSB0"
        self.baudrate = 921600
        self.imuId = "imu_frame"
        self.imu_topic = "imu"
        self.imu_freq = 200
        self.magId = "mag"
        self.mag_topic = "mag"
        self.mag_freq = 70

        self.current_time = rospy.Time.now()
        self.previous_time = self.current_time
        self.serialIDLE_flag = 0
        self.ImuErrFlag = False
        self.Circleloop = queue(capacity=1024*80)
        self.Gyro = [0, 0, 0]
        self.Accel = [0, 0, 0]
        self.Quat = [0, 0, 0, 0]
        self.Mag = [0, 0, 0]
        self.firmware_version = [0, 0, 0]
        self.hardware_version = [0, 0, 0]
        self.last_cmd_vel_time = rospy.Time.now()
        self.last_ackermann_cmd_time = rospy.Time.now()

        try:
            self.serial = serial.Serial(
                self.device_port, self.baudrate, timeout=10)
            rospy.loginfo("Opening Sensor")
            try:
                if self.serial.in_waiting:
                    self.serial.readall()
            except:
                rospy.loginfo("Opening Sensor Try Faild")
                pass
        except:
            rospy.logerr("Can not open Serial"+self.device_port)
            self.serial.close()
            sys.exit(0)
        rospy.loginfo("Sensor Open Succeed")

        self.imu_pub = rospy.Publisher(self.imu_topic, Imu, queue_size=10)
        self.mag_pub = rospy.Publisher(
            self.mag_topic, MagneticField, queue_size=10)

        self.timer_communication = rospy.Timer(
            rospy.Duration(1.0/1000.0), self.timerCommunicationCB)
        self.timer_imu = rospy.Timer(rospy.Duration(
            1.0/500.0), self.timerIMUCB)

        time.sleep(0.1)

    def timerCommunicationCB(self, event):
        length = self.serial.in_waiting
        if length:
            reading = self.serial.read_all()
            if len(reading) != 0:
                for i in range(0, len(reading)):
                    data = (int(reading[i].encode('hex'), 16))
                    try:
                        self.Circleloop.enqueue(data)
                    except:
                        rospy.logerr("Circleloop.enqueue Faild")
        else:
            pass

        if self.Circleloop.is_empty() == False:
            if self.Circleloop.is_empty() == False:
                data = self.Circleloop.get_front()
            else:
                pass
            if data == 0x5a:
                queue_length = self.Circleloop.get_queue_length()
                if queue_length >= 59:
                    if self.Circleloop.get_front_second() == 0x5a:
                        databuf = []
                        for i in range(59):
                            databuf.append(self.Circleloop.get_front())
                            self.Circleloop.dequeue()
                        checkSum = 0
                        for i in range(2, 58):
                            checkSum += databuf[i]
                        checkSum = checkSum & 0xff
                        sumStr = '{0:08b}'.format(checkSum)
                        checkStr = '{0:08b}'.format(databuf[58])

                        if sumStr == checkStr:
                            self.current_time = rospy.Time.now()

                            for i in range(3):
                                x = databuf[i*4+2:i*4+6]
                                self.Gyro[i] = struct.unpack(
                                    '<f', struct.pack('4B', *x))[0]

                            for i in range(3):
                                x = databuf[i*4+14:i*4+18]
                                self.Accel[i] = struct.unpack(
                                    '<f', struct.pack('4B', *x))[0]

                        else:
                            pass
                else:
                    pass
            else:
                self.Circleloop.dequeue()
        else:
            # rospy.loginfo("Circle is Empty")
            pass

    def timerIMUCB(self, event):
        self.serialIDLE_flag = 0
        msg = Imu()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.imuId

        msg.angular_velocity.x = self.Gyro[0]*self.degree2Rad
        msg.angular_velocity.y = self.Gyro[1]*self.degree2Rad
        msg.angular_velocity.z = self.Gyro[2]*self.degree2Rad

        msg.linear_acceleration.x = self.Accel[0]*self.G
        msg.linear_acceleration.y = self.Accel[1]*self.G
        msg.linear_acceleration.z = self.Accel[2]*self.G

        msg.orientation.w = 1
        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0

        self.imu_pub.publish(msg)


# main function
if __name__ == "__main__":
    try:
        rospy.init_node('ky_imu102n_b0_driver', anonymous=True)
        bc = Sensor()
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
        print("Shutting down")
