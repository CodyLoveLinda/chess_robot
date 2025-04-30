#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import time
import rospy
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot

from chess_robot_service.srv import gripper, gripperResponse
self.mc.set_gripper_value(100, 50)

mc = None

def get_timestamp():
    return round(time.time())

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data)
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(value)

    data_list[5] = data_list[5]-0.7853981633974483
    mc.send_radians(data_list, 80)

def handle_service_request(req):
    global mc
    rospy.loginfo("Received command: %s", req.command)
    if req.command == 'open':
        mc.set_gripper_value(40, 50)
    if req.command == 'close':
        mc.set_gripper_value(40, 50)
    feedback = True  #
    return gripperResponse(feedback=feedback)

def service_server():
    global mc
    rospy.init_node("mycobot_reciver", anonymous=True)

    port = rospy.get_param("~port", '/dev/ttyUSB0')
    baud = rospy.get_param("~baud", 1000000)
    print(port, baud)
    mc = MyCobot(port, baud)

    rospy.Subscriber("joint_states", JointState, callback)

    #rospy.init_node('string_service_server')
    service = rospy.Service('string_service', gripper, handle_service_request)
    rospy.loginfo("Service server ready.")
    rospy.spin()

if __name__ == "__main__":
    service_server()
