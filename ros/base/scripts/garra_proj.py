#! /usr/bin/env python
# -*- coding:utf-8 -*-
from __future__ import print_function
import rospy
import sys
import rospy

import garra_demo

tutorial = garra_demo.MoveGroupPythonIntefaceTutorial()


def pega_creeper():
    '''
    Manipula a garra para pegar o creeper.
    '''
    print("\n----------------------------------------------------------")
    print("\n============ Press `Enter` to open gripper  ...\n")
    input()
    tutorial.open_gripper()
    rospy.sleep(0.2)

    print("\n============ Press `Enter` to close gripper  ...\n")
    input()
    tutorial.close_gripper()
    rospy.sleep(0.2)

    print("\n============ Press `Enter` to go to init joint state ...\n")
    input()
    tutorial.go_to_init_joint_state()
    rospy.sleep(0.2)

    print("\n============ Press `Enter` to go to home joint state ...\n")
    input()
    tutorial.go_to_home_joint_state()
    rospy.sleep(0.2)

    print("\n============ Press `Enter` to open gripper  ...\n")
    input()
    tutorial.open_gripper()
    rospy.sleep(0.2)

    print("\n============ Press `Enter` to go to init goal ...\n")
    input()
    tutorial.go_to_zero_position_goal()
    rospy.sleep(0.2)

    print("\n============ Press `Enter` to close gripper  ...\n")
    input()
    tutorial.close_gripper()
    rospy.sleep(0.2)

    print("\n============ Press `Enter` to go to home goal ...\n")
    input()
    tutorial.go_to_home_position_goal()
    rospy.sleep(0.2)

    pegou = True