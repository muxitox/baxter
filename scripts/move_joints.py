#!/usr/bin/env python

import rospy
import baxter_interface


rospy.init_node("move_arm")

side = 'right'
limb = baxter_interface.Limb(side)
gripper = baxter_interface.Gripper(side)

angles = [-0.459, -0.202, 1.807, 1.714, -0.906, -1.545, -0.276]

angles_dict = dict(zip(limb.joint_names(), angles)) 
limb.move_to_joint_positions(angles_dict)

gripper.open()
