#!/usr/bin/env python

import rospy
import baxter_interface


rospy.init_node("move_arm")

side = 'right'
limb = baxter_interface.Limb(side)
gripper = baxter_interface.Gripper(side)

#joint_names = limb.joint_names()
joint_names = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
angles = [0.4886142711056683, -0.7764225832358347, 0.8916709194109393, 0.22549400829659486, 0.8595084014738843, -1.5043151825662207, -0.6262580966375515]

joint_names = ['right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0', 'right_e1']
angles = [0.4621411865360754, -0.9958563918171109, -0.19181393201472519, -0.6815311173282016, 0.38051581068549245, 0.2184000027288132, 1.6489499224904172]
angles = [0.4610823722338945, -1.0228559191726059, -0.25255873086283237, -0.6381903868602483, 0.3792666085567302, 0.2255645801564961, 1.6609615115919942]


angles_dict = dict(zip(joint_names, angles)) 
limb.move_to_joint_positions(angles_dict)

