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
angles = [0.7241458575900799, 0.3999019701723015, 0.02614341138253233, -1.2026141129047758, -2.9135921280396446, -0.7378757274568828, 0.5253737338418242]



angles_dict = dict(zip(joint_names, angles)) 
limb.move_to_joint_positions(angles_dict)

