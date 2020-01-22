#!/usr/bin/env python

import rospy
import baxter_interface


rospy.init_node("move_arm")

side = 'right'
limb = baxter_interface.Limb(side)
gripper = baxter_interface.Gripper(side)

#joint_names = limb.joint_names()
joint_names = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']

angles_0 = [0.8764725942919336, 1.5562521109309468, -1.7016798893475187, -1.4716329185751462, -0.3670479393311279, 0.3030074113979131, -0.20293509149000855]
angles_1 = [1.7594962249613975, 1.8523185500652843, -1.0038370415321767, 0.07553092519912763, -0.36832549045606644, 0.3014800928082826, -0.2029696812809494]


angles_dict = dict(zip(joint_names, angles_1)) 
limb.move_to_joint_positions(angles_dict)

