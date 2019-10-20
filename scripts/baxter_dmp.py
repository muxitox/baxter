#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *
from baxter_pykdl import baxter_kinematics
import csv


#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                   D_gain, num_bases):
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print "Starting LfD..."
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "LfD done"    
            
    return resp;


#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"   
            
    return resp;

# Read the angles from csv file
def read_trajectory_angles(input_file):
    print('')
    print('#### Start reading angles in ', input_file, '...')
    with open(input_file) as csvfile:
        cartesian_reader = csv.reader(csvfile)
        angles_list = list()
        for row in cartesian_reader:
            angles = [float(i) for i in row]
            angles_list.append(angles)

    return angles_list

if __name__ == '__main__':
    rospy.init_node('baxter_dmp')

    input_file = '../data/baxter_angles.csv'

    #Create a DMP from a 2-D trajectory
    dims = 7                
    dt = 1.0                
    K = 100                 
    D = 2.0 * np.sqrt(K)      
    num_bases = 4          
    traj = read_trajectory_angles(input_file)
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    #Start inverse kinematics system
    kin = baxter_kinematics('right')

    #Now, generate a plan
    x_0_pose = [0.579687640225,-0.183309445372,0.11368179783]
    x_0_orientation = [-0.140765802936,0.989646583677,-0.0116585947259,0.0254696939174]
    x_0 = kin.inverse_kinematics(x_0_pose, x_0_orientation)
    print(x_0)
    x_dot_0 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]   
    t_0 = 0                

    goal_pose = [0.663752423932,-0.218316410266,0.0838890619813]
    goal_orientation = [-0.063026429341,0.991362603102,0.0303186030565,0.110944313435]
    goal = kin.inverse_kinematics(goal_pose, goal_orientation, list(x_0))

    goal_thresh = [0.1,0.1,0.1,0.1,0.1,0.1,0.1]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * resp.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)


    print plan
    print(len(plan.plan.points))
    print(plan.plan.times)
