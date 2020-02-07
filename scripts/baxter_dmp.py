#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *
from baxter_pykdl import baxter_kinematics
import csv
import argparse
import baxter_interface



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
        angles_reader = csv.reader(csvfile)
        angles_list = list()
        for row in angles_reader:
            angles = [float(i) for i in row]
            angles_list.append(angles)

    return angles_list

# Read the cartesian points from csv file
def read_trajectory_points(input_file, fix_orientation):
    print('')
    print('#### Start reading cartesian points in ', input_file, '...')
    # fixed_orientation = [x, y, z, w]
    fixed_orientation = [0.527010502809, 0.551633072459, 0.491366471717, 0.420142682633]
    with open(input_file) as csvfile:
        cartesian_reader = csv.reader(csvfile, delimiter=' ')
        points_list = list()
        for row in cartesian_reader:
            point = [float(i.replace(',','.')) for i in row]
            point = point[0:3]

            if fix_orientation:
                point.extend(fixed_orientation)
            else:
                #TODO  Make calculations to calculate orientation
                print('TODO:')
                point.extend(fixed_orientation)
            points_list.append(point)

    return points_list

if __name__ == '__main__':

    # Read the console arguments

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)

    # Select angles or cartesian points
    arg_group = parser.add_mutually_exclusive_group(required=True)
    arg_group.add_argument("-a", "--angles", dest="angles",
        action='store_true', default=False, help="set if trainin with angles (pre-conversion)")
    arg_group.add_argument("-c", "--cartesian", dest="cartesian",
        action='store_true', default=False, help="set if training with cartesian points (post-conversion)")

    # Input file
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-i', '--input-file', dest='input_file', required=False,
        default='~/catkin_ws/src/dmp/data/Data/deAtrasAlante.csv', help='the file name to read from'
    )
    # Output file
    required.add_argument(
        '-o', '--output-file', dest='output_file', required=False,
        help='the file name to record to'
    )
    required.add_argument("-f", action="store_true", dest="fix", help="define if you want to fix the orientation")
    args = parser.parse_args(rospy.myargv()[1:])

    input_file = args.input_file
    angles = args.angles
    cartesian = args.cartesian

    rospy.init_node('baxter_dmp')

    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()

    # Read the trajectory
    points = read_trajectory_points(input_file, args.fix)

    #Start inverse kinematics system
    side = 'right'
    kin = baxter_kinematics(side)

    #Now, generate a plan

    # Set initial position and goal
    x_0_pose = [0.02284265,-0.2563786, 0.1964981]
    x_0_orientation = [0.527010502809, 0.551633072459, 0.491366471717, 0.420142682633]
    t_0 = 0  

    goal_pose = [0.4781504, -0.167219, 0.2498018]
    goal_orientation = [0.527010502809, 0.551633072459, 0.491366471717, 0.420142682633]

    # Make previous conversion to angles if requested:
    if angles:

        print('Converting to angles before DMP...')
        #seed = [0.18820418589865534, -0.24293864083427794, 0.21197731608154377,  0.527010502809, 0.551633072459, 0.491366471717, 0.420142682633]


        traj = []
        first_iter = True
        for point in points:
            point_pose = point[0:3]
            point_orientation = point[3:]

            # TODO: INTRODUCE AN INITIAL SEED TO CHANGE THE FORM OF THE MOVEMENT
            if first_iter:
                import pdb; pdb.set_trace()

                traj_i = kin.inverse_kinematics(point_pose, point_orientation)
                first_iter = False
            else:
                traj_i = kin.inverse_kinematics(point_pose, point_orientation, list(traj_i))


            print(traj_i)
            traj.append(traj_i)

                      
        x_0 = kin.inverse_kinematics(x_0_pose, x_0_orientation, list(seed))


        goal_pose = [0.4781504, -0.167219, 0.2498018]
        goal_orientation = [0.527010502809, 0.551633072459, 0.491366471717, 0.420142682633]
        goal = kin.inverse_kinematics(goal_pose, goal_orientation, list(x_0))

    else:
        print('Using cartesian points for DMP...')
        x_0_pose.extend(x_0_orientation)
        x_0 = x_0_pose               

        goal_pose.extend(goal_orientation)
        goal = goal_pose

        traj = points

    x_dot_0 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]


    #Create a DMP from a trajectory
    dims = 7                # Number of dimensions
    dt = 1.0                # Time resolition of plan
    K = 100                 # List of proportional gains
    D = 2.0 * np.sqrt(K)    # D_gains
    num_bases = 200           # Number of basis functions to use

    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    # Threshold in each dimension, the threshold is the same as both cases have the same number of dimensions
    # And we are not very restrictive
    goal_thresh = [0.1,0.1,0.1,0.1,0.1,0.1,0.1]

    seg_length = -1          #Plan until convergence to goal
    #tau = 2 * resp.tau       #Desired plan should take twice as long as demo
    tau = resp.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)

    ###
    # Make posterior conversion to angles if needed
    ###

    plan_list = []
    first_iter = True
    for point in plan.plan.points:
        position = list(point.positions)

        # If trained with cartesian points, convert them to angles, o.w. it's already in angles
        if cartesian:
            if first_iter:
                converted = kin.inverse_kinematics(position[0:3], position[3:])
                first_iter = False
            else:
                converted = kin.inverse_kinematics(position[0:3], position[3:], list(converted))
            converted = position
            

        plan_list.append(position)

    limb = baxter_interface.Limb(side)
    #gripper = baxter_interface.Gripper(side)

    joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']


    for position in plan_list:
        angles_dict = dict(zip(joint_names, position))
        print(angles_dict)
        limb.move_to_joint_positions(angles_dict)
    

    '''
    # Save the plan if requested
    if args.output_file is not None:
        print('Saving plan...')
    '''
