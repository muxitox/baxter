#!/usr/bin/env python
import roslib;
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *
import csv
import argparse
import math

import baxter_interface

from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


# My class for doing de DMPs
class DynamicMovementPrimitives:

    def __init__(self, _dims, _dt, _K_gain, _D_gain, _num_bases, _hi, _alpha):
        self.dims = _dims
        self.dt = _dt
        self.K_gain = _K_gain
        self.D_gain = _D_gain
        self.num_bases = _num_bases
        self.hi = _hi
        self.alpha = alpha

    def fit(self,  _trajectory):
        self.trajectory = _trajectory

        self.resp = self.makeLFDRequest(self.dims, self.trajectory, self.dt, self.K_gain,
                                        self.D_gain, self.num_bases, self.hi, self.alpha)

        self.makeSetActiveRequest(self.resp.dmp_list)

        return self.resp

    # Learn a DMP from demonstration data
    def makeLFDRequest(self, dims, traj, dt, K_gain, D_gain, num_bases, _hi, _alpha):
        demotraj = DMPTraj()
            
        for i in range(len(traj)):
            pt = DMPPoint();
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(dt*i)
                
        k_gains = [K_gain]*dims
        d_gains = [D_gain]*dims
            
        # print "Starting LfD..."
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj, k_gains, d_gains, num_bases, _hi, _alpha)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        # print "LfD trained"
                
        return resp

    # Set a DMP as active for planning
    def makeSetActiveRequest(self, dmp_list):
        try:
            sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    #
    def predict(self, _x_0, _x_dot_0, _t_0, _goal, _goal_thresh, _seg_length, _tau, _dt, _integrate_iter):
        return self.makePlanRequest(_x_0, _x_dot_0, _t_0, _goal, _goal_thresh, _seg_length, _tau, _dt, _integrate_iter)

    # Generate a plan from a DMP
    def makePlanRequest(self, _x_0, _x_dot_0, _t_0, _goal, _goal_thresh, _seg_length, _tau, _dt, _integrate_iter):
        # print "Starting DMP planning..."
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
            resp = gdp(_x_0, _x_dot_0, _t_0, _goal, _goal_thresh, _seg_length, _tau, _dt, _integrate_iter)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        # print "DMP planning done"
                
        return resp


# Read the cartesian points from csv file
def read_trajectory_joints(_input_file):
    print('')
    print('#### Start reading joints in ', input_file, '...')

    header = True
    with open(_input_file) as csvfile:
        _cartesian_reader = csv.reader(csvfile, delimiter=',')
        _joints_list = list()
        for row in _cartesian_reader:
            if header:
                _joint_names = row
                _joint_names = _joint_names[1:]
                header = False
            else:
                _joints = [float(i) for i in row]
                _joints_list.append(_joints[1:])

    return _joint_names, _joints_list


'''
Main
'''
if __name__ == '__main__':

    # Read the console arguments
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)


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
    args = parser.parse_args(rospy.myargv()[1:])

    input_file = args.input_file

    # Start a node
    rospy.init_node('baxter_dmp')

    # Enable the robot
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()

    # Read the trajectory
    joint_names, traj = read_trajectory_joints(input_file)

    alpha = [0.1, 0.2, 0.3, 0.5, 0.6, 0.8, 0.01, 0.02, 0.03, 0.04, 0.05, 0.08, 0.001]
    #alpha_log_10 = [-math.log(i, 10) for i in alpha]
    alpha_log = [-math.log(i) for i in alpha]
    #alpha_log.extend(alpha_log_10)
    list_num_bases = [1, 2, 3, 4, 5, 7, 10, 13, 15, 20, 25, 40, 50, 65, 75, 100, 125, 150, 175, 185, 200]
    list_hi = [0.0000001, 0.0000003, 0.0000005, 0.0000008, 0.000001, 0.000003, 0.000005, 0.000008, 0.00001, 0.00003,
               0.00005, 0.00008, 0.0001, 0.0003, 0.0005, 0.0008, 0.001, 0.003, 0.005, 0.008, 0.01, 0.03, 0.05, 0.08,
               0.1, 0.3, 0.5, 0.8, 1, 1.3, 1.5, 1.8, 2, 5, 10]

    len_num_bases = len(list_num_bases)
    len_hi = len(list_hi)
    len_alpha = len(alpha_log)
    total_combs = len_num_bases * len_hi * len_alpha

    current_min = float("inf")
    best_hi = 0
    best_nb = 0
    best_alpha = 0

    i = 1
    for hi in list_hi:
        for num_bases in list_num_bases:
            for alpha in alpha_log:

                # Create DMP class with initial parameters
                dims = 7                # Number of dimensions
                dt = 0.5                # Time resolution of the plan
                K = 100                 # List of proportional gains
                D = np.sqrt(K)    # D_gains
                # num_bases = 100           # Number of basis functions to use
                # hi = 3

                DMPs = DynamicMovementPrimitives(dims, dt, K, D, num_bases, hi, alpha)

                # Start inverse kinematics system
                side = "right"

                # Train the DMPs
                resp = DMPs.fit(traj)

                # Make the query with the new initial position and goal
                # Set initial position
                x_0 = traj[0]
                x_dot_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                t_0 = 0

                # Set goal
                goal = traj[-1]

                # Threshold in each dimension
                goal_thresh = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]

                seg_length = -1          # Plan until convergence to goal
                # tau = 2 * resp.tau      # Desired plan should take twice as long as demo
                tau = resp.tau       # Desired plan should take twice as long as demo
                dt = 0.5
                integrate_iter = 5       # dt is rather large, so this is > 1
                plan = DMPs.predict(x_0, x_dot_0, t_0, goal, goal_thresh, seg_length, tau, dt, integrate_iter)


                # Extract the plan as a list of points
                plan_list = []
                for point in plan.plan.points:
                    position = list(point.positions)
                    plan_list.append(position)

                plan_list = plan_list[:len(traj)]

                error = 0
                for source, target in zip(traj, plan_list):
                    source = np.array(source)
                    target = np.array(target)

                    error += np.linalg.norm(source - target)

                print(i, '/', total_combs)
                print('Num_bases', num_bases, 'hi', hi, 'alhpa', alpha, 'Error', error)

                i += 1

                if error < current_min:
                    current_min = error
                    best_hi = hi
                    best_nb = num_bases
                    best_alpha = alpha

                    #print('Num_bases', num_bases, 'hi', hi, 'alhpa', alpha, 'Error', error)

    print('Min error', current_min, 'Best num bases', best_nb, 'alpha', alpha, 'Best hi', best_hi)

    '''
    print('Writing to file... ', args.output_file)

    # Save plan if requested
    header_list = ['time']
    header_list.extend(joint_names)
    header = ','.join(header_list)

    f = open(args.output_file, 'w')
    f.write(header+'\n')

    for (point, timestamp) in zip(plan_list, plan.plan.times):
        line = [timestamp-dt]
        line.extend(point)
        line = ','.join(map(str, line))
        f.write(line+'\n')

    f.close()
    '''
