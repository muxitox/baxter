#!/usr/bin/env python
import roslib;
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *
import csv
import argparse
import actionlib
import sys
from copy import copy

import baxter_interface

from sensor_msgs.msg import JointState

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
# My class for doing de DMPs
class DynamicMovementPrimitives:

    def __init__(self, _dims, _dt, _K_gain, _D_gain, _num_bases, _hi):
        self.dims = _dims
        self.dt = _dt
        self.K_gain = _K_gain
        self.D_gain = _D_gain
        self.num_bases = _num_bases
        self.hi = _hi

    def fit(self,  _trajectory):
        self.trajectory = _trajectory

        self.resp = self.makeLFDRequest(self.dims, self.trajectory, self.dt, self.K_gain,
                                        self.D_gain, self.num_bases, self.hi)

        self.makeSetActiveRequest(self.resp.dmp_list)

        return self.resp

    # Learn a DMP from demonstration data
    def makeLFDRequest(self, dims, traj, dt, K_gain, D_gain, num_bases, hi):
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
            resp = lfd(demotraj, k_gains, d_gains, num_bases, hi)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "LfD trained"
                
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
        print "Starting DMP planning..."
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
            resp = gdp(_x_0, _x_dot_0, _t_0, _goal, _goal_thresh, _seg_length, _tau, _dt, _integrate_iter)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "DMP planning done"   
                
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

    # Move left arm to a position where it does not trouble
    limb = baxter_interface.Limb("left")
    joint_names = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    position = [-0.005368932757598948, 2.6173547193294873, 2.3531265286162246, -1.6965827514012677, -2.1456556270547225,
                0.11313108310654926, 0.08858739050038264]
    angles_dict = dict(zip(joint_names, position))
    limb.move_to_joint_positions(angles_dict)

    # Read the trajectory
    joint_names, traj = read_trajectory_joints(input_file)


    # Create DMP class with initial parameters
    dims = 7                # Number of dimensions
    dt = 0.5                # Time resolution of the plan
    K = 100                 # List of proportional gains
    D = 2.0 * np.sqrt(K)    # D_gains
    num_bases = 100           # Number of basis functions to use
    hi = 3

    DMPs = DynamicMovementPrimitives(dims, dt, K, D, num_bases, hi)

    # Start inverse kinematics system
    side = "right"

    # Train the DMPs
    resp = DMPs.fit(traj)

    # Make the query with the new initial position and goal
    # Set initial position
    x_0 = traj[0]
    x_0 = [-0.6757185370635248, -0.7708253459124204, 1.426985627930406, 1.4611167004608565, 0.49700977527487405, -0.003451456772742181, -1.2732040539448934]
    x_dot_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    t_0 = 0

    # Set goal
    goal = traj[-1]
    goal = [-0.1606844875309971, -0.5526165788357204, 1.4530633013244583, 1.716524501643778, 0.2949078064709708, -0.66306319556347, -1.2755050251267215]

    # Threshold in each dimension
    goal_thresh = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

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


    '''
    ##
    # Move the robot according to the plan
    ##
    limb = baxter_interface.Limb(side)
    rate = rospy.Rate(100)

    for position in plan_list:
        angles_dict = dict(zip(joint_names, position))
        limb.move_to_joint_positions(angles_dict)

        rate.sleep()

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


