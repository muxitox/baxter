#!/usr/bin/env python
import roslib;
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *
import csv
import argparse
import sys
from copy import copy
import math

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


'''
Inverse Kinematics Solver (With manual seeds)
'''
class IKSolver:

    def __init__(self, _limb,  _verbose=False):
        self._verbose = _verbose
        ns = "ExternalTools/" + _limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)

        # Seeds for mover liquido

        self.seed_list = [[0.09357282806101024, 1.6662866308405306, 0.6224127046845066, -1.1846166634445108,
                           -0.13614079492483047, -0.438335010138257, 0.11888351106111957],
                          [0.3390097541226764, 1.4733885467639398, 0.16988837225830958, -1.3598739684604193,
                           -1.3675438723998463, -0.2703641138648042, 1.2440584189750705],
                          [0.2592427531526349, 1.3123205640359714, 0.3796602450016399, -1.2950632801722606,
                           -1.6988837225830957, -0.17487380981893716, 1.6072283705069423]]


        self.joint_names = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']

        '''
        # Joint seeds for obstacle
        self.seed_list = [[-0.49931074645670215, 1.9239954032052802, 0.795369038518587, -0.48857288094150425, -0.854043803655204, -0.8107088463974411, -2.3726847836617635],
                          [-0.21283983431910117, 1.4795244699154815, 0.8770535154734853, -0.09127185687918211, 0.03298058693953639, -1.236005019838672, -2.9594324350279346],
                          [-0.1679708962734528, 1.3077186216723151, 0.6220292094875353, 0.07669903939427068, -0.26959712347086146, -1.4112623248545806, -2.8823499004366924],
                          [-0.16643691548556738, 1.087208883413787, 0.9242234247009617, 0.19136410328870537, 0.2044029399857314, -1.2068593848688494, -3.0495538063162027]]
        '''

    def ik_request(self, pose, seed_list=None, _joint_names=None):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))

        if seed_list is not None:
            for seed_angles in seed_list:

                seed = JointState()
                seed.name = _joint_names
                seed.position = seed_angles
                seed.header = Header(stamp=rospy.Time.now(), frame_id='base')

                ikreq.seed_angles.append(seed)

        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    # Process the trajectory to feed it to the DMPs
    def cartesian_list_to_joints(self, _points, _use_initial_seed=False):
        _traj = []
        _times = []
        first_iter = True

        # Call the IK Solver for all points
        for _point in _points:

            pose = Pose(position=Point(x=_point[1], y=_point[2], z=_point[3]),
                        orientation=Quaternion(x=_point[4], y=_point[5], z=_point[6], w=_point[7]))


            if first_iter:
                if _use_initial_seed:
                    _angles_dict = self.ik_request(pose, self.seed_list, self.joint_names)

                else:
                    _angles_dict = self.ik_request(pose)

                first_iter = False
                self.initial_angles = _angles_dict.values()

            else:
                _angles_dict = self.ik_request(pose, [traj_k], _joint_names)

            traj_k = _angles_dict.values()
            _joint_names = _angles_dict.keys()
            _traj.append(traj_k)
            _times.append(_point[0])
        self.final_angles = traj_k
        self.final_joint_names = _joint_names
        return _traj, _joint_names, _times

    '''
    Process one cartesian position and convert it to axial
    
    mode 0 -> Use initial manual seed
    mode 1 -> Use initial angles
    mode 2 -> Use final angles
    otherwise -> Don't use any seed
    '''
    def cartesian_to_joints(self, pose, mode=None):

        if mode == 0:
            _angles_dict = self.ik_request(pose, self.seed_list, self.joint_names)
        elif mode == 1:
            _angles_dict = self.ik_request(pose, [self.initial_angles], self.final_joint_names)
        elif mode == 2:
            _angles_dict = self.ik_request(pose, [self.final_angles], self.final_joint_names)
        else:
            _angles_dict = self.ik_request(pose)

        _joint_names = _angles_dict.keys()
        axial_point = _angles_dict.values()

        return axial_point, _joint_names


# My class for doing de DMPs
class DynamicMovementPrimitives:

    def __init__(self, _dims, _dt, _K_gain, _D_gain, _num_bases, _hi, _alpha):
        self.dims = _dims
        self.dt = _dt
        self.K_gain = _K_gain
        self.D_gain = _D_gain
        self.num_bases = _num_bases
        self.hi = _hi
        self.alpha = _alpha

    def fit(self, _trajectory):
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
            demotraj.times.append(dt * i)

        k_gains = [K_gain] * dims
        d_gains = [D_gain] * dims

        # print "Starting LfD..."
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj, k_gains, d_gains, num_bases, _hi, _alpha)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
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
        print "Starting DMP planning..."
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
            resp = gdp(_x_0, _x_dot_0, _t_0, _goal, _goal_thresh, _seg_length, _tau, _dt, _integrate_iter)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "DMP planning done"   
                
        return resp


# Trajectory client object
class Trajectory(object):
    def __init__(self, _limb, _joint_names):
        ns = 'robot/limb/' + _limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(_joint_names)

    def add_point(self, positions, time=None):
        _point = JointTrajectoryPoint()
        _point.positions = copy(positions)
        # point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(_point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, _joint_names):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        # self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
        #                                     ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
        self._goal.trajectory.joint_names = _joint_names


# Read the cartesian points from csv file
def read_trajectory_points(_input_file, _fix_orientation):
    print('')
    print('#### Start reading cartesian points in ', input_file, '...')
    # fixed_orientation = [x, y, z, w]
    _fixed_orientation = [-0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011]
    with open(_input_file) as csvfile:
        _cartesian_reader = csv.reader(csvfile, delimiter=',')
        _points_list = list()
        for row in _cartesian_reader:
            _point = [float(i) for i in row]

            if _fix_orientation:
                _point = _point[0:4]
                _point.extend(_fixed_orientation)

            _points_list.append(_point)

    return _points_list


'''
Main
'''
if __name__ == '__main__':

    # Read the console arguments
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)

    # Select angles or cartesian points
    arg_group = parser.add_mutually_exclusive_group(required=True)
    arg_group.add_argument("-a", "--angles", dest="angles",
        action='store_true', default=False, help="set if training with angles (pre-conversion)")
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
    # Output file
    required.add_argument(
        '-oa', '--output-file-angles', dest='output_file_angles', required=False,
        help='the file name to record to'
    )
    required.add_argument("-f", action="store_true", dest="fix", help="define if you want to fix the orientation")
    args = parser.parse_args(rospy.myargv()[1:])

    input_file = args.input_file
    angles = args.angles

    # Start a node
    rospy.init_node('baxter_dmp')

    # Enable the robot
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()


    # Read the trajectory
    traj = read_trajectory_points(input_file, args.fix)

    # Create DMP class with initial parameters
    dims = 7  # Number of dimensions
    dt = 0.5  # Time resolution of the plan
    K = 100  # List of proportional gains
    D = np.sqrt(K)  # D_gains
    num_bases = 13  # Number of basis functions to use
    hi = 10
    alpha = -math.log(0.000005, 10)
    alpha = 6.907755278982137

    DMPs = DynamicMovementPrimitives(dims, dt, K, D, num_bases, hi, alpha)

    # Start inverse kinematics system
    side = "right"
    kin = IKSolver(side, False)

    # Make previous conversion to angles if requested:
    if angles:
        print('Converting to angles before DMP...')
        use_initial_seed = True
        traj, joint_names, times = kin.cartesian_list_to_joints(traj, use_initial_seed)

    #print traj

    if args.output_file_angles and angles:
        # Save plan if requested
        header_list = ['time']
        header_list.extend(joint_names)
        header = ','.join(header_list)

        f = open(args.output_file_angles, 'w')
        f.write(header + '\n')

        for (point, timestamp) in zip(traj, times):
            line = [timestamp]
            line.extend(point)
            line = ','.join(map(str, line))
            f.write(line + '\n')

        f.close()

    # Train the DMPs
    resp = DMPs.fit(traj)

    # POSITIONS FOR MANOARRIBA
    # Make the query with the new initial position and goal

    # Set initial position
    # x_0_position = [1.057178400000, -0.372620000000, 0.500260180000]
    x_0_position = [1.056356046628,-0.50,0.60]
    x_0_orientation = [0.679558241454,0.126662326600,0.713324911380,0.115433194086]
    x_dot_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    t_0 = 0

    # Set goal
    #goal_position = [1.023647346900,-0.134444726073,0.233769178968]
    goal_position = [1.023647346900,0.10,0.16]
    goal_orientation = [0.601120337846,0.596000430247,0.465588801541,0.258195457848]
    goal_orientation = [ 0.347545234546, 0.721356382885, 0.352070247993, 0.484668772764 ]




    '''
    # Make the query with the new initial position and goal

    x_0_position = [1.056356046628,-0.350334254965,0.498158707178]
    x_0_orientation = [0.679558241454,0.126662326600,0.713324911380,0.115433194086]
    x_dot_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    t_0 = 0

    goal_position = [0.995756006189,0.05,0.05]
    goal_orientation = [0.790470908830,0.335521723000,0.501183212652,0.106753468534]
    '''

    if angles:

        # Call the IK Solver for initial and final positions
        x_0_pose = Pose(position=Point(x=x_0_position[0], y=x_0_position[1], z=x_0_position[2]),
                        orientation=Quaternion(x=x_0_orientation[0], y=x_0_orientation[1], z=x_0_orientation[2],
                                               w=x_0_orientation[3]))
        x_0, _ = kin.cartesian_to_joints(x_0_pose, 0)
        #x_0 = traj[0]

        goal_pose = Pose(position=Point(x=goal_position[0], y=goal_position[1], z=goal_position[2]),
                         orientation=Quaternion(x=goal_orientation[0], y=goal_orientation[1], z=goal_orientation[2],
                                                w=goal_orientation[3]))

        goal, _ = kin.cartesian_to_joints(goal_pose, 2)

        #goal = traj[-1]

        # Threshold in each dimension

        goal_thresh = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]


    else:
        print('Using cartesian points for DMP...')
        x_0_position.extend(x_0_orientation)
        x_0 = x_0_position             

        goal_position.extend(goal_orientation)
        goal = goal_position

        # Threshold in each dimension
        goal_thresh = [0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03]

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

    # If the plan has been learnt in cartesian, then convert to angles
    if not angles:
        plan_list, joint_names = kin.cartesian_list_to_joints(plan_list, True)

    plan_list_short = plan_list[:len(traj)]

    error = 0
    for source, target in zip(traj, plan_list_short):
        source = np.array(source)
        target = np.array(target)

        error += np.linalg.norm(source - target)

    print('MSE', error)

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

    if args.output_file:
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


