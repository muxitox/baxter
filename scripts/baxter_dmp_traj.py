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


'''
Inverse Kinematics Solver (With manual seeds)
'''
class IKSolver:

    def __init__(self, _limb,  _verbose=False):
        self._verbose = _verbose
        ns = "ExternalTools/" + _limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)

    def ik_request(self, pose, seed=None):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))

        if seed is not None:
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
    def cartesian_to_joints(self, _points, _use_initial_seed=False):
        traj = []
        first_iter = True
        seed = JointState()

        # Call the IK Solver for all points
        for point in _points:

            pose = Pose(position=Point(x=point[0], y=point[1],z=point[2]),
                        orientation=Quaternion(x=point[3], y=point[4], z=point[5], w=point[6]))

            if first_iter:
                # Make initial query just to know the joint names
                _angles_dict = self.ik_request(pose)
                _joint_names = _angles_dict.keys()
                seed.name = _joint_names

                if _use_initial_seed:
                    # ['right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0', 'right_e1']
                    seed_0 = [0.07999999960926552, -0.9999845805105894, -0.669996713128465, 1.030008743719245,
                              0.4999996890795071, 1.1899720112608447, 1.9400294781400014]

                    seed.position = seed_0
                    seed.header = Header(stamp=rospy.Time.now(), frame_id='base')
                    _angles_dict = self.ik_request(pose, seed)

                else:
                    _angles_dict = self.ik_request(pose)

                first_iter = False

            else:
                seed.position = traj_k
                seed.header = Header(stamp=rospy.Time.now(), frame_id='base')
                _angles_dict = self.ik_request(pose, seed)

            traj_k = _angles_dict.values()
            traj.append(traj_k)

        return traj, _joint_names

# My class for doing de DMPs
class DynamicMovementPrimitives:

    def __init__(self, _dims, _dt, _K_gain, _D_gain, _num_bases):
        self.dims = _dims
        self.dt = _dt
        self.K_gain = _K_gain
        self.D_gain = _D_gain
        self.num_bases = _num_bases

    def fit(self,  _trajectory):
        self.trajectory = _trajectory

        self.resp = self.makeLFDRequest(self.dims, self.trajectory, self.dt, self.K_gain,
                                        self.D_gain, self.num_bases)

        self.makeSetActiveRequest(self.resp.dmp_list)

        return self.resp

    # Learn a DMP from demonstration data
    def makeLFDRequest(self, dims, traj, dt, K_gain, D_gain, num_bases):
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
                _point = _point[0:3]
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

    # Move left arm to a position where it does not trouble
    limb = baxter_interface.Limb("left")
    joint_names = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    position = [-0.005368932757598948, 2.6173547193294873, 2.3531265286162246, -1.6965827514012677, -2.1456556270547225,
                0.11313108310654926, 0.08858739050038264]
    angles_dict = dict(zip(joint_names, position))
    limb.move_to_joint_positions(angles_dict)

    # Read the trajectory
    traj = read_trajectory_points(input_file, args.fix)

    # Create DMP class with initial parameters
    dims = 7                # Number of dimensions
    dt = 1.0                # Time resolution of the plan
    K = 100                 # List of proportional gains
    D = 2.0 * np.sqrt(K)    # D_gains
    num_bases = 200           # Number of basis functions to use

    DMPs = DynamicMovementPrimitives(dims, dt, K, D, num_bases)

    # Start inverse kinematics system
    side = "right"
    kin = IKSolver(side, False)

    # Make previous conversion to angles if requested:
    if angles:
        print('Converting to angles before DMP...')
        use_initial_seed = True
        traj, joint_names = kin.cartesian_to_joints(traj, use_initial_seed)

    # Train the DMPs
    resp = DMPs.fit(traj)

    # Make the query with the new initial position and goal

    # Set initial position
    x_0_position = [0.1442207, -0.4007651, 0.2935641]
    x_0_orientation = [-0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011]
    x_dot_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    t_0 = 0

    # Set goal
    goal_position = [0.2755853, 0.1018015, 0.2359535]
    goal_orientation = [-0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011]

    if angles:

        # Call the IK Solver for initial and final positions
        x_0_pose = Pose(position=Point(x=x_0_position[0], y=x_0_position[1], z=x_0_position[2]),
                        orientation=Quaternion(x=x_0_orientation[0], y=x_0_orientation[1], z=x_0_orientation[2],
                                               w=x_0_orientation[3]))
        x_0_dict = kin.ik_request(x_0_pose)
        x_0 = x_0_dict.values()

        goal_pose = Pose(position=Point(x=goal_position[0], y=goal_position[1], z=goal_position[2]),
                         orientation=Quaternion(x=goal_orientation[0], y=goal_orientation[1], z=goal_orientation[2],
                                                w=x_0_orientation[3]))
        goal_dict = kin.ik_request(goal_pose)
        goal = goal_dict.values()

        # Threshold in each dimension
        goal_thresh = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    else:
        print('Using cartesian points for DMP...')
        x_0_position.extend(x_0_orientation)
        x_0 = x_0_position             

        goal_position.extend(goal_orientation)
        goal = goal_position

        # Threshold in each dimension
        goal_thresh = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    seg_length = -1          # Plan until convergence to goal
    # tau = 2 * resp.tau      # Desired plan should take twice as long as demo
    tau = 2 * resp.tau       # Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       # dt is rather large, so this is > 1
    plan = DMPs.predict(x_0, x_dot_0, t_0, goal, goal_thresh, seg_length, tau, dt, integrate_iter)

    # Extract the plan as a list of points
    plan_list = []
    for point in plan.plan.points:
        position = list(point.positions)
        plan_list.append(position)

    # If the plan has been learnt in cartesian, then convert to angles
    if not angles:
        plan_list, joint_names = kin.cartesian_to_joints(plan_list)

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

    # gripper = baxter_interface.Gripper(side)
    # joint_names = ['right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0', 'right_e1']

    # Move arm to the initial position of the trajectory
    # first_position = plan_list[0]
    # angles_dict = dict(zip(joint_names, first_position))
    # limb.move_to_joint_positions(angles_dict)

    # time = 0

    # traj = Trajectory(side, joint_names)
    # rospy.on_shutdown(traj.stop)
    # for position in plan_list:
    #    traj.add_point(position, time)
    #    time += 1

    # traj.start()
    # traj.wait(20)
    # print('Test completed')
    
    '''

    # TODO: Save plan if requested

    header_list = ['time']
    header_list.extend(joint_names)
    header = ','.join(header_list)

    f = open(args.output_file, 'w')
    f.write(header+'\n')

    timestamp = 0
    time_increment = 0.5
    for point in plan_list:
        line = [timestamp]
        line.extend(point)
        line = ','.join(map(str, line))
        f.write(line+'\n')
        timestamp += 0.5
    f.close()


