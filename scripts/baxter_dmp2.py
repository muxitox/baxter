#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *
import csv
import argparse
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


class IKSolver:

    def __init__(self, limb,  verbose=True):
        self._verbose = verbose
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
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
    fixed_orientation = [-0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011]
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
    kin = IKSolver(side, False)

    #Now, generate a plan

    # Set initial position and goal
    x_0_position = [0.1442207,-0.4007651, 0.2935641]
    x_0_orientation = [-0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011]
    t_0 = 0  

    goal_position = [0.2755853, 0.1018015, 0.2359535]
    goal_orientation = [-0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011]

    # Make previous conversion to angles if requested:
    if angles:

        print('Converting to angles before DMP...')
        seed_0 = [0.07999999960926552, -0.9999845805105894, -0.669996713128465, 1.030008743719245, 0.4999996890795071, 1.1899720112608447, 1.9400294781400014]
        #['right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0', 'right_e1']

        traj = []
        first_iter = True
        seed = JointState()

        # Call the IK Solver for all points
        for point in points:

            pose = Pose(
                        position=Point(x=point[0], y=point[1],z=point[2]),
                        orientation=Quaternion(x=point[3], y=point[4], z=point[5], w=point[6]))

            # TODO: INTRODUCE AN INITIAL SEED TO CHANGE THE FORM OF THE MOVEMENT
            if first_iter:
                angles_dict = kin.ik_request(pose)

                joint_names = angles_dict.keys()
                seed.name = joint_names
                traj_k = angles_dict.values()
                seed.position = traj_k

                angles_dict = kin.ik_request(pose, seed)


                first_iter = False
            else:
                
                seed.position = traj_k
                seed.header = Header(stamp=rospy.Time.now(), frame_id='base')

                angles_dict = kin.ik_request(pose, seed)

                traj_k = angles_dict.values()

            traj.append(traj_k)


        # Call the IK Solver for initial and final positions
        x_0_pose = Pose(
                        position=Point(x=x_0_position[0], y=x_0_position[1],z=x_0_position[2]),
                        orientation=Quaternion(x=x_0_orientation[0], y=x_0_orientation[1], z=x_0_orientation[2], w=x_0_orientation[3]))

        x_0_dict = kin.ik_request(x_0_pose)
        x_0 = x_0_dict.values()

        goal_pose = Pose(
                        position=Point(x=goal_position[0], y=goal_position[1],z=goal_position[2]),
                        orientation=Quaternion(x=goal_orientation[0], y=goal_orientation[1], z=goal_orientation[2], w=x_0_orientation[3]))

        goal_dict = kin.ik_request(goal_pose)
        goal = goal_dict.values()

    else:
        print('Using cartesian points for DMP...')
        x_0_position.extend(x_0_orientation)
        x_0 = x_0_position             

        goal_position.extend(goal_orientation)
        goal = goal_position

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
    tau = 2* resp.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)

    ###
    # Make posterior conversion to angles if needed
    ###

    plan_list = []
    first_iter = True

    seed = JointState()

    for point in plan.plan.points:
        position = list(point.positions)

        # If trained with cartesian points, convert them to angles, o.w. it's already in angles
        if cartesian:
            pose = Pose(
                        position=Point(x=position[0], y=position[1],z=position[2]),
                        orientation=Quaternion(x=position[3], y=position[4], z=position[5], w=position[6]))


            if first_iter:

                angles_dict = kin.ik_request(pose)

                joint_names = angles_dict.keys()
                seed.name = joint_names
                converted = angles_dict.values()
                
                first_iter = False


            else:
                seed.position = converted
                seed.header = Header(stamp=rospy.Time.now(), frame_id='base')

                angles_dict = kin.ik_request(pose, seed)

                converted = angles_dict.values()

            position = converted
            

        plan_list.append(position)

    limb = baxter_interface.Limb(side)
    #gripper = baxter_interface.Gripper(side)

    # joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']


    for position in plan_list:
        angles_dict = dict(zip(joint_names, position))
        limb.move_to_joint_positions(angles_dict)
    

    '''
    # Save the plan if requested
    if args.output_file is not None:
        print('Saving plan...')
    '''
