#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse

import rospy

import baxter_interface
from baxter_examples import JointRecorder

from baxter_interface import CHECK_VERSION




def main():
    # Prepare the recorder

    epilog = """
Related examples:
  joint_position_file_playback.py; joint_trajectory_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file', dest='filename', required=True,
        help='the file name to record to'
    )
    parser.add_argument(
        '-r', '--record-rate', type=int, default=10, metavar='RECORDRATE',
        help='rate at which to record (default: 100)'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_record_joints")
    #print("Getting robot state... ")
    #rs = baxter_interface.RobotEnable(CHECK_VERSION)
    #print("Enabling robot... ")
    #rs.enable()

    print("Create Recorder")
    recorder = JointRecorder(args.filename, args.record_rate)
    print("Create thread")
    recorder_thread = threading.Thread(target=recorder.record())
    print("Launch thread")
    rospy.sleep(1.0)
    recorder_thread.start()

    # Recorder ready
    # Move the joints

    angles_0 = [0.0007631914876728985, -2.7617363369540485e-07, -0.020833228167125926, -1.1879388330468545, 1.941939444992581, -0.07999999263457713, -0.9976007838930174, 0.6697204397280547, 1.0299254770209938, -0.5000007755219995, 0.0001359207064124563, 4.542713971417686e-06, 0.8764725942919336, 1.5562521109309468, -1.7016798893475187, -1.4716329185751462, -0.3670479393311279, 0.3030074113979131, -0.20293509149000855]
    angles_1 = [0.0009464061500512599, -2.213174724695102e-07, -0.020833182551364657, -1.1879388290637056, 1.9419394411801107, -0.0800000065987323, -0.99760078218275, 0.669720430125615, 1.029925473985287, -0.5000007670930158, -6.474204016433322e-06, -0.0002432110695115592, 1.7594962249613975, 1.8523185500652843, -1.0038370415321767, 0.07553092519912763, -0.36832549045606644, 0.3014800928082826, -0.2029696812809494]

    print("Move to first position")
    angles_dict = dict(zip(limb.joint_names(), angles_0)) 
    limb.move_to_joint_positions(angles_dict)

    angles_dict = dict(zip(limb.joint_names(), angles_1)) 
    limb.move_to_joint_positions(angles_dict)
    print("Finish")

    # Stop the recorder at the end
    recorder.stop()


    print("\nDone.")

if __name__ == '__main__':
    main()
