#!/usr/bin/python

# Copyright (c) 2013-2014, Rethink Robotics
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

import rospy
import csv
from baxter_pykdl import baxter_kinematics


def main():
    rospy.init_node('baxter_kinematics')
    kin = baxter_kinematics('right')
    
    angles = list()
    first_iter = True

    input_file = '../data/baxter_endpoints.csv'
    output_file = '../data/baxter_angles.csv'

    print('')
    print('#### Start reading endpoints in ', input_file, '...')
    # with open('/home/angelp/catkin_ws/src/dmp/data/baxter_endpoints.csv') as csvfile:
    with open(input_file) as csvfile1:
        with open(output_file, 'w') as csvfile2:

            cartesian_reader = csv.reader(csvfile1)
            angles_writer = csv.writer(csvfile2, delimiter=',')
            for row in cartesian_reader:

                r = [float(i) for i in row]
                position = r[:3]
                orientation = r[3:]
                if first_iter:
                    angles = kin.inverse_kinematics(position, orientation)
                    angles_writer.writerow(angles)
                    first_iter = False
                else:
                    angles = kin.inverse_kinematics(position, orientation, list(angles))
                    angles_writer.writerow(angles)

    print('#### Finished transforming angles in ', output_file ,'...')






if __name__ == "__main__":
    main()
