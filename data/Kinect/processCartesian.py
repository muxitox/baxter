#!/usr/bin/python

import csv
import numpy as np
import math


def main():
    
    first_iter = True

    input_file = 'Pre/23-02-2020-18-2-30-NEW_MANOALTA.csv'
    output_file = 'Post/NEWMANOALTA.csv'


    # Period to create groups of vectors to process the records
    period = 0.5
    time_processed = 0

    pose_list = []
    pose_matrix = []

    first_iter = True

    print('')
    print('#### Start reading endpoints in ', input_file, '...')
    # with open('/home/angelp/catkin_ws/src/dmp/data/baxter_endpoints.csv') as csvfile:
    with open(input_file) as csvfile1:
        cartesian_reader = csv.reader(csvfile1, delimiter=',')
        for row in cartesian_reader:
            # Process the line, convert to float
            line = [float(i) for i in row]
            # Offsets
            line[1] += 0.6
            line[4] += 0.6
            line[7] += 0.6
            line[2] -= 0.05
            line[5] -= 0.05
            line[8] -= 0.05

            # Create the vectors
            # [position, vectorHW, vector WT]
            wh_vector = np.subtract(line[4:7], line[1:4])
            wt_vector = np.subtract(line[7:10], line[1:4])
            time = line[0]

            if first_iter:
                initial_time = time
                first_iter = False

            time -= initial_time

            # Calculate the angle between the vectors to filter
            cos_alpha = np.dot(wh_vector, wt_vector) / (np.linalg.norm(wh_vector) * np.linalg.norm(wt_vector))

            # If cos of the angle between both vectors is approx 1 -> discard this record
            if cos_alpha > 0.95:
                continue

            # Append to create a new vector
            record = line[1:4] + list(wh_vector) + list(wt_vector)

            if time < period+time_processed:
                pose_list.append(record)
            else:
                pose_matrix.append(pose_list)
                pose_list = [record]
                time_processed += period

        if pose_list is not []:
            pose_matrix.append(pose_list)

    median_list = []
    for pose_list in pose_matrix:
        pose_array = np.array(pose_list)
        #median = pose_list[0]
        median = np.median(pose_array, 0)

        # Create the vectors for defining hand orientation as [n o a] notation
        wh_vector = median[3:6]
        wh_norm = np.linalg.norm(wh_vector)
        wt_vector = median[6:9]
        # wt_norm = np.linalg.norm(wt_vector)

        # Vector a, in direction of the hand
        a = np.divide(wh_vector, wh_norm)

        # Normal vector between a and wt
        n_aux_vector = np.cross(wt_vector, a)
        n_aux_norm = np.linalg.norm(n_aux_vector)
        n = np.divide(n_aux_vector, n_aux_norm)
        # Make sure it is a unitary vector
        n = np.divide(n, np.linalg.norm(n))

        # Vector a, defining the rotation angle
        o = np.cross(a, n)
        # Make sure it is unitary
        o = np.divide(o, np.linalg.norm(o))

        print n, o, a
        # Calculate the quaternion from the matrix, according to Robotics Book
        w = math.sqrt(n[0]+o[1]+a[2]+1)/2
        x = math.sqrt(n[0]-o[1]-a[2]+1)/2
        y = math.sqrt(-n[0]+o[1]-a[2]+1)/2
        z = math.sqrt(-n[0]-o[1]+a[2]+1)/2

        final_pose = list(median[0:3])
        final_pose.extend([x, y, z, w])
        # Append everything
        median_list.append(final_pose)

    output_array = np.array(median_list)
    np.savetxt(output_file, output_array, fmt='%f', delimiter=",")

    print('#### Finished processing file in ', output_file, '...')


if __name__ == "__main__":
    main()
