#!/usr/bin/python

import csv
import numpy as np
import math
from scipy.ndimage.filters import gaussian_filter1d
import matplotlib.pyplot as plt


def main():
    
    first_iter = True

    '''
    input_file = 'Pre/23-02-2020-18-2-30-NEW_MANOALTA_BUENO.csv'
    output_file = 'Post/NEWMANOALTA_MODIFY_YZ.csv'
    '''

    '''
    input_file = 'Pre/23-02-2020-18-11-37-NEW_OBSTACULOBAJO.csv'
    output_file = 'Post/OBSTACULOBAJO.csv'
    '''

    input_file = 'Pre/23-02-2020-18-2-30-NEW_MANOALTA_BUENO.csv'
    output_file = 'Post/pruebas.csv'


    # Period to create groups of vectors to process the records
    period = 0.5
    time_processed = 0

    pose_list = []
    pose_matrix = []

    first_iter = True

    discarded_number_list = []
    discarded_number_total = 0
    discarded_number_period = 0
    num_elem_period = 0
    num_elem_total = 0


    print('')
    print('#### Start reading endpoints in ', input_file, '...')
    # with open('/home/angelp/catkin_ws/src/dmp/data/baxter_endpoints.csv') as csvfile:
    with open(input_file) as csvfile1:
        cartesian_reader = csv.reader(csvfile1, delimiter=',')
        for row in cartesian_reader:
            # Process the line, convert to float
            line = [float(i) for i in row]

            # Create the vectors
            # [position, vectorHW, vector WT]
            wh_vector = np.subtract(line[4:7], line[1:4])
            wt_vector = np.subtract(line[7:10], line[1:4])
            time = line[0]

            if first_iter:
                initial_time = time
                first_iter = False

            num_elem_period += 1
            num_elem_total += 1

            time -= initial_time

            # Calculate the angle between the vectors to filter
            cos_alpha = np.dot(wh_vector, wt_vector) / (np.linalg.norm(wh_vector) * np.linalg.norm(wt_vector))

            # If cos of the angle between both vectors is approx 1 -> discard this record
            if cos_alpha > 0.95:
                discarded_number_period += 1
                discarded_number_total += 1
                continue


            # Append to create a new vector
            record = [time] + line[1:4] + list(wh_vector) + list(wt_vector)

            if time < period+time_processed:
                pose_list.append(record)
            else:
                pose_matrix.append(pose_list)
                pose_list = [record]
                time_processed += period


                discarded_number_list.append(float(discarded_number_period)/float(num_elem_period))
                discarded_number_period = 0
                num_elem_period = 0

        if pose_list is not []:
            pose_matrix.append(pose_list)
            discarded_number_list.append(float(discarded_number_period) / float(num_elem_period+1))

        plt.style.use('ggplot')
        plt.figure(1)
        plt.hist(discarded_number_list, 10)
        plt.xlabel('Proportion of discarded records every period')
        plt.ylabel('Frequency')
        #plt.show()
        print(float(discarded_number_total)/float(num_elem_total))

    xw_diff_list = []
    yw_diff_list = []
    zw_diff_list = []
    xh_diff_list = []
    yh_diff_list = []
    zh_diff_list = []
    xt_diff_list = []
    yt_diff_list = []
    zt_diff_list = []

    median_list = []
    std_list = []
    for pose_list in pose_matrix:
        pose_array = np.array(pose_list)
        # median = pose_list[0]
        median = np.median(pose_array, 0)

        std = np.std(pose_array[:,1:], 0)

        std_list.append(std)

        print(pose_array[0,1:])

        xw_diff = (max(pose_array[:, 1]) - min(pose_array[:, 1]))
        yw_diff = (max(pose_array[:, 2]) - min(pose_array[:, 2]))
        zw_diff = (max(pose_array[:, 3]) - min(pose_array[:, 3]))

        xh_diff = (max(pose_array[:, 4]) - min(pose_array[:, 4]))
        yh_diff = (max(pose_array[:, 5]) - min(pose_array[:, 5]))
        zh_diff = (max(pose_array[:, 6]) - min(pose_array[:, 6]))

        xt_diff = (max(pose_array[:, 7]) - min(pose_array[:, 7]))
        yt_diff = (max(pose_array[:, 8]) - min(pose_array[:, 8]))
        zt_diff = (max(pose_array[:, 9]) - min(pose_array[:, 9]))

        xw_diff_list.append(xw_diff)
        yw_diff_list.append(yw_diff)
        zw_diff_list.append(zw_diff)
        xh_diff_list.append(xh_diff)
        yh_diff_list.append(yh_diff)
        zh_diff_list.append(zh_diff)
        xt_diff_list.append(xt_diff)
        yt_diff_list.append(yt_diff)
        zt_diff_list.append(zt_diff)

        # Create the vectors for defining hand orientation as [n o a] notation
        wh_vector = median[4:7]
        wh_norm = np.linalg.norm(wh_vector)
        wt_vector = median[7:10]
        # wt_norm = np.linalg.norm(wt_vector)

        # Vector a, in direction of the hand
        a = np.divide(wh_vector, wh_norm)

        # Normal vector between a and wt
        n_aux_vector = np.cross(a, wt_vector)
        n_aux_norm = np.linalg.norm(n_aux_vector)
        n = np.divide(n_aux_vector, n_aux_norm)
        # Make sure it is a unitary vector
        n = np.divide(n, np.linalg.norm(n))

        # Vector a, defining the rotation angle
        o = np.cross(a, n)
        # Make sure it is unitary
        o = np.divide(o, np.linalg.norm(o))

        # Calculate the quaternion from the matrix, according to Robotics Book
        w = math.sqrt(n[0]+o[1]+a[2]+1)/2
        x = math.sqrt(n[0]-o[1]-a[2]+1)/2
        y = math.sqrt(-n[0]+o[1]-a[2]+1)/2
        z = math.sqrt(-n[0]-o[1]+a[2]+1)/2

        median[1] += 0.6
        median[2] = (median[2]*1.8)+0.20
        median[3] = (median[3]*1.8)-0.32

        '''
        median[1] += 0.64
        median[2] = (median[2]*1.8)
        median[3] = (median[3]*1.8) - 0.45
        '''

        final_pose = [pose_list[0][0]] + list(median[1:4])
        final_pose.extend([x, y, z, w])
        # Append everything
        median_list.append(final_pose)

    plt.figure(2)
    plt.subplot(3, 3, 1)
    plt.hist(xw_diff_list, 5)
    plt.ylabel('Frequency')
    plt.title('(a) Wrist X')

    plt.subplot(3, 3, 2)
    plt.hist(yw_diff_list, 5)
    plt.title('(b) Wrist Y')

    plt.subplot(3, 3, 3)
    plt.hist(zw_diff_list, 5)
    plt.title('(c) Wrist Z')

    plt.subplot(3, 3, 4)
    plt.hist(xh_diff_list, 5)
    plt.ylabel('Frequency')
    plt.title('(d) Hand X')

    plt.subplot(3, 3, 5)
    plt.hist(yh_diff_list, 5)
    plt.title('(e) Hand Y')

    plt.subplot(3, 3, 6)
    plt.hist(zh_diff_list, 5)
    plt.title('(f) Hand Z')

    plt.subplot(3, 3, 7)
    plt.hist(xt_diff_list, 5)
    plt.ylabel('Frequency')
    plt.title('(g) Thumb X')

    plt.subplot(3, 3, 8)
    plt.hist(yt_diff_list, 5)
    plt.title('(h) Thumb Y')

    plt.subplot(3, 3, 9)
    plt.hist(zt_diff_list, 5)
    plt.title('(i) Thumb Z')

    plt.suptitle('Max Min Difference every 0.5 s')

    plt.show()

    std_array = np.array(std_list)

    print(std_array.shape)

    plt.figure(3)
    plt.subplot(3, 3, 1)
    plt.hist(std_array[:,0], 5)
    plt.ylabel('Frequency')
    plt.title('(a) Wrist X')

    plt.subplot(3, 3, 2)
    plt.hist(std_array[:,1], 5)
    plt.title('(b) Wrist Y')

    plt.subplot(3, 3, 3)
    plt.hist(std_array[:,1], 5)
    plt.title('(c) Wrist Z')

    plt.subplot(3, 3, 4)
    plt.hist(std_array[:,3], 5)
    plt.ylabel('Frequency')
    plt.title('(d) Hand X')

    plt.subplot(3, 3, 5)
    plt.hist(std_array[:,4], 5)
    plt.title('(e) Hand Y')

    plt.subplot(3, 3, 6)
    plt.hist(std_array[:,5], 5)
    plt.title('(f) Hand Z')

    plt.subplot(3, 3, 7)
    plt.hist(std_array[:,6], 5)
    plt.ylabel('Frequency')
    plt.title('(g) Thumb X')

    plt.subplot(3, 3, 8)
    plt.hist(std_array[:,7], 5)
    plt.title('(h) Thumb Y')

    plt.subplot(3, 3, 9)
    plt.hist(std_array[:,8], 5)
    plt.title('(i) Thumb Z')

    plt.suptitle('Standard Deviation every 0.5 s')

    plt.show()


    output_array = np.array(median_list)

    plt.figure(4)
    plt.subplot(4, 2, 1)
    plt.plot(output_array[:, 0], output_array[:, 1])
    plt.xlabel('Time [s]')
    plt.title('(a) Position X')

    plt.subplot(4, 2, 3)
    plt.plot(output_array[:, 0], output_array[:, 2])
    plt.xlabel('Time [s]')
    plt.title('(b) Position Y')

    plt.subplot(4, 2, 5)
    plt.plot(output_array[:, 0], output_array[:, 3])
    plt.xlabel('Time [s]')
    plt.title('(c) Position Z')

    plt.subplot(4, 2, 2)
    plt.plot(output_array[:, 0], output_array[:, 4])
    plt.xlabel('Time [s]')
    plt.title('(d) Orientation X')

    plt.subplot(4, 2, 4)
    plt.plot(output_array[:, 0], output_array[:, 5])
    plt.xlabel('Time [s]')
    plt.title('(d) Orientation Y')

    plt.subplot(4, 2, 6)
    plt.plot(output_array[:, 0], output_array[:, 6])
    plt.xlabel('Time [s]')
    plt.title('(f) Orientation Z')

    plt.subplot(4, 2, 8)
    plt.plot(output_array[:, 0], output_array[:, 7])
    plt.xlabel('Time [s]')
    plt.title('(g) Orientation W')

    plt.suptitle('Endpoint Pose')

    plt.show()


    output_array[:, 1] = gaussian_filter1d(output_array[:, 1], 2)
    output_array[:, 2] = gaussian_filter1d(output_array[:, 2], 2)
    output_array[:, 3] = gaussian_filter1d(output_array[:, 3], 2)

    np.savetxt(output_file, output_array, fmt='%.12f', delimiter=",")

    print('#### Finished processing file in ', output_file, '...')

if __name__ == "__main__":
    main()
