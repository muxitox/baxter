#!/usr/bin/python

import csv
import numpy as np
import math


def main():
    
    first_iter = True

    input_file = 'Pre/dmptraj1.csv'
    output_file = 'Post/dmptraj1.csv'


    # Period to create groups of vectors to process the records
    period = 0.5
    time_processed = 0

    joint_list = []
    joint_matrix = []

    first_iter = True
    header = True

    print('')
    print('#### Start reading endpoints in ', input_file, '...')
    # with open('/home/angelp/catkin_ws/src/dmp/data/baxter_endpoints.csv') as csvfile:
    with open(input_file) as csvfile1:
        axial_reader = csv.reader(csvfile1, delimiter=',')
        for row in axial_reader:
            # Process the line, convert to float
            if header:
                line = row
                joint_names = line[-8:-1]
                header = False

            else:
                line = [float(i) for i in row]

                positions = line[-8:-1]
                time = line[0]

                record = [line[0]]
                record.extend(positions)



                if first_iter:
                    initial_time = time
                    first_iter = False

                time -= initial_time



                if time < period+time_processed:
                    joint_list.append(record)
                else:
                    joint_matrix.append(joint_list)
                    joint_list = [record]
                    time_processed += period

        if joint_list is not []:
            joint_matrix.append(joint_list)

    records_list = []
    for joint_list in joint_matrix:
        list_len = len(joint_list)
        mid_elem = int(list_len/2)
        element = joint_list[0]

        # Append everything
        records_list.append(element)

    output_array = np.array(records_list)
    header_list = ['time']
    header_list.extend(joint_names)
    header = ','.join(header_list)
    np.savetxt(output_file, output_array, fmt='%.12f', delimiter=",", header=header)

    print('#### Finished processing file in ', output_file, '...')


if __name__ == "__main__":
    main()
