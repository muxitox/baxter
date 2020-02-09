#!/usr/bin/python

import csv
import numpy as np
import math


def main():
    
    
    first_iter = True

    input_file = 'Pre/horizontalConElevacion.csv'
    output_file = 'Post/horizontalConElevacion.csv'

    cartesian_list = []

    print('')
    print('#### Start reading endpoints in ', input_file, '...')
    # with open('/home/angelp/catkin_ws/src/dmp/data/baxter_endpoints.csv') as csvfile:
    with open(input_file) as csvfile1:
        cartesian_reader = csv.reader(csvfile1, delimiter=' ')
        for row in cartesian_reader:

            line = [float(i.replace(',','.')) for i in row]
            cartesian_list.append(line)
            
    cartesian_array = np.array(cartesian_list)

    N = 5
    len_array = len(cartesian_array)

    output_list = []
    for i in range(0,int(math.ceil(len_array/N))):
        inf = i*N
        sup = i*N+N
        if sup > len_array:
            sup = len_array
            inf = len_array - N
        _median = np.median(cartesian_array[inf:sup],0)
        output_list.append(_median)
    
    output_array = np.array(output_list)

    np.savetxt(output_file, output_array, delimiter=" ")



    print('#### Finished processing file in ', output_file ,'...')






if __name__ == "__main__":
    main()
