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
    '''
    '''
    input_file = 'Pre/23-02-2020-18-11-37-NEW_OBSTACULOBAJO.csv'
    '''

    input_file = 'Pre/23-02-2020-18-2-30-NEW_MANOALTA_BUENO.csv'

    discarded_number_list = []
    discarded_number_total = 0
    discarded_number_period = 0
    num_elem_period = 0
    num_elem_total = 0

    data = np.genfromtxt(input_file, delimiter=',', dtype=float)

    plt.style.use('ggplot')
    plt.figure(1)

    plt.subplot(3,3,1)
    plt.plot(data[:, 0], data[:, 1])
    plt.xlabel('Time [s]')
    plt.title('(a) Wrist X')

    plt.subplot(3, 3, 2)
    plt.plot(data[:, 0], data[:, 2])
    plt.xlabel('Time [s]')
    plt.title('(b) Wrist Y')

    plt.subplot(3, 3, 3)
    plt.plot(data[:, 0], data[:, 3])
    plt.xlabel('Time [s]')
    plt.title('(c) Wrist Z')

    plt.subplot(3, 3, 4)
    plt.plot(data[:, 0], data[:, 4])
    plt.xlabel('Time [s]')
    plt.title('(d) Hand X')

    plt.subplot(3, 3, 5)
    plt.plot(data[:, 0], data[:, 5])
    plt.xlabel('Time [s]')
    plt.title('(e) Hand Y')

    plt.subplot(3, 3, 6)
    plt.plot(data[:, 0], data[:, 6])
    plt.xlabel('Time [s]')
    plt.title('(f) Hand Z')

    plt.subplot(3, 3, 7)
    plt.plot(data[:, 0], data[:, 7])
    plt.xlabel('Time [s]')
    plt.title('(g) Thumb X')

    plt.subplot(3, 3, 8)
    plt.plot(data[:, 0], data[:, 8])
    plt.xlabel('Time [s]')
    plt.title('(h) Thumb Y')

    plt.subplot(3, 3, 9)
    plt.plot(data[:, 0], data[:, 9])
    plt.xlabel('Time [s]')
    plt.title('(i) Thumb Z')

    plt.suptitle('Joint Positions [m]')

    plt.show()

if __name__ == "__main__":
    main()
