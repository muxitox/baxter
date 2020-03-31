#!/usr/bin/python

import csv
import numpy as np
import math
from scipy.ndimage.filters import gaussian_filter1d
import matplotlib.pyplot as plt


def main():
    
    first_iter = True

    '''
    input_file = 'TrajIK/NEWMANOALTA_MODIFY_YZ.csv'
    '''
    '''
    input_file = 'TrajIK/OBSTACULOBAJO.csv'
    '''

    source_file = 'TrajIK/FINAL_OBSTACULOBAJO_0.csv'

    source_data = np.genfromtxt(source_file, delimiter=',', dtype=float)

    target_file = 'TrajDMP/FINAL_OBSTACULOBAJO_INI_RIGHTUP_END_LEFTUP.csv'
    target_data = np.genfromtxt(target_file, delimiter=',', dtype=float)


    plt.style.use('ggplot')
    plt.figure(figsize = (7, 2))
    plt.plot(source_data[:-1, 0], source_data[:-1, 1], label='Source')
    plt.plot(target_data[:, 0], target_data[:, 1], linestyle='dashed', label='Planned')
    plt.autoscale(enable=True, axis='x', tight=True)
    plt.xlabel('Time [s]')
    plt.ylabel('Angle')
    plt.subplots_adjust(bottom=0.26)
    plt.title('(a) right_s0')
    plt.legend()

    plt.figure(figsize=(7, 2))
    plt.plot(source_data[:-1, 0], source_data[:-1, 2], label='Source')
    plt.plot(target_data[:, 0], target_data[:, 2], linestyle='dashed', label='Planned')
    plt.autoscale(enable=True, axis='x', tight=True)
    plt.xlabel('Time [s]')
    plt.ylabel('Angle')
    plt.subplots_adjust(bottom=0.26)
    plt.title('(b) right_s1')
    plt.legend()


    plt.figure(figsize=(7, 2))
    plt.plot(source_data[:-1, 0], source_data[:-1, 6], label='Source')
    plt.plot(target_data[:, 0], target_data[:, 6], linestyle='dashed', label='Planned')
    plt.autoscale(enable=True, axis='x', tight=True)
    plt.xlabel('Time [s]')
    plt.ylabel('Angle')
    plt.subplots_adjust(bottom=0.26)
    plt.title('(c) right_e0')
    plt.legend()

    plt.figure(figsize=(7, 2))
    plt.plot(source_data[:-1, 0], source_data[:-1, 7], label='Source')
    plt.plot(target_data[:, 0], target_data[:, 7], linestyle='dashed', label='Planned')
    plt.autoscale(enable=True, axis='x', tight=True)
    plt.xlabel('Time [s]')
    plt.ylabel('Angle')
    plt.subplots_adjust(bottom=0.26)
    plt.title('(d) right_e1')
    plt.legend()

    plt.figure(figsize=(7, 2))
    plt.plot(source_data[:-1, 0], source_data[:-1, 3], label='Source')
    plt.plot(target_data[:, 0], target_data[:, 3], linestyle='dashed', label='Planned')
    plt.autoscale(enable=True, axis='x', tight=True)
    plt.xlabel('Time [s]')
    plt.ylabel('Angle')
    plt.subplots_adjust(bottom=0.26)
    plt.title('(e) right_w0')
    plt.legend()

    plt.figure(figsize=(7, 2))
    plt.plot(source_data[:-1, 0], source_data[:-1, 4], label='Source')
    plt.plot(target_data[:, 0], target_data[:, 4], linestyle='dashed', label='Planned')
    plt.autoscale(enable=True, axis='x', tight=True)
    plt.xlabel('Time [s]')
    plt.ylabel('Angle')
    plt.subplots_adjust(bottom=0.26)
    plt.title('(f) right_w1')
    plt.legend()

    plt.figure(figsize=(7, 2))
    plt.plot(source_data[:-1, 0], source_data[:-1, 5], label='Source')
    plt.plot(target_data[:, 0], target_data[:, 5], linestyle='dashed', label='Planned')
    plt.autoscale(enable=True, axis='x', tight=True)
    plt.xlabel('Time [s]')
    plt.ylabel('Angle')
    plt.title('(g) right_w2')
    plt.legend()

    #right_s0,right_s1,right_w0,right_w1,right_w2,right_e0,right_e1


    plt.show()

if __name__ == "__main__":
    main()
