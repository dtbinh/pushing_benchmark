# -*- coding: utf-8 -*-
"""
Created on Fri Nov  3 11:28:14 2017

@author: fhogan
"""

import json
import pdb
from pprint import pprint
import matplotlib.pyplot as plt
import numpy as np

def column(matrix, i):
    return [row[i] for row in matrix]
    
def plot_experiment(filename):
    title = filename
    experiments = json.load(open(filename))
    target = json.load(open('../../../../../Simulation/Data/'+filename))
    #~ pprint(data)
    x_experiments = experiments['xc'][0][:]
    y_experiments = experiments['xc'][1][:]
    x_target = column(target['Matrices']['xc_star'], 0)
    y_target = column(target['Matrices']['xc_star'], 1)
    fig, ax = plt.subplots()
    ax.plot(x_experiments,y_experiments, 'r--', label='Experiments')
    ax.plot(x_target, y_target, 'k', label='Target')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title(title)
    legend = ax.legend(loc='upper right', shadow=True)
    frame = legend.get_frame()

        # Set the fontsize
    for label in legend.get_texts():
        label.set_fontsize('large')

    for label in legend.get_lines():
        label.set_linewidth(1.5)  # the legend line width
    
    plt.axis('equal')
    plt.savefig(title + '.png')
    plt.show()

def plot_experiments():
    for i in [18,17,16,15,14,13,12, 11, 10]:
        filename='8Track_line_pusher_radius_0_'+str(i)+'_vel_0_05.json'
        title = 'line_pusher_radius_'+str(i)
        experiments = json.load(open(filename))
        target = json.load(open('../../../../../Simulation/Data/'+filename))
        #~ pprint(data)
        x_experiments = experiments['xc'][0][:]
        y_experiments = experiments['xc'][1][:]
        x_target = column(target['Matrices']['xc_star'], 0)
        y_target = column(target['Matrices']['xc_star'], 1)
        fig, ax = plt.subplots()
        ax.plot(x_experiments,y_experiments, 'r--', label='Experiments')
        ax.plot(x_target, y_target, 'k', label='Target')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.title(title)
        legend = ax.legend(loc='upper right', shadow=True)
        frame = legend.get_frame()

            # Set the fontsize
        for label in legend.get_texts():
            label.set_fontsize('large')

        for label in legend.get_lines():
            label.set_linewidth(1.5)  # the legend line width
        
        plt.axis('equal')
        plt.savefig(title + '.png')
        
def convert2pusher(x_vec,y_vec, theta_vec, ry_vec):
    x_new = []
    y_new = []
    for i in range(len(x_vec)):
        x = x_vec[i]
        y = y_vec[i]
        theta = theta_vec[i]
        ry = ry_vec[i]
        
        ribi = np.array([x,y])
        rbpb = np.array([-0.045,ry])
        Cbi = np.array([[np.cos(theta), np.sin(theta)],[-np.sin(theta), np.cos(theta)]])
        ripi = ribi + np.dot(Cbi.transpose(),rbpb)

        x_new.append(ripi[0])
        y_new.append(ripi[1])
        
    return np.array(x_new), np.array(y_new)
    
 
def plot_nominal():
    filename='8Track_line_pusher_radius_0_15_vel_0_05_open_loop.json'
    filename_target='8Track_point_pusher_radius_0_15_vel_0_05.json'
    title = 'Open Loop Trajectory'
    experiments = json.load(open(filename))
    target = json.load(open('../../../../../Simulation/Data/'+filename_target))

    x_experiments = np.array(experiments['q_pusher_sensed'][0][:])- np.array(experiments['q_pusher_sensed'][0][0])
    y_experiments = experiments['q_pusher_sensed'][1][:]
    x_target = column(target['Matrices']['xc_star'], 0)
    y_target = column(target['Matrices']['xc_star'], 1)
    theta_target = column(target['Matrices']['xc_star'], 2)
    ry_target = column(target['Matrices']['xc_star'], 3)
    x_new, y_new = convert2pusher(x_target, y_target, theta_target, ry_target)
    
    fig, ax = plt.subplots()
    ax.plot(x_experiments,y_experiments, 'r--', label='Robot Pusher Trajectory')
    ax.plot(x_new, y_new, 'b--', label='Desired Pusher Trajectory')
    ax.plot(x_target, y_target, 'k', label='Desired Object Trajectory')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title(title)
    legend = ax.legend(loc='upper right', shadow=True)
    frame = legend.get_frame()

        # Set the fontsize
    for label in legend.get_texts():
        label.set_fontsize('large')

    for label in legend.get_lines():
        label.set_linewidth(1.5)  # the legend line width
    
    plt.axis('equal')
    plt.savefig(title + '.png')
    plt.show()
    pdb.set_trace()

    

if __name__ == "__main__":
    #~ plot_experiment('8Track_point_pusher_radius_0_15_vel_0_02.json')
    #~ plot_experiments()

    plot_nominal()
    pdb.set_trace()
