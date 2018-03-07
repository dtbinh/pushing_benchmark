%% Author: Francois Hogan
%% Date: 07/03/2018
%--------------------------------------------------------
% Description:
% This script simulates the motion of a square object subject to robot
% velocites. The simulation total time (line 18), object and robot initial
% configurations (line 24), and robot applied velocities (line 40)
%--------------------------------------------------------

clear all;
close all;
clc;

%% Simulation data and video are stored in /home/Data/<simulation_name>
simulation_name = 'test';

%% Simulation time
sim_time = 10;

%% Initial conditions 
% x0 = [x;y;theta;xp;yp]
% x: x position of object, y: y position of object, theta: orientation of object
% xp: x position of pusher, yp: y position of pusher
x0 = [0;0;0;-0.1;0]; 

%%Initiaze system
initialize_system();


%% Perform Simulation
for i1=1:simulator.N
        %display current time 
        disp(simulator.t(i1));
        
        %Define loop variables
        xs = simulator.xs_state(i1,:)';
          
        %% edit velocity (us = [vx, vy] -> velocity of pusher in world frame in m/s)
        %----------------------------
        us = [0.02;0.005];
        %--------------------------------
        
        %Get velocities from current object state and robot velocity 
        dxs = simulator.pointSimulator(xs,us);

        %Perform Euler Integration
        if i1<simulator.N
            simulator.t(i1+1)  = simulator.t(i1) + simulator.h;
            simulator.xs_state(i1+1, :) = simulator.xs_state(i1, :) + simulator.h*dxs';       
        end
        
        %Save data 
        simulator.us_state(i1,:) = us;
end

%% Graphics
simulator.Animate1pt()

%% Post-Processing
save(simulator.FileName, 'simulator');