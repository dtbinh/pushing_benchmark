%% Author: Francois Hogan
%% Date: 07/03/2018
%--------------------------------------------------------
% Description:
% This script simulates the motion of a square object subject to robot
% velocites. The simulation total time (line 18), object and robot initial
% configurations (line 24), and robot applied velocities (line 40)
%--------------------------------------------------------
clear
clc
close all

run(strcat(getenv('HOME'),'/pushing_benchmark/Simulation/Simulator/setup.m'));

%% Simulation data and video are stored in /home/Data/<simulation_name>
simulation_name = 'trajectory_test';
%% Simulation time
sim_time = 120;

%% Initial conditions 
% x0 = [x;y;theta;xp;yp]
% x: x position of object, y: y position of object, theta: orientation of object
% xp: x position of pusher, yp: y position of pusher
x0_c = [0.0;-0.00*1;0*pi/180*1;.009*1];
%%Initiaze system
initialize_system();
planner = Planner(planar_system, [], [], [], [], '8Track', 0.05); %8track
%send planned trajectory to simulator for plotting
simulator.x_star = planner.xs_star;
simulator.u_star = planner.us_star;
simulator.t_star = planner.t_star;

[xs_d,us_d] = simulator.find_nominal_state(0);
simulator.initialize_plot(x0, xs_d, sim_time);
%% Perform Simulation
for i1=1:simulator.N
        %display current time 
        disp(simulator.t(i1));
        
        %find nominal traj
        [xs_d,us_d] = simulator.find_nominal_state(simulator.t(i1));
        
        %Define loop variables
        xs = simulator.xs_state(i1,:)';
        
        %apply lqr controller
        xc = planar_system.coordinateTransformSC(xs);

        us = us_d;
% us = [.05;0];
        %simulate forward
        %1. analytical model
        xs_next = simulator.get_next_state_i(xs, us, simulator.h);
        %2. gp model
%         xs_next = simulator.get_next_state_gpData_i(xs, us, simulator.h);
        %update plot
        simulator.update_plot(xs_next, simulator.t(i1));
%       %Perform Euler Integration
        if i1<simulator.N
            simulator.t(i1+1)  = simulator.t(i1) + simulator.h;
            simulator.xs_state(i1+1, :) = xs_next;%simulator.xs_state(i1, :) + simulator.h*dxs';       
        end
        %Save data 
        simulator.us_state(i1,:) = us;
%          pause(.1);
end

%% Graphics
% simulator.Animate1pt(simulator.x_star)
close(simulator.v);
%% Post-Processing
save(simulator.FileName, 'simulator');