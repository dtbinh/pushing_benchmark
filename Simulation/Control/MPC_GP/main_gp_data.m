%% Author: Francois Hogan
%% Date: 07/03/2018
%--------------------------------------------------------
% Description:
% This script simulaLtes the motion of a square object subject to robot
% velocites. The simulation total time (line 18), object and robot initial
% configurations (line 24), and robot applied velocities (line 40)
%--------------------------------------------------------
clear
clc
close all

run(strcat(getenv('HOME'),'/pushing_benchmark/Simulation/Simulator/setup.m'));

symbolic_linearize_data;

%% Simulation data and video are stored in /home/Data/<simulation_name>
simulation_name = 'gp_data';
%% Simulation time
sim_time = 45;

%% Initial conditions 
% x0 = [x;y;theta;xp;yp]
% x: x position of object, y: y position of object, theta: orientation of object
% xp: x position of pusher, yp: y position of pusher
x0_c = [0.0;-0.03*1;15*pi/180*1;.00*1];
%%Initiaze system
is_gp=true;
initialize_system();
load('learning_output_model_from_train_size_4000');
simulator.data = data;
planner = Planner(planar_system, simulator, Linear, data, object, '8Track_gp', 0.05, 0.15); %8track
planner.ps.num_ucStates = 2;
%Controller setup
Q = 1*diag([1,1,.01,0.1]);
Qf=  1*2000*diag([1,1,.1,.1]);
R = 1*diag([1,1]);
mpc = MPC(planner, Q, Qf, R, Linear, data, object);
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
        
        %Define loop variables
        xs = simulator.xs_state(i1,:)';
        
        %apply lqr controller
        xc = planar_system.coordinateTransformSC(xs);
        uc = mpc.solveMPC(xc, simulator.t(i1));
%          uc
        us = mpc.get_robot_vel(xc, uc);
% us = [.05;0];
        %simulate forward
        %1. analytical model
%         xs_next = simulator.get_next_state_i(xs, us, simulator.h);
        %2. gp model
        xs_next = simulator.get_next_state_gpData_i(xs, us, simulator.h);
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