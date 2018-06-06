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

gp_model_name = 'trained_data_residual_new_inputs_outputs_validation_side_0_only_5000';
Linear = symbolic_linearize_residual_v2(gp_model_name);
load(gp_model_name);

%% Simulation data and video are stored in /home/Data/<simulation_name>
simulation_name = 'mpc_force1';
%% Simulation time
sim_time = 45;

%% Initial conditions 
% x0 = [x;y;theta;xp;yp]
% x: x position of object, y: y position of object, theta: orientation of object
% xp: x position of pusher, yp: y position of pusher
x0_c = [0.0;0.03*1;0*pi/180*1;.00];
%%Initiaze system
is_gp=true;
initialize_system();


des_velocity = 0.05;
des_dist = 0.15;
num_laps=1;
planner = Planner(planar_system, simulator, Linear, data, object, 'Square', des_velocity, des_dist, num_laps); %8track

% planner = Planner(planar_system, simulator, Linear, data, object, '8Track_residual', 0.05, 0.15, 1 ); %8track

%Controller setup
Q = 10*diag([1,10,1,1]);
Qf=  2000*diag([1,10,1,1]);
R = .01*diag([1,1,1]);
mpc = MPCforce(planner, 'is_fom', Q, Qf, R, Linear, data, object);
%send planned trajectory to simulator for plotting
simulator.x_star = planner.xs_star;
simulator.u_star = planner.us_star;
simulator.t_star = planner.t_star;
simulator.data = data;

[xs_d,us_d] = simulator.find_nominal_state(0);
simulator.initialize_plot(x0, xs_d, sim_time);

%% Perform Simulation
for i1=1:simulator.N
        %display current time 
        disp(simulator.t(i1));
        
        %Define loop variables
        xs = simulator.xs_state(i1,:)';
        
        %get error
        [xcStar, ucStar, xsStar, usStar,A,B] = mpc.getStateNominal(simulator.t(i1));
        
        %apply lqr controller
        xc = planar_system.coordinateTransformSC(xs);
        uc = mpc.solveMPC(xc, simulator.t(i1));
        us = planar_system.force2Velocity(xc, uc);
        %simulate forward
        %1. analytical model
        xs_next = simulator.get_next_state_i(xs, us, simulator.h);
        %2. gp model
%         xs_next = simulator.get_next_state_gp_i(xs, us, simulator.h);
        %3. residual
%         xs_next = simulator.get_next_state_gpResidual_i(xs, us, simulator.h);
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


        %% Graphics
        % simulator.Animate1pt(simulator.x_star)
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