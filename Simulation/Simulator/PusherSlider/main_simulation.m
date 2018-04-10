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

symbolic_linearize;

%% Simulation data and video are stored in /home/Data/<simulation_name>
simulation_name = 'analytical_simulation';
%% Simulation time
sim_time = 25;

%% Initial conditions 
% x0 = [x;y;theta;xp;yp]
% x: x position of object, y: y position of object, theta: orientation of object
% xp: x position of pusher, yp: y position of pusher
x0_c = [0.0;0.03;0*pi/180;-0.02];
%%Initiaze system
initialize_system();
planner = Planner(planar_system, 'Straight', 0.05); 
simulator.x_star = planner.xs_star;
simulator.u_star = planner.us_star;
simulator.t_star = planner.t_star;

%Set up lqr controller parameters
A = double(subs(A_fun([0;0;0;0],[0.05;0])));
B = double(subs(B_fun([0;0;0;0],[0.05;0])));
Q = diag([1,10,1,1]);
R = .1*eye(size(B,2));
[K, S, e] = lqr(A,B,Q, R);
Ain = [eye(2);-eye(2)];
bin = [.15;.2;.2;0];

H = R;
% F = x_bar'*S*B;
% u_bar = quadprog(H,F)

[xs_d,us_d] = simulator.find_nominal_state(0);
simulator.initialize_plot(x0, xs_d);
%% Perform Simulation
for i1=1:simulator.N
        %display current time 
        disp(simulator.t(i1));
        
        %Define loop variables
        xs = simulator.xs_state(i1,:)';
        
        %apply lqr controller
        xc = planar_system.coordinateTransformSC(xs);
        [xs_d,us_d] = simulator.find_nominal_state(simulator.t(i1));
        xc_d =  planar_system.coordinateTransformSC(xs_d);
        x_bar = xc - xc_d;
        u_bar = -K*x_bar;
%         F = x_bar'*S*B;
%         u_bar = quadprog(H,F, Ain, bin);
        
        
          
        %% edit velocity (us = [vx, vy] -> velocity of pusher in world frame in m/s)
        %----------------------------
%         u_bar(1) = max(-0.05, u_bar(1));
%          us = us_d + u_bar;
%          u_bar
%          us
         us = [0.02;0.005];
        %--------------------------------
        xs_next = simulator.get_next_state_i(xs, us, simulator.h);
        %update plot
        simulator.update_plot(xs_next, simulator.t(i1));
        
        %Get velocities from current object state and robot velocity 
%         dxs = simulator.pointSimulator(xs,us);
% 
%       %Perform Euler Integration
        if i1<simulator.N
            simulator.t(i1+1)  = simulator.t(i1) + simulator.h;
            simulator.xs_state(i1+1, :) = xs_next;%simulator.xs_state(i1, :) + simulator.h*dxs';       
        end
        
        %Save data 
        simulator.us_state(i1,:) = us;
         pause(.1);
end

%% Graphics
simulator.Animate1pt()

%% Post-Processing
save(simulator.FileName, 'simulator');