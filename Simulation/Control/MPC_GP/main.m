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

symbolic_linearize;

%% Simulation data and video are stored in /home/Data/<simulation_name>
simulation_name = 'gp_lqr_6_gp_model';
%% Simulation time
sim_time = 5;

%% Initial conditions 
% x0 = [x;y;theta;xp;yp]
% x: x position of object, y: y position of object, theta: orientation of object
% xp: x position of pusher, yp: y position of pusher
x0_c = [0.0;0.03*0;0*pi/180*1;-.009];
%%Initiaze system
is_gp = true;

initialize_system();

planner = Planner(planar_system, simulator, Linear, data, object, 'inf_circle', 0.05); %8track
planner.ps.num_ucStates = 2;
mpc = MPC(planner, Linear, data, object);
% mpc = MPC(planner, [], [], [], A, B);
simulator.x_star = planner.xs_star;
simulator.u_star = planner.us_star;
simulator.t_star = planner.t_star;

%Set up lqr controller parameters
Q = diag([1,1,.01,.01]);
R = 1*eye(2);
% [K, S, e] = lqr(A,B,Q, R);
Ain = [1 0;0 1;-1 0;0 -1];
bin = [.15;.15; 0; .15];

H = R;
% F = x_bar'*S*B;
% u_bar = quadprog(H,F)

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
        [xs_d,us_d] = simulator.find_nominal_state(simulator.t(i1));
        xc_d =  planar_system.coordinateTransformSC(xs_d);
        x_bar = xc - xc_d;
%         u_bar_b = -K*x_bar;
%         F = x_bar'*S*B;
%         u_bar_b = quadprog(H,F, [], []);
        Cbi = Helper.C3_2d(xc(3));
        u_bar_b = mpc.solveMPC(xc, simulator.t(i1));

        u_bar = Cbi'*u_bar_b;
        %% edit velocity (us = [vx, vy] -> velocity of pusher in world frame in m/s)
%         %----------------------------
%         u_bar(1) = max(-0.05, u_bar(1));
%         u_bar(1) = min(0.15, u_bar(1));
%          us = us_d + u_bar;
        us = u_bar;
%          u_bar
%          us
%          us = [0.05;0.00];
        %--------------------------------
        is_gp = true;
%         xs_next = simulator.get_next_state_i(xs, us, simulator.h);
        xs_next = simulator.get_next_state_gp_i(xs, us, simulator.h);
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