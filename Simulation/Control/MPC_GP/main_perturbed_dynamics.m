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
simulation_name = 'mpc_perturbed_dynamics1';
%% Simulation time
sim_time = 15;

%% Initial conditions 
% x0 = [x;y;theta;xp;yp]
% x: x position of object, y: y position of object, theta: orientation of object
% xp: x position of pusher, yp: y position of pusher
x0_c = [0.0;0.03*1;15*pi/180*1;-.009];
%%Initiaze system
is_gp=true;
%Define objects
pusher_controller = PointPusher(.3);
object_controller = Square();
surface_controller = Surface(.35);
planar_system_controller = PlanarSystem(pusher_controller, object_controller, surface_controller);
simulator_controller = Simulator(planar_system_controller, simulation_name, is_gp);

pusher_sim = PointPusher(.7);
object_sim = Square();
surface_sim = Surface(.35);
planar_system_sim = PlanarSystem(pusher_sim, object_sim, surface_sim);
simulator_sim = Simulator(planar_system_sim, strcat(simulation_name,'_sim'), is_gp);

x0 = planar_system_sim.coordinateTransformCS(x0_c);

%Build variables
t0 = 0;
tf = sim_time;
h_step = 0.01;


planner = Planner(planar_system_controller, simulator_controller, Linear, data, object_gp, 'inf_circle', 0.05); %8track
%Controller setup
Q = 10*diag([3,3,.1,0.1]);
Qf=  1*2000*diag([3,3,1,.1]);
R = 1*diag([1,1,.1]);
mpc = MPCforce(planner, 'is_fom', Q, Qf, R, Linear, data, object_gp);

%build simulator object properties
simulator_sim.h = h_step;
simulator_sim.N = floor((1/simulator_sim.h)*(tf-t0));
simulator_sim.t = zeros(simulator_sim.N,1);
simulator_sim.xs_state = zeros(simulator_sim.N, planar_system_controller.num_xsStates);
simulator_sim.us_state = zeros(simulator_sim.N, planar_system_controller.num_usStates);
simulator_sim.xs_state(1,:) = x0';simulator_sim.x_star = planner.xs_star;
simulator_sim.u_star = planner.us_star;
simulator_sim.t_star = planner.t_star;

[xs_d,us_d] = simulator_sim.find_nominal_state(0);
simulator_sim.initialize_plot(x0, xs_d, sim_time);
%% Perform Simulation
for i1=1:simulator_sim.N
        %display current time 
        disp(simulator_sim.t(i1));
        
        %Define loop variables
        xs = simulator_sim.xs_state(i1,:)';
        
        %get error
        [xcStar, ucStar, xsStar, usStar,A,B] = mpc.getStateNominal(simulator_sim.t(i1));
        
        %apply lqr controller
        xc = planar_system_controller.coordinateTransformSC(xs);
        uc = mpc.solveMPC(xc, simulator_sim.t(i1));
%         uc = mpc.solveMPC_gp(xc, simulator_sim.t(i1));
        us = planar_system_controller.force2Velocity(xc, uc);
        %simulate forward
        %1. analytical model
        xs_next = simulator_sim.get_next_state_i(xs, us, simulator_sim.h);
        %2. gp model
%         xs_next = simulator.get_next_state_gp_i(xs, us, simulator.h);
        %update plot
        simulator_sim.update_plot(xs_next, simulator_sim.t(i1));
%       %Perform Euler Integration
        if i1<simulator_sim.N
            simulator_sim.t(i1+1)  = simulator_sim.t(i1) + simulator_sim.h;
            simulator_sim.xs_state(i1+1, :) = xs_next;%simulator.xs_state(i1, :) + simulator.h*dxs';       
        end
        %Save data 
        simulator_sim.us_state(i1,:) = us;
%          pause(.1);
end

%% Graphics
% simulator.Animate1pt(simulator.x_star)
close(simulator_sim.v);
%% Post-Processing
save(simulator_sim.FileName, 'simulator');