%% Author: Francois Hogan
%% Date: 01/08/2017
%% Clear
clear all;
close all;
clc;

%Setup externals
filename = 'data_03_18_2018_v2.mat'
simulation_name='data_collection'
try
    load(filename);
catch
    data.input = []
    data.output = []
end

% run('../setup.m');

%%Initiaze system
is_gp=true;
%Define objects
pusher_controller = PointPusher(.3);
object_controller = Square();
surface_controller = Surface(.35);
planar_system_controller = PlanarSystem(pusher_controller, object_controller, surface_controller);
simulator_controller = Simulator(planar_system_controller, simulation_name, is_gp);

pusher_sim = PointPusher(.3);
object_sim = Square();
surface_sim = Surface(.35);
planar_system_sim = PlanarSystem(pusher_sim, object_sim, surface_sim);
planar_system_sim.f_max = planar_system_sim.f_max/2;
planar_system_sim.m_max = planar_system_sim.m_max*2;
planar_system_sim.c = planar_system_sim.m_max/planar_system_sim.f_max; 
planar_system_sim.symbolicLinearize();
simulator_sim = Simulator(planar_system_sim, strcat(simulation_name,'_sim'), is_gp);

vt_range = [-.2, .2];
vn_range = [0, .2];
ry_range = [-object_controller.a/2, object_controller.a/2];

init_counter = length(data.input) + 1;
number_ite = 200;

vbpi = zeros(2,1);

for i=init_counter : init_counter + number_ite - 1
    i
    vbpi(1) = (vn_range(2)-vn_range(1)).*rand(1) + vn_range(1);
    vbpi(2) = (vt_range(2)-vt_range(1)).*rand(1) + vt_range(1);
    ry = (ry_range(2)-ry_range(1)).*rand(1) + ry_range(1);
    data.input(i,:) = [vbpi;ry]';
    twist_b_sim = simulator_sim.pointSimulatorAnalytical_b(vbpi,ry);
    twist_b_controller = simulator_controller.pointSimulatorAnalytical_b(vbpi,ry);
    data.output(i,:) = twist_b_sim-twist_b_controller;
    save(filename, 'data');
end