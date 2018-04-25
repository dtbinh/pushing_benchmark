%% Author: Francois Hogan
%% Date: 12/08/2016
%% Clear
clear all;
close all;
clc;
%Setup externals
run('setup.m');

%% Simulation Parameters
t0 = 0;
tf = 5;
h_step = 0.01;

%% Build Simulation object
pusher = LinePusher(.3);
% pusher = PointPusher(0.3);
object = Square();
surface = Surface(0.35);
planar_system = PlanarSystem(pusher, object, surface);
% simulator = Simulator(planar_system, simulation_name);
% planner = Planner(planar_system, [], [], [], [], '8Track', 0.05); %8track
planner = Planner(planar_system, [], [], [], [], 'Straight'); %8track

jsonFile = struct('xc_star', planner.xc_star,...
                   'uc_star', planner.uc_star,...
                   'xs_star', planner.xs_star,...
                   'us_star', planner.us_star,...
                   't_star', planner.t_star...
                    );              

% JsonFile = savejson('Matrices', jsonFile,  '../../../Data/8Track_point_pusher_radius_0_15_vel_0_05_3_laps.json');
% JsonFile = savejson('Matrices', jsonFile,  '../../../Data/Straight_point_pusher_vel_0_05.json');
JsonFile = savejson('Matrices', jsonFile,  '../../../Data/Straight_line_pusher_vel_0_05.json');


json2data=loadjson(JsonFile);
