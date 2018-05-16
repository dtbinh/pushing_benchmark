%% Author: Francois Hogan
%% Date: 07/03/2018
clear
clc
close all

% symbolic_linearize_residual_v2;
symbolic_linearize_data;

%% Simulation data and video are stored in /home/Data/<simulation_name>
simulation_name = 'analytical';
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
% load('learning_output_model_from_train_size_4000');
% load('trained_new_inputs_outputs_validation_side_0_only_5000');
% simulator.data = data;


for lv1=1:1
%     planner = Planner(planar_system, simulator, [], [], object, '8Track', 0.05, 0.15); %8track
%     planner = Planner(planar_system, simulator, Linear, data, object, '8Track_residual', 0.05, 0.15); %8track
    planner = Planner(planar_system, simulator, Linear, data, object, '8Track_gp', 0.05, 0.15); %8track

    jsonFile = struct('xc_star', planner.xc_star,...
                       'uc_star', planner.uc_star,...
                       'xs_star', planner.xs_star,...
                       'us_star', planner.us_star,...
                       'A_star', reshape(planner.A_star, length(planner.A_star), size(planner.xc_star,2)*size(planner.xc_star,2)),...
                       'B_star', reshape(planner.B_star, length(planner.B_star), size(planner.xc_star,2)*size(planner.uc_star,2)),...
                       't_star', planner.t_star...
                        );              


%     file_name = '../../../Data/8Track_point_pusher_radius_0_15_vel_0_05_1_laps_analytical_controller.json';
%     file_name = '../../../Data/8Track_point_pusher_radius_0_15_vel_0_05_1_laps_hybrid_controller.json';
    file_name = '../../../Data/8Track_point_pusher_radius_0_15_vel_0_05_1_laps_gp_controller.json';

    JsonFile = savejson('Matrices', jsonFile, file_name);
end

json2data=loadjson(JsonFile);