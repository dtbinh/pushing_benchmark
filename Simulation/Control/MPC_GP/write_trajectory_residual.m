%% Author: Francois Hogan
%% Date: 07/03/2018

clear
clc
close all

%edit
des_velocity = 0.05;
des_radius = 0.15;
num_laps = 1;

data_list = [100,200,500,1000,2000, 5000];

for counter=1:length(data_list)
    gp_model_name = strcat('trained_new_inputs_outputs_validation_side_0_only_',num2str(data_list(counter)),'.mat');

    gp_model_name = 'trained_gp_model_data_residual_05_09_2018';

    json_filename = strcat('../../../Data/8Track_point_pusher_radius_',num2str(des_radius),'_vel_',num2str(des_velocity),'_',num2str(num_laps),'_laps_hybrid_controller.json');
    Linear = symbolic_linearize_residual_v2(gp_model_name);
    load(gp_model_name);

    %% Simulation data and video are stored in /home/Data/<simulation_name>
    simulation_name = 'gp_residual';
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
        planner = Planner(planar_system, simulator, Linear, data, object, '8Track_residual', des_velocity, des_radius, num_laps); %8track

        jsonFile = struct('xc_star', planner.xc_star,...
                           'uc_star', planner.uc_star,...
                           'xs_star', planner.xs_star,...
                           'us_star', planner.us_star,...
                           'A_star', reshape(planner.A_star, length(planner.A_star), size(planner.xc_star,2)*size(planner.xc_star,2)),...
                           'B_star', reshape(planner.B_star, length(planner.B_star), size(planner.xc_star,2)*size(planner.uc_star,2)),...
                           't_star', planner.t_star...
                            );              


        JsonFile = savejson('Matrices', jsonFile, json_filename);
    end

    json2data=loadjson(JsonFile);
end