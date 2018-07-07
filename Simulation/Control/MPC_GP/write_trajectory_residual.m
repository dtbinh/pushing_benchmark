%% Author: Francois Hogan
%% Date: 07/03/2018
clear
clc
close all

%%%%%%%%%%%%%
%% to edit %%
%%%%%%%%%%%%%
des_velocity = 0.10;
des_radius = 0.15; %8track: denoted curvature of circles (0.15), square: denoted curvature of corners (0.05), 
des_dist = 0.15; %only used for square denotes line lengths
is_reversed = false;%only used for 8track
is_square = false; %options are true: square, false: 8track
data_list = [5000]; %this is not actually used! dummy variable required
%------------

if is_reversed && not(is_square)
    des_radius=-des_radius;
end

for counter=1:length(data_list)
    gp_model_name = strcat('trained_data_residual_new_inputs_outputs_validation_side_0_only_',num2str(data_list(counter)),'.mat');
    num_laps = 1;
    data_list = [5000]; %this is not actually used! dummy variable required
    if is_square
        json_filename = strcat('../../../Data/square_point_pusher_radius_',num2str(des_radius),'_dist_', num2str(des_dist),'_vel_',num2str(des_velocity),'_',num2str(num_laps),'_laps_data_',num2str(data_list(counter)),'_hybrid_controller.json');
    else
        if is_reversed
            json_filename = strcat('../../../Data/reversed_8Track_point_pusher_radius_',num2str(des_radius),'_vel_',num2str(des_velocity),'_',num2str(num_laps),'_laps_data_',num2str(data_list(counter)),'_hybrid_controller.json');
        else
            json_filename = strcat('../../../Data/8Track_point_pusher_radius_',num2str(des_radius),'_vel_',num2str(des_velocity),'_',num2str(num_laps),'_laps_data_',num2str(data_list(counter)),'_hybrid_controller.json');
        end
    end

    Linear = symbolic_linearize_residual_v2(gp_model_name);
    load(gp_model_name);

    %% Simulation data and video are stored in /home/Data/<simulation_name>
    simulation_name = 'gp_residual';
    sim_time = 45;
    x0_c = [0.0;-0.03*1;15*pi/180*1;.00*1];
    is_gp=true;
    initialize_system();

    for lv1=1:1
        if is_square
            planner = Planner(planar_system, simulator, Linear, data, object, 'SquareCurved',    des_velocity, des_radius,num_laps,des_dist);
        else
            planner = Planner(planar_system, simulator, Linear, data, object, '8Track_residual', des_velocity, des_radius, num_laps); %8track
        end

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
