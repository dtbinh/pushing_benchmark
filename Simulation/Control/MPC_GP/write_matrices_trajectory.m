%% Author: Francois Hogan
%% Date: 07/03/2018
clear
clc
close all

%%%%%%%%%%%%%
%% to edit %%
%%%%%%%%%%%%%
des_velocity = 0.08;
des_radius = 0.05; %8track: denoted curvature of circles (0.15), square: denoted curvature of corners (0.05), 
des_dist = 0.15; %only used for square denotes line lengths
is_reversed = false;%only used for 8track
is_square = true; %options are true: square, false: 8track
data_list = [5000]; %used for flag 1-2 only. datapoints in dataset [5000,2000,1000,500,200,100,50,20,10,5,2]; 
version_list = [1]; %only used for flag3
flag = 1;%1:old data, 2: new data, 3: 5 datapoints only (most recent)
%------------

if is_reversed && not(is_square)
    des_radius=-des_radius;
end

for lv1=1:length(version_list)
    for counter=1:length(data_list)
        num_laps=1;
        if flag==1
            gp_model_name = strcat('trained_new_inputs_outputs_validation_side_0_only_',num2str(data_list(counter)),'.mat');
        elseif flag==2
            gp_model_name = strcat('trained_new_inputs_outputs_with_05_data_only_',num2str(data_list(counter)),'.mat');
        elseif flag==3
            gp_model_name = strcat('trained_new_inputs_outputs_5_data_v',num2str(version_list(lv1)),'.mat');
        end

         if is_square
             if flag==1
                 json_filename = strcat('../../../Data/square_point_pusher_radius_',num2str(des_radius),'_dist_', num2str(des_dist),'_vel_',num2str(des_velocity),'_',num2str(num_laps),'_laps_data_',num2str(data_list(counter)),'_gpdata_controller.json');
             elseif flag==2
                 json_filename = strcat('../../../Data/new_data_square_point_pusher_radius_',num2str(des_radius),'_dist_', num2str(des_dist),'_vel_',num2str(des_velocity),'_',num2str(num_laps),'_laps_data_',num2str(data_list(counter)),'_gpdata_controller.json');
             elseif flag==3
                 json_filename = strcat('../../../Data/5_data_square_point_pusher_radius_',num2str(des_radius),'_dist_', num2str(des_dist),'_vel_',num2str(des_velocity),'_',num2str(num_laps),'_laps_version_',num2str(version_list(counter)),'_gpdata_controller.json');
             end
        else
            if is_reversed
                 if flag==1
                     json_filename = strcat('../../../Data/reversed_8Track_point_pusher_radius_',num2str(des_radius),'_vel_',num2str(des_velocity),'_',num2str(num_laps),'_laps_data_',num2str(data_list(counter)),'_gpdata_controller.json');
                 elseif flag==2
                     json_filename = strcat('../../../Data/reversed_new_data_8Track_point_pusher_radius_',num2str(des_radius),'_vel_',num2str(des_velocity),'_',num2str(num_laps),'_laps_data_',num2str(data_list(counter)),'_gpdata_controller.json');
                 elseif flag==3
                     json_filename = strcat('../../../Data/reversed_5_data_8Track_point_pusher_radius_',num2str(des_radius),'_dist_', num2str(des_dist),'_vel_',num2str(des_velocity),'_',num2str(num_laps),'_laps_version_',num2str(version_list(lv1)),'_gpdata_controller.json');
                 end
            else
                 if flag==1
                     json_filename = strcat('../../../Data/8Track_point_pusher_radius_',num2str(des_radius),'_vel_',num2str(des_velocity),'_',num2str(num_laps),'_laps_data_',num2str(data_list(counter)),'_gpdata_controller.json');
                 elseif flag==2
                     json_filename = strcat('../../../Data/new_data_8Track_point_pusher_radius_',num2str(des_radius),'_vel_',num2str(des_velocity),'_',num2str(num_laps),'_laps_data_',num2str(data_list(counter)),'_gpdata_controller.json');
                 elseif flag==3
                     json_filename = strcat('../../../Data/5_data_8Track_point_pusher_radius_',num2str(des_radius),'_dist_', num2str(des_dist),'_vel_',num2str(des_velocity),'_',num2str(num_laps),'_laps_version_',num2str(version_list(lv1)),'_gpdata_controller.json');
                 end
            end
        end

        Linear = symbolic_linearize_data(gp_model_name);
        load(gp_model_name);
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
        % load('learning_output_model_from_train_size_4000');
        simulator.data = data;

        for lv1=1:1
            if is_square
                planner = Planner(planar_system, simulator, Linear, data, object, 'SquareCurved_gp', des_velocity, des_radius,num_laps,des_dist); %8track
            else
                planner = Planner(planar_system, simulator, Linear, data, object, '8Track_gp', des_velocity, des_radius,num_laps);
            end

            planner.ps.num_ucStates = 2;

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
end
