%% Author: Francois Hogan
%% Date: 01/08/2017
%% Clear
clear all;
close all;
clc;

%% Load data
data_list = [5000,2000,1000,500,200,100,50,20,10,5,2];

for counter=1:length(data_list)
    filename = strcat('new_inputs_outputs_validation_side_0_only_',num2str(data_list(counter)),'.mat')
    load(filename);
    
    output_residual_training = [];
    %Setup externals
    filename2 = strcat('data_residual_',filename);
    simulation_name='data_collection_residual'

    % run('../setup.m');

    %%Initiaze system
    is_gp=true;
    %Define objects
    pusher = PointPusher(.3);
    object = Square();
    surface = Surface(.35);
    planar_system = PlanarSystem(pusher, object, surface);
    simulator = Simulator(planar_system, simulation_name);
%     load(filename);
%     simulator.data = data;


    for i=1:length(input_training)
        i
        %get c, phi inputs
        I_23 = input_training(i,:);
        c=I_23(1);
        phi=I_23(2);
        %convert to vn,vt,ry space
        ry = object.a*(1/2 - c);
        vt_tmp = tan(phi);
        vn_tmp=1;
        vt = .02*vt_tmp/sqrt(vt_tmp^2+vn_tmp^2);
        vn = .02*vn_tmp/sqrt(vt_tmp^2+vn_tmp^2);
        %get quasi-static output
        vbpi(1)=vn;
        vbpi(2)=vt;
        twist_b = simulator.pointSimulatorAnalytical_b(vbpi,ry);
        %convert to frame c
        Ccb = Helper.C3_2d(phi);
        Cbc = [transpose(Ccb) [0; 0];0 0 1];
        Cbc=transpose(Cbc);
        twist_c = Cbc'*twist_b;
        %convert to discrete variation delta_xc, delta_yc, delta_thetac space
        delta_xc_model = .2*twist_c';
        %get gp prediction
        delta_xc_gp = output_training(i,:);
    %     delta_xc_gp_test = simulator.pointSimulatorGPDataRaw(c,phi);
        %get new residual data point
        res = delta_xc_gp-delta_xc_model;
        output_residual_training = [output_residual_training;res];

    end
    save(filename2, 'input_training', 'output_training', 'output_residual_training');
end