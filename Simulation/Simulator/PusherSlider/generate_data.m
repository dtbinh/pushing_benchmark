%% Author: Francois Hogan
%% Date: 01/08/2017
%% Clear
clear all;
close all;
clc;

%Setup externals
filename = 'data_03_06_2018.mat'
try
    load(filename);
catch
    data.input = []
    data.output = []
end

run('../setup.m');

%Define objects
pusher = PointPusher();
object = Square();
surface = Surface();
planar_system = PlanarSystem(pusher, object, surface);
simulator = Simulator(planar_system, filename);

vt_range = [-.2, .2];
vn_range = [0, .2];
ry_range = [-object.a/2, object.a/2];

init_counter = length(data.input) + 1;
number_ite = 5000;

vbpi = zeros(2,1);

for i=init_counter : init_counter + number_ite - 1
    i
    vbpi(1) = (vn_range(2)-vn_range(1)).*rand(1) + vn_range(1);
    vbpi(2) = (vt_range(2)-vt_range(1)).*rand(1) + vt_range(1);
    ry = (ry_range(2)-ry_range(1)).*rand(1) + ry_range(1);
    data.input(i,:) = [vbpi;ry]';
    twist_b = simulator.pointSimulatorAnalytical_b(vbpi,ry);
    data.output(i,:) = twist_b;
    save(filename, 'data');
end
