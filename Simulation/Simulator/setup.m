%Setup external functions, libraries, and directories

% PATH
addpath(strcat(getenv('HOME'),'/pushing_benchmark/Simulation/Simulator/PusherSlider'));
addpath(strcat(getenv('HOME'),'/pushing_benchmark/Simulation/Simulator'));
addpath(strcat(getenv('HOME'),'/pushing_benchmark/software/externals/jsonlab-1.0'));
addpath(strcat(getenv('HOME'),'/pushing_benchmark/Simulation/System'));
addpath(strcat(getenv('HOME'),'/pushing_benchmark/Simulation/GP_learning'));

% add gurobi path
if ismac()
    run('/Users/Francois/Dropbox (MIT)/Matlab/MIT/Drake/drake/addpath_drake');
    addpath('/Library/gurobi650/mac64/matlab');
    gurobi_setup;
elseif isunix() && ~ismac()
    baseName = getenv('HOME');
    drakePath = strcat(baseName, '/software/drake-distro/drake/addpath_drake');
    gurobiPath = strcat(baseName, '/software/gurobi702/linux64/matlab');
    run(drakePath);
    addpath(gurobiPath);
    gurobi_setup;
elseif ispc()
    addpath('C:\gurobi701\win64\matlab');
end