clear
close all
clc

%simulation parameters
x0 = [pi; 0];
tspan = 15;
dt = 0.05;
N = tspan/dt;

%Define Dynamical system
sys = Pendulum();

%Define controller object
MPPI = MPPI(sys);

x = zeros(N, 2);
x(1,:) = x0';

for i=1:N
    %get action 
    u = MPPI.controller(x(i,:));
    %get next state
    x(i+1,:) = sys.get_next_state(x(i,:), u, dt, 1);
    % Update plot
    sys.update_plot(x(i,:));
end