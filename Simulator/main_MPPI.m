clear
close all
clc

%simulation parameters
x0_c = [-0.05;0.0;-pi*1/2;0]*1;
tspan = 5;
dt = 0.05;
N = tspan/dt;

%Define Dynamical system
% sys = Pendulum();

pusher = PointPusher();
object = Square();
surface = Surface();
planar_system = PlanarSystem(pusher, object, surface);
planner = Planner(planar_system, 'Straight', 0.05); 
sys = Simulator(planar_system, 'MPPI_nonlinear_error');

x0 = planar_system.coordinateTransformCS(x0_c);

%Define controller object
MPPI = MPPI(.5, 0.05, 300, 1, diag([.005,.005]), @sys.get_next_state_b, @sys.q_cost, @sys.phi_cost, planner.t_star, planner.xs_star, planner.us_star);
x = zeros(N+1, length(x0));
x(1,:) = x0';
sys.initialize_plot(x0, planner.xs_star(1,:)');
t = zeros(N+1,1);

MPC_trajectories = {};

for i=1:N
    t(i)
    [x_star, u_star] = MPPI.find_nominal_state(t(i));
    %get action 
    u = MPPI.controller(x(i,:), t(i));
    
    %get next state�
    x(i+1,:) = sys.get_next_state_b(x(i,:)', u, dt);
    t(i+1) = t(i) + dt;
    
    MPC_trajectories{i} = MPPI.X;
    MPC_best{i} = MPPI.x;
    
    sys.update_plot(x(i+1,:), MPPI, t(i));
end

sys.xs_state = x;
sys.t = t;

sys.Animate1pt(planner.xs_star, MPC_trajectories, MPC_best, MPPI)