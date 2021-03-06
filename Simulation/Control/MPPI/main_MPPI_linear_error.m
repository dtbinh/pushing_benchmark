clear
close all
clc

%simulation parameters
x0_c = [-0.05;0.0;-pi*.5;0]*1;
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
sys = Simulator(planar_system, 'MPPI_linear_error_1');
controller = Controller(planner, 'is_fom');

x0 = planar_system.coordinateTransformCS(x0_c);

%Define controller object
MPPI = MPPI(.5, 0.05, 300, .1, diag([.0005,.005]), @sys.get_next_state_nonlinear_error_b, @sys.q_cost_nonlinear_error, @sys.phi_cost_nonlinear_error, @sys.u_constraints_nonlinear_error);

x = zeros(N+1, length(x0));
x(1,:) = x0';
sys.x_star = planner.xs_star;
sys.u_star = planner.us_star;
sys.t_star = planner.t_star;
sys.initialize_plot(x0, planner.xs_star(1,:)', MPPI);
t = zeros(N+1,1);

MPC_trajectories = {};

for i=1:N
%     t(i)
    [x_star, u_star] = sys.find_nominal_state(t(i));
    %
    x_bar = x(i,:) - x_star';
    %get action 
    u_bar = MPPI.controllerFOM(x_bar, t(i), @sys.find_nominal_state, @controller.solveFOM_delta, planar_system);
    %
    [xd, ud] = find_nominal_state(t(i));
    u_tilde = solveFOM(coordinateTransformSC(x(i,:)), t(i)); 
    u_bar = sys.u_constraints_nonlinear_error(x_bar', u_bar + u_tilde, t(i), true);
    
    u = u_star + u_bar;
    
    %get vector of nominal states
    x_des = zeros(floor(MPPI.T/MPPI.dt), length(x(i,:)));
    t_tmp = t(i);
    for lv1=1:floor(MPPI.T/MPPI.dt)
        [x_star, u_star] = sys.find_nominal_state(t_tmp);
        x_des(lv1,:) = x_star;
        t_tmp = t_tmp + MPPI.dt;
    end
    %get next state�
    x(i+1,:) = sys.get_next_state_b(x(i,:)', u, dt);
    t(i+1) = t(i) + dt;
    
    MPC_trajectories{i} = MPPI.X;
    MPC_best{i} = MPPI.x;
    weights{i} = MPPI.w;
    des_traj{i} = x_des;
 
    sys.update_plot_nonlinear_error(x(i+1,:), MPPI, x_des, t(i));
end

sys.xs_state = x;
sys.t = t;

sys.Animate1pt(planner.xs_star, MPC_trajectories, MPC_best, MPPI, weights, des_traj)