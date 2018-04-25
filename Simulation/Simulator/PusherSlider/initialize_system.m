try
    run('../setup.m');
catch
end

%Define objects
% pusher = PointPusher(0.3);
pusher = LinePusher(0.3);
object = Square();
surface = Surface(0.35);
planar_system = PlanarSystem(pusher, object, surface);
simulator = Simulator(planar_system, simulation_name);

x0 = planar_system.coordinateTransformCS(x0_c);

%Build variables
t0 = 0;
tf = sim_time;
h_step = 0.05;

simulator.h = h_step;
simulator.N = floor((1/simulator.h)*(tf-t0));
simulator.t = zeros(simulator.N,1);
simulator.xs_state = zeros(simulator.N, planar_system.num_xsStates);
simulator.us_state = zeros(simulator.N, planar_system.num_usStates);
simulator.xs_state(1,:) = x0';
