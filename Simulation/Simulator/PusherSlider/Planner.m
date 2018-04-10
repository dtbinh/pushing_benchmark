classdef Planner < dynamicprops 
    % Contains all object properties
    properties (Constant)  
    end
    
    properties
        ps;
        t_star;
        xc_star;
        uc_star
        xs_star;
        us_star;
        uc_eq;  
        xc_eq;
    end
    methods
        %% Constructor
        function obj = Planner(ps, name, vel)  
            obj.ps = ps;
            if strcmp(name,'Straight')
                obj.buildStraightTrajectory();
            elseif strcmp(name, '8Track')
                obj.build8track(-0.15, vel, [0;0;0*pi/180;0]);
            elseif strcmp(name, 'inf_circle')
                obj.build_inf_circle(-0.15, vel, [0;0;0*pi/180;0]);
            end
%              obj.buildStraightTrajectory();
%              obj.buildCircleTrajectory(-0.15, 0.05, [0;0;90*pi/180;0]);
%              obj.build8track(-0.15, 0.05, [0;0;90*pi/180;0]);

             %               obj.buildOvalTrajectory();
%             obj.setNominalState();
        end
%         %% set nominal input
%         function obj = setNominalState(obj)
%             obj.xc_eq = obj.xc_star(1,:)';
%             obj.uc_eq = obj.uc_star(1,:)';
%         end
        %% Build nominal trajectory
        function obj = buildStraightTrajectory(obj)
            velocity = 0.05;
            radius = inf;
            tf = 25;
            t0 = 0;
            h_star = 0.01;
            N_star = (1/h_star)*(tf-t0);
            xc = zeros(4,1);
            uc = zeros(obj.ps.p.num_contacts * 2 + 1, 1);
            obj.t_star = zeros(N_star, 1);
            %Perform inverse dynamics
            nominal_states = obj.inverseDynamics(radius, velocity);
            fn = nominal_states(1:obj.ps.p.num_contacts);
            ft = nominal_states(obj.ps.p.num_contacts + 1: obj.ps.p.num_contacts * 2);
            ry = nominal_states(obj.ps.p.num_contacts * 2 + 1);
            uc = [fn; ft; 0];
            %initial state
            xsStarTemp = obj.ps.coordinateTransformCS(xc);
            usStarTemp = obj.ps.force2Velocity(xc, uc);
            obj.xs_star(1,:) = xsStarTemp';
            obj.us_star(1,:) = usStarTemp';
            obj.xc_star(1,:) = xc';
            obj.uc_star(1,:) = uc';
            for lv1=1:N_star
                %get object velocity
                dxs = obj.ps.forceSimulator(xc, uc);
                dxc= [dxs(1:3); 0];
                xc = xc + h_star*dxc;
                %convert to state coordinates
                xcStarTemp = xc;
                ucStarTemp = uc;
                xsStarTemp = obj.ps.coordinateTransformCS(xcStarTemp);
                usStarTemp = obj.ps.force2Velocity(xcStarTemp, ucStarTemp);
                %Build control matrices A and B (symbolic linearization of motion
                %equations)
                obj.xs_star(lv1+1,:) = xsStarTemp';
                obj.us_star(lv1+1,:) = usStarTemp';
                obj.xc_star(lv1+1,:) = xcStarTemp';
                obj.uc_star(lv1+1,:) = ucStarTemp';
                if lv1<N_star
                    obj.t_star(lv1+1)  = obj.t_star(lv1) + h_star;
                end
            end
        end
        
        function obj = build8track(obj, radius, velocity, x0)
            x0_initial = x0;
            obj.buildCircleTrajectory(radius, velocity, x0);
            xs_star = obj.xs_star;
            us_star = obj.us_star;
            xc_star = obj.xc_star;
            uc_star = obj.uc_star;
            t_star = obj.t_star;
            x0(3) = x0(3)-2*pi;
            obj.buildCircleTrajectory(-radius, velocity, x0);
            obj.xs_star = [xs_star; obj.xs_star];
            obj.us_star = [us_star; obj.us_star];
            obj.xc_star = [xc_star; obj.xc_star];
            obj.uc_star = [uc_star; obj.uc_star];
            obj.t_star = [t_star; t_star(end) + obj.t_star];
            x0 = x0_initial;
            %repeat same operation
            obj.buildCircleTrajectory(radius, velocity, x0);
            xs_star = obj.xs_star;
            us_star = obj.us_star;
            xc_star = obj.xc_star;
            uc_star = obj.uc_star;
            t_star = [t_star; t_star(end) + obj.t_star];
            x0(3) = x0(3)-2*pi;
            obj.buildCircleTrajectory(-radius, velocity, x0);
            obj.xs_star = [xs_star; obj.xs_star];
            obj.us_star = [us_star; obj.us_star];
            obj.xc_star = [xc_star; obj.xc_star];
            obj.uc_star = [uc_star; obj.uc_star];
            obj.t_star = [t_star; t_star(end) + obj.t_star];
x0 = x0_initial;
%repeat same operation
            obj.buildCircleTrajectory(radius, velocity, x0);
            xs_star = obj.xs_star;
            us_star = obj.us_star;
            xc_star = obj.xc_star;
            uc_star = obj.uc_star;
            t_star = [t_star; t_star(end) + obj.t_star];
            x0(3) = x0(3)-2*pi;
            obj.buildCircleTrajectory(-radius, velocity, x0);
            obj.xs_star = [xs_star; obj.xs_star];
            obj.us_star = [us_star; obj.us_star];
            obj.xc_star = [xc_star; obj.xc_star];
            obj.uc_star = [uc_star; obj.uc_star];
            obj.t_star = [t_star; t_star(end) + obj.t_star];
x0 = x0_initial;                        %repeat same operation
            obj.buildCircleTrajectory(radius, velocity, x0);
            xs_star = obj.xs_star;
            us_star = obj.us_star;
            xc_star = obj.xc_star;
            uc_star = obj.uc_star;
            t_star = [t_star; t_star(end) + obj.t_star];
            x0(3) = x0(3)-2*pi;
            obj.buildCircleTrajectory(-radius, velocity, x0);
            obj.xs_star = [xs_star; obj.xs_star];
            obj.us_star = [us_star; obj.us_star];
            obj.xc_star = [xc_star; obj.xc_star];
            obj.uc_star = [uc_star; obj.uc_star];
            obj.t_star = [t_star; t_star(end) + obj.t_star];
x0 = x0_initial;
%repeat same operation
            obj.buildCircleTrajectory(radius, velocity, x0);
            xs_star = obj.xs_star;
            us_star = obj.us_star;
            xc_star = obj.xc_star;
            uc_star = obj.uc_star;
            t_star = [t_star; t_star(end) + obj.t_star];
            x0(3) = x0(3)-2*pi;
            obj.buildCircleTrajectory(-radius, velocity, x0);
            obj.xs_star = [xs_star; obj.xs_star];
            obj.us_star = [us_star; obj.us_star];
            obj.xc_star = [xc_star; obj.xc_star];
            obj.uc_star = [uc_star; obj.uc_star];
            obj.t_star = [t_star; t_star(end) + obj.t_star];
x0 = x0_initial;
            
        end
        
        function obj = build_inf_circle(obj, radius, velocity, x0)
            obj.buildCircleTrajectory(radius, velocity, x0);
            xs_star = obj.xs_star;
            us_star = obj.us_star;
            xc_star = obj.xc_star;
            uc_star = obj.uc_star;
            t_star = obj.t_star;
            x0 = xc_star(end,:)';
            obj.buildCircleTrajectory(radius, velocity, x0);
            %repeat same operation
            xs_star = [xs_star; obj.xs_star ];
            us_star = [us_star; obj.us_star];
            xc_star = [xc_star; obj.xc_star];
            uc_star = [uc_star; obj.uc_star];
            t_star  = [t_star; t_star(end) + obj.t_star];
            x0 = xc_star(end,:)';
            obj.buildCircleTrajectory(radius, velocity, x0);
            %repeat same operation
            xs_star = [xs_star; obj.xs_star];
            us_star = [us_star; obj.us_star];
            xc_star = [xc_star; obj.xc_star];
            uc_star = [uc_star; obj.uc_star];
            t_star = [t_star; t_star(end) + obj.t_star];
            x0 = xc_star(end,:)';
            obj.buildCircleTrajectory(radius, velocity, x0);
            %repeat same operation
            xs_star = [xs_star; obj.xs_star];
            us_star = [us_star; obj.us_star];
            xc_star = [xc_star; obj.xc_star];
            uc_star = [uc_star; obj.uc_star];
            t_star = [t_star; t_star(end) + obj.t_star];
            
            obj.xs_star=xs_star;
            obj.us_star=us_star;
            obj.xc_star=xc_star;
            obj.uc_star=uc_star;
            obj.t_star=t_star;
        end

        %% Build nominal trajectory
        function obj = buildCircleTrajectory(obj, radius, velocity, x0)
            
            xc = x0;
            uc = zeros(obj.ps.p.num_contacts * 2 + 1, 1);
            tf = abs(2*pi*radius)/velocity;
            t0 = 0;
            h_star = 0.01;
            N_star = (1/h_star)*(tf-t0);
            obj.t_star = zeros(floor(N_star), 1);
            %Perform inverse dynamics
            nominal_states = obj.inverseDynamics(radius, velocity);
            fn = nominal_states(1:obj.ps.p.num_contacts);
            ft = nominal_states(obj.ps.p.num_contacts + 1: obj.ps.p.num_contacts * 2);
            ry = nominal_states(obj.ps.p.num_contacts * 2 + 1);
            uc = [fn; ft; 0];
            xc(4) = ry;
            %initial state
            xsStarTemp = obj.ps.coordinateTransformCS(xc);
            usStarTemp = obj.ps.force2Velocity(xc, uc);
            obj.xs_star(1,:) = xsStarTemp';
            obj.us_star(1,:) = usStarTemp';
            obj.xc_star(1,:) = xc';
            obj.uc_star(1,:) = uc';
            for lv1=1:N_star
                %get object velocity
                dxs = obj.ps.forceSimulator(xc, uc);
                dxc= [dxs(1:3); 0];
                xc = xc + h_star*dxc;
                
                %convert to state coordinates
                xcStarTemp = xc;
                ucStarTemp = uc;
                xsStarTemp = obj.ps.coordinateTransformCS(xcStarTemp);
                usStarTemp = obj.ps.force2Velocity(xcStarTemp, ucStarTemp);
                %Build control matrices A and B (symbolic linearization of motion
                %equations)
                obj.xs_star(lv1+1,:) = xsStarTemp';
                obj.us_star(lv1+1,:) = usStarTemp';
                obj.xc_star(lv1+1,:) = xcStarTemp';
                obj.uc_star(lv1+1,:) = ucStarTemp';
                if lv1<N_star
                    obj.t_star(lv1+1)  = obj.t_star(lv1) + h_star;
                end
            end
        end
        %% Build nominal trajectory
        function obj = buildOvalTrajectory(obj)
            
            obj.buildStraightTrajectory();
            xc_star_straight = obj.xc_star;
            uc_star_straight = obj.uc_star;
            xs_star_straight = obj.xs_star;
            us_star_straight = obj.us_star;
            t_star_straight = obj.t_star;
            
            xc_star_straight2 = -obj.xc_star + [0.2,-0.3,-3.1416,0];
            uc_star_straight2 = obj.uc_star;
            xs_star_straight2 = -obj.xs_star + [0.2,-0.3,-3.1416,0.2,-0.3,-3.1416];
            us_star_straight2 = -obj.us_star;
            
            obj.buildCircleTrajectory();
            xc_star_circle = obj.xc_star;
            uc_star_circle = obj.uc_star;
            xs_star_circle = obj.xs_star;
            us_star_circle = obj.us_star;
            t_star_circle = obj.t_star;
            
            xc_star_circle2 = obj.xc_star;
            uc_star_circle2 = obj.uc_star;
            xs_star_circle2 = obj.xs_star;
            us_star_circle2 = obj.us_star;
            t_star_circle = obj.t_star;
            
            t_star_oval = t_star_straight;
            xc_star_oval = zeros(2500,4);
            uc_star_oval = zeros(2500,5);
            xs_star_oval = zeros(2500,6);
            us_star_oval = zeros(2500,3);
            
            xc_star_oval(1:401,:) = xc_star_straight(1:401,:);
            uc_star_oval(1:401,:) = uc_star_straight(1:401,:);
            xs_star_oval(1:401,:) = xs_star_straight(1:401,:);
            us_star_oval(1:401,:) = us_star_straight(1:401,:);
            
            xc_star_oval(402:1345,:) = xc_star_circle(1:944,:) + [0.2 0 0 0];
            uc_star_oval(402:1345,:) = uc_star_circle(1:944,:);
            xs_star_oval(402:1345,:) = xs_star_circle(1:944,:)+ [0.2 0 0 0.2 0 0];
            us_star_oval(402:1345,:) = us_star_circle(1:944,:);
            
            xc_star_oval(1346:1746,:) = xc_star_straight2(1:401,:);
            uc_star_oval(1346:1746,:) = uc_star_straight2(1:401,:);
            xs_star_oval(1346:1746,:) = xs_star_straight2(1:401,:);
            us_star_oval(1346:1746,:) = us_star_straight2(1:401,:);
            
            xc_star_oval(1747:end,:) = xc_star_circle2(945:1698,:);
            uc_star_oval(1747:end,:) = uc_star_circle2(945:1698,:);
            xs_star_oval(1747:end,:) = xs_star_circle2(945:1698,:);
            us_star_oval(1747:end,:) = us_star_circle2(945:1698,:);
            
            obj.xc_star=xc_star_oval;
            obj.uc_star=uc_star_oval;
            obj.xs_star=xs_star_oval;
            obj.us_star=us_star_oval;
            
        end
        %% Inverse dynamics 
        function x = inverseDynamics(obj, radius, velocity)
            x0 = zeros(2*obj.ps.p.num_contacts+1,1);
           [x_object, dx_object] = obj.circularVelocities(radius, velocity);
            x = fmincon(@obj.cost,x0,[],[],[],[],[],[],@(x)obj.nonlcon(x, x_object, dx_object));
        end
        %% Inverse dynamics 
        function [cost] = cost(obj,x)
            cost=0; %dummy cost
        end
        %% Inverse dynamics 
        function [c, ceq] = nonlcon(obj,x, x_object, dx_object)
            %extract variables
            num_contacts = obj.ps.p.num_contacts;
            fn = x(1:num_contacts);
            ft = x(num_contacts+1: num_contacts*2);
            ry = x(num_contacts*2+1);
            %controller state
            xc = [x_object;ry];
            uc = [fn;ft;0];
            %motion equations
            f_non = obj.ps.twist_object_i_fun(xc,uc);
            c = [-fn; ft-obj.ps.p.nu_p*fn];
            ceq = [dx_object - f_non];
        end
        %% Inverse dynamics 
        function [x_object, dx_object] = circularVelocities(obj, radius, velocity)
            dx = velocity;
            dy = 0;
            dtheta = velocity/radius;
            x_object = [0;0;0];
            dx_object = [dx;dy;dtheta];
        end
        %%
    end
end
