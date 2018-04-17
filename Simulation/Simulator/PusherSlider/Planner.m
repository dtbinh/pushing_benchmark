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
        A_star;
        B_star;
        Linear;
        data;
        object;
        simulator;
    end
    methods
        %% Constructor
        function obj = Planner(ps, simulator, Linear, data, object, name, vel)  
            obj.ps = ps;
            obj.Linear=Linear;
            obj.data=data;
            obj.object=object;
            obj.simulator=simulator;
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
            [obj.A_star{1}, obj.B_star{1}] = GP_linearization(obj.xc_star(1,:)', obj.us_star(1,1:2)', obj.Linear, obj.data, obj.object);
            for lv1=1:N_star
                lv1
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
                [obj.A_star{lv1+1}, obj.B_star{lv1+1}] = GP_linearization(obj.xc_star(lv1+1,:)', obj.us_star(lv1+1,1:2)', obj.Linear, obj.data, obj.object);

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
            A_star = obj.A_star;
            B_star = obj.B_star;
            t_star = obj.t_star;
            x0(3) = x0(3)-2*pi;
            obj.buildCircleTrajectory(-radius, velocity, x0);
            obj.xs_star = [xs_star; obj.xs_star];
            obj.us_star = [us_star; obj.us_star];
            obj.xc_star = [xc_star; obj.xc_star];
            obj.uc_star = [uc_star; obj.uc_star];
            obj.t_star = [t_star; t_star(end) + obj.t_star];
            obj.A_star = [A_star;obj.A_star];
            obj.B_star = [B_star;obj.B_star];
%             obj.t_star = [t_star; t_star(end) + obj.t_star];
%             x0 = x0_initial;
%             %repeat same operation
%             obj.buildCircleTrajectory(radius, velocity, x0);
%             xs_star = obj.xs_star;
%             us_star = obj.us_star;
%             xc_star = obj.xc_star;
%             uc_star = obj.uc_star;
%             t_star = [t_star; t_star(end) + obj.t_star];
%             x0(3) = x0(3)-2*pi;
%             obj.buildCircleTrajectory(-radius, velocity, x0);
%             obj.xs_star = [xs_star; obj.xs_star];
%             obj.us_star = [us_star; obj.us_star];
%             obj.xc_star = [xc_star; obj.xc_star];
%             obj.uc_star = [uc_star; obj.uc_star];
%             obj.A_star = {A_star,obj.A_star};
%             obj.B_star = {B_star,obj.B_star};
%             obj.t_star = [t_star; t_star(end) + obj.t_star];
% x0 = x0_initial;
%repeat same operation
%             obj.buildCircleTrajectory(radius, velocity, x0);
%             xs_star = obj.xs_star;
%             us_star = obj.us_star;
%             xc_star = obj.xc_star;
%             uc_star = obj.uc_star;
%             t_star = [t_star; t_star(end) + obj.t_star];
%             x0(3) = x0(3)-2*pi;
%             obj.buildCircleTrajectory(-radius, velocity, x0);
%             obj.xs_star = [xs_star; obj.xs_star];
%             obj.us_star = [us_star; obj.us_star];
%             obj.xc_star = [xc_star; obj.xc_star];
%             obj.uc_star = [uc_star; obj.uc_star];
%             obj.t_star = [t_star; t_star(end) + obj.t_star];
% x0 = x0_initial;                        %repeat same operation
%             obj.buildCircleTrajectory(radius, velocity, x0);
%             xs_star = obj.xs_star;
%             us_star = obj.us_star;
%             xc_star = obj.xc_star;
%             uc_star = obj.uc_star;
%             t_star = [t_star; t_star(end) + obj.t_star];
%             x0(3) = x0(3)-2*pi;
%             obj.buildCircleTrajectory(-radius, velocity, x0);
%             obj.xs_star = [xs_star; obj.xs_star];
%             obj.us_star = [us_star; obj.us_star];
%             obj.xc_star = [xc_star; obj.xc_star];
%             obj.uc_star = [uc_star; obj.uc_star];
%             obj.t_star = [t_star; t_star(end) + obj.t_star];
% x0 = x0_initial;
% %repeat same operation
%             obj.buildCircleTrajectory(radius, velocity, x0);
%             xs_star = obj.xs_star;
%             us_star = obj.us_star;
%             xc_star = obj.xc_star;
%             uc_star = obj.uc_star;
%             t_star = [t_star; t_star(end) + obj.t_star];
%             x0(3) = x0(3)-2*pi;
%             obj.buildCircleTrajectory(-radius, velocity, x0);
%             obj.xs_star = [xs_star; obj.xs_star];
%             obj.us_star = [us_star; obj.us_star];
%             obj.xc_star = [xc_star; obj.xc_star];
%             obj.uc_star = [uc_star; obj.uc_star];
%             obj.t_star = [t_star; t_star(end) + obj.t_star];
% x0 = x0_initial;
            
        end
          function obj = build_inf_circle(obj, radius, velocity, x0)
            obj.buildCircleTrajectory(radius, velocity, x0);
            xs_star = obj.xs_star;
            us_star = obj.us_star;
            xc_star = obj.xc_star;
            uc_star = obj.uc_star;
            t_star = obj.t_star;
            x0 = xc_star(end,:)';
%             obj.buildCircleTrajectory(radius, velocity, x0);
%             %repeat same operation
%             xs_star = [xs_star; obj.xs_star ];
%             us_star = [us_star; obj.us_star];
%             xc_star = [xc_star; obj.xc_star];
%             uc_star = [uc_star; obj.uc_star];
%             t_star  = [t_star; t_star(end) + obj.t_star];
%             x0 = xc_star(end,:)';
%             obj.buildCircleTrajectory(radius, velocity, x0);
%             %repeat same operation
%             xs_star = [xs_star; obj.xs_star];
%             us_star = [us_star; obj.us_star];
%             xc_star = [xc_star; obj.xc_star];
%             uc_star = [uc_star; obj.uc_star];
%             t_star = [t_star; t_star(end) + obj.t_star];
%             x0 = xc_star(end,:)';
%             obj.buildCircleTrajectory(radius, velocity, x0);
%             %repeat same operation
%             xs_star = [xs_star; obj.xs_star];
%             us_star = [us_star; obj.us_star];
%             xc_star = [xc_star; obj.xc_star];
%             uc_star = [uc_star; obj.uc_star];
%             t_star = [t_star; t_star(end) + obj.t_star];
            
            obj.xs_star=xs_star;
            obj.us_star=us_star;
            obj.xc_star=xc_star;
            obj.uc_star=uc_star;
            obj.t_star=t_star;
          end
        
        function obj = build_inf_circle_vel_space(obj, radius, velocity, x0)
            obj.buildCircleTrajectory_vel_space(radius, velocity, x0);
            xs_star = obj.xs_star;
            us_star = obj.us_star;
            xc_star = obj.xc_star;
            uc_star = obj.uc_star;
            t_star = obj.t_star;
            x0 = xc_star(end,:)';
%             obj.buildCircleTrajectory(radius, velocity, x0);
%             %repeat same operation
%             xs_star = [xs_star; obj.xs_star ];
%             us_star = [us_star; obj.us_star];
%             xc_star = [xc_star; obj.xc_star];
%             uc_star = [uc_star; obj.uc_star];
%             t_star  = [t_star; t_star(end) + obj.t_star];
%             x0 = xc_star(end,:)';
%             obj.buildCircleTrajectory(radius, velocity, x0);
%             %repeat same operation
%             xs_star = [xs_star; obj.xs_star];
%             us_star = [us_star; obj.us_star];
%             xc_star = [xc_star; obj.xc_star];
%             uc_star = [uc_star; obj.uc_star];
%             t_star = [t_star; t_star(end) + obj.t_star];
%             x0 = xc_star(end,:)';
%             obj.buildCircleTrajectory(radius, velocity, x0);
%             %repeat same operation
%             xs_star = [xs_star; obj.xs_star];
%             us_star = [us_star; obj.us_star];
%             xc_star = [xc_star; obj.xc_star];
%             uc_star = [uc_star; obj.uc_star];
%             t_star = [t_star; t_star(end) + obj.t_star];
            
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
            [obj.A_star(1,:,:), obj.B_star(1,:,:)] = GP_linearization_u(obj.xc_star(1,:)', obj.uc_star(1,:)', obj.Linear, obj.data, obj.object);
            
            for lv1=1:N_star
                lv1
                %get object velocity
                xs = obj.ps.coordinateTransformCS(xc);
                us = obj.ps.force2Velocity(xc, uc);
                dxs = obj.simulator.pointSimulatorGP(xs,us);
%                 dxs = obj.ps.forceSimulator(xc, uc);
%                 dxs = obj.pointSimulatorGP(xs,us);
                 dxc= [dxs(1:3); 0];
                 xs = xs + h_star*dxs;
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
                [obj.A_star(lv1+1,:,:), obj.B_star(lv1+1,:,:)] = GP_linearization_u(obj.xc_star(lv1+1,:)', obj.uc_star(lv1+1,:)', obj.Linear, obj.data, obj.object);
                if lv1<N_star
                    obj.t_star(lv1+1)  = obj.t_star(lv1) + h_star;
                end
            end
        end
        
        %% Build nominal trajectory
        function obj = buildCircleTrajectory_vel_space(obj, radius, velocity, x0)
            
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
            Cbi = Helper.C3_2d(xc(3));
            usStarTemp = Cbi*usStarTemp;
            obj.xs_star(1,:) = xsStarTemp';
            obj.us_star(1,:) = usStarTemp';
            obj.xc_star(1,:) = xc';
            obj.uc_star(1,:) = usStarTemp';
            [obj.A_star(1,:,:), obj.B_star(1,:,:)] = GP_linearization(obj.xc_star(1,:)', obj.us_star(1,1:2)', obj.Linear, obj.data, obj.object);
            
            for lv1=1:N_star
                lv1
                %get object velocity
                dxs = obj.ps.forceSimulator(xc, uc);
                dxc= [dxs(1:3); 0];
                xc = xc + h_star*dxc;
                
                %convert to state coordinates
                xcStarTemp = xc;
                ucStarTemp = uc;
                xsStarTemp = obj.ps.coordinateTransformCS(xcStarTemp);
                usStarTemp = obj.ps.force2Velocity(xcStarTemp, ucStarTemp);
                Cbi = Helper.C3_2d(xc(3));
                usStarTemp = Cbi*usStarTemp;
                %Build control matrices A and B (symbolic linearization of motion
                %equations)
                obj.xs_star(lv1+1,:) = xsStarTemp';
                obj.us_star(lv1+1,:) = usStarTemp';
                obj.xc_star(lv1+1,:) = xcStarTemp';
                obj.uc_star(lv1+1,:) = usStarTemp';
                [obj.A_star(lv1+1,:,:), obj.B_star(lv1+1,:,:)] = GP_linearization(obj.xc_star(lv1+1,:)', obj.us_star(lv1+1,1:2)', obj.Linear, obj.data, obj.object);
                if lv1<N_star
                    obj.t_star(lv1+1)  = obj.t_star(lv1) + h_star;
                end
            end
            obj.uc_star(lv1+1,:) = obj.us_star(lv1+1,:);
        end
                %% Build nominal trajectory
        function obj = buildCircleTrajectory_gp(obj, radius, velocity, x0)
            
            xc = x0;
            uc = zeros(obj.ps.p.num_contacts * 2 + 1, 1);
            tf = abs(2*pi*radius)/velocity;
            t0 = 0;
            h_star = 0.01;
            N_star = (1/h_star)*(tf-t0);
            obj.t_star = zeros(floor(N_star), 1);
            %Perform inverse dynamics
            nominal_states = obj.inverseDynamics_gp(radius, velocity);
            us = nominal_states(1:2);
            uc=us;
            xc(4) = nominal_states(3);
            %initial state
            xsStarTemp = obj.ps.coordinateTransformCS(xc);
            xs = xsStarTemp;
            Cbi = Helper.C3_2d(xc(3));
            usStarTemp = us;
            obj.xs_star(1,:) = xsStarTemp';
            obj.us_star(1,:) = usStarTemp';
            obj.xc_star(1,:) = xc';
            obj.uc_star(1,:) = uc';
            [obj.A_star{1}, obj.B_star{1}] = GP_linearization(obj.xc_star(1,:)', obj.us_star(1,1:2)', obj.Linear, obj.data, obj.object);
            
            for lv1=1:N_star
                lv1
                %get object velocity
                dxs = obj.simulator.pointSimulatorGP(obj.xs_star(lv1,:)', Cbi'*us);
%                 xs = xs+h_star*dxs;
%                 xc = obj.ps.coordinateTransformSC(xs);
                dxc= [dxs(1:3); 0];
                xc = xc + h_star*dxc;
                xs = obj.ps.coordinateTransformCS(xc);
                %convert to state coordinates
                xcStarTemp = xc;
                ucStarTemp = uc;
                xsStarTemp = xs;
                usStarTemp = us;
                Cbi = Helper.C3_2d(xc(3));
%                 usStarTemp = Cbi*usStarTemp;
                %Build control matrices A and B (symbolic linearization of motion
                %equations)
                obj.xs_star(lv1+1,:) = xsStarTemp';
                obj.us_star(lv1+1,:) = usStarTemp';
                obj.xc_star(lv1+1,:) = xcStarTemp';
                obj.uc_star(lv1+1,:) = ucStarTemp';
%                 obj.uc_star(lv1+1,4)=0.009;
                [obj.A_star{lv1+1}, obj.B_star{lv1+1}] = GP_linearization(obj.xc_star(lv1+1,:)', obj.us_star(lv1+1,1:2)', obj.Linear, obj.data, obj.object);
                if lv1<N_star
                    obj.t_star(lv1+1)  = obj.t_star(lv1) + h_star;
                end
            end
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
        function x = inverseDynamics_gp(obj, radius, velocity)
            x0 = zeros(3,1);
           [x_object, dx_object] = obj.circularVelocities(radius, velocity);
            x = fmincon(@(x)obj.cost_gp(x, x_object, dx_object),x0,[],[],[],[],[],[],@(x)obj.nonlcon_gp(x, x_object, dx_object));
            
        end
        %% Inverse dynamics 
        function [cost] = cost_gp(obj,x, x_object, dx_object)
                        %extract variables
            vn = x(1);
            vt = x(2);
            ry = x(3);
            %controller state
            xc = [x_object;ry];
            xs = obj.ps.coordinateTransformCS(xc);
            us = [vn;vt];
            %motion equations
%             f_non = obj.ps.twist_object_i_fun(xc,uc);
            f_non = obj.simulator.pointSimulatorGP(xs, us);
%             c = [-fn; ft-obj.ps.p.nu_p*fn];
            cost = norm([dx_object - f_non(1:3)]);
        end
        %% Inverse dynamics 
        function [c, ceq] = nonlcon_gp(obj,x, x_object, dx_object)
            %extract variables
            vn = x(1);
            vt = x(2);
            ry = x(3);
            %controller state
            xc = [x_object;ry];
            xs = obj.ps.coordinateTransformCS(xc);
            us = [vn;vt];
            %motion equations
%             f_non = obj.ps.twist_object_i_fun(xc,uc);
%             f_non = obj.simulator.pointSimulatorGP(xs, us);
             c = [];
%             ceq = [dx_object - f_non];
            ceq = [ry-0.009];
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
