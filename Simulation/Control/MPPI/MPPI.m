classdef MPPI < dynamicprops
    properties (Constant)
    end

    properties
        N;
        U;
        X;
        F;
        u;
        x;
        Sigma;
        t_star;
        x_star;
        u_star;
        K;
        T;
        dt;
        lambda;
        q_cost;
        phi_cost;
        u_constraints;
        w;
    end

    methods
        function obj = MPPI(T, dt, K, lambda, Sigma, F, q_cost, phi_cost, u_constraints)
            obj.K = K;
            obj.T = T;
            obj.dt = dt;
            obj.q_cost = q_cost;
            obj.phi_cost = phi_cost;
            obj.lambda = lambda;
            obj.F = F;
            obj.N =  floor(obj.T/obj.dt);
            obj.u = zeros(length(Sigma), obj.N);
            obj.Sigma =  Sigma; 
            obj.u_constraints = u_constraints;
        end    
        
        function u_control = controllerFOM(obj, x0, t0, find_nominal_state, solveFOM_delta, planner)
            %1. Initialize variables
            S = zeros(obj.K,1);
            u = obj.u;
            u_clamped = obj.u*0;
            u_desired = obj.u*0;
            u_tilde = obj.u*0;
            x_dummy = obj.F(x0', u_clamped(:,1), obj.dt, 0);
            x = zeros(length(x_dummy), obj.N);
            %Loop through sample trajectories
            for k=1:obj.K
                x(:, 1) = x0';
                %rng default  % For reproducibility
                E{k} = mvnrnd(zeros(length(obj.Sigma),1), obj.Sigma, obj.N)';
                %Rollout dynamics and compute cost
                t = t0;
                for n=2:obj.N+1
                    [xd, ud] = find_nominal_state(t);
                    x_nonlinear = x(:, n-1) + xd;
                    xc = planner.coordinateTransformSC(x_nonlinear);
                    delta_uc = solveFOM_delta(xc, t); 
                    u_tilde(:,n-1) = planner.force2Velocity(xc, uc);
                    u_desired(:,n-1) = u(:,n-1) + E{k}(:,n-1) + u_tilde(:,n-1);
                    u_clamped(:,n-1) = obj.u_constraints(x(:, n-1), u_desired(:,n-1), t, false);
                    xn = obj.F(x(:, n-1), u_clamped(:,n-1), obj.dt, t);
                    x(:, n) = xn';
                    S(k) = S(k) + obj.q_cost(x(:,n),u_clamped(:,n-1),t) + 1*obj.lambda*u(:, n-1)'*inv(obj.Sigma)*E{k}(:,n-1);
                    t = t + obj.dt;
                end
%                 [xd, ud] = obj.find_nominal_state(t);
                S(k) = S(k) + obj.phi_cost(x(:,obj.N+1),u_clamped(:,n-1),t);
                obj.U{k} = u + E{k};
                obj.X{k} = x;
            end
            %Find rollout with minimum cost
            [beta, index] = min(S);
            %Compute normalization constant eta
            eta = 0;
            for k=1:obj.K
                eta = eta + exp(-1/obj.lambda *(S(k)-beta));
            end
            
            %Get weights for each rollots trajectory
            for k=1:obj.K
                w(k) = (1/eta)*exp(-1/obj.lambda *(S(k)-beta));
            end   
            obj.w = w;
            %Get control input update
            for n=1:obj.N
                W = 0;
                for k=1:obj.K
                    W = W + w(k)*E{k}(:, n);
                end
                u(:, n) = u(:, n) + W;
            end
            %Generate trajectory rollout for computed action sequence
            t = t0;
            for n=2:obj.N+1
%                 [xd, ud] = obj.find_nominal_state(t);
%                 xn = obj.F(x(:, n-1), u(:,n-1), xd', ud', obj.dt);
                [xd, ud] = find_nominal_state(t);
                x_nonlinear = x(:, n-1) + xd;
                xc = planner.coordinateTransformSC(x_nonlinear);
                uc = solveFOM(xc, t); 
                u_tilde(:,n-1) = planner.force2Velocity(xc, uc);
                u_clamped(:,n-1) = obj.u_constraints(x(:, n-1), u(:,n-1) + u_tilde(:,n-1), t, false);
                xn = obj.F(x(:, n-1), u_clamped(:,n-1),  obj.dt, t);
                x(:, n) = xn';
                t = t + obj.dt;       
            end
            %save best state and action
            obj.x = x;%obj.X{index};
            obj.u = u;%obj.U{index};
            %send control command to actuators
            u_control = u(:,1) + u_tilde(:,1);%obj.U{index}(:,1)
            %Reinitialize controls for next controller solution      
            for n=2:obj.N
                u(:, n-1) = u(:, n);
            end
            u(:, obj.N) = 0;
            obj.u = u;
        end

        function u_control = controller(obj, x0, t0)
            %1. Initialize variables
            S = zeros(obj.K,1);
            u = obj.u;
            u_clamped = obj.u*0;
            x_dummy = obj.F(x0', u_clamped(:,1), obj.dt, 0);
            x = zeros(length(x_dummy), obj.N);
            %Loop through sample trajectories
            for k=1:obj.K
                x(:, 1) = x0';
                %rng default  % For reproducibility
                E{k} = mvnrnd(zeros(length(obj.Sigma),1), obj.Sigma, obj.N)';
                %Rollout dynamics and compute cost
                t = t0;
                for n=2:obj.N+1
%                     [xd, ud] = obj.find_nominal_state(t);
%                     xn = obj.F(x(:, n-1), u(:,n-1) + E{k}(:,n-1), xd', ud', obj.dt);
                    u_clamped(:,n-1) = obj.u_constraints(x(:, n-1), u(:,n-1) + E{k}(:,n-1), t, false);
                    xn = obj.F(x(:, n-1), u_clamped(:,n-1), obj.dt, t);
                    x(:, n) = xn';
                    S(k) = S(k) + obj.q_cost(x(:,n),u(:,n-1) + E{k}(:,n-1),t) + 1*obj.lambda*u(:, n-1)'*inv(obj.Sigma)*E{k}(:,n-1);
                    t = t + obj.dt;
                end
%                 [xd, ud] = obj.find_nominal_state(t);
                S(k) = S(k) + obj.phi_cost(x(:,obj.N+1),u_clamped(:,n-1),t);
                obj.U{k} = u + E{k};
                obj.X{k} = x;
            end
            %Find rollout with minimum cost
            [beta, index] = min(S);
            %Compute normalization constant eta
            eta = 0;
            for k=1:obj.K
                eta = eta + exp(-1/obj.lambda *(S(k)-beta));
            end
            
            %Get weights for each rollots trajectory
            for k=1:obj.K
                w(k) = (1/eta)*exp(-1/obj.lambda *(S(k)-beta));
            end   
            obj.w = w;
            %Get control input update
            for n=1:obj.N
                W = 0;
                for k=1:obj.K
                    W = W + w(k)*E{k}(:, n);
                end
                u(:, n) = u(:, n) + W;
            end
            %Generate trajectory rollout for computed action sequence
            t = t0;
            for n=2:obj.N+1
%                 [xd, ud] = obj.find_nominal_state(t);
%                 xn = obj.F(x(:, n-1), u(:,n-1), xd', ud', obj.dt);
                u_clamped(:,n-1) = obj.u_constraints(x(:, n-1), u(:,n-1), t, false);
                xn = obj.F(x(:, n-1), u_clamped(:,n-1),  obj.dt, t);
                x(:, n) = xn';
                t = t + obj.dt;
                
                    
                    
            end
            %save best state and action
            obj.x = x;%obj.X{index};
            obj.u = u;%obj.U{index};
            %send control command to actuators
            u_control = u(:,1);%obj.U{index}(:,1)
            %Reinitialize controls for next controller solution      
            for n=2:obj.N
                u(:, n-1) = u(:, n);
            end
            u(:, obj.N) = 0;
            obj.u = u;
        end

    end
end