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
    end

    methods
        function obj = MPPI(T, dt, K, lambda, Sigma, F, q_cost, phi_cost, t_star, x_star, u_star)
            obj.K = K;
            obj.T = T;
            obj.dt = dt;
            obj.q_cost = q_cost;
            obj.phi_cost = phi_cost;
            obj.lambda = lambda;
            obj.F = F;
            obj.N =  floor(obj.T/obj.dt);
            obj.u = zeros(length(Sigma), obj.N);%[.05*ones(1, obj.N); zeros(1, obj.N)];
            obj.Sigma =  Sigma; %1*diag([.005,.015]);%.05*eye(obj.F.a_length); 
            obj.t_star = t_star;
            obj.x_star = x_star;
            obj.u_star = u_star;

        end
        
        function [state, action] = find_nominal_state(obj, t)
            diff = abs(obj.t_star - t);
            [val ind] = min(diff);
            state = obj.x_star(ind,:);
            action = obj.u_star(ind,:);
        end
        
        function u_control = controller(obj, x0, t0)
            
            %1. Initialize variables
            S = zeros(obj.K,1);
            u = obj.u;
            x = zeros(size(obj.x_star,2), obj.N);
            %Loop through sample trajectories
            for k=1:obj.K
                x(:, 1) = x0';
                %rng default  % For reproducibility
                E{k} = mvnrnd(zeros(length(obj.Sigma),1), obj.Sigma, obj.N)';
                %Rollout dynamics and compute cost
                t = t0;
                for n=2:obj.N+1
                    [xd, ud] = obj.find_nominal_state(t);
                    xn = obj.F(x(:, n-1), u(:,n-1) + E{k}(:,n-1), xd', ud', obj.dt);
                    x(:, n) = xn';
                    S(k) = S(k) + obj.q_cost(x(:,n),u(:,n-1),xd') + 1*obj.lambda*u(:, n-1)'*inv(obj.Sigma)*E{k}(:,n-1);
                    t = t + obj.dt;
                end
                S(k) = S(k) + obj.phi_cost(x(:,obj.N+1),u(:,n-1),xd');
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
                [xd, ud] = obj.find_nominal_state(t);
                xn = obj.F(x(:, n-1), u(:,n-1), xd', ud', obj.dt);
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
            u(:, n-1) = 0;
            obj.u = u;
        end

    end
end