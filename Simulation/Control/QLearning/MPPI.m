classdef MPPI < dynamicprops
    %TODO: Add 
    properties (Constant)
        K = 300;
        T = 1.5;
        dt = 0.05;
        Sigma =  [.9];%.9;
        lambda = 1;
    end

    properties
        N;
        U;
        X;
        F;
        u;
        x;
    end

    methods
        function obj = MPPI(F)
            obj.F = F;
            obj.N =  obj.T/obj.dt;
            obj.u = zeros(obj.F.a_length, obj.N);
        end
        
        function u_control = controller(obj, x0)
            
            %1. Sample deviations in action (Loop over K action sequences)
            S = zeros(obj.K,1);
            u = obj.u;
            x = zeros(obj.F.x_length, obj.N);
            for k=1:obj.K
                x(:, 1) = x0';
                %rng default  % For reproducibility
                E{k} = mvnrnd(zeros(obj.F.a_length,1), obj.Sigma, obj.N)';
                %Initialize Error
                S(k) = 0;
                %Rollout dynamics and compute cost
                for n=2:obj.N+1
                    xn = obj.F.get_next_state(x(:, n-1)', u(:,n-1)' + E{k}(:,n-1)', obj.dt, 1);
                    x(:, n) = xn';
                    S(k) = S(k) + obj.q_cost(x(:,n)) + obj.lambda*u(:, n-1)'*inv(obj.Sigma)*E{k}(:,n-1);
                end
                S(k) = S(k) + obj.phi_cost(x(:,obj.N+1));
                obj.U{k} = u + E{k};
                obj.X{k} = x;
            end
            
            [beta, index] = min(S);
            eta = 0;
            for k=1:obj.K
                eta = eta + exp(-1/obj.lambda *(S(k)-beta));
            end
            
            for k=1:obj.K
                w(k) = (1/eta)*exp(-1/obj.lambda *(S(k)-beta));
            end   
            
            for n=1:obj.N
                for k=1:obj.K
                    u(:, n) = u(:, n) + w(k)*E{k}(:, n);
                end
            end
            
            %save best state and action
            obj.x = obj.X{index};
            obj.u = obj.U{index};
            
            for n=2:obj.N
                u(:, n-1) = u(:, n);
            end
            u(:, n-1) = 0;
            obj.u = u;
            
            u_control = u(:,1);
  
        end
        
        function q = q_cost(obj,x)
            q = x'*diag([1,1])*x;
        end
        
        function phi = phi_cost(obj,x)
            phi = x'*diag([50,50])*x;
        end
        
        function video_setup()           
            if obj.doVid
                obj.writerObj = VideoWriter('qlearnVid.mp4','MPEG-4');
                obj.writerObj.FrameRate = 60;
                open(obj.writerObj);
            end
        end

    end
end