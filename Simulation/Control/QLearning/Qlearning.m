classdef Qlearning < dynamicprops
    %TODO: Add 
    properties (Constant)
        alpha = 0.01;
        gamma = 0.9;
        epsilon = 0.1;
    end
    
    properties
        ps;
        S0;
        S_terminal;
        Q;
        simulator;
        Ax_bar_set;
        Ay_bar_set;
        Sx_bar_set;
        Sy_bar_set;
        Sry_bar_set;
        
    end
    
    methods
        %% Constructor
        function obj = Qlearning(simulator, ps, S0, Sf)  
            obj.ps = ps;
            obj.S0 = S0;
            obj.S_terminal = Sf;
            obj.simulator = simulator;
            obj.Ax_bar_set = linspace(0,0.1,4);
            obj.Ay_bar_set = linspace(-0.1,0.1,4);
            obj.Sx_bar_set = linspace(0,0.5,500);
            obj.Sy_bar_set = linspace(-0.1,0.1,500);
            obj.Sry_bar_set = linspace(-0.0450,0.0450,500);
            obj.Q = zeros(length(obj.Sx_bar_set),length(obj.Sy_bar_set),length(obj.Sry_bar_set),length(obj.Ax_bar_set),length(obj.Ay_bar_set));
            obj.q_learning()
        end
        
        function out = A_discretize(obj,value)
            [val1, ind1] = min(abs(value(1)-obj.Ax_bar_set));
            [val2, ind2] = min(abs(value(2)-obj.Ay_bar_set));
            out = [obj.Ax_bar_set(ind1) obj.Ay_bar_set(ind2)];
        end
        
        function out = S_discretize(obj,value)
            [val1, ind1] = min(abs(value(1)-obj.Sx_bar_set));
            [val2, ind2] = min(abs(value(2)-obj.Sy_bar_set));
            [val3, ind3] = min(abs(value(3)-obj.Sry_bar_set));
            out = [obj.Sx_bar_set(ind1) obj.Sy_bar_set(ind2) obj.Sry_bar_set(ind3)];
        end

        function obj = q_learning(obj)
            counter = 1;
            for episode=1:200
                disp(episode);
                S_bar = obj.S_discretize([0.0,0.1,0.02]*rand(1));
                for i =1:10000
                
                    A_bar = obj.choose_action(S_bar);
                    S_bar_next = obj.S_discretize(obj.get_next_state(S_bar,A_bar));
                    R = obj.get_reward(S_bar_next,A_bar);
                    state_index = obj.get_state_index(S_bar);
                    action_index = obj.get_action_index(A_bar);
                    state_next_index =  obj.get_state_index(S_bar);
                    obj.Q(state_index(1),state_index(2),state_index(3),action_index(1),action_index(2)) = R + obj.alpha*(R + obj.gamma*obj.get_maxQ(S_bar_next) - obj.Q(state_index(1),state_index(2),state_index(3),action_index(1),action_index(2)));
                    
                    S_bar = S_bar_next;
                    if S_bar(1)>obj.S_terminal(1) 
                        break
                    end
                end
            end
            
            disp(1);
            
        end
        
        function out = get_action_index(obj,A)
            [value, index1] = min(abs(obj.Ax_bar_set-A(1)));
            [value, index2] = min(abs(obj.Ay_bar_set-A(2)));
            out = [index1, index2];        
        end
       
        function out = get_state_index(obj, S)
            [value, index1] = min(abs(obj.Sx_bar_set-S(1)));
            [value, index2] = min(abs(obj.Sy_bar_set-S(2)));
            [value, index3] = min(abs(obj.Sry_bar_set-S(3)));
            out = [index1, index2, index3];
        end
        
        function out = get_maxQ(obj, S_next)
            Q = zeros(length(obj.Ax_bar_set), length(obj.Ay_bar_set));
            for counter=1:length(obj.Ax_bar_set)
                for counter2=1:length(obj.Ay_bar_set)
                    state_next_index = obj.get_state_index(S_next);
                    Q(counter, counter2) = obj.Q(state_next_index(1), state_next_index(2), counter, counter2);
                end
            end
            %find best action
            [value, index] = min(Q(:));
            [I,J] = ind2sub([size(Q,1) size(Q,2)],index);

            out = obj.Q(state_next_index(1), state_next_index(2), I, J);

        end
        
        
        function action = choose_action(obj, S)
            Q = zeros(length(obj.Ax_bar_set), length(obj.Ay_bar_set));
            for counter=1:length(obj.Ax_bar_set)
                for counter2=1:length(obj.Ay_bar_set)
                    index_vec = obj.get_state_index(S);
                    Q(counter, counter2) = obj.Q(index_vec(1), index_vec(2), counter, counter2);
                end
            end
            %find best action
            
            [value, index] = min(Q(:));
            [I,J] = ind2sub([size(Q,1) size(Q,2)],index);
            
            if rand(1)<obj.epsilon
                x_action_index = ceil(length(obj.Ax_bar_set)*rand(1));
                y_action_index = ceil(length(obj.Ay_bar_set)*rand(1));
                A = [obj.Ax_bar_set(x_action_index);obj.Ay_bar_set(y_action_index)];
            else
                A = [obj.Ax_bar_set(I);obj.Ay_bar_set(J)];
            end

            action = A;

        end

        function reward = get_reward(obj, S, A)
            reward = -(S-obj.S_terminal)*(S-obj.S_terminal)';

        end

        function S = get_next_state(obj, S, A)
%        
S
A
            dxs = obj.simulator.pointSimulator([S(1);S(2);0;S(1)-.09/2;S(2)+S(3)],A);
            S = S + obj.simulator.h*[dxs(1:2);dxs(5)]';  
            
        end

    end
    
end