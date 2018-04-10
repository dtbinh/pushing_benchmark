classdef QlearnPend < dynamicprops
    %TODO: Add 
    properties (Constant)
        %%% Hyperparameters
        learnRate = 0.99; % How is new value estimate weighted against the old (0-1). 1 means all new and is ok for no noise situations.
        epsilonDecay = 0.98; % Decay factor per iteration.
        discount = 0.9; % When assessing the value of a state & action, how important is the value of the future states?
        successRate = 1; % How often do we do what we intend to do?
        maxEpi = 2000; % Each episode is starting with the pendulum down and doing continuous actions for awhile.
        maxit = 1500; % Iterations are the number of actions taken in an episode.
        substeps = 2; % Number of physics steps per iteration (could be 1, but more is a little better integration of the dynamics)
        dt = 0.05; % Timestep of integration. Each substep lasts this long
        winBonus = 100;  % Option to give a very large bonus when the system reaches the desired state (pendulum upright).
        doVid = false;
        transpMap = true;
    end
    
    properties
        MDP;
        states;
        actions;
        states_index;
        actions_index;
        epsilon = 0.5; % Initial value
        writerObj;
        num_states;
        num_actions;
        x;
        a;
        Q;
    end
    
    methods
        %% Example reinforcement learning - Q-learning code
        function obj = QlearnPend(MDP)
            obj.MDP = MDP;
            obj.x{1} = linspace(-0.1, 0.25, 100);
            obj.x{2} = linspace(-0.2, 0.2, 20);
            obj.a{1} = linspace(0, 0.1, 3);
            obj.a{2} = linspace(-.05,.05,3);
        end
        
        
        function sIdx = get_state_index(obj, z1)
            [~,sIdx] = min(sum((obj.states - repmat(z1,[size(obj.states,1),1])).^2,2));
        end
                
        function [T, aIdx] = get_action(obj,Q, sIdx, episodes)
            if (rand()>obj.epsilon || episodes == obj.maxEpi) && rand()<=obj.successRate % Pick according to the Q-matrix it's the last episode or we succeed with the rand()>epsilon check. Fail the check if our action doesn't succeed (i.e. simulating noise)
                    [~,aIdx] = max(Q(sIdx,:)); % Pick the action the Q matrix thinks is best!
            else
                aIdx = randi(length(obj.actions),1); % Random action!
            end
                T = obj.actions(aIdx);
        end
        
        function obj = generate_states(obj)
            %define state space
            x_size = length(obj.x);
            state_length  = 1;
            x_length = {};
            a_lenght = {};
            for i=1:length(obj.x)
                state_length = state_length*length(obj.x{i});
                x_length{i} = 1:length(obj.x{i});
            end
            %define action space
            a_size = length(obj.a);
            action_length  = 1;
            for i=1:a_size(1)
               action_length = action_length*length(obj.a{i});
               a_length{i} = 1:length(obj.a{i});
            end
            
            obj.states=obj.combine_states(obj.x);%zeros(state_length,x_size(1)); % 2 Column matrix of all possible combinations of the discretized state.
            obj.actions=obj.combine_states(obj.a);%zeros(action_length,a_size(1));
            
            obj.states_index = obj.combine_states(x_length);
            obj.actions_index = obj.combine_states(a_length);
 
        end
        
        function video_setup()           
            if obj.doVid
                obj.writerObj = VideoWriter('qlearnVid.mp4','MPEG-4');
                obj.writerObj.FrameRate = 60;
                open(obj.writerObj);
            end
        end
        
        function out_reverse = combine_states(obj, input)

            reverse_input = {};
            for i=1:length(input)
                reverse_input{length(input)+1-i} = input{i};
            end
            
            out = allcomb(reverse_input);
            
            out_reverse = [];
            for i=1:size(out, 2)
                out_reverse(:, size(out, 2)+1-i) = out(:,i);
            end
            
        end
              
        
        function obj = main(obj, x0, xf)
        close all;

        %% Discretize state
        obj.generate_states();
        
        %%Initialize Reward Matrix
        R = obj.MDP.get_reward(obj.states(:,1),obj.states(:,2), xf(1), xf(2)); % Initialize the "cost" of a given state to be quadratic error from the goal state. Note the signs mean that -angle with +velocity is better than -angle with -velocity

        %% Initialize Q function
        obj.Q = zeros(length(obj.states), length(obj.actions));%repmat(R,[1,3]); % Q is length(x1) x length(x2) x length(actions) - IE a bin for every action-state combination.

        % initialize plot of value function
%         [f, panel, pathmap, map, Vorig] = obj.MDP.initialize_plot(Q, R, obj.states, obj.x{1}, obj.x{2});

        %% Start learning!

        % Number of episodes or "resets"
        for episodes = 1:obj.maxEpi
            episodes
            z1 = x0; % Reset the pendulum on new episode.
            % Number of actions we're willing to try before a reset
            for g = 1:obj.maxit
                %discretize staet
                sIdx = obj.get_state_index(z1);
                %% PICK AN ACTION
                [T, aIdx] = obj.get_action(obj.Q, sIdx, episodes);
                %simulate dynamics
                z2 = obj.MDP.get_(z1, T, obj.dt, obj.substeps);
                %% UPDATE Q-MATRIX
                [bonus, success] = obj.MDP.get_terminal_state(z2, obj.winBonus);
                %discretize next state
                snewIdx = obj.get_state_index(z2); 
                % Update Q
                if episodes ~= obj.maxEpi % On the last iteration, stop learning and just execute. Otherwise...
                    obj.Q(sIdx,aIdx) = obj.Q(sIdx,aIdx) + obj.learnRate * ( R(snewIdx) + obj.discount*max(obj.Q(snewIdx,:)) - obj.Q(sIdx,aIdx) + bonus );
                end

                z1 = z2; % Old state = new state
                obj.epsilon = obj.epsilon*obj.epsilonDecay;
%                 obj.MDP.update_plot(z1, obj.x{1}, obj.x{2}, snewIdx,  Q, Vorig, episodes, f, pathmap, map, obj.transpMap, obj.doVid)
                % End this episode if we've hit the goal point (upright pendulum).
                if success
                    break;
                end

            end
        end
% 
%         if obj.doVid
%             close(obj.writerObj);
%         end

        end

    end
end
