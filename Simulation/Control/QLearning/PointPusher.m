classdef PointPusher < dynamicprops
    %TODO: Add 
    properties (Constant)

    end
    
    properties

    end
    
    methods
        function reward = get_reward(obj, x, y, xf, yf)
            reward = (-(abs(x-xf)).^2 + (abs(y-yf)).^2);
        end
        
        function z2 = get_next_state(obj, z1, T, dt, substeps)
            for i = 1:substeps
                k1 = obj.Dynamics(z1,T);
                k2 = obj.Dynamics(z1+dt/2*k1,T);
                k3 = obj.Dynamics(z1+dt/2*k2,T);
                k4 = obj.Dynamics(z1+dt*k3,T);

                z2 = z1 + dt/6*(k1 + 2*k2 + 2*k3 + k4);

            end
        end
        
        function zdot = Dynamics(obj, xs, us)
        % Pendulum with motor at the joint dynamics. IN - [angle,rate] & torque.
        % OUT - [rate,accel]
        g = 1;
        L = 1;
        z = z';
        zdot = [z(2) g/L*sin(z(1))+T];
        end
        
        function [bonus, success] = get_terminal_state(obj, z2, winBonus)
        % End condition for an episode
            if norm(z2)<0.02 % If we've reached upright with no velocity (within some margin), end this episode.
                success = true;
                bonus = winBonus; % Give a bonus for getting there.
            else
                bonus = 0;
                success = false;
            end
        end
        
        function [f, panel, pathmap, map, Vorig] = initialize_plot(obj, Q, R, states, x1, x2)
            V = zeros(size(states,1),1);

            Vorig = reshape(max(Q,[],2),[length(x2),length(x1)]);

            %% Set up the pendulum plot
            panel = figure;
            panel.Position = [680 558 1000 400];
            panel.Color = [1 1 1];
            subplot(1,4,1)

            hold on
            % Axis for the pendulum animation
            f = plot(0,0,'b','LineWidth',10); % Pendulum stick
            axPend = f.Parent;
            axPend.XTick = []; % No axis stuff to see
            axPend.YTick = [];
            axPend.Visible = 'off';
            axPend.Position = [0.01 0.5 0.3 0.3];
            axPend.Clipping = 'off';
            axis equal
            axis([-1.2679 1.2679 -1 1]);
            plot(0.001,0,'.k','MarkerSize',50); % Pendulum axis point

            hold off

            %% Set up the state-value map plot (displays the value of the best action at every point)
            colormap('hot');
            subplot(1,4,[2:4]);
            hold on
            map = imagesc(reshape(R,[length(x2),length(x1)]));
            axMap = map.Parent;
            axMap.XTickLabels = {'-pi' '0' 'pi'};
            axMap.XTick = [1 floor(length(x1)/2) length(x1)];
            axMap.YTickLabels = {'-pi' '0' 'pi'};
            axMap.YTick = [1 floor(length(x2)/2) length(x2)];
            axMap.XLabel.String = 'Angle (rad)';
            axMap.YLabel.String = 'Angular rate (rad/s)';
            axMap.Visible = 'on';
            axMap.Color = [0.3 0.3 0.5];
            axMap.XLim = [1 length(x1)];
            axMap.YLim = [1 length(x2)];
            axMap.Box = 'off';
            axMap.FontSize = 14;
            caxis([3*min(R),max(R)])
            pathmap = plot(NaN,NaN,'.g','MarkerSize',30); % The green marker that travels through the state map to match the pendulum animation
            map.CData = V;
            hold off
        end
        
        function obj = update_plot(obj, z1, x1, x2, snewIdx,  Q, Vorig, episodes, f, pathmap, map, transpMap, doVid)
                            %% UPDATE PLOTS
                if episodes>0
                    % Pendulum state:
                    set(f,'XData',[0 -sin(z1(1))]);
                    set(f,'YData',[0 cos(z1(1))]);

                    % Green tracer point:
                    [newy,newx] = ind2sub([length(x2), length(x1)],snewIdx); % Find the 2d index of the 1d state index we found above
    
                    set(pathmap,'XData',newx);
                    set(pathmap,'YData',newy);

                    % The heat map of best Q values
                    V = max(Q,[],2); % Best estimated value for all actions at each state.
                    fullV = reshape(V,[length(x2),length(x1)]); % Make into 2D for plotting instead of a vector.

                    set(map,'CData',fullV);
                    if transpMap
                        set(map,'AlphaData',fullV~=Vorig); % Some spots have not changed from original. If not, leave them transparent.
                    end
                    drawnow;

                    % Take a video frame if turned on.
                    if doVid
                        frame = getframe(panel);
                        writeVideo(obj.writerObj,frame);
                    end
                end
        end
        
    end
end