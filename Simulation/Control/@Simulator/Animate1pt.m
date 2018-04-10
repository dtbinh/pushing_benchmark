%% Main Animation 
function obj = Animate1pt(obj, xs_star, MPC_trajectories, MPC_best_traj, MPPI, weights, des_traj)

obj.Fig_weights= figure('Color', 'w', 'OuterPosition', [0, 0, 960, 1080], 'PaperPosition', [0, 0, 6, (6/8)*6]);
bar_weights = bar([0 0]);
xlim([0 1200]);
ylim([0 1]);

%Initialize figure
obj.Ani = figure('Color', 'w', 'OuterPosition', [0, 0, 960, 1080], 'PaperPosition', [0, 0, 6, (6/8)*6]);
%figure properties
font_size  = 25;
line_size  = 15;
line_width = 2;
% set(gcf,'Renderer','OpenGL');
set(gca,'FontSize',20)
set(obj.Ani, 'PaperPositionMode', 'auto');
box off
% plot(xs_exp(:,1), xs_exp(:,2), 'b');
axis equal
xlabel('x(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
ylabel('y(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
% xlim([-.25 .25]);
% ylim([-.35 .4]);
xlim([-.1 .65]);
ylim([-.2 .2]);
view([90 90])
xticks([0 pi 2*pi])
yticks([-0.3, -0.2,-0.1,0,0.1,0.2,0.3])
view([90 90])

% plot(xs_exp(:,1), xs_exp(:,2), 'b');

%Animation parameters
tf = obj.t(end);
N = length(obj.t);
accFactor = 15;
x_state = obj.xs_state;
%create movie file
videoname = strcat(obj.FilePath,'/',(obj.SimName),'.avi');
v = VideoWriter(videoname);
fps = int64(N/(accFactor*tf));
fps = double(fps);
v.FrameRate = fps;
open(v);
%create movie file
videoname = strcat(obj.FilePath,'/',(obj.SimName),'weights','.avi');
v2 = VideoWriter(videoname);
v2.FrameRate = fps;
open(v2);

%% initialize plots
%nominal trajectory (red)
lv1=1;
Data{lv1} = obj.Data1pt(xs_star(lv1,:));
Slider{lv1} = patch(Data{lv1}.x1b, Data{lv1}.y1b,'r', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', 'r','FaceColor','NONE','LineWidth',1.);
hold on 
Pusher_c{lv1} = patch(Data{lv1}.X_circle_p,Data{lv1}.Y_circle_p,'r', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',1.);
hold on
%actual trajectories (blue)
for lv1=2:2
    Data{lv1} = obj.Data1pt(obj.xs_state(lv1,:)); 
    hold on 
    Slider{lv1} = patch(Data{lv1}.x1b, Data{lv1}.y1b,'red', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',2.);
    hold on 
    Pusher_c{lv1} = patch(Data{lv1}.X_circle_p,Data{lv1}.Y_circle_p,'r', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',2.);
    hold on
end
for lv1=1:1200
    MPC_traj{lv1} = plot([0,0],[0,0], 'b','LineWidth',1.);
    hold on;
end
MPC_best = plot([0,0],[0,0], 'r','LineWidth',2.);
Des_traj = plot([0,0],[0,0], 'k','LineWidth',2.);

%% update figures
%nominal trajectory (red)
counter=1;
is_tick = [25, 48, 114];
for i1=1:accFactor:length(obj.t)
    lv1=1;
    %update data
    Data{lv1} = obj.Data1pt(xs_star(i1,:));
    %update figures
    Slider{lv1}.XData = Data{lv1}.x1b;
    Slider{lv1}.YData = Data{lv1}.y1b;
    Pusher_c{lv1}.XData = Data{lv1}.X_circle_p;
    Pusher_c{lv1}.YData = Data{lv1}.Y_circle_p;
    %actual trajectories (blue)
    for lv1=2:2
        Data{lv1} = obj.Data1pt(obj.xs_state(i1,:));
        Slider{lv1}.XData = Data{lv1}.x1b;
        Slider{lv1}.YData = Data{lv1}.y1b;
        Pusher_c{lv1}.XData = Data{lv1}.X_circle_p;
        Pusher_c{lv1}.YData = Data{lv1}.Y_circle_p;
        Slider_thin{lv1} = patch(Data{lv1}.x1b, Data{lv1}.y1b,'red', 'FaceAlpha', .6,'EdgeAlpha', .2,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',0.3);
        Pusher_thin_c{lv1} = patch(Data{lv1}.X_circle_p,Data{lv1}.Y_circle_p,'red', 'EdgeAlpha', .2,'FaceAlpha', .6, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.3);
    end
    Des_traj.XData = des_traj{i1}(:,1);
    Des_traj.YData = des_traj{i1}(:,2);
    
    t = obj.t(i1);
	xd_vec = zeros(floor(MPPI.T/MPPI.dt)+1, 5);
    for lv2 = 1:floor(MPPI.T/MPPI.dt)+1
        [xd, ud] = obj.find_nominal_state(t);
        xd_vec(lv2,:) = xd';
        t = t+MPPI.dt;
    end
    
    for lv1=1:MPPI.K
        MPC_traj{lv1}.XData = MPC_trajectories{i1}{lv1}(1,:)+ xd_vec(:,1)';
        MPC_traj{lv1}.YData = MPC_trajectories{i1}{lv1}(2,:)+ xd_vec(:,2)';
        MPC_traj{lv1}.Color(4) = weights{i1}(lv1);
    end
    MPC_best.XData = MPC_best_traj{i1}(1,:)+ xd_vec(:,1)';
    MPC_best.YData = MPC_best_traj{i1}(2,:)+ xd_vec(:,2)';
    bar_weights.YData = weights{i1};

    %update and save frame
    frame = getframe(obj.Ani);
    frame2 = getframe(obj.Fig_weights);
    writeVideo(v,frame);
    writeVideo(v2,frame2);
    counter=counter+1;
end
% saveas(obj.Ani, 'MPPI_nonlinear_straight_line','png')

%%%%%%%%%%%%%%%%%%%%%%%%

% %Initialize figure
% obj.Ani = figure('Color', 'w', 'OuterPosition', [0, 0, 960, 1080], 'PaperPosition', [0, 0, 11, (6/8)*11]);
% %figure properties
% font_size  = 25;
% line_size  = 15;
% line_width = 2;
% % set(gcf,'Renderer','OpenGL');
% traj =  plot(xs_exp(:,1), xs_exp(:,2), 'color', [0,0,1]*.8);
% traj.Color(4) = 0.8;
% set(gca,'FontSize',20)
% set(obj.Ani, 'PaperPositionMode', 'auto');
% box off
% % plot(xs_exp(:,1), xs_exp(:,2), 'b');
% axis equal
% xlabel('x(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
% ylabel('y(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
% xlim([-.25 .25]);
% ylim([-.35 .35]);view([90 90])
% % xticks([0 pi 2*pi])
% yticks([-0.2,-0.1,0,0.1,0.2])
% view([90 90])
% 
% % plot(xs_exp(:,1), xs_exp(:,2), 'b');
% 
% %Animation parameters
% tf = obj.t(end);
% N = length(obj.t);
% accFactor = 30;
% x_state = obj.xs{1};
% %create movie file
% videoname = strcat(obj.FilePath,'/',(obj.SimName),'.avi');
% v = VideoWriter(videoname);
% fps = int64(N/(accFactor*tf));
% fps = double(fps);
% v.FrameRate = fps;
% open(v);
% %% initialize plots
% %nominal trajectory (red)
% lv1=1;
% Data{lv1} = obj.Data1pt(1,lv1);
% Slider{lv1} = patch(Data{lv1}.x1b, Data{lv1}.y1b,'r', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', 'r','FaceColor','NONE','LineWidth',1.);
% hold on 
% Pusher_c{lv1} = patch(Data{lv1}.X_circle_p,Data{lv1}.Y_circle_p,'r', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',1.);
% hold on
% %actual trajectories (blue)
% for lv1=2:obj.NumSim+1
%     Data{lv1} = obj.Data1pt(1,lv1); 
%     hold on 
%     Slider{lv1} = patch(Data{lv1}.x1b, Data{lv1}.y1b,'red', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',2.);
%     hold on 
%     Pusher_c{lv1} = patch(Data{lv1}.X_circle_p,Data{lv1}.Y_circle_p,'r', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',2.);
%     hold on
% end
% %% update figures
% %nominal trajectory (red)
% counter=1;
% for i1=1:accFactor:length(obj.t)
%     lv1=1;
%     %update data
%     Data{lv1} = obj.Data1pt(i1,lv1);
%     %update figures
% %     Slider{lv1}.XData = Data{lv1}.x1b;
% %     Slider{lv1}.YData = Data{lv1}.y1b;
% %     Pusher_c{lv1}.XData = Data{lv1}.X_circle_p;
% %     Pusher_c{lv1}.YData = Data{lv1}.Y_circle_p;
% %     %actual trajectories (blue)
%     for lv1=2:obj.NumSim+1
%         Data{lv1} = obj.Data1pt(i1,lv1);
% %         Slider{lv1}.XData = Data{lv1}.x1b;
% %         Slider{lv1}.YData = Data{lv1}.y1b;
% %         Pusher_c{lv1}.XData = Data{lv1}.X_circle_p;
% %         Pusher_c{lv1}.YData = Data{lv1}.Y_circle_p;
%         t_range=linspace(1.,.2,30);
%         t_range2=linspace(2.,.1,30);
%         if counter<30
%             Slider_thin{lv1} = patch(Data{lv1}.x1b, Data{lv1}.y1b,'red', 'FaceAlpha', .6,'EdgeAlpha', t_range(counter),'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',t_range2(counter));
%         else
%             Slider_thin{lv1} = patch(Data{lv1}.x1b, Data{lv1}.y1b,'red', 'FaceAlpha', .6,'EdgeAlpha', .2,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',0.3);
%         end
% %         hold on 
% %         Pusher_thin_c{lv1} = patch(Data{lv1}.X_circle_p,Data{lv1}.Y_circle_p,'red', 'EdgeAlpha', .2,'FaceAlpha', .6, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.3);
%     end
%     %update and save frame
%     frame = getframe(obj.Ani);
%     writeVideo(v,frame);
%     counter=counter+1;
% end
saveas(obj.Ani, 'point_pusher_infinite','png')

close(v);



