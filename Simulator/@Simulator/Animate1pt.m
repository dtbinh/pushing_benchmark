%% Main Animation 
function obj = Animate1pt(obj)

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
xlim([-.25 .25]);
ylim([-.35 .4]);
view([90 90])
% xticks([0 pi 2*pi])
% yticks([-0.3, -0.2,-0.1,0,0.1,0.2,0.3])
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
%% initialize plots
%nominal trajectory (red)
% lv1=1;
% Data{lv1} = obj.Data1pt(1,lv1);
% Slider{lv1} = patch(Data{lv1}.x1b, Data{lv1}.y1b,'r', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', 'r','FaceColor','NONE','LineWidth',1.);
% hold on 
% Pusher_c{lv1} = patch(Data{lv1}.X_circle_p,Data{lv1}.Y_circle_p,'r', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',1.);
% hold on
%actual trajectories (blue)
for lv1=1:1
    Data{lv1} = obj.Data1pt(1+accFactor,lv1); 
    hold on 
    Slider{lv1} = patch(Data{lv1}.x1b, Data{lv1}.y1b,'red', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',2.);
    hold on 
    Pusher_c{lv1} = patch(Data{lv1}.X_circle_p,Data{lv1}.Y_circle_p,'r', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',2.);
    hold on
end
%% update figures
%nominal trajectory (red)
counter=1;
is_tick = [25, 48, 114];
for i1=1+accFactor:accFactor:length(obj.t)
    lv1=1;
    %update data
    Data{lv1} = obj.Data1pt(i1,lv1);
    %update figures
    Slider{lv1}.XData = Data{lv1}.x1b;
    Slider{lv1}.YData = Data{lv1}.y1b;
    Pusher_c{lv1}.XData = Data{lv1}.X_circle_p;
    Pusher_c{lv1}.YData = Data{lv1}.Y_circle_p;
    %actual trajectories (blue)
    for lv1=1:1
        
        Data{lv1} = obj.Data1pt(i1,lv1);
        Slider{lv1}.XData = Data{lv1}.x1b;
        Slider{lv1}.YData = Data{lv1}.y1b;
        Pusher_c{lv1}.XData = Data{lv1}.X_circle_p;
        Pusher_c{lv1}.YData = Data{lv1}.Y_circle_p;
        Slider_thin{lv1} = patch(Data{lv1}.x1b, Data{lv1}.y1b,'red', 'FaceAlpha', .6,'EdgeAlpha', .2,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',0.3);
        hold on 
        Pusher_thin_c{lv1} = patch(Data{lv1}.X_circle_p,Data{lv1}.Y_circle_p,'red', 'EdgeAlpha', .2,'FaceAlpha', .6, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.3);
    end
    %update and save frame
    frame = getframe(obj.Ani);
    writeVideo(v,frame);
    counter=counter+1;
end
% saveas(obj.Ani, 'point_pusher_infinite','png')

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
% % saveas(obj.Ani, 'point_pusher_infinite','png')

close(v);



