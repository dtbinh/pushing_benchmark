classdef Simulator < dynamicprops 
    % Class used during Euler integration to determine time derivative of
    % slider and pusher coordinates in world frame

    properties (Constant)
    end
    
    properties
        h;
        ps;
        t;
        xs_state;
        us_state;
        N;
        FileName;
        SimName;
        FilePath;
        NumSim;
        Ani;
        Fig_weights;
        x_length;
        a_length;
        
        Pusher_c;
        Slider;
        MPC_traj;
        MPC_best;
        Des_traj;
        Slider_des;
        x_star;
        u_star;
        t_star;
        bar_weights;
    end
   
    methods
        %% Constructor
        function obj = Simulator(ps, SimName)  
            obj.ps = ps;
            
                %% Save data in new folder
            obj.SimName = SimName;
            homeName = getenv('HOME');
            tempName = strcat(homeName, '/Data/pushing_simulation/',obj.SimName);
            mkdir(tempName);
            obj.FilePath = tempName;
            obj.FileName = strcat(obj.FilePath,'/',obj.SimName);
            obj.x_length = 5;
            obj.a_length = 2;
        end
        
        function z2 = get_next_state_i(obj, z1, T, dt, t)
            k1 = obj.pointSimulatorAnalytical(z1,T);
            k2 = obj.pointSimulatorAnalytical(z1+dt/2*k1,T);
            k3 = obj.pointSimulatorAnalytical(z1+dt/2*k2,T);
            k4 = obj.pointSimulatorAnalytical(z1+dt*k3,T);

            z2 = z1 + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        end
        
        function z2 = get_next_state_b(obj, z1, u_b, dt, t)
            theta = z1(3);
            T = Helper.C3_2d(theta)'*u_b;
            k1 = obj.pointSimulatorAnalytical(z1,T);
            k2 = obj.pointSimulatorAnalytical(z1+dt/2*k1,T);
            k3 = obj.pointSimulatorAnalytical(z1+dt/2*k2,T);
            k4 = obj.pointSimulatorAnalytical(z1+dt*k3,T);

            z2 = z1 + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        end
        
        function z2 = get_next_state_nonlinear_error_b(obj, xs_bar, u_b_bar, dt, t)
            [xd, ud] = obj.find_nominal_state(t);
            xs = xs_bar + xd;
            
            theta = xs(3);
            us_bar = Helper.C3_2d(theta)'*u_b_bar;
            us_star = Helper.C3_2d(theta)'*ud;
            us = us_star + us_bar;
            
            dz = obj.pointSimulatorAnalytical(xs, us);
            dz_star = obj.pointSimulatorAnalytical(xd, us_star);
            delta_dz = dz-dz_star;

            z2 = xs_bar + dt*delta_dz;
        end
        
        function [state, action] = find_nominal_state(obj, t)
            diff = abs(obj.t_star - t);
            [val ind] = min(diff);
            state = obj.x_star(ind,:)';
            action = obj.u_star(ind,:)';
        end
        
        function u_clamp = u_constraints(obj, x, u, t, is_print)
            [xd, ud] = obj.find_nominal_state(t);
            theta = x(3);
            rbbi = Helper.C3_2d(theta)*x(1:2);
            rbpi = Helper.C3_2d(theta)*x(4:5);
            rbpb = rbpi - rbbi;
            rx = rbpb(1);
            ry = rbpb(2);
            u_clamp = u;

            if ry>0.09/3
                u_clamp(2) = min(u(2), 0);
            elseif ry<-0.09/3
                u_clamp(2) = max(0, u(2));
            end
            
            u_clamp(1) = max(0, u_clamp(1));
            u_clamp(1) = min(0.2, u_clamp(1));
            
            u_clamp(2) = max(-0.2, u_clamp(2));
            u_clamp(2) = min(0.2, u_clamp(2));
        end

        function u_clamp_bar = u_constraints_nonlinear_error(obj, x_bar, u_bar, t, is_print)
            [xd, ud] = obj.find_nominal_state(t);
            x = xd+x_bar;
            u = ud+u_bar;
            theta = x(3);
            rbbi = Helper.C3_2d(theta)*x(1:2);
            rbpi = Helper.C3_2d(theta)*x(4:5);
            rbpb = rbpi - rbbi;
            rx = rbpb(1);
            ry = rbpb(2);
            u_clamp = u;

            if ry>0.09/3
                u_clamp(2) = min(u(2), 0);
            elseif ry<-0.09/3
                u_clamp(2) = max(0, u(2));
            end
            
            u_clamp(1) = max(0, u_clamp(1));
            u_clamp(1) = min(0.2, u_clamp(1));
            
            u_clamp(2) = max(-0.2, u_clamp(2));
            u_clamp(2) = min(0.2, u_clamp(2));

            if is_print
                ry
                u_clamp
            end
            u_clamp_bar = u_clamp - ud;
        end
        
        function q = q_cost(obj, x, u, t)
            [xd, ud] = obj.find_nominal_state(t);
            q = 10000*((x - xd)'*diag([1,1,.01,0.,0.])*(x - xd) +0*(u-[0.05;0])'*eye(2)*(u-[0.05;0]));
        end
        
        function phi = phi_cost(obj,x, u, t)
            [xd, ud] = obj.find_nominal_state(t);
            phi = 10000*(x - xd)'*500*diag([1,3,.01,0.,0.])*(x - xd);
        end
        
        function q = q_cost_nonlinear_error(obj, x, u, t)
            q = 10000*((x)'*diag([1,1,.01,0.,0.])*(x) );
        end
        
        function phi = phi_cost_nonlinear_error(obj,x, u, t)
            phi = 10000*(x)'*500*diag([1,3,.01,0.,0.])*(x);
        end
        
        
        function twist_i = pointSimulatorAnalytical(obj, xs, us)
            %
            ribi = xs(1:2);
            theta  = xs(3);
            ripi = xs(4:5);
            vipi = us;
            %
            Cbi = Helper.C3_2d(theta);
            rbbi = Cbi*ribi;
            rbpi = Cbi*ripi;
            vbpi = Cbi*vipi;
            %Find rx, ry: In body frame
            rbpb = rbpi - rbbi;
            rx = rbpb(1);
            ry = rbpb(2);

            %Build output vector
               twist_b =obj.pointSimulatorAnalytical_b(vbpi,ry);
%                twist_b =twist_b_gp(vbpi,ry);
            
            vbbi = twist_b(1:2);
            dtheta = twist_b(3);
            vibi = Cbi'*vbbi;
            twist_i = [vibi;dtheta;us];
        end
        
        function twist_b = pointSimulatorAnalytical_b(obj, vbpi, ry)
            
            %Find rx, ry: In body frame
            rx = -obj.ps.o.a/2;
%             ry = rbpb(2);
            %Compute friction cone vectors
            gamma_top    = (obj.ps.p.nu_p*obj.ps.c^2 - rx*ry + obj.ps.p.nu_p*rx^2)/(obj.ps.c^2 + ry^2 - obj.ps.p.nu_p*rx*ry);
            gamma_bottom = (-obj.ps.p.nu_p*obj.ps.c^2 - rx*ry - obj.ps.p.nu_p*rx^2)/(obj.ps.p.nu_p^2 + ry^2 + obj.ps.p.nu_p*rx*ry);
            try
                gamma = vbpi(2)/vbpi(1);
            catch
                if vbpi(2)>0
                    gamma = 1000000;
                else
                    gamma = -1000000;
                end
            end        
            %Motion Cone condition for sliding/sticking
            if gamma>gamma_top
                vMC = [1;gamma_top];
                kappa = (vbpi(1))/(vMC(1));
                vo = kappa*vMC;
        %         disp('Sliding Up');
            elseif gamma<gamma_bottom
                vMC = [1;gamma_bottom];
                kappa = (vbpi(1))/(vMC(1));
                vo = kappa*vMC;
        %         disp('Sliding Down');
            else
                vo = vbpi;
        %         disp('Sticking');
            end

            if vo(1)<=0 || abs(ry)>obj.ps.o.b/2 || abs(rx+obj.ps.o.a/2)>.005
                vo=[0;0];
%                 disp('not in contact');
            end
%             vo = vbpi;
            vpx = vo(1);
            vpy = vo(2);
            
            %Compute derivative: In body frame
            vx = ((obj.ps.c^2 + rx^2)*vpx + rx*ry*vpy)/(obj.ps.c^2 + rx^2 + ry^2);
            vy = ((obj.ps.c^2 + ry^2)*vpy + rx*ry*vpx)/(obj.ps.c^2 + rx^2 + ry^2);
            dtheta = (rx*vy - ry*vx)/(obj.ps.c^2); 
            %Build output vector
            twist_b = [vx;vy;dtheta];

        end
        
        function obj = initialize_plot(obj, xs, xd, MPPI)
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
%             for lv1=1:MPPI.K
%                 obj.MPC_traj{lv1} = plot([0,0],[0,0], 'b','LineWidth',1.);
%                 hold on;
%             end
%             obj.MPC_best = plot([0,0],[0,0], 'r','LineWidth',2.);
%             obj.Des_traj = plot([0,0],[0,0], 'k','LineWidth',2.);
            hold on;
            axis equal
            xlabel('x(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
            ylabel('y(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
            xlim([-.1 1.6]);
            ylim([-.2 .2]);
            view([90 90])
            Data = obj.Data1pt(xs); 
            obj.Slider = patch(Data.x1b, Data.y1b,'red', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',2.);
            obj.Pusher_c = patch(Data.X_circle_p,Data.Y_circle_p,'r', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',2.);

            switch nargin
                case 3
            Data_xd = obj.Data1pt(xd); 
            obj.Slider_des = patch(Data_xd.x1b, Data_xd.y1b,'r', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', 'r','FaceColor','NONE','LineWidth',1.);
            end
%             switch nargin
%                 case 3
%                     
%                      obj.Fig_weights= figure('Color', 'w', 'OuterPosition', [0, 0, 960, 1080], 'PaperPosition', [0, 0, 6, (6/8)*6]);
%                      obj.bar_weights = bar([0 0]);
%             end
            
        end
        
        function obj = update_plot(obj, xs, t)
                        
            Data = obj.Data1pt(xs); 
            obj.Slider.XData = Data.x1b;
            obj.Slider.YData = Data.y1b;
            obj.Pusher_c.XData = Data.X_circle_p;
            obj.Pusher_c.YData = Data.Y_circle_p;
            
            [xd, ud] = obj.find_nominal_state(t);
            xd=xd';
            Data_xd = obj.Data1pt(xd);
            obj.Slider_des.XData = Data_xd.x1b;
            obj.Slider_des.YData = Data_xd.y1b;
%             obj.Des_traj.XData = x_des(:,1);
%             obj.Des_traj.YData = x_des(:,2);

            switch nargin
                case 4

                     for lv1=1:MPPI.K
                         obj.MPC_traj{lv1}.XData = MPPI.X{lv1}(1,:);
                         obj.MPC_traj{lv1}.YData = MPPI.X{lv1}(2,:);
                         obj.MPC_traj{lv1}.Color(4) = MPPI.w(lv1);
                     end
                     obj.MPC_best.XData = MPPI.x(1,:);
                     obj.MPC_best.YData = MPPI.x(2,:);
                     obj.bar_weights.YData = MPPI.w;
            end
            
            drawnow;
        end
                
        function obj = update_plot_nonlinear_error(obj, xs, MPPI, x_des, t)
            
            [xd, ud] = obj.find_nominal_state(t);
            xd=xd';
            Data_xd = obj.Data1pt(xd);
            obj.Slider_des.XData = Data_xd.x1b;
            obj.Slider_des.YData = Data_xd.y1b;
            
            Data = obj.Data1pt(xs); 
            obj.Slider.XData = Data.x1b;
            obj.Slider.YData = Data.y1b;
            obj.Pusher_c.XData = Data.X_circle_p;
            obj.Pusher_c.YData = Data.Y_circle_p;
            obj.Des_traj.XData = x_des(:,1);
            obj.Des_traj.YData = x_des(:,2);
            xd_vec = zeros(floor(MPPI.T/MPPI.dt)+1, 5);
            for lv2 = 1:floor(MPPI.T/MPPI.dt)+1
                [xd, ud] = obj.find_nominal_state(t);
                xd_vec(lv2,:) = xd';
                t = t+MPPI.dt;
            end
            for lv1=1:MPPI.K
                obj.MPC_traj{lv1}.XData = MPPI.X{lv1}(1,:) + xd_vec(:,1)';
                obj.MPC_traj{lv1}.YData = MPPI.X{lv1}(2,:) + xd_vec(:,2)';
                obj.MPC_traj{lv1}.Color(4) = MPPI.w(lv1);
            end
            obj.MPC_best.XData = MPPI.x(1,:)+ xd_vec(:,1)';
            obj.MPC_best.YData = MPPI.x(2,:)+ xd_vec(:,2)';
            obj.bar_weights.YData = MPPI.w;
            
            drawnow;
        end
        
                %% line Simulator
        function [dxs, F_gen] = pointSimulator(obj,xs,us)
        %Extract variables from xs
        ribi = xs(1:2);
        theta = xs(3);
        ripi = xs(4:5);
        %Extract variables from us
        vipi = us(1:2);
        %Direction Cosine Matrices 
        Cbi = Helper.C3_2d(theta);
        %Convert state to body frame
        rbbi = Cbi*ribi;
        rbpi = Cbi*ripi;
        vbpi = Cbi*vipi;
        %Find rx, ry: In body frame   
        rbpb = rbpi - rbbi;
        rpx = rbpb(1);
        rpy = rbpb(2);
        rb{1} = rbpb;
        %kinematics
        npa_b = [1;0];
        tpa_b = [0;1];
        Dpa_b = [tpa_b -tpa_b];
        Jpa = [1 0 -rbpb(2);...
               0 1 rbpb(1)];
        vpa = vbpi; 
        %find distances
        rbzi = rbbi-[obj.ps.o.a/2;0];
        rbpz = rbpi-rbzi;
        d{1} = -rbpz'*npa_b;
        %build useful matrices pt a
        N{1} = npa_b'*Jpa;
        L{1} = Dpa_b'*Jpa;
        E{1} = [1;1];
        aMat{1} = -npa_b'*vpa;
        bMat{1} = -Dpa_b'*vpa;
        %collision check
        counter = 1;
        index_vec = [];
        for lv1=1:1
            if d{lv1}<0.0001 && abs(rb{lv1}(2))<obj.ps.o.a/2
                index_vec(counter) = lv1;
                counter = counter+1;
            end
        end
        %number if contact points
        m=length(index_vec);
        %concatenate matrices
        N_tot= [];
        L_tot=[];
        E_tot=[];
        a_tot = [];
        b_tot = [];
        nu_tot = [];
        F_gen = [];
        for lv1=1:m
            N_tot = [N_tot;N{index_vec(lv1)}];
            L_tot = [L_tot;L{index_vec(lv1)}];
            E_tot = blkdiag(E_tot,E{index_vec(lv1)});
            a_tot = [a_tot;aMat{index_vec(lv1)}];
            b_tot = [b_tot;bMat{index_vec(lv1)}];
            nu_tot = blkdiag(nu_tot,obj.ps.p.nu_p);
        end
        %solve LCP
        if m
            %LCP matrices
            M = [N_tot*obj.ps.A_ls*N_tot' N_tot*obj.ps.A_ls*L_tot' zeros(m,m);...
                L_tot*obj.ps.A_ls*N_tot' L_tot*obj.ps.A_ls*L_tot' E_tot;...
                nu_tot -E_tot' zeros(m,m)];
            q = [a_tot;b_tot;zeros(m,1)];
            F_gen = LCP(M,q);
            fn = F_gen(1:m);
            ft = F_gen(m+1:m+1+2*m-1);
            twist = obj.ps.A_ls*(N_tot'*fn + L_tot'*ft);
        else
            twist=[0;0;0];
        end
        %Compute twist (in body frame)
        twist_body_b = twist;
        %convert to inertial frame
        S_ib = [Cbi' [0;0];0 0 1];
        twist_body_i = S_ib*twist_body_b;
        %Return combined velocites of slider and pusher
        dxs = [twist_body_i; us];
        end
        
    end
end

