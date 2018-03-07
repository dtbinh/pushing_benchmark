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

