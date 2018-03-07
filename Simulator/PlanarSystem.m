classdef PlanarSystem < dynamicprops
    properties (Constant)
    end
    
    properties
        p;
        s;
        o;
        c;
        m_max;
        f_max; 
        A_ls;
        A_fun;
        B_fun;
        twist_object_i_fun;
        f_non_fun;
        A_linear;
        B_linear;
        A_bar;
        B_bar;
        num_xcStates;
        num_ucStates;
        num_xsStates;
        num_usStates;
    end
    
    methods
        %% Constructor
        function obj = PlanarSystem(p,o,s) 
            obj.p=p;
            obj.o=o;
            obj.s=s;
            obj.initialize_variables();
            %Compute f_max and m_max
            obj.f_max = (obj.s.nu*obj.o.m*Helper.g);
            obj.m_max = obj.m_max_funct();
            obj.c = obj.m_max/obj.f_max; 
            %build limit surface
            obj.build_limit_surface();
            %symbolic manipulation
%             obj.symbolicLinearize();
%             obj.symbolicLinearize_cpp();
        end
        %% Initialize variable dimensions
        function obj = initialize_variables(obj)
            obj.num_xcStates = 4;
            obj.num_ucStates = length(obj.p.uc);
            obj.num_xsStates = 3+length(obj.p.xp);
            obj.num_usStates = length(obj.p.xp);
        end
        %% Build limit surface
        function n_f = build_limit_surface(obj)     
            %Build limit surface representation
            syms fx fy m 
            F  = [fx;fy;m];
            %% 1) Limit surface
            H = fx^2/obj.f_max^2 + fy^2/obj.f_max^2 + m^2/obj.m_max^2;
            obj.A_ls = double(hessian(H,F));
        end
        %% max friction coefficients
        function n_f = m_max_funct(obj)     
            n_f_integrand = @(p1,p2) (obj.s.nu*obj.o.m*Helper.g/obj.o.A) * sqrt([p1;p2;0]'*[p1;p2;0]);
            n_f = Helper.DoubleGaussQuad(n_f_integrand,-obj.o.a/2,obj.o.a/2,-obj.o.b/2,obj.o.b/2);
        end
        %% Coordinate transform
        function [xc] = coordinateTransformSC(obj, xs)
            %Extract variables from xs
            ribi  = xs(1:2);
            theta = xs(3);
            ripi  = xs(4:5);
            %Direction Cosine Matrices 
            Cbi = Helper.C3_2d(theta);
            %Convert state to body frame
            rbbi = Cbi*ribi;
            rbpi = Cbi*ripi;
            %Build xc
            rbpb = rbpi - rbbi;
            ry = rbpb(2);
            %Output
            xc = [ribi;theta;ry];
        end
        %% Coordinate transform
        function [xs] = coordinateTransformCS(obj, xc)
            %Extract variables
            ribi = xc(1:2);
            theta = xc(3);
            ry = xc(4);
            %Direction cosine matrices
            Cbi = Helper.C3_2d(theta);
            %Convert state to body frame
            rbbi = Cbi*ribi;
            rbpb = [-obj.o.a/2;ry];
            ripb = Cbi'*rbpb;
            ripi = ribi+ripb;
            %Forces
            if length(obj.p.xp)==2
                xs = [ribi;theta;ripi];
            elseif length(obj.p.xp)==3
                xs = [ribi;theta;ripi;theta];
            end
        end
                %% Symbolic linearize
        function obj = symbolicLinearize_cpp(obj)
            %% Symbolic variables
            rbpb = sym('rbpb', [2,1]); %x = [x;y;theta]
            A_limit = sym('A_ls', [3,3]); %x = [x;y;theta]
            syms d;
            rx = rbpb(1);
            ry = rbpb(2);
            %% Build states       
            xc = [obj.o.xo;ry];
            uc = [obj.p.uc]; %[fn1;ft1;ry_dot];
            fn = obj.p.fn;
            ft = obj.p.ft;
            %% DCM Matrices
            Cbi = Helper.C3_2d(obj.o.xo(3));
            %% Kinematics
            sign_vec = [1 -1]*1;
            N_tot=[];
            T_tot=[];
            for lv1=1:obj.p.num_contacts %lv1 represents contact point 1 and 2
                rb{lv1} = [rx*1;ry*1]+sign_vec(lv1)*[0;d]; %position of contact point lv1
                Jb{lv1} = [1 0 -rb{lv1}(2);... 
                             0 1 rb{lv1}(1)];
                for lv2=1:2 %lv2 represents left of right border (FC/MC)
                    n{lv1} = [1;0];
                    t{lv1} = [0;1];
                    N{lv1} = transpose(n{lv1})*Jb{lv1};
                    T{lv1} = transpose(t{lv1})*Jb{lv1};
                end
                N_tot = [N_tot;N{lv1}];
                T_tot = [T_tot;T{lv1}];
            end
            %Motion equations (nonlinear)
            Vb = A_limit*(transpose(N_tot)*fn + transpose(T_tot)*ft);
            C_tilde = [transpose(Cbi) [0; 0];0 0 1];
            f_non1 = C_tilde*Vb;
            
            f_non2 = obj.p.dry;
            f_non = [f_non1;f_non2];
            %Linearization
            A = jacobian(f_non,xc);
            B = jacobian(f_non,uc);
            % Substitute equilibrium states
%             A_fun = matlabFunction(A, 'FILE', 'A_fun', 'Vars', {xc,uc});
%             B_fun = matlabFunction(B, 'FILE', 'B                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          _fun', 'Vars', {xc,uc});
            A_fun = matlabFunction(A, 'File', 'A_fun_cpp', 'Vars', {xc,uc, rbpb, d, A_limit});
            B_fun = matlabFunction(B, 'File', 'B_fun_cpp', 'Vars', {xc,uc, rbpb, d, A_limit});
            twist_object_i_fun = matlabFunction(f_non1, 'File', 'twist_object_i_fun_cpp', 'Vars', {xc,uc, rbpb, d, A_limit});
            f_non_fun = matlabFunction(f_non, 'File', 'f_non_fun_cpp', 'Vars', {xc,uc, rbpb, d, A_limit});
        end
        %% Symbolic linearize
        function obj = symbolicLinearize(obj)
            %% Symbolic variables
            syms ry
            %% Build states       
            xc = [obj.o.xo;ry];
            uc = [obj.p.uc]; %[fn1;ft1;ry_dot];
            fn = obj.p.fn;
            ft = obj.p.ft;
            %% DCM Matrices
            Cbi = Helper.C3_2d(obj.o.xo(3));
            %% Kinematics
            sign_vec = [1 -1]*1;
            rx = -obj.o.a/2;
            N_tot=[];
            T_tot=[];
            for lv1=1:obj.p.num_contacts %lv1 represents contact point 1 and 2
                rb{lv1} = [rx*1;ry*1]+sign_vec(lv1)*[0;obj.p.d]; %position of contact point lv1
                Jb{lv1} = [1 0 -rb{lv1}(2);... 
                             0 1 rb{lv1}(1)];
                for lv2=1:2 %lv2 represents left of right border (FC/MC)
                    n{lv1} = [1;0];
                    t{lv1} = [0;1];
                    N{lv1} = transpose(n{lv1})*Jb{lv1};
                    T{lv1} = transpose(t{lv1})*Jb{lv1};
                end
                N_tot = [N_tot;N{lv1}];
                T_tot = [T_tot;T{lv1}];
            end
            %Motion equations (nonlinear)
            Vb = obj.A_ls*(transpose(N_tot)*fn + transpose(T_tot)*ft);
            C_tilde = [transpose(Cbi) [0; 0];0 0 1];
            f_non1 = C_tilde*Vb;
            
            f_non2 = obj.p.dry;
            f_non = [f_non1;f_non2];
            %Linearization
            A = jacobian(f_non,xc);
            B = jacobian(f_non,uc);
            % Substitute equilibrium states
%             A_fun = matlabFunction(A, 'FILE', 'A_fun', 'Vars', {xc,uc});
%             B_fun = matlabFunction(B, 'FILE', 'B                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          _fun', 'Vars', {xc,uc});
            obj.A_fun = matlabFunction(A, 'Vars', {xc,uc});
            obj.B_fun = matlabFunction(B, 'Vars', {xc,uc});
            obj.twist_object_i_fun = matlabFunction(f_non1, 'Vars', {xc,uc});
            obj.f_non_fun = matlabFunction(f_non, 'Vars', {xc,uc});
        end
        %% Convert reaction forces to associated robot velocity
        function [us] = force2Velocity(obj, xc, uc)
            %Extract variables
            ribi = xc(1:2);
            theta = xc(3);
            ry = xc(4);
            rx = -obj.o.a/2;
            dry = uc(end);
            %Direction cosine matrices
            Cbi = Helper.C3_2d(theta);
            rbpb = [rx;ry];
            ripb = Cbi'*rbpb;
            drbpb = [0;dry];
            twist_body_i = obj.twist_object_i_fun(xc, uc);
            
            %Forces
            if length(obj.p.xp)==2
                twist_pusher_i = zeros(2,1);
                twist_pusher_i(1:2) = twist_body_i(1:2) + Cbi'*drbpb + Helper.cross3d(twist_body_i(3), ripb); 
             
            elseif length(obj.p.xp)==3
                twist_pusher_i = zeros(3,1);
                twist_pusher_i(1:2) = twist_body_i(1:2) + Cbi'*drbpb + Helper.cross3d(twist_body_i(3), ripb); 
                twist_pusher_i(3) = twist_body_i(3);
            end
            us = [twist_pusher_i ];
        end
                %% Convert reaction forces to associated robot velocity
        function [us] = velocity2force(obj, xs_object, twist_object_i)
            %Extract variables
            ribi = xs_object(1:2);
            theta = xs_object(3);
            Cbi = Helper.C3_2d(theta);
            if length(obj.p.xp)==2
                twist_pusher_i = zeros(2,1);
                twist_pusher_i(1:2) = twist_body_i(1:2) + Cbi'*drbpb + Helper.cross3d(twist_body_i(3), ripb); 
            elseif length(obj.p.xp)==3
                twist_pusher_i = zeros(3,1);
                twist_pusher_i(1:2) = twist_body_i(1:2) + Cbi'*drbpb + Helper.cross3d(twist_body_i(3), ripb); 
            end
            %Direction cosine matrices
            Cbi = Helper.C3_2d(theta);
            rbpb = [rx;ry];
            ripb = Cbi'*rbpb;
            drbpb = [0;dry];
            twist_body_i = obj.twist_object_i_fun(xc, uc);
            %Forces
            if length(obj.p.xp)==2
                twist_pusher_i = zeros(2,1);
                twist_pusher_i(1:2) = twist_body_i(1:2) + Cbi'*drbpb + Helper.cross3d(twist_body_i(3), ripb); 

            elseif length(obj.p.xp)==3
                twist_pusher_i = zeros(3,1);
                twist_pusher_i(1:2) = twist_body_i(1:2) + Cbi'*drbpb + Helper.cross3d(twist_body_i(3), ripb); 
            end
            us = twist_pusher_i;
        end
        %% point Simulator
        function [dxs] = forceSimulator(obj, xc, uc) 
            %Extract variables
            ribi = xc(1:2);
            theta = xc(3);
            ry = xc(4);
            %Direction cosine matrices
            Cbi = Helper.C3_2d(theta);
            twist_body_i = obj.twist_object_i_fun(xc, uc);
            %Compute twist of pusher
            vel_pusher_i = obj.force2Velocity(xc, uc);
            vp_b = Cbi*vel_pusher_i(1:2);
            %output
            dxs = [twist_body_i;vel_pusher_i];
        end
        
    end
end