classdef MPCforce < dynamicprops  
    properties (Constant)
        h_opt = 0.01;
        NumFam = 3;
        steps = 35;
        num_int = 8;
    end
    
    properties
        planner;
        Opt;
        Opt_tmp;
        Opt_fom;
        Q;
        Qf;
        R;
        solver_flag;
        mode_index_vec;
        Linear;
        data;
        object;
        A;
        B;
    end
   
    methods
        %% Constructor
        function obj = MPCforce(planner, solver_flag, Q, Qf, R, Linear, data, object)  
            obj.planner = planner;
            obj.Linear=Linear;
            obj.data=data;
            try
                obj.object=object;
            catch
            end
            obj.solver_flag = solver_flag;
            %define weight matrices for MPC
            obj.Q = Q;
            obj.Qf=  Qf;
            obj.R = R;
            %use MIQP or FOM formulation
            t_init = 0;
            if strcmp(solver_flag,'is_miqp')
                obj.buildMIQPProgram(t_init);
            elseif strcmp(solver_flag,'is_fom')
                obj.buildProgram(t_init, [0;0;0;0]);
            elseif strcmp(solver_flag,'is_learning')
                obj.buildProgram(t_init);
                obj.buildMIQPProgram(t_init);
            end
        end
        %% solve MPC function using FOM
        function [uc] = solveMPC(obj, xc, t)
            %Define variables
            x = xc(1);
            y = xc(2);
            theta = xc(3);
            ry = xc(4);
            %Nominal coordinates
            [xcStar, ucStar, xsStar, usStar] = obj.getStateNominal(t);
            obj.buildProgram(t,xc);
            %Build error state vector
%             xc(4) = unwrap_custom(xc(4));
%             xcStar(4) = unwrap_custom(xcStar(4));

            delta_xc = [xc - xcStar];

            A_nom = obj.planner.ps.A_fun(xcStar, ucStar);
            B_nom = obj.planner.ps.B_fun(xcStar, ucStar);
            A_bar = eye(4)+obj.h_opt*A_nom;
            B_bar = obj.h_opt*B_nom;
 
            %Loop through family of modes
            fVal = [];
            for lv2=1:3
            %Build optimization program
                %Update initial conditions
                obj.Opt_fom{lv2}.beq(1:4) = zeros(4,1);
                obj.Opt_fom{lv2}.beq(1:4) = [A_bar*delta_xc];
                % Solve Opt Program   
                options = optimoptions('quadprog','Display','none');
                tic
                [obj.Opt_fom{lv2}, solvertime{lv2}, fval{lv2}] = obj.Opt_fom{lv2}.solve;
                toc
                
                out_delta_u{lv2} = obj.Opt_fom{lv2}.vars.u.value';
                out_delta_x{lv2} = obj.Opt_fom{lv2}.vars.x.value';
                fVal = [fVal; fval{lv2}];
            end
            %Find mode schedule with lowest cost
            [minFOM indexFOM] = min(fVal);
            
            %Return first element of control sequence
           delta_u = out_delta_u{indexFOM}(1,1:obj.planner.ps.num_ucStates)';
            %Add feedforward and feedback controls together
            uc = delta_u + ucStar;
        end
                %% solve MPC function using FOM
        function [uc] = solveMPC_gp(obj, xc, t)
            %Define variables
            x = xc(1);
            y = xc(2);
            theta = xc(3);
            ry = xc(4);
            %Nominal coordinates
            [xcStar, ucStar, xsStar, usStar,A,B] = obj.getStateNominal(t);
            obj.buildProgram(t,xc);
            %Build error state vector
%             xc(4) = unwrap_custom(xc(4));
%             xcStar(4) = unwrap_custom(xcStar(4));

            delta_xc = [xc - xcStar];
%             A(:,4) = zeros(4,1);
%             B(:,3) = zeros(4,1);
            A_nom = A*0 + 1*obj.planner.ps.A_fun(xcStar, ucStar);
            B_nom = B*0 + 1*obj.planner.ps.B_fun(xcStar, ucStar);
            A_bar = eye(4)+obj.h_opt*A_nom;
            B_bar = obj.h_opt*B_nom;
 
            %Loop through family of modes
            fVal = [];
            for lv2=1:3
            %Build optimization program
                %Update initial conditions
                obj.Opt_fom{lv2}.beq(1:4) = zeros(4,1);
                obj.Opt_fom{lv2}.beq(1:4) = [A_bar*delta_xc];
                % Solve Opt Program   
                options = optimoptions('quadprog','Display','none');
                tic
                [obj.Opt_fom{lv2}, solvertime{lv2}, fval{lv2}] = obj.Opt_fom{lv2}.solve;
                toc
                
                out_delta_u{lv2} = obj.Opt_fom{lv2}.vars.u.value';
                out_delta_x{lv2} = obj.Opt_fom{lv2}.vars.x.value';
                fVal = [fVal; fval{lv2}];
            end
            %Find mode schedule with lowest cost
            [minFOM indexFOM] = min(fVal);
            
            %Return first element of control sequence
           delta_u = out_delta_u{indexFOM}(1,1:obj.planner.ps.num_ucStates)';
            %Add feedforward and feedback controls together
            uc = delta_u*1 + ucStar;
        end
        %% solve MPC function using FOM
        function delta_uc = solveFOM_delta(obj, xc, t)
            %Define variables
            x = xc(1);
            y = xc(2);
            theta = xc(3);
            ry = xc(4);
            %Nominal coordinates
            [xcStar, ucStar, usStar, usStar, A_gp, B_gp] = obj.getStateNominal(t);
            obj.buildProgram(t,xc);
            %Build error state vector
            delta_xc = [xc - xcStar];

            A_nom = obj.planner.ps.A_fun(xcStar, ucStar);
            B_nom = obj.planner.ps.B_fun(xcStar, ucStar);
            A_bar = eye(4)+obj.h_opt*A_nom;
            B_bar = obj.h*B_nom;
 
            %Loop through family of modes
            fVal = [];
            for lv2=1:3
            %Build optimization program
                %Update initial conditions
                obj.Opt_fom{lv2}.beq(1:4) = zeros(4,1);
                obj.Opt_fom{lv2}.beq(1:4) = [A_bar*delta_xc];
                % Solve Opt Program   
                options = optimoptions('quadprog','Display','none');
                tic
                [obj.Opt_fom{lv2}, solvertime{lv2}, fval{lv2}] = obj.Opt_fom{lv2}.solve;
                toc
                
                out_delta_u{lv2} = obj.Opt_fom{lv2}.vars.u.value';
                out_delta_x{lv2} = obj.Opt_fom{lv2}.vars.x.value';
                fVal = [fVal; fval{lv2}];
            end
            %Find mode schedule with lowest cost
            [minFOM indexFOM] = min(fVal);
            
            %Return first element of control sequence
           delta_uc = out_delta_u{indexFOM};
            %Add feedforward and feedback controls together
%             uc = delta_u + ucStar;
        end
        %% solve MPC function using FOM
        function [uc, fval] = solveMPC_Mode(obj, xc, t, mode_schedule)
            %Define variables
            x = xc(1);
            y = xc(2);
            theta = xc(3);
            ry = xc(4);
            %Nominal coordinates
            [xcStar, ucStar, usStar, usStar] = obj.getStateNominal(t);
            %Build error state vector
            delta_xc = [xc - xcStar];
            %Define linear matrices
            A_nom = obj.planner.ps.A_fun(xcStar, ucStar);
            B_nom = obj.planner.ps.B_fun(xcStar, ucStar);
            A_bar = eye(4)+obj.h*A_nom;
            B_bar = obj.h*B_nom;         
            %Build optimization program
            obj.buildProgram_mode(mode_schedule, t);
            %Update initial conditions
            obj.Opt_tmp.beq(1:4) = zeros(4,1);
            obj.Opt_tmp.beq(1:4) = [A_bar*delta_xc];
            % Solve Opt Program   
            options = optimoptions('quadprog','Display','none');
            [obj.Opt_tmp, solvertime, fval] = obj.Opt_tmp.solve;
            out_delta_u= obj.Opt_tmp.vars.u.value';
            out_delta_x = obj.Opt_tmp.vars.x.value';            
            %Return first element of control sequence
            delta_u = out_delta_u(1,1:obj.planner.ps.num_ucStates)';
            
            %Add feedforward and feedback controls together
            uc = delta_u + ucStar;
%             clear obj.Opt
        end
        %% solve MPC optimization problem
        function [uc, modes] = solveMPC_MIQP(obj, xc, t)
            %Define variables
            x = xc(1);
            y = xc(2);
            theta = xc(3);
            ry = xc(4);
            %Nominal coordinates
            [xcStar, ucStar, usStar, usStar] = obj.getStateNominal(t);
            %Build error state vector
            delta_xc = [xc - xcStar];
            
            A_nom = obj.planner.ps.A_fun(xcStar, ucStar);
            B_nom = obj.planner.ps.B_fun(xcStar, ucStar);
            A_bar = eye(4)+obj.h*A_nom;
            B_bar = obj.h*B_nom;
            %Loop through family of modes
            fVal = [];
            %Build optimization program
            obj.buildMIQPProgram(t);
            %Build optimization program
            obj.Opt.beq(1:4) = zeros(4,1);
            obj.Opt.beq(1:4) = [A_bar*delta_xc];
            % Solve Opt Program   
            options = optimoptions('quadprog','Display','none');
            tic
            [obj.Opt, solvertime, fval] = obj.Opt.solve;
            toc
            out_delta_u = obj.Opt.vars.u.value';
            out_delta_x = obj.Opt.vars.x.value';
            %Return first element of control sequence
            delta_u = out_delta_u(1,1:obj.planner.ps.num_ucStates)';
            %Add feedforward and feedback controls together
            uc = delta_u + ucStar;
            modes = obj.Opt.vars.region.value';
        end
        %% Get nominal trajectory values at time T
        function [xcStar, ucStar, xsStar, usStar, A, B] = getStateNominal(obj, t)
            vecDif = t - obj.planner.t_star;
            [valDif, indexDif] = min(abs(vecDif));
            xcStar = obj.planner.xc_star(indexDif,:)';
            ucStar = obj.planner.uc_star(indexDif,:)'; 
            xsStar = obj.planner.xs_star(indexDif,:)';
            usStar = obj.planner.us_star(indexDif,:)'; 
            try
                A_Star = obj.planner.A_star(indexDif,:)';
                B_Star = obj.planner.B_star(indexDif,:)';

                A = reshape(obj.planner.A_star(indexDif,:,:), 4,4);
                B = reshape(obj.planner.B_star(indexDif,:,:), 4,2); 
            catch
                A=zeros(size(obj.planner.xc_star,2),size(obj.planner.xc_star,2));
                B=zeros(size(obj.planner.xc_star,2),size(obj.planner.uc_star,2));
            end
        end
        %Get nominal trajectory values at time T
        function [tStar] = getxStateNominal(obj, x)
            vecDif = x -  obj.planner.xc_star(:,1)';
            [valDif, indexDif] = min(abs(vecDif));
            tStar = obj.t_star(indexDif); 
        end

       %% Build optimization program and constraints
        function obj = buildProgram(obj, t_init, xc)
            t0 = t_init;
            for lv1=1:3
                Opt = MixedIntegerConvexProgram(false);
                Opt = Opt.addVariable('u', 'C', [obj.planner.ps.num_ucStates, obj.steps], -1000*ones(obj.planner.ps.num_ucStates,obj.steps), 1000*ones(obj.planner.ps.num_ucStates,obj.steps));
                Opt = Opt.addVariable('x', 'C', [obj.planner.ps.num_xcStates, obj.steps], -1000*ones(obj.planner.ps.num_xcStates,obj.steps), 1000*ones(obj.planner.ps.num_xcStates,obj.steps));
                %Loop through steps of MPC
                for lv3=1:obj.steps 
                    [xcStar, ucStar, usStar, usStar, A, B] = obj.getStateNominal(t_init);  

                    %Add cost
                    Opt = obj.buildCost(Opt, lv3);
                    %Add dynamic constraints
                    Opt = obj.addMotionConstraints(Opt, lv3, xcStar, ucStar, A, B);
                    Opt = obj.addForceIndConstraints(Opt, lv3, xcStar, ucStar);
%                     if lv3 ==1
%                         Opt = obj.addVelocityConstraints(Opt, lv3, xcStar, ucStar,xc);
%                     end
                    
                    if lv3==1
                        if lv1==1 
                            %Add mode dependant constraints
                            Opt = obj.addForceDepConstraints(Opt, lv3, 1, xcStar, ucStar);%1: Sticking, 2: Sliding left(up), 3: Slide right(down)
                        elseif lv1==2 
                            %Add mode dependant constraints
                            Opt = obj.addForceDepConstraints(Opt, lv3, 2, xcStar, ucStar);%1: Sticking, 2: Sliding left(up), 3: Slide right(down)
                        elseif lv1==3 
                            %Add mode dependant constraints
                            Opt = obj.addForceDepConstraints(Opt, lv3, 3, xcStar, ucStar);%1: Sticking, 2: Sliding left(up), 3: Slide right(down)
                        end
                    else
                        %Add mode dependant constraints
                         Opt = obj.addForceDepConstraints(Opt, lv3, 1, xcStar, ucStar);%1: Sticking, 2: Sliding left(up), 3: Slide right(down)
                    end  
                    t_init = t_init + obj.h_opt;
                end
                obj.Opt_fom{lv1} = Opt;
                t_init = t0;
            end
        end
        %% Build optimization program and constraints
        function obj = buildMIQPProgram(obj, t_init)
            Opt = MixedIntegerConvexProgram(false);
            Opt = Opt.addVariable('u', 'C', [obj.planner.ps.num_ucStates, obj.steps], -1000*ones(obj.planner.ps.num_ucStates,obj.steps), 1000*ones(obj.planner.ps.num_ucStates,obj.steps));
            Opt = Opt.addVariable('x', 'C', [obj.planner.ps.num_xcStates, obj.steps], -1000*ones(obj.planner.ps.num_xcStates,obj.steps), 1000*ones(obj.planner.ps.num_xcStates,obj.steps));
            Opt = Opt.addVariable('region', 'B', [3, obj.num_int], 0, 1);
            index_vec = [1,5,5,5,5,5,5,4];
            obj.mode_index_vec = [];
            for lv4=1:length(index_vec)
                obj.mode_index_vec = [obj.mode_index_vec lv4*ones(1,index_vec(lv4))];
            end
            %Loop through steps of MPC
            for lv3=1:obj.steps 
                [xcStar, ucStar, usStar, usStar] = obj.getStateNominal(t_init);
                %Add cost
                Opt = obj.buildCost(Opt, lv3);
                %Add dynamic constraints
                Opt = obj.addMotionConstraints(Opt, lv3, xcStar, ucStar);
                Opt = obj.addForceIndConstraints(Opt, lv3, xcStar, ucStar);
                Opt = obj.addMIQPForceConstraints(Opt, obj.mode_index_vec(lv3), lv3, xcStar, ucStar);
                t_init = t_init + obj.h_opt;
            end
            obj.Opt = Opt;
end
        %% Build optimization program and constraints
        function obj = buildProgram_mode(obj, mode_schedule, t_init)
            Opt = MixedIntegerConvexProgram(false);
            Opt = Opt.addVariable('u', 'C', [obj.planner.ps.num_ucStates, obj.steps], -1000*ones(obj.planner.ps.num_ucStates,obj.steps), 1000*ones(obj.planner.ps.num_ucStates,obj.steps));
            Opt = Opt.addVariable('x', 'C', [obj.planner.ps.num_xcStates, obj.steps], -1000*ones(obj.planner.ps.num_xcStates,obj.steps), 1000*ones(obj.planner.ps.num_xcStates,obj.steps));
            %Loop through steps of MPC
            for lv3=1:obj.steps 
                [xcStar, ucStar, usStar, usStar] = obj.getStateNominal(t_init);
                %Add cost
                Opt = obj.buildCost(Opt, lv3);
                %Add dynamic constraints
                Opt = obj.addMotionConstraints(Opt, lv3, xcStar, ucStar);
                Opt = obj.addForceIndConstraints(Opt, lv3, xcStar, ucStar);
                Opt = obj.addForceDepConstraints(Opt, lv3, mode_schedule(lv3), xcStar, ucStar);
                t_init = t_init + obj.h_opt;
            end
            obj.Opt_tmp = Opt;
        end
        %% Build cost matrix
        function Opt = buildCost(obj, Opt, lv1)
            %Initialize matrices
            H = zeros(Opt.nv, Opt.nv);
            % State Cost
            if lv1<obj.steps
                H(Opt.vars.x.i(1:length(obj.Q),lv1), Opt.vars.x.i(1:length(obj.Q),lv1)) = obj.Q;
            else
                H(Opt.vars.x.i(1:length(obj.Qf),lv1), Opt.vars.x.i(1:length(obj.Qf),lv1)) = obj.Qf;
            end
            if strcmp(obj.solver_flag,'is_miqp')
    %             H(Opt.vars.region.i(2:3,obj.mode_index_vec(lv1)), Opt.vars.region.i(2:3,obj.mode_index_vec(lv1))) = eye(10);
                if obj.mode_index_vec(lv1) == 2
                    H(Opt.vars.region.i(2, obj.mode_index_vec(lv1)), Opt.vars.region.i(2, obj.mode_index_vec(lv1))) = .3;  
                    H(Opt.vars.region.i(3, obj.mode_index_vec(lv1)), Opt.vars.region.i(3, obj.mode_index_vec(lv1))) = .3;  
                elseif obj.mode_index_vec(lv1) == 3
                    H(Opt.vars.region.i(2, obj.mode_index_vec(lv1)), Opt.vars.region.i(2, obj.mode_index_vec(lv1))) = .1;  
                    H(Opt.vars.region.i(3, obj.mode_index_vec(lv1)), Opt.vars.region.i(3, obj.mode_index_vec(lv1))) = .1;
                elseif obj.mode_index_vec(lv1) == 4
                    H(Opt.vars.region.i(2, obj.mode_index_vec(lv1)), Opt.vars.region.i(2, obj.mode_index_vec(lv1))) = .1;  
                    H(Opt.vars.region.i(3, obj.mode_index_vec(lv1)), Opt.vars.region.i(3, obj.mode_index_vec(lv1))) = .1;
                elseif obj.mode_index_vec(lv1) == 5
                    H(Opt.vars.region.i(2, obj.mode_index_vec(lv1)), Opt.vars.region.i(2, obj.mode_index_vec(lv1))) = .1;  
                    H(Opt.vars.region.i(3, obj.mode_index_vec(lv1)), Opt.vars.region.i(3, obj.mode_index_vec(lv1))) = .1;
                end
            end
                %Inputs Cost
            H(Opt.vars.u.i(1:length(obj.R),lv1), Opt.vars.u.i(1:length(obj.R),lv1)) = obj.R;
            %Add cost to program
            Opt = Opt.addCost(H, [], []);
        end
                %% Build dynamic constraints
        function Opt = addMotionConstraints(obj, Opt, lv1, xcStar, ucStar, A, B)
%             A(:,4) = zeros(4,1);
%             B(:,3) = zeros(4,1);
            A_nom = A*0+1*obj.planner.ps.A_fun(xcStar, ucStar);
            B_nom = B*0+1*obj.planner.ps.B_fun(xcStar, ucStar);
            A_bar = eye(obj.planner.ps.num_xcStates)+obj.h_opt*A_nom;
            B_bar = obj.h_opt*B_nom;

            numConstraints = obj.planner.ps.num_xcStates;
            Aeq = zeros(numConstraints, Opt.nv);
            %Special case of initial conditions
            if lv1 ~=1
            Aeq(:,Opt.vars.x.i(1:obj.planner.ps.num_xcStates,lv1-1))= -A_bar;
            end
            Aeq(:,Opt.vars.x.i(1:obj.planner.ps.num_xcStates,lv1))  = eye(obj.planner.ps.num_xcStates);
            Aeq(:,Opt.vars.u.i(1:obj.planner.ps.num_ucStates,lv1))=  -B_bar;
            beq = zeros(obj.planner.ps.num_xcStates,1);
            Opt = Opt.addLinearConstraints([], [], Aeq, beq);
        end
        %% Build forces constraints
        function Opt = addMIQPForceConstraints(obj, Opt, mode_var, lv1, xcStar, ucStar)
            xn = xcStar;
            un = ucStar;
            M = 10;
            
            [Aeq, beq, Ain, bin] = obj.forceIndConstraintMatrices(xn,un);
            num_ineq_constraints = size(Ain,1);
            num_eq_constraints = size(Aeq,1);

            %Add inequality constraints
            Amatrix = zeros(num_ineq_constraints, Opt.nv);
            bmatrix = zeros(num_ineq_constraints, 1);
            Amatrix(:,Opt.vars.u.i(:,lv1)) = Ain;
            bmatrix = bin;
            Opt =  Opt.addLinearConstraints(Amatrix, bmatrix, [], []);
            clear Amatrix bmatrix num_ineq_constraints

            %Add equality constraints
            Amatrix = zeros(num_eq_constraints, Opt.nv);
            bmatrix = zeros(num_eq_constraints, 1);
            Amatrix(:,Opt.vars.u.i(:,lv1)) = Aeq;
            bmatrix  =  beq;
            Opt =  Opt.addLinearConstraints([],[],Amatrix, bmatrix);
            clear Amatrix bmatrix  num_eq_constraints
 
            clear Aeq beq Ain bin
                
            for lv2=1:3
                [Aeq{lv2}, beq{lv2}, Ain{lv2}, bin{lv2}] = obj.forceDepConstraintMatrices(xn,un, lv2);
                num_ineq_constraints = size(Ain{lv2},1);
                num_eq_constraints = size(Aeq{lv2},1);
                
                %Add inequality constraints
                Amatrix = zeros(num_ineq_constraints, Opt.nv);
                bmatrix = zeros(num_ineq_constraints, 1);
                Amatrix(:,Opt.vars.u.i(:,lv1)) = Ain{lv2};
                Amatrix(:,Opt.vars.region.i(lv2,mode_var)) = M;
                bmatrix = bin{lv2} + M;
                Opt =  Opt.addLinearConstraints(Amatrix, bmatrix, [], []);
                clear Amatrix bmatrix num_ineq_constraints

                %Add equality constraints
                Amatrix = zeros(num_eq_constraints, Opt.nv);
                bmatrix = zeros(num_eq_constraints, 1);
                Amatrix2 = zeros(num_eq_constraints, Opt.nv);
                bmatrix2 = zeros(num_eq_constraints, 1);
                Amatrix(:,Opt.vars.u.i(:,lv1)) = Aeq{lv2};
                Amatrix2(:,Opt.vars.u.i(:,lv1)) = -Aeq{lv2};
                Amatrix(:,Opt.vars.region.i(lv2,mode_var)) = M;
                Amatrix2(:,Opt.vars.region.i(lv2,mode_var)) = M;
                bmatrix  =  beq{lv2} + M;
                bmatrix2 = -beq{lv2} + M;
                Opt =  Opt.addLinearConstraints(Amatrix, bmatrix,[],[]);
                Opt =  Opt.addLinearConstraints(Amatrix2, bmatrix2, [], []);
                clear Amatrix bmatrix  num_eq_constraints
            end
            % sum(region) == 1
            Aeq_comp = zeros(1, Opt.nv);
            Aeq_comp(1, Opt.vars.region.i(:,mode_var)) = 1;
            beq_comp = 1;
            Opt = Opt.addLinearConstraints([], [], Aeq_comp, beq_comp);
        end
        %% Build force (dep) constraints
        function Opt = addForceDepConstraints(obj, Opt, lv1, mode, xcStar, ucStar)
            xn = xcStar;
            un = ucStar;
            
            [Aeq, beq, Ain, bin] = obj.forceDepConstraintMatrices(xn,un,mode);
            numConstraints = size(Ain,1);
            Amatrix = zeros(numConstraints, Opt.nv);
            bmatrix = zeros(numConstraints, 1);
            Amatrix(:,Opt.vars.u.i(:,lv1)) = Ain;
            bmatrix = bin;
            Opt =  Opt.addLinearConstraints(Amatrix, bmatrix, [], []);
            clear Amatrix bmatrix
            
            numConstraints = size(Aeq,1);
            Amatrix = zeros(numConstraints, Opt.nv);
            bmatrix = zeros(numConstraints, 1);
            Amatrix(:,Opt.vars.u.i(:,lv1)) = Aeq;
            bmatrix = beq;
            Opt =  Opt.addLinearConstraints([], [], Amatrix, bmatrix);
            clear Amatrix bmatrix   
        end
        %% Build force (ind) constraints
        function Opt = addForceIndConstraints(obj, Opt, lv1, xcStar, ucStar)
            xn = xcStar;
            un = ucStar;
            
            [Aeq, beq, Ain, bin] = obj.forceIndConstraintMatrices(xn,un);
            numConstraints = size(Ain,1);
            Amatrix = zeros(numConstraints, Opt.nv);
            bmatrix = zeros(numConstraints, 1);
            Amatrix(:,Opt.vars.u.i(:,lv1)) = Ain;
            bmatrix = bin;
            Opt =  Opt.addLinearConstraints(Amatrix, bmatrix, [], []);
            clear Amatrix bmatrix
            
            numConstraints = size(Aeq,1);
            Amatrix = zeros(numConstraints, Opt.nv);
            bmatrix = zeros(numConstraints, 1);
            Amatrix(:,Opt.vars.u.i(:,lv1)) = Aeq;
            bmatrix = beq;
            Opt =  Opt.addLinearConstraints([], [], Amatrix, bmatrix);
            clear Amatrix bmatrix   
        end
        
                %% Build force (ind) constraints
        function Opt = addVelocityConstraints(obj, Opt, lv1, xcStar, ucStar, xc)
            xn = xcStar;
            un = ucStar;
            
            [Aeq, beq, Ain, bin] = obj.velocityConstraintMatrices(xn, un, xc);
            numConstraints = size(Ain,1);
            Amatrix = zeros(numConstraints, Opt.nv);
            bmatrix = zeros(numConstraints, 1);
            Amatrix(:,Opt.vars.u.i(:,lv1)) = Ain;
            bmatrix = bin;
            Opt =  Opt.addLinearConstraints(Amatrix, bmatrix, [], []);
            clear Amatrix bmatrix
            
            numConstraints = size(Aeq,1);
            Amatrix = zeros(numConstraints, Opt.nv);
            bmatrix = zeros(numConstraints, 1);
            Amatrix(:,Opt.vars.u.i(:,lv1)) = Aeq;
            bmatrix = beq;
            Opt =  Opt.addLinearConstraints([], [], Amatrix, bmatrix);
            clear Amatrix bmatrix   
        end
        
        
        %% Dynamic constraints
        function [A, B] = motionConstraintMatrices(obj, xn, un)
            A = planner.ps.A_fun(xn, un);
            B = planner.ps.B_fun(xn, un);
        end

        %% Force (mode dependant) constraints
        function [Aeq, beq, Ain, bin] = forceDepConstraintMatrices(obj, xn, un, mode)
            %Initialize matrices
            Aeq = [];
            beq = [];
            Ain = [];
            bin = [];
            num_contacts = obj.planner.ps.p.num_contacts;
            if mode==1
                %sticking constraints
                num_constraints = 1;
                index = [2*num_contacts+1];
                Aeq_tmp = 1;
                beq_tmp =  - Aeq_tmp*un(index);
                Aeq = [Aeq;zeros(num_constraints, length(un))];
                beq = [beq;zeros(num_constraints, 1)];
                Aeq(end-num_constraints+1:end,index) = Aeq_tmp;
                beq(end-num_constraints+1:end) = beq_tmp;
                clear Aeq_tmp beq_tmp
            elseif mode==2
                %sliding up
                num_constraints = 1;
                index = [2*num_contacts+1];
                Ain_tmp = -1;
                bin_tmp =  - Ain_tmp*un(index);
                Ain = [Ain;zeros(num_constraints, length(un))];
                bin = [bin;zeros(num_constraints, 1)];
                Ain(end-num_constraints+1:end,index) = Ain_tmp;
                bin(end-num_constraints+1:end) = bin_tmp;
                clear Ain_tmp bin_tmp
                for j=1:num_contacts
                    %tangential force of contact point 1 must lie on bondary of FC
                    num_constraints = 1;
                    index = [j,j+num_contacts];
                    Aeq_tmp = [-obj.planner.ps.p.nu_p 1];
                    beq_tmp =  - Aeq_tmp*un(index);
                    Aeq = [Aeq;zeros(num_constraints, length(un))];
                    beq = [beq;zeros(num_constraints, 1)];
                    Aeq(end-num_constraints+1:end,index) = Aeq_tmp;
                    beq(end-num_constraints+1:end) = beq_tmp;
                    clear Aeq_tmp beq_tmp
                end
            elseif mode==3
                %sliding up
                num_constraints = 1;
                index = [2*num_contacts+1];
                Ain_tmp = 1;
                bin_tmp =  -Ain_tmp*un(index);
                Ain = [Ain;zeros(num_constraints, length(un))];
                bin = [bin;zeros(num_constraints, 1)];
                Ain(end-num_constraints+1:end,index) = Ain_tmp;
                bin(end-num_constraints+1:end) = bin_tmp;
                clear Ain_tmp bin_tmp
                for j=1:num_contacts
                    %tangential force of contact point 1 must lie on bondary of FC
                    num_constraints = 1;
                    index = [j,j+num_contacts];
                    Aeq_tmp = [obj.planner.ps.p.nu_p 1];
                    beq_tmp =  - Aeq_tmp*un(index);
                    Aeq = [Aeq;zeros(num_constraints, length(un))];
                    beq = [beq;zeros(num_constraints, 1)];
                    Aeq(end-num_constraints+1:end,index) = Aeq_tmp;
                    beq(end-num_constraints+1:end) = beq_tmp;
                    clear Aeq_tmp beq_tmp
                end
            end
        end
        
         %% velocity (mode independant) constraints
        function [Aeq, beq, Ain, bin] = velocityConstraintMatrices(obj, xn, un, xc)
            %Initialize constraint matrices
            Aeq = [];
            beq = [];
            Ain = [];
            bin = [];
            %
%             A = obj.planner.ps.A_fun(xn, un);
%             B = obj.planner.ps.B_fun(xn, un);
            num_contacts = obj.planner.ps.p.num_contacts;
            %~state variables
            x = xc(1);
            y = xc(2);
            theta = xc(3);
            ry = xc(4);
            %
            rx = -obj.planner.ps.o.a/2;
            rbpb = [rx;ry];
            Cbi = Helper.C3_2d(theta);
            R = [transpose(Cbi) [0; 0];0 0 1];
            G = zeros(2, length(un));
            G(end,end) = 1;
            I12 = [1 0 0;0 1 0];
            n3 = [0;0;1];
            sign_vec = [1 -1]*1;
            %~For each contact point do:
            N_tot=[];
            T_tot=[];
            for lv1=1:num_contacts %lv1 represents contact point 1 and 2
                rb{lv1} = [rx*1;ry*1]+sign_vec(lv1)*[0;obj.planner.ps.p.d]; %position of contact point lv1
                Jb{lv1} = [1 0 -rb{lv1}(2);0 1 rb{lv1}(1)];
                n{lv1} = [1;0];
                t{lv1} = [0;1];
                N{lv1} = transpose(n{lv1})*Jb{lv1};
                T{lv1} = transpose(t{lv1})*Jb{lv1};
                
                N_tot = [N_tot;N{lv1}];
                T_tot = [T_tot;T{lv1}];
            end

            E = [N_tot' T_tot' zeros(3,1)];
            F = I12*obj.planner.ps.A_ls*E;
            O = [0 -1;1 0]*rbpb*n3'*obj.planner.ps.A_ls*E;
            H = [F + G + O; n3'*obj.planner.ps.A_ls*E];
            
            %Build constraint matrices in velocity space
            %Velocity upper bound
            tp_max = [.3;.4;1];
            Ain_tp = blkdiag(eye(3));
            bin_tp = [tp_max];
            Ain_hat = Ain_tp*H;
            bin_hat = bin_tp - Ain_tp*H*un;
          
            num_constraints = 3;
            index = [1:length(un)];
            Ain_tmp = Ain_hat;
            bin_tmp = bin_hat;
            Ain = [Ain;zeros(num_constraints, length(un))];
            bin = [bin;zeros(num_constraints, 1)];
            Ain(end-num_constraints+1:end,index) = Ain_tmp;
            bin(end-num_constraints+1:end) = bin_tmp;
            clear Ain_tmp bin_tmp
            %Velocity lower bound
            tp_max = [-.03;.4;1];
            Ain_tp = blkdiag(eye(3));
            bin_tp = [tp_max];
            Ain_hat = -Ain_tp*H;
            bin_hat = bin_tp + Ain_tp*H*un;
          
            num_constraints = 3;
            index = [1:length(un)];
            Ain_tmp = Ain_hat;
            bin_tmp = bin_hat;
            Ain = [Ain;zeros(num_constraints, length(un))];
            bin = [bin;zeros(num_constraints, 1)];
            Ain(end-num_constraints+1:end,index) = Ain_tmp;
            bin(end-num_constraints+1:end) = bin_tmp;
            clear Ain_tmp bin_tmp
        end

        %% Force (mode independant) constraints
        function [Aeq, beq, Ain, bin] = forceIndConstraintMatrices(obj, xn, un)
            %Initialize matrices
            Aeq = [];
            beq = [];
            Ain = [];
            bin = [];
            num_contacts = obj.planner.ps.p.num_contacts;
            %% tangential velocity bounds
            num_constraints = 1;
            index = [2*num_contacts+1];
            Ain_tmp = 1;
            bin_tmp = 0.05; %upper limit (i.e. dry<0.05)
            Ain = [Ain;zeros(num_constraints, length(un))];
            bin = [bin;zeros(num_constraints, 1)];
            Ain(end-num_constraints+1:end,index) = Ain_tmp;
            bin(end-num_constraints+1:end) = bin_tmp;
            clear Ain_tmp bin_tmp
            %% tangential velocity bounds
            num_constraints = 1;
            index = [2*num_contacts+1];
            Ain_tmp = -1;
            bin_tmp = 0.05; %lower limit (i.e. dry>-0.05)
            Ain = [Ain;zeros(num_constraints, length(un))];
            bin = [bin;zeros(num_constraints, 1)];
            Ain(end-num_constraints+1:end,index) = Ain_tmp;
            bin(end-num_constraints+1:end) = bin_tmp;
            clear Ain_tmp bin_tmp
            % positive normal force
            num_constraints = num_contacts;
            index = [1:num_contacts];
            Ain_tmp = -eye(num_constraints);
            bin_tmp = zeros(num_constraints,1) - Ain_tmp*un(index);
            Ain = [Ain;zeros(num_constraints, length(un))];
            bin = [bin;zeros(num_constraints, 1)];
            Ain(end-num_constraints+1:end,index) = Ain_tmp;
            bin(end-num_constraints+1:end) = bin_tmp;
            clear Ain_tmp bin_tmp
            %%  normal force bounds
%             upper bound (applied on deviation only, aka only linear term)
            num_constraints = num_contacts;
            index = [1:num_contacts];
            Ain_tmp = eye(num_constraints);
            max_normal = 0.15; %upper limit (i.e. delta_fn< 0.05)
            bin_tmp = [max_normal]*ones(num_constraints,1); 
            Ain = [Ain;zeros(num_constraints, length(un))];
            bin = [bin;zeros(num_constraints, 1)];
            Ain(end-num_constraints+1:end,index) = Ain_tmp;
            bin(end-num_constraints+1:end) = bin_tmp;
            clear Ain_tmp bin_tmp
%             lower bound (applied on deviation only, aka only linear term)
            num_constraints = num_contacts;
            index = [1:num_contacts];
            Ain_tmp = -eye(num_constraints);
            max_normal = 0.13; %lower limit (i.e. delta_fn> -0.1)
            bin_tmp = [max_normal]*ones(num_constraints,1); 
            Ain = [Ain;zeros(num_constraints, length(un))];
            bin = [bin;zeros(num_constraints, 1)];
            Ain(end-num_constraints+1:end,index) = Ain_tmp;
            bin(end-num_constraints+1:end) = bin_tmp;
            clear Ain_tmp bin_tmp
            %%  friction cone (contact point 1)
            for j=1:num_contacts
                %upper border
                index = [j,j+num_contacts];
                num_constraints = 1;
                Ain_tmp = [-obj.planner.ps.p.nu_p 1];
                bin_tmp =  - Ain_tmp*un(index);
                Ain = [Ain;zeros(num_constraints, length(un))];
                bin = [bin;zeros(num_constraints, 1)];
                Ain(end-num_constraints+1:end,index) = Ain_tmp;
                bin(end-num_constraints+1:end) = bin_tmp;
                clear Ain_tmp bin_tmp
                %lower border
                num_constraints = 1;
                Ain_tmp = -[obj.planner.ps.p.nu_p 1];
                bin_tmp =  - Ain_tmp*un(index);
                Ain = [Ain;zeros(num_constraints, length(un))];
                bin = [bin;zeros(num_constraints, 1)];
                Ain(end-num_constraints+1:end,index) = Ain_tmp;
                bin(end-num_constraints+1:end) = bin_tmp;
                clear Ain_tmp bin_tmp
            end
        end
    end
end