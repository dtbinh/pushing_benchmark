classdef MPC < dynamicprops  
    properties (Constant)
        h_opt = 0.01;
        steps = 50;
    end
    
    properties
        planner;
        Opt;
        Q;
        Qf;
        R;
        Linear;
        data;
        object;
        A;
        B;
    end
   
    methods
        %% Constructor
        function obj = MPC(planner, Linear, data, object)  
            obj.planner = planner;
            obj.Linear=Linear;
            obj.data=data;
            obj.object=object;
%             obj.A=A;
%             obj.B=B;
%Simulation
%                 obj.Q  = 10*diag([5,3,.1,0.1]);
%                 obj.Qf = 2000*diag([5,3,1,0.1]);
%                 obj.R  = .01*diag([1,10,0.5]);
%Experiments
            obj.Q = 1*diag([1,1,.01,0.0000]);
            obj.Qf=  1*2000*diag([1,1,.1,.0000]);
            obj.R = .01*diag([1,1]);

            %use MIQP or FOM formulation
            t_init = 0;
            obj.buildProgram(t_init);

        end
        %% solve MPC function using FOM
        function [uc] = solveMPC(obj, xc, t)
            %Define variables
            x = xc(1);
            y = xc(2);
            theta = xc(3);
            ry = xc(4);
            %Nominal coordinates
            [xcStar, ucStar, xsStar, usStar, A_nom, B_nom] = obj.getStateNominal(t);
            obj.buildProgram(t,xc);
            %Build error state vector
            delta_xc = [xc - xcStar];
%             A_nom = obj.A;
%             B_nom = obj.B;
%             [A_nom, B_nom] = GP_linearization(xcStar, usStar(1:2), obj.Linear, obj.data, obj.object);
            A_bar = eye(4)+obj.h_opt*A_nom;
            B_bar = obj.h_opt*B_nom;
 
            %Build optimization program
            %Update initial conditions
            obj.Opt.beq(1:4) = zeros(4,1);
            obj.Opt.beq(1:4) = [A_bar*delta_xc];
            % Solve Opt Program   
            options = optimoptions('quadprog','Display','none');
%             tic
            [obj.Opt, solvertime, fval] = obj.Opt.solve;
%             toc

            out_delta_u = obj.Opt.vars.u.value';
            out_delta_x = obj.Opt.vars.x.value';

            %Return first element of control sequence
            delta_u = out_delta_u(1,1:obj.planner.ps.num_ucStates)';
            %Add feedforward and feedback controls together
             uc = delta_u*1 + ucStar(1:2);
%              delta_u
%              uc
%              A_bar
        end
      
        %% Get nominal trajectory values at time T
        function [xcStar, ucStar, xsStar, usStar, A, B] = getStateNominal(obj, t)
            vecDif = t - obj.planner.t_star;
            [valDif, indexDif] = min(abs(vecDif));
            xcStar = obj.planner.xc_star(indexDif,:)';
            ucStar = obj.planner.uc_star(indexDif,:)'; 
            xsStar = obj.planner.xs_star(indexDif,:)';
            usStar = obj.planner.us_star(indexDif,:)';
            A = reshape(obj.planner.A_star(indexDif,:,:), 4,4);
            B = reshape(obj.planner.B_star(indexDif,:,:), 4,2); 
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
                Opt = obj.addVelocityConstraints(Opt, lv3, xcStar, ucStar);
%                 Opt = obj.addStateConstraints(Opt, lv3, xcStar, ucStar);

                t_init = t_init + obj.h_opt;
            end
            obj.Opt = Opt;
            t_init = t0;
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
            %Inputs Cost
            H(Opt.vars.u.i(1:length(obj.R),lv1), Opt.vars.u.i(1:length(obj.R),lv1)) = obj.R;
            %Add cost to program
            Opt = Opt.addCost(H, [], []);
        end
        
        %% Build dynamic constraints
        function Opt = addMotionConstraints(obj, Opt, lv1, xcStar, ucStar, A_nom, B_nom)

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
        
        %% Build force (dep) constraints
        function Opt = addVelocityConstraints(obj, Opt, lv1, xcStar, ucStar)
            xn = xcStar;
            un = ucStar;
            
            [Aeq, beq, Ain, bin] = obj.addVelocityConstraintsMatrices(xn,un);
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
        function Opt = addStateConstraints(obj, Opt, lv1, xcStar, ucStar)
            xn = xcStar;
            un = ucStar;
            
            [Aeq, beq, Ain, bin] = obj.addStateConstraintsMatrices(xn,un);
            numConstraints = size(Ain,1);
            Amatrix = zeros(numConstraints, Opt.nv);
            bmatrix = zeros(numConstraints, 1);
            Amatrix(:,Opt.vars.x.i(:,lv1)) = Ain;
            bmatrix = bin;
            Opt =  Opt.addLinearConstraints(Amatrix, bmatrix, [], []);
            clear Amatrix bmatrix
            
            numConstraints = size(Aeq,1);
            Amatrix = zeros(numConstraints, Opt.nv);
            bmatrix = zeros(numConstraints, 1);
            Amatrix(:,Opt.vars.x.i(:,lv1)) = Aeq;
            bmatrix = beq;
            Opt =  Opt.addLinearConstraints([], [], Amatrix, bmatrix);
            clear Amatrix bmatrix   
        end
        
        %% Velocity constraints
        function [Aeq, beq, Ain, bin] = addStateConstraintsMatrices(obj, xn, un)
            %Initialize matrices
            Aeq = [];
            beq = [];
            Ain = [];
            bin = [];
            %% tangential velocity bounds
            num_constraints = 2;
            index = [4];
            Ain_tmp = [1;-1];
            bin_tmp = [.03-xn(4);.03+xn(4)];
            Ain = [Ain;zeros(num_constraints, length(xn))];
            bin = [bin;zeros(num_constraints, 1)];
            Ain(end-num_constraints+1:end,index) = Ain_tmp;
            bin(end-num_constraints+1:end) = bin_tmp;
            clear Ain_tmp bin_tmp

        end
        
                %% Force (mode independant) constraints
        function [Aeq, beq, Ain, bin] = addVelocityConstraintsMatrices(obj, xn, un)
            %Initialize matrices
            Aeq = [];
            beq = [];
            Ain = [];
            bin = [];
            %% tangential velocity bounds
            num_constraints = 4;
            Ain_tmp = [1 0;0 1;-1 0;0 -1];
            bin_tmp = [.15;.15; 0.03; .15];
            Ain = [Ain;zeros(num_constraints, 2)];
            bin = [bin;zeros(num_constraints, 1)];
            Ain(end-num_constraints+1:end,:) = Ain_tmp;
            bin(end-num_constraints+1:end) = bin_tmp;
            clear Ain_tmp bin_tmp

        end
    end
end