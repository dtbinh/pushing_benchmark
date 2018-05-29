function [A, B] = GP_linearization_residual(x, u, Linear, data, object)

    rx = -object.a/2;
    %Build A and B matrices
    x_star = x;
    u_star = u;
    v_star = Linear.Gc_fun(x_star)*u_star;
    v=v_star;
    
    V_star =sqrt(v_star(1)^2+v_star(2)^2);%sqrt(v(1)^2+v(2)^2);%I(1);%
    c_star = 1/2-x(4)/object.a;
    phi_star = atan(v(2)/v(1)); %atan(v(2)/v(1));%I(3);%
%     I_star = [V_star, phi_star, c_star]';
    gp_input_star = [c_star; phi_star];
    V_nom = .2*.02;

    %build large derivative matrices and gp function output
    N = length(data.X{1});
    D = 4;
    g = [];
    dg_dx = [];
    dg_dv = [];
    dg_dI=[];
    dg_dphi=[];
    dg_dc=[];
    for lv1=1:3
        %initialize
        dK_dx{lv1} = zeros(N,4);
        dK_dv{lv1} = zeros(N,2);
        k_star{lv1} = zeros(N,1);
        K1_star_sym{lv1}=[];
        dK1_dx_star_sym{lv1}=[];
        dK1_dv_star_sym{lv1}=[];
        tic

        k_star{lv1}=Linear.k_fun{lv1}(x_star,gp_input_star,data.X{lv1}');
        dK_dc{lv1}=Linear.dk_dc_fun{lv1}(x_star,gp_input_star,data.X{lv1}');
        dK_dphi{lv1}=Linear.dk_dphi_fun{lv1}(x_star,gp_input_star,data.X{lv1}');
        
%         dK_dx{lv1}=[zeros(3,length(dK_ry{lv1}));dK_ry{lv1}];
%         dK_dv{lv1}=Linear.dk_dv_fun{lv1}(x_star,v_star,data.X{lv1}');
    %     for n=1:N
    %         n;
    %         %build k_star column matrix with equilibrium data 
    %         k_star{lv1}(n) = Linear.k_fun{lv1}(x_star,v_star,data.X{lv1}(n,:)');
    %         %concat kernel derivatives and sub in equilibrium numbers
    %         dK_dx{lv1}(n,:) = Linear.dk_dx_fun{lv1}(x_star,v_star,data.X{lv1}(n,:)');
    %         dK_dv{lv1}(n,:) = Linear.dk_du_fun{lv1}(x_star,v_star,data.X{lv1}(n,:)');
    %     end
    %     [~, K1star] = feval(data.covfunc1{lv1}{:}, data.theta1{lv1}, data.X{lv1}, [1 2 3]./exp(data.lengthscales{lv1}(:)'));
    %     twist_b = [twist_b;K1star'*obj.data.alpha{lv1}];
        %concat gp output into a numerical column vector
        g = [g;k_star{lv1}*data.alpha{lv1}];
        dg_dphi_tmp = dK_dphi{lv1}*data.alpha{lv1};
        dg_dc_tmp = dK_dc{lv1}*data.alpha{lv1};
        dg_dphi = [dg_dphi;dg_dphi_tmp];
        dg_dc = [dg_dc;dg_dc_tmp];
        dg_dI = [dg_dI;0 dg_dc_tmp dg_dphi_tmp];

        %concat gp derivatives together
%         dg_dx_tmp = dg_dI
%         dg_dx = [dg_dx;transpose(dK_dx{lv1}*data.alpha{lv1})];
%         dg_dv = [dg_dv;transpose(dK_dv{lv1}*data.alpha{lv1})];

    end
    
%     dg_dx = double(dg_dI*Linear.dI_dx);
%     dg_dv = double(dg_dI*Linear.dI_dv_fun(v_star));

    %compute partial derivatives
    dxb_dV = (1/V_nom)*Linear.Rbc_fun(x_star)*g;
    dxb_dphi = (V_star/V_nom)*(Linear.dRbc_dphi_fun(x_star)*g+Linear.Rbc_fun(x_star)*dg_dphi);
    dxb_dc = (V_star/V_nom)*(Linear.Rbc_fun(x_star)*dg_dc);
    dxb_dI = [dxb_dV dxb_dc dxb_dphi];
    
    dg_dv = double(dxb_dI*Linear.dI_dv_fun(v_star)*Linear.Gc_fun(x_star));
    dg_dx = double(dxb_dI*Linear.dI_dx + dxb_dI*Linear.dI_dv_fun(v_star)*Linear.dv_dx_fun(x_star, u_star));
    g = (V_star/V_nom)*g;


    %build expression for dry=dx(4) (note: dry = vt-)

    %% Need to account for missing expression
%     vbpi=v_star;
    % vbbi=g();
    % dry_dx = 0 - 0 - 0;
    % dry_dv = 1 - dgp2_u - diff(gp(3)xrbpb. u);
    % dg_gx2 = [dK1_dx_star_Fun{1}(x_star, v_star)'*data.alpha{1}, dK1_dx_star_Fun{2}(x_star, v_star)'*data.alpha{2}, dK1_dx_star_Fun{3}(x_star, v_star)'*data.alpha{3}]';
    dry_dx = [0, 0, 0, -dg_dx(2,4)-rx*dg_dx(3,4)];
    dry_dv = [0-dg_dv(2,1)-rx*dg_dv(3,1), 1-dg_dv(2,2)-rx*dg_dv(3,2)];

    dR_dx = [zeros(size(Linear.dRib_dtheta_fun(x_star)))*g zeros(size(Linear.dRib_dtheta_fun(x_star)))*g Linear.dRib_dtheta_fun(x_star)*g zeros(size(Linear.dRib_dtheta_fun(x_star)))*0*g];

    A = [dR_dx + R_fun(x_star)*(dg_dx); dry_dx*0];
    B = [R_fun(x_star)*dg_dv; zeros(1,3)];
end