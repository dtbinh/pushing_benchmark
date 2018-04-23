function [A, B] = GP_linearization(x, v, Linear, data, object)

    rx = -object.a/2;
    %Build A and B matrices
    x_star = x;
    v_star = v;
    
    V_star = 0.05;
    phi_star = 0;
    c_star = -rx;
%     I_star = [V_star, phi_star, c_star]';
    gp_input_star = [phi_star;c_star];

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
        dK_dphi{lv1}=Linear.dk_dphi_fun{lv1}(x_star,gp_input_star,data.X{lv1}');
        dK_dc{lv1}=Linear.dk_dc_fun{lv1}(x_star,gp_input_star,data.X{lv1}');
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
        dg_dI = [dg_dI;0 dg_dphi_tmp dg_dc_tmp];

        %concat gp derivatives together
%         dg_dx_tmp = dg_dI
%         dg_dx = [dg_dx;transpose(dK_dx{lv1}*data.alpha{lv1})];
%         dg_dv = [dg_dv;transpose(dK_dv{lv1}*data.alpha{lv1})];

    end
    
    dg_dx = double(dg_dI*Linear.dI_dx);
    dg_dv = double(dg_dI*Linear.dI_dv_fun(v_star));

    %compute partial derivatives
    dxb_dV = Linear.Rbc_fun(x_star)*g;
    dxb_dphi = V_star*(Linear.dRbc_dphi_fun(x_star)*g+Linear.Rbc_fun(x_star)*dg_dphi);
    dxb_dc = V_star*(Linear.Rbc_fun(x_star)*dg_dc);
    dxb_dI = [dxb_dV dxb_dphi dxb_dc];

    %build expression for dry=dx(4) (note: dry = vt-)

    %% Need to account for missing expression
    vbpi=v_star;
    % vbbi=g();
    % dry_dx = 0 - 0 - 0;
    % dry_dv = 1 - dgp2_u - diff(gp(3)xrbpb. u);
    % dg_gx2 = [dK1_dx_star_Fun{1}(x_star, v_star)'*data.alpha{1}, dK1_dx_star_Fun{2}(x_star, v_star)'*data.alpha{2}, dK1_dx_star_Fun{3}(x_star, v_star)'*data.alpha{3}]';
    dry_dx = [0, 0, 0, -dg_dx(2,4)-rx*dg_dx(3,4)];
    dry_dv = [0-dg_dv(2,1)-rx*dg_dv(3,1), 1-dg_dv(2,2)-rx*dg_dv(3,2)];

    dR_dx = [zeros(size(Linear.dRib_dtheta_fun(x_star)))*g zeros(size(Linear.dRib_dtheta_fun(x_star)))*g Linear.dRib_dtheta_fun(x_star)*g zeros(size(Linear.dRib_dtheta_fun(x_star)))*0*g];

    A = [dR_dx + R_fun(x_star)*dg_dx; dry_dx];
    B = [R_fun(x_star)*dg_dv; dry_dv];
end