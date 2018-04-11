function [A, B] = GP_linearization(x, u, Linear, data, object)

rx = -object.a/2;
%Build A and B matrices
x_star = x;
u_star = u;

%build large derivative matrices and gp function output
N = length(data.X{1});
D = 4;
g = [];
dg_dx = [];
dg_du = [];
for lv1=1:3
    %initialize
    dK_dx{lv1} = zeros(N,4);
    dK_du{lv1} = zeros(N,2);
    k_star{lv1} = zeros(N,1);
    K1_star_sym{lv1}=[];
    dK1_dx_star_sym{lv1}=[];
    dK1_du_star_sym{lv1}=[];
    tic
    k_star{lv1} = Linear.k_fun{lv1}(x_star,u_star,data.X{lv1}(:,:)');
    dK_dx{lv1} = Linear.dk_dx_fun{lv1}(x_star,u_star,data.X{lv1}(:,:)');
    dK_du{lv1} = Linear.dk_du_fun{lv1}(x_star,u_star,data.X{lv1}(:,:)');
%     for n=1:N
%         n;
%         %build k_star column matrix with equilibrium data 
%         k_star{lv1}(n) = Linear.k_fun{lv1}(x_star,u_star,data.X{lv1}(n,:)');
%         %concat kernel derivatives and sub in equilibrium numbers
%         dK_dx{lv1}(n,:) = Linear.dk_dx_fun{lv1}(x_star,u_star,data.X{lv1}(n,:)');
%         dK_du{lv1}(n,:) = Linear.dk_du_fun{lv1}(x_star,u_star,data.X{lv1}(n,:)');
%     end
%     [~, K1star] = feval(data.covfunc1{lv1}{:}, data.theta1{lv1}, data.X{lv1}, [1 2 3]./exp(data.lengthscales{lv1}(:)'));
%     twist_b = [twist_b;K1star'*obj.data.alpha{lv1}];
    %concat gp output into a numerical column vector
    g = [g;k_star{lv1}*data.alpha{lv1}];
    %concat gp derivatives together
    dg_dx = [dg_dx;transpose(dK_dx{lv1}*data.alpha{lv1})];
    dg_du = [dg_du;transpose(dK_du{lv1}*data.alpha{lv1})];

end

%build expression for dry=dx(4) (note: dry = vt-)

%% Need to account for missing expression
vbpi=u;
% vbbi=g();
% dry_dx = 0 - 0 - 0;
% dry_du = 1 - dgp2_u - diff(gp(3)xrbpb. u);
% dg_gx2 = [dK1_dx_star_Fun{1}(x_star, u_star)'*data.alpha{1}, dK1_dx_star_Fun{2}(x_star, u_star)'*data.alpha{2}, dK1_dx_star_Fun{3}(x_star, u_star)'*data.alpha{3}]';
dry_dx = [0, 0, 0, -dg_dx(2,4)-rx*dg_dx(3,4)];
dry_du = [0-dg_du(2,1)-rx*dg_du(3,1), 1-dg_du(2,2)-rx*dg_du(3,2)];

dR_dx = [zeros(size(Linear.dR_dtheta_fun(x_star)))*g zeros(size(Linear.dR_dtheta_fun(x_star)))*g Linear.dR_dtheta_fun(x_star)*g zeros(size(Linear.dR_dtheta_fun(x_star)))*0*g];

A = [dR_dx + R_fun(x_star)*dg_dx; dry_dx];
B = [R_fun(x_star)*dg_du; dry_du];
end