clear 
clc
close all;

load('learning_output.mat');

x_star = [0;0;0;0];
u_star = [0;0];

%build variables
xo = sym('xo', [3,1]);
ry = sym('ry', [1,1]);
x = [xo;ry];
u = sym('u', [2,1]);
x_data = sym('x_data', [3,1]);
gp_input = [u;ry];

%rotational kinematics
Cbi = Helper.C3_2d(x(3));
R = [transpose(Cbi) [0; 0];0 0 1];
R_fun = matlabFunction(R,  'Vars', {x});
dR_dtheta = diff(R, x(3));

%Build symbolic functions
for lv1=1:3
    %build symbolic derivatives of kernel
    k_sym{lv1} = covFunc(data.theta1{lv1}, x_data, gp_input./exp(data.lengthscales{lv1}(:)));
    dk_dx{lv1} = jacobian(k_sym{lv1}, x);
    dk_du{lv1} = jacobian(k_sym{lv1}, u);
    %convert to matlab function
    k_fun{lv1} = matlabFunction(k_sym{lv1}, 'Vars', {x,u,x_data});
    dk_dx_fun{lv1} = matlabFunction(dk_dx{lv1}, 'Vars', {x,u,x_data});
    dk_du_fun{lv1} = matlabFunction(dk_du{lv1}, 'Vars', {x,u,x_data});
end

%build large derivative matrices and gp function output
tic
N = 5000;
D = 4;
g = [];
dg_dx = [];
dg_du = [];
for lv1=1:3
    %initialize
    dK_dx{lv1} = zeros(N,4);
    dK_du{lv1} = zeros(N,2);
    k_star{lv1} = zeros(N,1);
    for n=1:N
        %build k_star column matrix with equilibrium data 
        k_star{lv1}(n) = k_fun{lv1}(x_star,u_star,data.X{lv1}(n,:)');
        %concat kernel derivatives and sub in equilibrium numbers
        dK_dx{lv1}(n,:) = dk_dx_fun{lv1}(x_star,u_star,data.X{lv1}(n,:)');
        dK_du{lv1}(n,:) = dk_du_fun{lv1}(x_star,u_star,data.X{lv1}(n,:)');
    end
    %concat gp output into a numerical column vector
    g = [g;transpose(k_star{lv1})*data.alpha{lv1}];
    %concat gp derivatives together
    dg_dx = [dg_dx;transpose(transpose(dK_dx{lv1})*data.alpha{lv1})];
    dg_du = [dg_du;transpose(transpose(dK_du{lv1})*data.alpha{lv1})];
end
toc
%build expression for dry=dx(4) (note: dry = vt-)

%% Need to account for missing expression
vbpi=u;
% vbbi=g();
dry_dx = 0 - 0 - diff(gp(3)xrbpb. x);
dry_du = 1 - dgp2_u - diff(gp(3)xrbpb. u);


dgp_u2 = transpose(transpose(dK_du{2})*data.alpha{2});
dh_du = [0, 1 - dgp_u2(2)];

dR_dx = [zeros(dR_dtheta) dR_dtheta*0*g dR_dtheta*g dR_dtheta*0*g];
dR_dx_fun =  matlabFunction(dR_dx, 'Vars', {x});

A = [dR_dx_fun(x_star) + R_fun(x_star)*dg_dx; [0 0 0 0]];
B = [R_fun(x_star)*dg_du; dh_du];

