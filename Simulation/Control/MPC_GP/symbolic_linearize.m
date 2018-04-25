clear
%clc
close all

pusher_gp = PointPusher(.3);
object_gp = Square();
surface_gp = Surface(.35);
planar_system_gp = PlanarSystem(pusher_gp, object_gp, surface_gp);
load('learning_output.mat');

%build variables
xo = sym('xo', [3,1]);
rx = -object_gp.a/2;
ry = sym('ry', [1,1]);
x = [xo;ry];
v = sym('v', [2,1]);
u = sym('u', [3,1]);
x_data = sym('x_data', [3,1]);
gp_input = [v;ry];
theta = sym('theta', [5,1]);

%Build symbolic functions
for lv1=1:3
    %build symbolic derivatives of kernel
    [k_sym{lv1} dk_sym{lv1}] = covFunc(data.theta1{lv1}, x_data, gp_input./exp(data.lengthscales{lv1}(:)), exp(data.lengthscales{lv1}(:)));
%     [k_sym{lv1} dk_sym{lv1}] = covFunc(theta, x_data, gp_input);
    dk_dv_sym{lv1} = dk_sym{lv1}(1:2,:);
    dk_dry_sym{lv1} = dk_sym{lv1}(3,:);
    dk_dx_sym_test{lv1} = jacobian(k_sym{lv1}, x);
    dk_dv_sym_test{lv1} = jacobian(k_sym{lv1}, v);
    %convert to matlab function
    Linear.k_fun{lv1}=matlabFunction(k_sym{lv1}, 'Vars', {x,v,x_data}, 'File',strcat('k_fun', num2str(lv1)));
    Linear.dk_fun{lv1}=matlabFunction(dk_sym{lv1}, 'Vars', {x,v,x_data}, 'File',strcat('dk_fun', num2str(lv1)));
    Linear.dk_dry_fun{lv1}=matlabFunction(dk_dry_sym{lv1}, 'Vars', {x,v,x_data}, 'File', strcat('dk_dx_fun', num2str(lv1)));
    Linear.dk_dv_fun{lv1}=matlabFunction(dk_dv_sym{lv1}, 'Vars', {x,v,x_data}, 'File', strcat('dk_dv_fun', num2str(lv1)));
    Linear.dk_dx_fun_test{lv1}=matlabFunction(dk_dx_sym_test{lv1}, 'Vars', {x,v,x_data}, 'File', strcat('dk_dx_fun_test', num2str(lv1)));
    Linear.dk_dv_fun_test{lv1}=matlabFunction(dk_dv_sym_test{lv1}, 'Vars', {x,v,x_data}, 'File', strcat('dk_dv_fun_test', num2str(lv1)));
end

%rotational kinematics
Cbi = Helper.C3_2d(x(3));
R = [transpose(Cbi) [0; 0];0 0 1];
R_fun = matlabFunction(R,  'Vars', {x}, 'File', 'R_fun');
dR_dtheta = diff(R, x(3));
Linear.dR_dtheta_fun = matlabFunction(dR_dtheta,  'Vars', {x}, 'File', 'dR_fun');
v_var = planar_system_gp.Gc_fun(x)*u;
dv_dx = jacobian(v_var, x);
dv_du = jacobian(v_var, u);
Linear.dv_dx_fun = matlabFunction(dv_dx,'Vars', {x,u});
Linear.dv_du_fun = matlabFunction(dv_du,'Vars', {x});
Linear.Gc_fun = planar_system_gp.Gc_fun;

% [A,B] = GP_linearization_u([0;0;3.1416;0], [.32;0;0], Linear, data, object);
return
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rx = -object.a/2;
%Build A and B matrices
x_star = [0;0;0;0];
u_star = [.32;0;0];
v_star = Linear.Gc_fun(x_star)*u_star;

%build large derivative matrices and gp function output
N = length(data.X{1});
D = 4;
g = [];
dg_dx = [];
dg_dv = [];
for lv1=1:3
    %initialize
    dK_dx{lv1} = zeros(N,4);
    dK_dv{lv1} = zeros(N,2);
    k_star{lv1} = zeros(N,1);
    K1_star_sym{lv1}=[];
    dK1_dx_star_sym{lv1}=[];
    dK1_dv_star_sym{lv1}=[];
    tic
    
    k_star{lv1}=Linear.k_fun{lv1}(x_star,v_star,data.X{lv1}(1:N,:)');
    dK_ry{lv1}=Linear.dk_dry_fun{lv1}(x_star,v_star,data.X{lv1}(1:N,:)');
    dK_dx{lv1}=[zeros(3,length(dK_ry{lv1}));dK_ry{lv1}];
    dK_dv{lv1}=Linear.dk_dv_fun{lv1}(x_star,v_star,data.X{lv1}(1:N,:)');
%     for n=1:N
%         n;
%         %build k_star column matrix with equilibrium data 
%         k_star{lv1}(n) = Linear.k_fun{lv1}(x_star,v_star,data.X{lv1}(n,:)');
%         %concat kernel derivatives and sub in equilibrium numbers
%         dK_dx{lv1}(n,:) = Linear.dk_dx_fun{lv1}(x_star,v_star,data.X{lv1}(n,:)');
%         dK_dv{lv1}(n,:) = Linear.dk_dv_fun{lv1}(x_star,v_star,data.X{lv1}(n,:)');
%     end
%     [~, K1star] = feval(data.covfunc1{lv1}{:}, data.theta1{lv1}, data.X{lv1}, [1 2 3]./exp(data.lengthscales{lv1}(:)'));
%     twist_b = [twist_b;K1star'*obj.data.alpha{lv1}];
    %concat gp output into a numerical column vector
    g = [g;k_star{lv1}*data.alpha{lv1}(1:N)];
    %concat gp derivatives together
    dg_dx = [dg_dx;transpose(dK_dx{lv1}*data.alpha{lv1}(1:N))];
    dg_dv = [dg_dv;transpose(dK_dv{lv1}*data.alpha{lv1}(1:N))];

end

%build expression for dry=dx(4) (note: dry = vt-)

%% Need to account for missing expression
vbpi=v;
% vbbi=g();
% dry_dx = 0 - 0 - 0;
% dry_dv = 1 - dgp2_u - diff(gp(3)xrbpb. u);
% dg_gx2 = [dK1_dx_star_Fun{1}(x_star, v_star)'*data.alpha{1}, dK1_dx_star_Fun{2}(x_star, v_star)'*data.alpha{2}, dK1_dx_star_Fun{3}(x_star, v_star)'*data.alpha{3}]';
dry_dx = [0, 0, 0, -dg_dx(2,4)-rx*dg_dx(3,4)];
dry_dv = [0-dg_dv(2,1)-rx*dg_dv(3,1), 1-dg_dv(2,2)-rx*dg_dv(3,2)];

dR_dx = [zeros(size(Linear.dR_dtheta_fun(x_star)))*g zeros(size(Linear.dR_dtheta_fun(x_star)))*g Linear.dR_dtheta_fun(x_star)*g zeros(size(Linear.dR_dtheta_fun(x_star)))*0*g];

A = [dR_dx + R_fun(x_star)*(dg_dx+dg_dv*Linear.dv_dx_fun(x_star, u_star)); dry_dx + dry_dv*Linear.dv_dx_fun(x_star, u_star)];
B = [R_fun(x_star)*dg_dv*Linear.Gc_fun(x_star); dry_dv*Linear.Gc_fun(x_star)];

% clear xo rx ry x u x_data gp_input Cbi
return
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v_new = Linear.Gc_fun(x)*u;
gp_input = [v_new;ry];

fb = twist_b_gp(gp_input);
fb_fun = matlabFunction(fb, 'Vars', {x,u});

dfb_dx = jacobian(fb, x);
dfb_du = jacobian(fb, u);
dfb_dx_fun = matlabFunction(dfb_dx, 'Vars', {x,u});
dfb_du_fun = matlabFunction(dfb_du, 'Vars', {x,u});

2
fi = R*fb;
dfi_dx = jacobian(fi, x);
dfi_du = jacobian(fi, u);
dfi_dx_fun = matlabFunction(dfi_dx, 'Vars', {x,u});
dfi_du_fun = matlabFunction(dfi_du, 'Vars', {x,u});
3
vbpi_test = v_new;
vbbi_test = fb(1:2);
vbpb_test = vbpi_test-vbbi_test-Helper.cross3d(fi(3), [rx;ry]);
dry_test = vbpb_test(2);
dry_dx_test = jacobian(dry_test,x);
dry_du_test = jacobian(dry_test,u);
dry_dx_fun_test = matlabFunction(dry_dx_test, 'Vars', {x,u});
dry_du_fun_test = matlabFunction(dry_du_test, 'Vars', {x,u});
4
% dry_fun = matlabFunction(dry, 'Vars', {x,v});
5
f_non = [fi;dry_test];
A_fun = jacobian(f_non, x);
B_fun = jacobian(f_non, u);
6
A_fun = matlabFunction(A_fun, 'Vars', {x,u});
B_fun = matlabFunction(B_fun, 'Vars', {x,u});
