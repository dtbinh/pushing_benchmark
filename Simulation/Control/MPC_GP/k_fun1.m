function out1 = k_fun1(in1,in2,in3)
%K_FUN1
%    OUT1 = K_FUN1(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    17-Apr-2018 16:43:18

ry1 = in1(4,:);
v1 = in2(1,:);
v2 = in2(2,:);
x_data1 = in3(1,:);
x_data2 = in3(2,:);
x_data3 = in3(3,:);
out1 = exp((v1.*5.989315239510151-x_data1.*2.416291805858003).*(v1.*2.47872182697048-x_data1).*(-1.0./2.0)-(v2.*4.820547867915905-x_data2.*7.016433306393015e-1).*(v2.*6.870367973887343-x_data2).*(1.0./2.0)-(ry1.*2.680358876367743e1-x_data3).*(ry1.*3.132548584960435e1-x_data3.*1.168704912084561).*(1.0./2.0)).*7.946801322558919e-3+1.15074330409099e-3;
