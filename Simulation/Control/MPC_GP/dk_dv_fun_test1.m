function out1 = dk_dv_fun_test1(in1,in2,in3)
%DK_DV_FUN_TEST1
%    OUT1 = DK_DV_FUN_TEST1(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    18-Apr-2018 09:44:57

ry1 = in1(4,:);
v1 = in2(1,:);
v2 = in2(2,:);
x_data1 = in3(1,:);
x_data2 = in3(2,:);
x_data3 = in3(3,:);
t2 = v1.*5.989315239510151;
t3 = t2-x_data1.*2.416291805858003;
t4 = v1.*2.47872182697048;
t5 = t4-x_data1;
t6 = v2.*4.820547867915905;
t7 = t6-x_data2.*7.016433306393015e-1;
t8 = v2.*6.870367973887343;
t9 = t8-x_data2;
t10 = ry1.*2.680358876367743e1;
t11 = t10-x_data3;
t12 = ry1.*3.132548584960435e1;
t13 = t12-x_data3.*1.168704912084561;
t14 = t3.*t5.*(-1.0./2.0)-t7.*t9.*(1.0./2.0)-t11.*t13.*(1.0./2.0);
t15 = exp(t14);
out1 = [t15.*(v1.*1.484584641278074e1-x_data1.*5.989315239510151).*(-7.946801322558919e-3),t15.*(v2.*3.311893768832035e1-x_data2.*4.820547867915905).*(-7.946801322558919e-3)];
