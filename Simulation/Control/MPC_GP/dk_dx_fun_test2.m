function out1 = dk_dx_fun_test2(in1,in2,in3)
%DK_DX_FUN_TEST2
%    OUT1 = DK_DX_FUN_TEST2(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    24-Apr-2018 09:25:19

ry1 = in1(4,:);
v1 = in2(1,:);
v2 = in2(2,:);
x_data1 = in3(1,:);
x_data2 = in3(2,:);
x_data3 = in3(3,:);
out1 = [0.0,0.0,0.0,exp((ry1.*2.148748630186113e1-x_data3.*2.858210115846079e-1).*(ry1.*7.517811998051959e1-x_data3).*(-1.0./2.0)-(v1.*5.049351444162502-x_data1.*3.383831090413436e-1).*(v1.*1.492199613174419e1-x_data1).*(1.0./2.0)-(v2.*1.942309355953989e1-x_data2.*4.86220912003827e-1).*(v2.*3.994705509372869e1-x_data2).*(1.0./2.0)).*(ry1.*1.615388823281087e3-x_data3.*2.148748630186113e1).*(-3.920564801172746e-5)];