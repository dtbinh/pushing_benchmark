function out1 = dk_dx_fun3(in1,in2,in3)
%DK_DX_FUN3
%    OUT1 = DK_DX_FUN3(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    18-Apr-2018 09:44:59

ry1 = in1(4,:);
v1 = in2(1,:);
v2 = in2(2,:);
x_data1 = in3(1,:);
x_data2 = in3(2,:);
x_data3 = in3(3,:);
out1 = exp((ry1.*2.545193306514084e1-x_data3.*7.671979948456739e-1).*(ry1.*3.31751819427795e1-x_data3).*(-1.0./2.0)-(v2.*7.17269234575182-x_data2.*6.319656041241728e-1).*(v2.*1.134981445025366e1-x_data2).*(1.0./2.0)-(v1.*5.19896119655713-x_data1).*(v1.*2.468479849543444-x_data1.*4.748025146212146e-1).*(1.0./2.0)).*(ry1.*6.6350363885559e1-x_data3.*2.0).*(-1.731793374998843e1);
