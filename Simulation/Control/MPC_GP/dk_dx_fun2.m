function out1 = dk_dx_fun2(in1,in2,in3)
%DK_DX_FUN2
%    OUT1 = DK_DX_FUN2(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    18-Apr-2018 09:44:58

ry1 = in1(4,:);
v1 = in2(1,:);
v2 = in2(2,:);
x_data1 = in3(1,:);
x_data2 = in3(2,:);
x_data3 = in3(3,:);
out1 = exp((v1.*2.767213338879429-x_data1.*4.615824127831468e-1).*(v1.*5.995058005339291-x_data1).*(-1.0./2.0)-(ry1.*2.165685256372107e1-x_data3.*7.042082895246099e-1).*(ry1.*3.075347576260565e1-x_data3).*(1.0./2.0)-(v2.*1.626458377802002e1-x_data2).*(v2.*5.451385378709441-x_data2.*3.351690675341136e-1).*(1.0./2.0)).*(ry1.*6.150695152521131e1-x_data3.*2.0).*(-1.521729509305749e-2);