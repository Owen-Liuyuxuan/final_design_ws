function [ss1] = omega_get_ss()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
m = ureal("m", 1500, 'Percentage', 5);
cf = ureal("cf", 19000, 'Percentage', 15);
cr = ureal("cr", 30000, 'Percentage', 15);
lf = ureal("lf", 1.2, 'Percentage', 10);
lr = ureal("lr", 1.3, 'Percentage', 10);
iz = ureal("Iz", 2500, 'Percentage', 20);
v = ureal("v", 10, 'Range', [5, 15]);
% m = 1500; cf = 19000; cr = 30000; lf = 1.2;
% lr = 1.6; iz = 2500; v = 15;
A = [-(cf + cr)/(m * v), (1 + (cf*lf - cr * lr)/(m * v^2));...
    -(cf*lf-cr*lr)/(iz), (cf * lf^2 + cr * lr^2)/(iz * v)];
B = [1, 0, -(1 + (cf*lf - cr * lr)/(m * v^2)) cf/m/v;...
    0, 1, -(cf * lf^2 + cr * lr^2)/(iz * v), cf*lf/iz];
C = [0 1];
D = 0;
ss1 = ss(A,B,C,D);
end

