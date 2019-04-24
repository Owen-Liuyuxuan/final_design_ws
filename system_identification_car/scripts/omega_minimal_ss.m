function [ss1] = omega_minimal_ss()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% m = 1600; cf = 30000; cr = 32000; lf = 1.2;
% lr = 1.6; iz = 2500; v = 15; steering_ratio = 14.8;


m = 1600; cf = 30000; cr = 32000; lf = 0.7;
lr = 0.9; iz = 2500; v = 5; steering_ratio = 1;

A = [-(cf + cr)/(m * v), -(1 + (cf*lf - cr * lr)/(m * v^2));...
    -(cf*lf-cr*lr)/(iz), -(cf * lf^2 + cr * lr^2)/(iz * v)];
B = [cf/m/v;...
    cf*lf/iz];
B = B./steering_ratio;
C = [0 1];
D = 0;

ss1 = ss(A, B, C, D);
end

