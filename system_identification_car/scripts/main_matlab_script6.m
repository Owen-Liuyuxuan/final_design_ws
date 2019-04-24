Ts = 0.04;
P = omega_minimal_ss();
[Ap,Bp,Cp,Dp]=ssdata(P);
discrete_P = c2d(P, Ts);
np=max(size(Ap));
% Bd = eye(np) * 5;
Bd = eye(np) * 5;
% We= 1 * tf([1],[1 0.005]); [Ae,Be,Ce,De]=ssdata(We);
We= 1 * tf([1],[1 0.005]); [Ae,Be,Ce,De]=ssdata(We);
ne=max(size(Ae));
% Wu= 5 * tf([0.01, 1],[0.001 3]); [Au,Bu,Cu,Du]=ssdata(Wu);
Wu= 0.3 * tf([0.01, 1],[0.001 5]); [Au,Bu,Cu,Du]=ssdata(Wu);
nu=max(size(Au));

A = [Ap, zeros(np,ne), zeros(np,nu);...
-Be*Cp, Ae, zeros(ne,nu);...
zeros(nu,np), zeros(nu,ne), Au];

B = [Bd, zeros(np, 1), Bp;...
    zeros(ne, np), Be, zeros(ne, nu);...
    zeros(nu, np + 1), Bu];

C = [-De*Cp, Ce, zeros(ne, nu); zeros(nu, np + ne), Cu; -Cp,zeros(1, ne), zeros(1, nu)];

D = [zeros(ne, np), De, zeros(ne, nu); zeros(nu, np + 1), Du; zeros(1, np), 1, zeros(1, nu)];

ss1 = ss(A,B,C,D);

ss1.StateName = {'beta', 'omega', 'error_cost', 'input_cost'};
ss1.InputName = {'beta_dist', 'omega_dist',...
    'omega_command', 'steering_angle'};
ss1.OutputName = {'omega_error', 'steering_angle_cost', 'omega_error_output'};

% W_delaying = tf(1, [0.1, 1]);
W_delaying = tf(1, [0.1, 1]);
W_delaying.u = 'steering_angle_cmd';
W_delaying.y = 'steering_angle';

W_noise = tf([1, 0.01], [1, 10]);
% W_noise = tf([1, 0.01], [1, 10]);
W_noise.u = 'd1';
W_noise.y = 'noise';
W_meas = sumblk('omega_meas = omega_error_output + noise');

entire_ss = connect(ss1, W_delaying, W_noise, W_meas,...
    {'beta_dist', 'omega_dist','omega_command', 'd1', 'steering_angle_cmd'},...
    {'omega_error', 'steering_angle_cost', 'omega_meas'});
K_2 = h2syn(entire_ss, 1, 1);
K_2 = reduce(K_2, 5);
K_2 = c2d(K_2, Ts);
svd(K_2.A)

% K_inf= hinfsyn(entire_ss, 1, 1);
% K_inf = reduce(K_inf, 4);
% K_inf = c2d(K_inf, Ts);
% svd(K_inf.A)
%%
G_miu = tf([252.2, 497.8, 2519], [1, 187.5, 3348, 872.6]);
[KA, KB, KC, KD] = ssdata(K_2);
save ./matlab_omega_control/H_2_control_matrix.mat  KA KB KC KD
% [KA, KB, KC, KD] = ssdata(K_inf);
% save ./matlab_omega_control/H_inf_control_matrix.mat  KA KB KC KD