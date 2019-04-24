%% Constants
Ts = 0.04;
T_delay = ureal('T_delay', 0.2, 'Range', [0.02, 0.5]);
V = ureal('V', 1, 'Range', [0.1, 5]);
l_sum = ureal('l_sum', 1.65, 'Range', [1.6, 1.7]); % 1.25
nominal_steering_ratio = 1;

P = omega_minimal_ss();
[Ap,Bp,Cp,Dp]=ssdata(P);

%% Structure
W_steering_delay = V/l_sum/nominal_steering_ratio * tf(1, [T_delay, 1]);
W_steering_delay.u = 'steering_cmd';
W_steering_delay.y = 'omega';

W_steering_forward = tf(nominal_steering_ratio * l_sum.NominalValue/V);
W_steering_forward.u = 'r';
W_steering_forward.y = 'cmd_1';
W_steering_command = sumblk('steering_cmd = cmd_1 + u');

%% noise
W_error = sumblk('omega_error = r - omega');
W_noise_trans =  tf([2, 1], [1, 50]);
W_noise_trans.u = 'd1';
W_noise_trans.y = 'noise';
W_meas = sumblk('y1 = noise + omega_error');

%% weights
W_error_weights = 5 * tf([1],[1 0.1]);
W_error_weights.u = 'omega_error';
W_error_weights.y = 'z1';
W_u_weights = tf([0.2, 1],[0.1 3]);
W_u_weights.u = 'u';
W_u_weights.y = 'z2';

%% Integration
ICInputs = {'d1', 'r', 'u'};
ICOutputs = {'z1', 'z2', 'y1'};
entiress = connect(W_steering_delay,W_steering_forward, W_steering_command,...
    W_error, W_noise_trans, W_meas,...
    W_error_weights, W_u_weights,...
    ICInputs, ICOutputs);

%% Controller synthesis
[K_2, CL, gamma, info] = h2syn(entiress, 1, 1);
index = 0;
K = 0;
for i = 1:size(K_2.A, 1)
    K_temp = reduce(K_2, i);
    K_temp = c2d(K_temp, Ts);
    if max(svd(K_temp.A)) < 1
        index = i;
    end
end
if index == 0
    K = c2d(K_2, Ts);
else
    K = c2d(reduce(K_2, index), Ts);
end
svd(K.A)


%% miu syn
% [K_miu, ~, gamma] = dksyn(entiress, 1, 1);

%%
% index = 0;
% for i = 1:size(K_miu.A, 1)
%     K_temp = reduce(K_miu, i);
%     K_temp = c2d(K_temp, Ts);
%     if max(svd(K_temp.A)) < 1
%         index = i;
%     end
% end
% if index == 0
%     Kmiu = c2d(K_miu, Ts);
% else
%     Kmiu = c2d(reduce(K_miu, index), Ts);
% end
% 
% [KmiuA, KmiuB, KmiuC, KmiuD] = ssdata(Kmiu);

%%
% [KA, KB, KC, KD] = ssdata(Kmiu);
% save ./matlab_omega_control/H_miu_control_matrix.mat KA KB KC KD

load ./matlab_omega_control/H_miu_control_matrix.mat
Kmiu = ss(KA, KB, KC, KD, Ts);
[KmiuA, KmiuB, KmiuC, KmiuD] = ssdata(Kmiu);

[KA, KB, KC, KD] = ssdata(K);

PID2 = 0.03 + 0.72 * Ts *tf(1, [1, -1], Ts), 0.0003*1/Ts*tf([1,-1],[1,0], Ts);
save all_current_space_2.mat

