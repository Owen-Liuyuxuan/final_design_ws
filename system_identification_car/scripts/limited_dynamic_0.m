%% Constants
Ts = 0.04;
T_delay = ureal('T_delay', 0.2, 'Range', [0.02, 0.5]);
V = ureal('V', 10, 'Range', [1, 15]);
l_sum = ureal('l_sum', 2.8, 'Range', [2.79, 2.9]); % 1.25
nominal_steering_ratio = ureal('steering_ratio', 14.8, 'Range', [14, 16]);

%% Structure
W_steering_delay = V/l_sum/nominal_steering_ratio * tf(1, [T_delay, 1]);
W_steering_delay.u = 'steering_cmd';
W_steering_delay.y = 'omega';

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
W_u_weights = 0.2 * tf([0.2, 1],[0.1 3]);
W_u_weights.u = 'steering_cmd';
W_u_weights.y = 'z2';

%% Integration
ICInputs = {'d1', 'r', 'steering_cmd'};
ICOutputs = {'z1', 'z2', 'r', 'y1'};
entiress = connect(W_steering_delay,...
    W_error, W_noise_trans, W_meas,...
    W_error_weights, W_u_weights,...
    ICInputs, ICOutputs);

%% Controller synthesis
[K_2, CL, gamma, info] = h2syn(entiress, 2, 1);
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
K_miu = dksyn(entiress, 2, 1);

%%
index = 0;
for i = 1:size(K_miu.A, 1)
    K_temp = reduce(K_miu, i);
    K_temp = c2d(K_temp, Ts);
    if max(svd(K_temp.A)) < 1
        index = i;
    end
end
if index == 0
    Kmiu = c2d(K_miu, Ts);
else
    Kmiu = c2d(reduce(K_miu, index), Ts);
end

[KmiuA, KmiuB, KmiuC, KmiuD] = ssdata(Kmiu);

%%
[KA, KB, KC, KD] = ssdata(Kmiu);
save ./matlab_omega_control/H_miu_control_matrix.mat KA KB KC KD
[KA, KB, KC, KD] = ssdata(K);

save all_current_space_2.mat