l_sum = 1.6;
ssused = Kmiu;
v = 10;
steering_ratio = 1;

%%
x = 0;
y = 0;
theta = 0;
omega = 0;
delta = 0;
alpha = Ts;
% alpha = 1;
index = 1;
Tmax = 20;
t_list = 0:Ts:Tmax;
omega_list = zeros([1, size(t_list,2)]);
omega_expectation = 0.1;
ssused_state = zeros(size(ssused.A, 1), 1);
for t=0:Ts:Tmax
    omega_error = omega_expectation - omega;
    input_vector = [omega_error];
    ssused_state = ssused.A*ssused_state + ssused.B*input_vector;
    delta_cmd = ssused.C*ssused_state + ssused.D * input_vector + steering_ratio * l_sum * omega_expectation/v;
    delta = delta_cmd * alpha + (1 - alpha) * delta;
    omega = tan(delta/steering_ratio)/l_sum * v;
    omega_list(1, index) = omega;
    index = index + 1;
end
%%
plot(t_list, omega_list)