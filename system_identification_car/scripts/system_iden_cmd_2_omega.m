load ./matlab_data/miu_sine_data.mat
% x, y, theta, v_command, omega_command, throttle_cmd, steering_cmd, omega
my_input = miu_sine(1:end-2, 5);
my_output = miu_sine(3:end, 8);
t = 1:size(my_input, 1);
plot(t, my_input,'r-', t, my_output, 'b-')