 load all_current_space_0.mat

%%
plot(noise_delay_deadzone_complex_miu_error.Time, noise_delay_deadzone_complex_miu_error.Data , 'b-',...
    noise_delay_deadzone_complex_PID_error.Time, noise_delay_deadzone_complex_PID_error.Data, 'r-')
disp('slow real')
sqrt(mean(noise_delay_deadzone_complex_miu_error.Data.^2))
sqrt(mean(noise_delay_deadzone_complex_PID_error.Data.^2))
%%
plot(noise_delay_deadzone_complex_miu_error_fast.Time, noise_delay_deadzone_complex_miu_error_fast.Data , 'b-',...
    noise_delay_deadzone_complex_PID_error_fast.Time, noise_delay_deadzone_complex_PID_error_fast.Data, 'r-')
disp('fast simulation')
sqrt(mean(noise_delay_deadzone_complex_miu_error_fast.Data.^2))
sqrt(mean(noise_delay_deadzone_complex_PID_error_fast.Data.^2))
save all_current_space_0.mat