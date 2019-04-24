function [error_y, error_phi, desired_phi_dot] = get_sine_info_2(data, amplitude)
%UNTITLED3 Summary of this function goes here
%   data(:,i) = x, y, theta, v_command, omega_command, throttle_cmd,
%   steering_cmd, omega
speed = data(:,4);
wavelength = 200 + 50 * sin(data(:,1) * 2 * pi / 2500);
w = 2 * pi ./ wavelength;
desired_y = amplitude * sin(data(:,1) .* w);
error_y = data(:,2) - desired_y;
desired_phi = atan2(w .* amplitude .* cos(data(:,1) .* w),1);
error_phi = data(:,3) - desired_phi;
desired_phi_dot = 1.0./(1+ (w.*amplitude.*cos(data(:,1).*w)).^2) .* (-w.^2.*amplitude.*sin(data(:,1).*w)) .* speed;
end