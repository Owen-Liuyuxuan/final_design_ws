function [output, state] = miu_controller(curvature, current_steering_angle, previous_state, speed, wheelbase)
    persistent KA KB KC KD
    if isempty(KA)
        S = load("./matlab_omega_control/H_miu_control_matrix.mat");
        KA = S.KA;
        KB = S.KB;
        KC = S.KC;
        KD = S.KD;
    end
    ts_ = 0.04;
    expected_steering_angle = arcsin(curvature/wheelbase);
    expected_omega = sin(expected_steering_angle) * speed / wheelbase;
    current_omega = sin(current_steering_angle) * speed / wheelbase;
    error_omega = expected_omega - current_omega;
    state = KA * previous_state + KB * error_omega;
    feedback_output = KC * state + KD  * error_omega;
    base_output = expected_steering_angle;
    trim_output = base_output + feedback_output;
    
    if trim_output > min(0.57, current_steering_angle + 2 * ts_)
        trim_output = min(0.57, current_steering_angle + 2 * ts_);
    elseif trim_output < max(-0.57, current_steering_angle - 2*ts_)
        trim_output = max(-0.57, current_steering_angle - 2*ts_);
    end
    output = trim_output;
end