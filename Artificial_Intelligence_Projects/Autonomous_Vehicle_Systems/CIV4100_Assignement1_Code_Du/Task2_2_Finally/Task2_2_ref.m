% Task2_2_ref.m

function [x_ref, y_ref, theta_ref, s_ref, v_ref, a_ref, omega_ref, alpha_ref, t_ref] = Task2_2_ref(dt, ~)
% =========================================================================
% Task 2.2 Reference Trajectory Wrapper (Fixed global path from Task 2.1)
% =========================================================================
% This function is identical to Task2_1_ref but used in Task 2.2.
% It does NOT accept dynamic x0 input to ensure trajectory consistency.
% =========================================================================

    % Simply call Task2_1_ref without dynamic start point
    [x_ref, y_ref, theta_ref, s_ref, v_ref, a_ref, omega_ref, alpha_ref, t_ref] = Task2_1_ref(dt);

end