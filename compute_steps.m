
function [commands] = compute_steps()
    % Constants ---------------------------------------------------------- 
    MAX_U = 3.2; % The max value of u in the parametric definition of path
    DELTA_T = 0.2; % The precision of dt to use when discretely approximating integrals
    DU_DT_SCAN = 0.05; % The accuracy used to parameter sweep du/dt
    DU_DT_SCAN_MAX = 2.0; % Max value to permit in sweep of du/dt
    MAX_NEETO_SPEED = 0.3; % Neeto cannot drive faster than +- 0.3 m/s.
    
    % Symbolic function u(t) maps time to the location defined by parametric
    % variable u
    syms u(t);
    assume(u(t), 'real');
    
    % Define function r(u) (u in [0,3.2]) to describe location of neeto
    ri = (0.3960*cos(2.65*(u+1.4)));
    rj = (-0.99*(sin(u+1.4)));
    rk =0*u;
    r =[ri,rj,rk];
    d = 0.245; 
        
    % Symbolically derive velocity/rotation with respect to u(t) and du/dt
    dr_dt = diff(r,t);
    V_symbolic = norm(dr_dt);
    T_hat_symbolic = dr_dt./norm(dr_dt);
    dT_hat_dt_symbolic = diff(T_hat_symbolic,t); 
    omega_u = cross(T_hat_symbolic, dT_hat_dt_symbolic);
    omega_u = omega_u(t); % these two lines get the third element of symbolic matrix 
    omega_u = omega_u(3);
    
    % Symbolic derivation of wheel commands with respect to u(t) and du/dt
    V_l_symbolic = V_symbolic - omega_u * (d/2);
    V_r_symbolic = V_symbolic + omega_u * (d/2);
    
    % Helpers to evaluate V_l_symbolic/V_r_symbolic for given u(t) and du/dt
    sub_du_dt = @(symbolic_expr, du_dt_sub) (subs(symbolic_expr, diff(u(t), t), du_dt_sub));
    sub_u = @(symbolic_expr, u_sub) (subs(symbolic_expr, u(t), u_sub));
    sub_eval_symb = @(symbolic_expr, u_sub, du_dt_sub) (double(sub_u(sub_du_dt(symbolic_expr,du_dt_sub),u_sub)));

    % Pre-compute the optimal du/dt values with brute force approximation
    commands = [0 0 0 0 0 0 0 0 0 0];
    current_u = 0;
    while(current_u < MAX_U + 0.1)
        [next_u, V_l_command, V_r_command, V_vec, dT_hat_dt] = neeto_step(current_u, DELTA_T, dr_dt, dT_hat_dt_symbolic, V_l_symbolic, V_r_symbolic, sub_eval_symb, DU_DT_SCAN, DU_DT_SCAN_MAX, MAX_NEETO_SPEED);
        commands = [commands; V_l_command V_r_command (DELTA_T * size(commands, 1)) current_u V_vec(1) V_vec(2) V_vec(3) dT_hat_dt(1) dT_hat_dt(2) dT_hat_dt(3)];
        current_u = next_u
    end
end

