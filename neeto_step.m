
function [next_u, V_l_command, V_r_command, V_vec, dT_hat_dt] = neeto_step(current_u, delta_t, V_symbolic, dT_hat_dt_symbolic, V_l_symbolic, V_r_symbolic, sub_eval_symb, DU_DT_SCAN, DU_DT_SCAN_MAX, MAX_NEETO_SPEED)
    optimal_du_dt = 0;
    V_l_command = 0;
    V_r_command = 0;
    
    du_min = DU_DT_SCAN;
    du_max = DU_DT_SCAN_MAX;
    while(abs(du_min - du_max) > 0.001)
        du_mid = (du_min + du_max) / 2;
        V_l_discrete = sub_eval_symb(V_l_symbolic, current_u, du_mid);
        V_r_discrete = sub_eval_symb(V_r_symbolic, current_u, du_mid);
        if(abs(V_l_discrete) < MAX_NEETO_SPEED && abs(V_r_discrete) < MAX_NEETO_SPEED)
            du_min = du_mid;
        else
            du_max = du_mid;
        end


    end
   
    optimal_du_dt = du_mid;
    V_l_command = sub_eval_symb(V_l_symbolic, current_u, du_mid);
    V_r_command = sub_eval_symb(V_r_symbolic, current_u, du_mid);
    V_vec = sub_eval_symb(V_symbolic, current_u, du_mid);
    dT_hat_dt = sub_eval_symb(dT_hat_dt_symbolic, current_u, du_mid);


    next_u = current_u + delta_t * optimal_du_dt
end

