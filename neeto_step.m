
function [next_u, V_l_command, V_r_command, V_vec, dT_hat_dt] = neeto_step(current_u, delta_t, V_symbolic, dT_hat_dt_symbolic, V_l_symbolic, V_r_symbolic, sub_eval_symb, MAX_NEETO_SPEED)
    optimal_du_dt = 0;
    V_l_command = 1000000;
    V_r_command = 1000000;
    
    du_min = 0.00001;
    du_max = 2.00000;
       

    while((abs(V_l_command) < MAX_NEETO_SPEED && abs(V_r_command) < MAX_NEETO_SPEED) == 0 || abs(du_max - du_min) > 0.001)
        optimal_du_dt = (du_min + du_max) / 2;
        V_l_command = sub_eval_symb(V_l_symbolic, current_u, optimal_du_dt);
        V_r_command = sub_eval_symb(V_r_symbolic, current_u, optimal_du_dt);
        if(abs(V_l_command) < MAX_NEETO_SPEED && abs(V_r_command) < MAX_NEETO_SPEED)
            du_min = optimal_du_dt;
        else
            du_max = optimal_du_dt;
        end
    end
   
    V_vec = sub_eval_symb(V_symbolic, current_u, optimal_du_dt);
    dT_hat_dt = sub_eval_symb(dT_hat_dt_symbolic, current_u, optimal_du_dt);
    next_u = current_u + delta_t * optimal_du_dt
end

