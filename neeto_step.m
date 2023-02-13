
function [next_u, V_l_command, V_r_command, V_vec, dT_hat_dt] = neeto_step(current_u, delta_t, V_symbolic, dT_hat_dt_symbolic, V_l_symbolic, V_r_symbolic, sub_eval_symb, DU_DT_SCAN, DU_DT_SCAN_MAX, MAX_NEETO_SPEED)
    optimal_du_dt = 0;
    V_l_command = 0;
    V_r_command = 0;
    for du_guess=DU_DT_SCAN:DU_DT_SCAN:DU_DT_SCAN_MAX
        V_l_discrete = sub_eval_symb(V_l_symbolic, current_u, du_guess);
        V_r_discrete = sub_eval_symb(V_r_symbolic, current_u, du_guess);
        %fprintf("%d %d %d\n", abs(V_l_discrete), abs(V_r_discrete), du_guess);
        if(abs(V_l_discrete) < MAX_NEETO_SPEED && abs(V_r_discrete) < MAX_NEETO_SPEED)
            optimal_du_dt = du_guess;
            V_l_command = V_l_discrete;
            V_r_command = V_r_discrete;
            V_vec = sub_eval_symb(V_symbolic, current_u, du_guess);
            dT_hat_dt = sub_eval_symb(dT_hat_dt_symbolic, current_u, du_guess);
        else
            break
        end

    end
    next_u = current_u + delta_t * optimal_du_dt;
end

