close all; 
clearvars -except commands;

if exist('commands','var') == 0
    commands = compute_steps()
end

syms u
ri = (0.3960*cos(2.65*(u+1.4)));
rj = (-0.99*(sin(u+1.4)));

figure(); 
hold on; 
axis equal; 
fplot(ri, rj, [0, 3.2], 'black');

points = [50, 300, 600, 900, 1200];
quiver_tan = 0; 
quiver_norm = 0;
for i=1:1:length(points)
    idx = points(i);
    location_x = double(subs(ri, u, commands(idx, 4)));
    location_y = double(subs(rj, u, commands(idx, 4)));
    vector_speed = [ commands(idx, 5), commands(idx, 6)];
    vector_speed = (vector_speed ./ norm(vector_speed)) * 0.5;

    vector_norm = [ commands(idx, 8),  commands(idx, 9)];
    vector_norm = (vector_norm ./ norm(vector_norm)) * 0.5;

    quiver_tan = quiver(location_x, location_y, vector_speed(1), vector_speed(2), 'b');
    quiver_norm = quiver(location_x, location_y, vector_norm(1), vector_norm(2), 'r');
end
legend([quiver_tan,quiver_norm], ["0.5 unit tangent vector", "0.5 unit normal vector"]);
xlabel("X Location (meters)");
ylabel("Y Location (meters)");
title("0.5 Unit Tangent and Normal Vectors on Parametric Plot");

exportgraphics(gcf,'plots/parametric-tangent-normal.png','Resolution',1000)


% Plot Theoretical Speed 
figure(); 
hold on; 
theoretical_vl = plot(commands(:, 3), commands(:, 1));
theoretical_vr = plot(commands(:, 3), commands(:, 2));
xlim([0, 13]) % 12.83 seconds to run BOD path
xlabel("Time (s)")
ylabel("Neato Wheel Speed (m/s)")

% Plot actual speed

% encoder readings are slow, remove duplicate rows
encoder_data =  table2array(readtable('real_encoder_data'));
encoder_data_clean = []; 
for i=2:size(encoder_data)
    if(encoder_data(i,2) ~= encoder_data(i-1,2))
        encoder_data_clean = [encoder_data_clean; encoder_data(i,:)];
    end
end
% differentiate 
measured_vl_data = diff(encoder_data_clean(:,2)) ./ diff(encoder_data_clean(:,1));
measured_vr_data = diff(encoder_data_clean(:,3)) ./ diff(encoder_data_clean(:,1));
measured_vl_plot = plot(encoder_data_clean(1:end-1,1), measured_vl_data, '--');
measured_vr_plot = plot(encoder_data_clean(1:end-1,1), measured_vr_data, '--');
legend([theoretical_vl,theoretical_vr, measured_vl_plot, measured_vr_plot], ["Theoretical (V_l)", "Theoretical (V_r)", "Actual (V_l)", "Actual (V_r)"]);
title("Neato Wheel Speeds Theoretical and Measured");
exportgraphics(gcf,'plots/neato-wheel-speeds.png','Resolution',1000)


% 
measured_v = (measured_vl_data + measured_vr_data) / 2; 
measured_w = (measured_vr_data - measured_vl_data) / 0.245; 

measured_theta = [0];
for i=1:size(measured_w)
    delta_t = encoder_data_clean(i+1,1) - encoder_data_clean(i,1);
    measured_theta = [measured_theta; measured_theta(end) + measured_w(i)*delta_t];
end

measured_r = [double(subs(ri,u,0)) double(subs(rj,u,0))]; 
for i=1:size(measured_v)
    delta_t = encoder_data_clean(i+1,1) - encoder_data_clean(i,1);
    next_x = measured_r(end, 1) + measured_v(i)*cos(measured_theta(i))*delta_t;
    next_y = measured_r(end, 2) + measured_v(i)*sin(measured_theta(i))*delta_t;
    measured_r = [measured_r; next_x next_y];
end

%Plot Planned Robot Path WITH TANGENTS -------------------------------
figure();
hold on;
axis equal; 
plot_parametric_theoretical = fplot(ri, rj, [0, 3.2]);
plot_parametric_measured = plot(measured_r(:,1), measured_r(:,2), '--');
xlabel("X Location (meters)");
ylabel("Y Location (meters)");
title("Measured and Actual Parametric Path");

% PLOT TANGENTS TO MEASURED
idx_measured = 2:20:58
meas_tangent = 1;
meas_norm = 1;
for i=1:size(idx_measured,2)
    idx = idx_measured(i)
    tangent_vec = measured_r(idx,:) - measured_r(idx-1,:);
    tangent_vec = [tangent_vec(1) tangent_vec(2) 0];
    tangent_vec = tangent_vec ./ norm(tangent_vec);

    omega_vec = [0 0 measured_w(i) ./ norm(measured_w(i))];

    normal_vec = -1*cross(tangent_vec, omega_vec);

    if(idx > 58/2)
        normal_vec = normal_vec * -1;
    end
    
    meas_tangent = quiver(measured_r(idx,1), measured_r(idx,2), tangent_vec(1) * 0.5, tangent_vec(2) * 0.5, 'b--');
    meas_norm = quiver(measured_r(idx,1), measured_r(idx,2), normal_vec(1) * 0.5, normal_vec(2) * 0.5, 'r--');
end

points = [300, 700, 1200];
quiver_tan = 0; 
quiver_norm = 0;
for i=1:1:length(points)
    idx = points(i);
    location_x = double(subs(ri, u, commands(idx, 4)));
    location_y = double(subs(rj, u, commands(idx, 4)));
    vector_speed = [ commands(idx, 5), commands(idx, 6)];
    vector_speed = (vector_speed ./ norm(vector_speed)) * 0.5;

    vector_norm = [ commands(idx, 8),  commands(idx, 9)];
    vector_norm = (vector_norm ./ norm(vector_norm)) * 0.5;

    quiver_tan = quiver(location_x, location_y, vector_speed(1), vector_speed(2), 'b');
    quiver_norm = quiver(location_x, location_y, vector_norm(1), vector_norm(2), 'r');
end

legend([plot_parametric_theoretical,plot_parametric_measured,quiver_tan,quiver_norm], ["Theoretical parametric plot", "Measured parametric plot","0.5 Unit Tangent Vector", "0.5 Unit Normal Vector"]);



exportgraphics(gcf,'plots/parametric-plot.png','Resolution',1000)




% Plot omega and speed
figure();
tiledlayout(2,1);
nexttile;
hold on; 
plot_measured_omega = plot(encoder_data_clean(1:end-1,1), measured_w);
plot_theoretical_omega = plot(commands(:,3), commands(:,11));
legend([plot_theoretical_omega, plot_measured_omega], ["Theoretical omega", "Measured omega"]);

title('Angular Speed With Respect to Time');
xlabel('Time (seconds)');
ylabel('Angular Speed (rad/sec)');

% Bottom plot
nexttile;
hold on;
theoretical_v_plot = plot(commands(:, 3), (commands(:,1) + commands(:,2)) / 2);
measured_v_plot = plot(encoder_data_clean(1:end-1,1), (measured_vl_data + measured_vr_data) / 2);
legend([theoretical_v_plot, measured_v_plot], ["Theoretical linear speed", "Measured linear speed"]);
title('Linear Speed With Respect to Time');
xlabel('Time (seconds)');
ylabel('Speed (m/s)');
exportgraphics(gcf,'plots/omega-and-velocity.png','Resolution',1000)


close all; 


