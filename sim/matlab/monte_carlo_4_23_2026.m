% Rocket Airbrake Monte Carlo Simulation - Energy Management Edition
clear; clc; close all;

%% 1. Constant Parameters (Imperial)
num_runs = 1000;         
target_alt = 10000;     
g = 32.17;              
W = 67.03;  
m = W / g;          
rho_0 = 0.00237786; 
B = 0.003566; 
T_0 = 549.67; 
S = 0.2045143;      
Cd_clean = 0.576;        
Cd_brakes = 0.9; 

% --- CONTROLLER SELECTION ---
% Options: 'BangBang', 'PID', 'Energy_Management', 'Clean_Flight'
controller_type = 'PID'; 

% Simulation Settings
alt_0 = 393 * 3.28;
dt = 0.01;
v0 = 1015.66;                
h0 = 1699.96;          
deploy_time = .24; 

% Storage for results
final_apogees = zeros(num_runs, 1);
errors = zeros(num_runs, 1);

%% 2. Monte Carlo Loop
fprintf('Running %d simulations using: %s\n', num_runs, controller_type);

for run = 1:num_runs
    % Set gains based on controller type
    switch controller_type
        case 'PID'
            Kp = 0.4; Ki = -0.004; Kd = -0.04;  
            u_min = -15; u_max = 30;
        case 'Energy_Management'
            Kp = 1; Ki = -0.04; Kd = 0.004; 
            u_min = 0; u_max = 0.05;
    end

    % Re-randomize noise-based parameters
    Cd_min = Cd_clean + normrnd(0,0.02);
    Cd_max = Cd_brakes + normrnd(0,0.05);
    max_rate = (Cd_max - Cd_min) / deploy_time;
    actual_Cd = Cd_min;
    
    % Re-initialize state
    t = 0:dt:30;
    h = zeros(size(t)); v = zeros(size(t));
    h(1) = h0; v(1) = v0;
    rho = rho_0;
    integral_error = 0;
    last_error = 0;
    
    % Simulation Loop
    for i = 1:(length(t)-1)
        % Noisy Measurements
        h_est = h(i) + normrnd(0,0.2);
        v_est = v(i) + normrnd(0,0.2);
        
        % --- Controller Logic ---
        switch controller_type
            case 'PID'
                k_current = 0.5 * rho * S * Cd_clean;    
                predicted_apogee = h_est + (m/(2*k_current)) * log(1 + (k_current * v_est^2)/(m*g));
                error = predicted_apogee - target_alt;
                integral_error = integral_error + (error * dt);
                derivative_error = v_est;
                u = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error);
                
            case 'Energy_Management'
                E_target = m * g * target_alt;
                PE_current = m * g * h_est;
                KE_current = 0.5 * m * v_est^2;
                E_total_current = PE_current + KE_current;
                
                dist_remaining = max(0, target_alt - h_est);
                avg_v_sq_factor = 0.4; 
                E_drag_loss = (0.5 * rho * (v_est^2 * avg_v_sq_factor) * Cd_min * S) * dist_remaining;
                
                E_excess = (E_total_current - E_drag_loss) - E_target;
                error = E_excess / E_target; 
                
                if error < 0, error = 0; end % Deadband
                
                integral_error = integral_error + (error * dt);
                derivative_error = (error - last_error) / dt;
                u = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error);
                last_error = error;
            case 'Clean_Flight'
                % default to clean flight
                desired_Cd = Cd_min;
                u = 0;
                u_max = 0;
                u_min = 0;
        end
        
        % Mapping and Rate Limiting
        u = max(min(u, u_max), u_min);
        u_mapped = (u - u_min) * (Cd_max - Cd_min) / (u_max - u_min);
        desired_Cd = Cd_min + u_mapped; 
        
        delta_Cd = desired_Cd - actual_Cd; 
        max_delta = max_rate * dt;
        if delta_Cd > max_delta
            actual_Cd = actual_Cd + max_delta;
        elseif delta_Cd < -max_delta
            actual_Cd = actual_Cd - max_delta;
        else
            actual_Cd = desired_Cd;
        end
        
        % Physics Engine
        Cd = actual_Cd;
        acc_noise = normrnd(0,0.5);
        Fd = 0.5 * rho * v(i)^2 * Cd * S * sign(v(i));
        a = (-W - Fd) / m + acc_noise;
        
        v(i+1) = v(i) + a * dt + normrnd(0,0.1);
        h(i+1) = h(i) + v(i) * dt;
        rho = rho_0*(1-((B*(h(i)+alt_0))/T_0))^(5.26-1);
        
        if v(i+1) < 0, break; end
    end
    
    final_apogees(run) = max(h);
    errors(run) = max(h) - target_alt;
    
    if mod(run, 50) == 0
        fprintf('Progress: %d/%d runs completed...\n', run, num_runs);
    end
end

%% 3. Visualization
figure('Color', 'w', 'Name', 'Monte Carlo Analysis');
histogram(errors, 50, 'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'w');
xline(0, 'r--', 'Target', 'LineWidth', 2);
grid on;
xlabel('Final Apogee Error (ft)');
ylabel('Frequency');
title(['Monte Carlo Results (', controller_type, '): ', num2str(num_runs), ' Runs']);

stats_str = {['Mean Error: ', num2str(mean(errors), '%.2f'), ' ft'], ...
             ['Std Dev: ', num2str(std(errors), '%.2f'), ' ft'], ...
             ['Max Miss: ', num2str(max(abs(errors)), '%.2f'), ' ft']};
annotation('textbox', [0.15, 0.7, 0.2, 0.15], 'String', stats_str, 'FitBoxToText', 'on', 'BackgroundColor', 'w');

fprintf('Monte Carlo Complete.\n');