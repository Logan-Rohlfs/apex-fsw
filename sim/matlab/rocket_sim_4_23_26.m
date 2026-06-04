% Rocket Airbrake Simulation
clear; clc; close all;

%% 1. Parameters (Imperial)
g = 32.17;             
W = 67.03;           
m = W / g;         
rho_0 = 0.00237786;   
rho = rho_0;
B = 0.003566; 
T_0 = 549.67; 
P_0 = 14.6959488; 
S = 0.2045143;      
Cd_clean = 0.576;        
Cd_brakes = 0.9; 
Cd_min = Cd_clean + normrnd(0,0.02);  
Cd_max = Cd_brakes + normrnd(0,0.05);  

% --- CONTROLLER SELECTION ---
% Options: 'BangBang', 'PID', 'Energy_Management', 'Clean_Flight'
controller_type = 'Energy_Management'; 

% Simulation Settings
sem = 393*3.28;
alt_0 = sem;
dt = 0.01;
v0 = 1015.66;                
h0 = 1699.96;          
target_alt = 10000; 

%% 2. Initialization
t = 0:dt:30;
h = zeros(size(t)); v = zeros(size(t));
h(1) = h0; v(1) = v0;
brake_state = zeros(size(t)); 
desired_brake_state = zeros(size(t));
p_hist = zeros(size(t)); i_hist = zeros(size(t)); d_hist = zeros(size(t)); u_hist = zeros(size(t));

% PID Gains
    switch controller_type
        case 'PID'
            Kp = 0.4; 
            Ki = -0.004; 
            Kd = -0.04;  

        case 'Energy_Management'
            Kp = 0.5; 
            Ki = 0.004; 
            Kd = 0.004; 
    end
integral_error = 0;
last_error = 0;

% Actuator Constraints
deploy_time = .24; 
max_rate = (Cd_max - Cd_min) / deploy_time; 
actual_Cd = Cd_min; 

%% 3. Simulation Loop
for i = 1:(length(t)-1)
    
    % State Estimation (with noise)
    h_est = h(i) + normrnd(0,0.2);
    v_est = v(i) + normrnd(0,0.2);
   
    % --- Controller Selection (The "elif" blocks) ---
    switch controller_type
        
        case 'BangBang'
            % Simple On/Off prediction logic
            k_current = 0.5 * rho * S * Cd_clean;    
            predicted_apogee = h_est + (m/(2*k_current)) * log(1 + (k_current * v_est^2)/(m*g));
            if predicted_apogee > target_alt && v_est > 0
                desired_Cd = Cd_max;
            else
                desired_Cd = Cd_min;
            end
            
        case 'PID'

            k_current = 0.5 * rho * S * Cd_clean;    
            predicted_apogee = h_est + (m/(2*k_current)) * log(1 + (k_current * v_est^2)/(m*g));
            
            % Standard PID Logic
            error = predicted_apogee - target_alt;
            integral_error = integral_error + (error * dt);
            derivative_error = v_est; % Using velocity as a proxy for D
            
            p_term = Kp * error;
            i_term = Ki * integral_error;
            d_term = Kd * derivative_error;
            
            u = p_term + i_term + d_term;
            u_min = -15; u_max = 30;
            u = max(min(u, u_max), u_min); % Manual clip function
            
            % Map Control Signal to Cd Range
            u_mapped = (u - u_min) * (Cd_max - Cd_min) / (u_max - u_min);
            desired_Cd = Cd_min + u_mapped;
            
            % Save PID history for plotting
            p_hist(i) = p_term; i_hist(i) = i_term; d_hist(i) = d_term; u_hist(i) = u;

        case 'Energy_Management'
            % 1. Energy at Target (Potential only)
            E_target = m * g * target_alt;
            
            % 2. Current Energy (PE + KE)
            PE_current = m * g * h_est;
            KE_current = 0.5 * m * v_est^2;
            E_total_current = PE_current + KE_current;
            
            % 3. Drag Compensation (Future Work)
            dist_remaining = (target_alt - h_est);
            dist_remaining = max(0, dist_remaining);
            
            % We use a higher avg_v_factor early to account for high-speed drag
            avg_v_sq_factor = 0.4; 
            E_drag_loss = (0.5 * rho * (v_est^2 * avg_v_sq_factor) * Cd_min * S) * dist_remaining;
            
            % 4. Energy Error (Absolute)
            % Excess energy above what is needed to reach target including drag loss
            E_excess = (E_total_current - E_drag_loss) - E_target;
            
            % Normalize error against E_target so Kp remains intuitive
            % This prevents the "denominator decay" that caused the late slam
            error = E_excess / E_target; 
            
            % 5. PID Logic
            % We only want to brake if error > 0 (over-energy)
            if error < 0
                error = 0; % Don't accumulate integral error if we are under-energy
            end
            
            integral_error = integral_error + (error * dt);
            derivative_error = (error - last_error) / dt;
            
            % GAIN TUNING: Since error is now E_excess/E_target, 
            % Kp likely needs to be higher (e.g., 5.0 - 15.0)
            u = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error);
            
            % Map to Cd
            u_min = 0; u_max = 0.05; % Very small max because error is normalized to total energy
            u_clipped = max(min(u, u_max), u_min);
            u_mapped = (u_clipped - u_min) * (Cd_max - Cd_min) / (u_max - u_min);
            
            desired_Cd = Cd_min + u_mapped;
            
            % Logging
            p_hist(i) = Kp * error;
            i_hist(i) = Ki * integral_error;
            d_hist(i) = Kd * derivative_error;
            u_hist(i) = u;
            last_error = error;
        case 'Clean_Flight'
            % default to clean flight
            desired_Cd = Cd_min;
        otherwise
            error('Invalid controller_type selected.');
    end
    
    % --- Physical Constraints (Rate Limiter & Clamping) ---
    desired_Cd = max(min(desired_Cd, Cd_max), Cd_min);
    
    delta_Cd = desired_Cd - actual_Cd; 
    max_delta = max_rate * dt; 
    
    if delta_Cd > max_delta
        actual_Cd = actual_Cd + max_delta;
    elseif delta_Cd < -max_delta
        actual_Cd = actual_Cd - max_delta;
    else
        actual_Cd = desired_Cd;
    end
    
    % Update Physics with Limited Cd
    Cd = actual_Cd;
    
    % Logging
    brake_state(i) = (actual_Cd - Cd_min) / (Cd_max - Cd_min);
    desired_brake_state(i) = (desired_Cd - Cd_min) / (Cd_max - Cd_min);
    
    % --- Physics Engine ---
    acc_noises = normrnd(0,0.5); 
    Fd = 0.5 * rho * v(i)^2 * Cd * S * sign(v(i));
    a = (-W - Fd) / m + acc_noises;
    
    v(i+1) = v(i) + a * dt + normrnd(0,0.1);
    h(i+1) = h(i) + v(i) * dt;
    
    % Atmospheric density update
    rho = rho_0*(1-((B*(h(i)+alt_0))/T_0))^(5.26-1);
    if v(i+1) < 0, break; end
end

%% 4. Plotting
% [Existing plotting code follows...]
% (Note: Added a small fix to the indexing to prevent dimension mismatch)
idx = 1:i;
figure('Color', 'w', 'Name', ['Rocket Analysis: ', controller_type]);
subplot(3,1,1);
plot(t(1:i+1), h(1:i+1), 'LineWidth', 2); hold on;
yline(target_alt, '--r', 'Target 10k');
grid on; ylabel('Altitude (ft)'); 
title(['Final Apogee: ', num2str(round(max(h))), ' ft (', controller_type, ')']);

subplot(3,1,2);
plot(t(idx), desired_brake_state(idx), '--', 'Color', [0.6 0.6 0.6]); hold on;
plot(t(idx), brake_state(idx), '-k', 'LineWidth', 1.5);
grid on; ylabel('Brake State');
legend('Desired', 'Actual');

subplot(3,1,3);
% Check for both modes that use PID terms
if strcmp(controller_type, 'PID') || strcmp(controller_type, 'Energy_Management')
    hold on;
    plot(t(idx), p_hist(idx), 'r--', 'DisplayName', 'P'); 
    plot(t(idx), i_hist(idx), 'g--', 'DisplayName', 'I');
    plot(t(idx), d_hist(idx), 'b--', 'DisplayName', 'D');
    plot(t(idx), u_hist(idx), 'k', 'DisplayName', 'Total U');
    grid on; ylabel('PID Terms');
    legend('show');
    % Dynamic Y-Lim based on the current mode's u_min/max
    ylim(min(u), max(u));
else
    text(0.5, 0.5, 'No PID data for this mode', 'HorizontalAlignment', 'center');
end
xlabel('Time (s)');