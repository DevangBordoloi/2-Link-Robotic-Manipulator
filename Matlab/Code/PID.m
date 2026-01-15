% Two-Link Manipulator PID Simulation

clc; clear; close all;
m1 = 5;    
m2 = 3;        
l1 = 0.25;      
l2 = 0.15;       
g  = 9.81;       

kp = 200; kd = 200; ki = 250;      
Kp = [kp, kp]; 
Kd = [kd, kd]; 
Ki = [ki, ki];               

q_initial = [0.2; 0.15; 0; 0];   
q_desired = [0; 0];             
e_int_prev = [0; 0];             
tspan = [0 10];

options = odeset('RelTol',1e-6,'AbsTol',1e-6,'MaxStep',0.01);

[t, q] = ode45(@(t, q) TwoLink_PID_Dynamics(t, q, m1, m2, l1, l2, g, Kp, Ki, Kd, ...
                 q_desired, e_int_prev, tspan), tspan, q_initial, options);
q1 = q(:,1);
q2 = q(:,2);

figure;
subplot(2,1,1);
plot(t, q1, 'r', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('q1 (rad)');
title(['Joint 1 Angle vs Time | PID Controller Kp=' num2str(kp) ', Kd=' num2str(kd) ', Ki=' num2str(ki)]);
subplot(2,1,2);
plot(t, q2, 'b', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('q2 (rad)');
title(['Joint 2 Angle vs Time | PID Controller Kp=' num2str(kp) ', Kd=' num2str(kd) ', Ki=' num2str(ki)]);

e1 = q_desired(1) - q1;
e2 = q_desired(2) - q2;

figure;
subplot(2,1,1);
plot(t, e1, 'r', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('e1 (rad)');
title('Error in Joint 1');
subplot(2,1,2);
plot(t, e2, 'g', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('e2 (rad)');
title('Error in Joint 2');
sgtitle('Joint Angle Errors vs Time (PID Control)');
