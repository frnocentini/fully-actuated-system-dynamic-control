%% Clean ENV
clear all
close all
clc

addpath('../../../../../Downloads/RVC2-copy/RVC2-copy/rvctools/')

% Load symbolic robot model, dynamics matrices, regressor multiplied by
% parameter vector and regressor
load("matrices.mat")
robot = matrices.robot; % symbolic model of Panda 5DoF
M_sym = matrices.M;     % symbolic mass matrix 
C_sym = matrices.C;     % symbolic coriolis matrix
G_sym = matrices.G;     % symbolic gravity vector
Y_pi = matrices.Y_pi;   % symbolic regressor multiplied by parameter vector
Y_sym = matrices.Y;     % symbolic regressor

% Symbolic parameters and variables
syms m1 m2 m3 m4 m5
syms q1 q2 q3 q4 q5
syms dq1 dq2 dq3 dq4 dq5
syms ddq1 ddq2 ddq3 ddq4 ddq5

M_sym = expand(M_sym);
% M_sym = subs(M_sym,[m6*r6x m6*r6y m6*r6z m7*r7x m7*r7y m7*r7z],...
    % [mr6x mr6y mr6z mr7x mr7y mr7z]);
% M_sym = subs(M_sym,[m4*r4x m4*r4y m4*r4z m5*r5x m5*r5y m5*r5z],...
%     [mr4x mr4y mr4z mr5x mr5y mr5z]);
G_sym = expand(G_sym);
% G_sym = subs(G_sym,[m6*r6x m6*r6y m6*r6z m7*r7x m7*r7y m7*r7z],...
%     [mr6x mr6y mr6z mr7x mr7y mr7z]);
% G_sym = subs(G_sym,[m4*r4x m4*r4y m4*r4z m5*r5x m5*r5y m5*r5z],...
%     [mr4x mr4y mr4z mr5x mr5y mr5z]);
C_sym = expand(C_sym);
% C_sym = subs(C_sym,[m6*r6x m6*r6y m6*r6z m7*r7x m7*r7y m7*r7z],...
%     [mr6x mr6y mr6z mr7x mr7y mr7z]);
% C_sym = subs(C_sym,[m4*r4x m4*r4y m4*r4z m5*r5x m5*r5y m5*r5z],...
%     [mr4x mr4y mr4z mr5x mr5y mr5z]);
% vars = [m6 mr6x mr6y mr6z m7 mr7x mr7y mr7z];
vars = [m1 m2 m3 m4 m5];
matlabFunction(M_sym,'File','MTilde','Vars',{[q2 q3 q4 vars]});
matlabFunction(G_sym,'File','GTilde','Vars',{[q2 q3 q4 vars]});
matlabFunction(C_sym,'File','CTilde','Vars',{[dq1,dq2,dq3,dq4,dq5,q2,q3,q4,vars]});

% Load real robot model
load("real_robot.mat")


%% Create trajectory
generate_trajectory_adaptive

num_of_joints = 5;  % number of joints
num_estim_param = 5;  % number of symbolic parameters


%% Dynamic parameters vector initialization
% Joint variables
q = zeros(length(t),num_of_joints); 
q_dot = zeros(length(t),num_of_joints); 
% Error vector
e = zeros(length(t),num_of_joints);
e_dot = zeros(length(t),num_of_joints);
% Torque
tau = zeros(length(t),num_of_joints); 
% Estimated parameters vector
piArray = zeros(length(t),num_estim_param); 

q0 = [0 pi/2 pi/2 pi/2 0 ];
q(1,:) = q0; 
q_dot0 = [0 0 0 0 0];
q_dot(1,:) = q_dot0; 

qr_dot = zeros(length(t),num_of_joints);
qr_ddot = zeros(length(t),num_of_joints); 

M_real = panda.inertia(q0);
C_real = panda.coriolis(q0,q_dot0);
G_real = panda.gravload(q0);

pi0 = zeros(1,num_estim_param); 
d = 10;  % add 2.5% of disturbance to the estimated parameters 
pi0(1,1) = panda.links(1).m * (1 + d/100);
pi0(1,2) = panda.links(2).m * (1 + d/100);
pi0(1,3) = panda.links(3).m * (1 + d/100);
pi0(1,4) = panda.links(4).m * (1 + d/100);
pi0(1,5) = panda.links(5).m * (1 + d/100);
 
piArray(1,:) = pi0; 


%% CONTROLLER
Kp = 1*diag([200 200 200 20 10]);
Kv = 0.1*diag([200 200 200 10 1]); 
Kd = 0.1*diag([200 200 200 20 1]);

R = diag([10 10 10 10 10]);
P = 0.01*eye(10);
lambda = diag([200, 200, 200, 200, 200])*0.03;

Mtilde = zeros(5,5);
Ctilde = zeros(5,5);
Gtilde = zeros(5,1);

Y = zeros(num_of_joints,num_estim_param);

qd_dot=dq_des';
qd_ddot=ddq_des';
qd=q_des';
t;
tic
for i = 2:length(t)

    if any(isnan(q(i-1,:)))
        fprintf('Simulation diverged! \n')
        return
    end
    
    e(i-1,:) = qd(i-1,:) - q(i-1,:); 
    e_dot(i-1,:) = qd_dot(i-1,:) - q_dot(i-1,:); 
    s = (e_dot(i-1,:) + e(i-1,:)*lambda);
    
    qr_dot(i-1,:) = qd_dot(i-1,:) + e(i-1,:)*lambda;
    if (i > 2)
        qr_ddot(i-1,:) = (qr_dot(i-1) - qr_dot(i-2)) / delta_t;
    end
    
    % Substitute estimated parameters
    subs_param = [q(i-1,2) q(i-1,3) q(i-1,4) piArray(i-1,1) piArray(i-1,2) piArray(i-1,3) piArray(i-1,4) piArray(i-1,5)];
    Mtilde = MTilde(subs_param);
    Gtilde = GTilde(subs_param);
    subs_param = [q_dot(i-1,1) q_dot(i-1,2) q_dot(i-1,3) q_dot(i-1,4) q_dot(i-1,5)...
        q(i-1,2) q(i-1,3) q(i-1,4) piArray(i-1,1) piArray(i-1,2) piArray(i-1,3) piArray(i-1,4) piArray(i-1,5)];
    Ctilde = CTilde(subs_param);

    
    % ADAPTIVE COMPUTED TORQUE    
    % tau(i,:) = qd_ddot(i-1,:)*Mtilde' + q_dot(i-1,:)*Ctilde'...
    %     + Gtilde + e_dot(i-1,:)*Kv' + e(i-1,:)*Kp';  

%  LI SLOTINE
%         tau(i,:) = qr_ddot(i-1,:)*Mtilde'...
%             + qr_dot(i-1,:)*Ctilde' + Gtilde + s*Kd'; 

%  ADAPTIVE BACKSTEPPING
    tau(i,:) = qr_ddot(i-1,:)*Mtilde' + qr_dot(i-1,:)*Ctilde' + Gtilde + s*Kd' + e(i-1,:)*Kp'; 

    
    M = panda.inertia(q(i-1,:)); 
    C = panda.coriolis(q(i-1,:),q_dot(i-1,:)); 
    G = panda.gravload(q(i-1,:)); 
    
    q_ddot = (tau(i,:) - q_dot(i-1,:)*C' - G) * (M')^(-1); 
    
    q_dot(i,:) = q_dot(i-1,:) + delta_t*q_ddot; 
    q(i,:) = q(i-1,:) + delta_t*q_dot(i,:); 
    
    q1_dot = q_dot(i,1);
    q2_dot = q_dot(i,2);
    q3_dot = q_dot(i,3); 
    q4_dot = q_dot(i,4);
    q5_dot = q_dot(i,5);

    qd1_dot = qd_dot(i,1);
    qd2_dot = qd_dot(i,2);
    qd3_dot = qd_dot(i,3);
    qd4_dot = qd_dot(i,4);
    qd5_dot = qd_dot(i,5);

    qd1_ddot = qd_ddot(i,1);
    qd2_ddot = qd_ddot(i,2);
    qd3_ddot = qd_ddot(i,3);
    qd4_ddot = qd_ddot(i,4);
    qd5_ddot = qd_ddot(i,5);

    g = 9.81;
    
    Y = subs(Y_sym,[q2 q3 q4 q5],[q(i-1,2) q(i-1,3) q(i-1,4) q(i-1,5)]);
    Y = subs(Y,[dq1 dq2 dq3 dq4 dq5],[q_dot(i-1,1) q_dot(i-1,2) q_dot(i-1,3) q_dot(i-1,4) q_dot(i-1,5)]);
    Y = subs(Y,[ddq1 ddq2 ddq3 ddq4 ddq5],[q_ddot(1,1) q_ddot(1,2) q_ddot(1,3) q_ddot(1,4) q_ddot(1,5)]);

    
    % COMPUTED TORQUE DYNAMICAL PARAMETERS DYNAMICS
    % piArray_dot = ( R^(-1) * Y' * (Mtilde')^(-1) * [zeros(num_of_joints) eye(num_of_joints)] * P * [e(i-1,:) e_dot(i-1,:)]' )';
    % 
    % piArray(i,:) = piArray(i-1,:) + delta_t*piArray_dot; 

    
    %BACKSTEPPING DYNAMICAL PARAMETERS DYNAMICS 
    piArray_dot = (R^(-1) * Y' * s')';  
    piArray(i,:) = piArray(i-1,:) + delta_t*piArray_dot;         
 

    if mod(i,100) == 0
        fprintf('Percent complete: %0.1f%%.',100*i/(length(t)-1));
        hms = fix(mod(toc,[0, 3600, 60])./[3600, 60, 1]);
        fprintf(' Elapsed time: %0.0fh %0.0fm %0.0fs. \n', ...
            hms(1),hms(2),hms(3));
    end
    
end

return


%% PLOT RESULTS
figure(5)
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t(1:10001),q(1:10001,j))
%     legend ()
    hold on
    plot (t,q_des(j,1:length(t)))
    legend ('Computed Torque','Desired angle')
    grid;
end


% Plot Dynamics parameter
figure(6)
subplot(2,1,1);
plot(t(1:10001),piArray(1:10001,1))
legend ('Mass Link 1')
hold on
subplot(2,1,2);
plot(t(1:10001),piArray(1:10001,2))
legend ('Mass Link 2')
grid;

% Plot error
figure(7)
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t(1:10001),e(1:10001,j))
%     legend ()
    % hold on
    % plot (t,q_des(j,1:length(t)))
    legend ('Joint error')
    grid;
end

%% Save data
switch choice
    case 1
        dataset_C = [];
        dataset_C.error = e;
        dataset_C.q_computed = q;
        dataset_C.q_desired = q_des;
        dataset_C.estimated_params = piArray;
        save('plot_data_Circumference','dataset_C');
    case 2
        dataset_H = [];
        dataset_H.error = e;
        dataset_H.q_computed = q;
        dataset_H.q_desired = q_des;
        dataset_H.estimated_params = piArray;
        save('plot_data_Helix','dataset_H');
    case 3
        dataset_L = [];
        dataset_L.error = e;
        dataset_L.q_computed = q;
        dataset_L.q_desired = q_des;
        dataset_L.estimated_params = piArray;
        save('plot_data_Lissajoux','dataset_L');
end



