%% Clean ENV
clear all
close all
clc

% Load symbolic robot model, dynamics matrices and regressor multiplied by
% parameter vector
load("matrices.mat")
robot = matrices.robot;
M_sym = matrices.M;
C_sym = matrices.C;
G_sym = matrices.G;
Y_pi = matrices.Y_pi;

syms m6 m7
syms r6x r6y r6z
syms r7x r7y r7z
syms mr6x mr6y mr6z 
syms mr7x mr7y mr7z

M_sym = expand(M_sym);
M_sym = subs(M_sym,[m6*r6x m6*r6y m6*r6z m7*r7x m7*r7y m7*r7z],...
    [mr6x mr6y mr6z mr7x mr7y mr7z]);
G_sym = expand(G_sym);
G_sym = subs(G_sym,[m6*r6x m6*r6y m6*r6z m7*r7x m7*r7y m7*r7z],...
    [mr6x mr6y mr6z mr7x mr7y mr7z]);
C_sym = expand(C_sym);
C_sym = subs(C_sym,[m6*r6x m6*r6y m6*r6z m7*r7x m7*r7y m7*r7z],...
    [mr6x mr6y mr6z mr7x mr7y mr7z]);
vars = [m6 mr6x mr6y mr6z m7 mr7x mr7y mr7z];
matlabFunction(M_sym,'File','MTilde','Vars',{vars});
matlabFunction(G_sym,'File','GTilde','Vars',{vars});
matlabFunction(C_sym,'File','CTilde','Vars',{vars});

% Load real robot model
addpath('../')
load('robot.mat')

%% Create trajectory
generate_trajectory_adaptive

num_of_joints = 7;  % number of joints
num_estim_param = 8;  % number of symboli parameters

%% Dynamic parameters vector initialization
q = zeros(length(t),num_of_joints); 
q_dot = zeros(length(t),num_of_joints); 
tau = zeros(length(t),num_of_joints); 
piArray = zeros(length(t),num_estim_param); 

q0 = [0 pi/3 0 pi/6 0 0 0]+[0 pi/3 0 pi/6 0 0 0]*0.01; 
q(1,:) = q0; 
q_dot0 = [0 0 0 0 0 0 0];
q_dot(1,:) = q_dot0; 

qr_dot = zeros(length(t),num_of_joints);
qr_ddot = zeros(length(t),num_of_joints); 

M_real = get_MassMatrix(q0);
C_real = get_CoriolisMatrix(q0,q_dot0);
G_real = get_GravityVector(q0);

pi0 = zeros(1,num_estim_param); 
% for j = 1:8
%     pi0((j-1)*10+1:j*10) = [robot.links(j).m robot.links(j).m*robot.links(j).r ...
%         robot.links(j).I(1,1) 0 0 robot.links(j).I(2,2) 0 robot.links(j).I(3,3)];
% end
d = 1;  % add 1% of disturbance
pi0(1,1) = PANDA.links(6).m * (1 + d/100);
pi0(1,2:4) = pi0(1,1) .* PANDA.links(6).r;
pi0(1,5) = PANDA.links(7).m * (1 + d/100);
pi0(1,6:8) = pi0(1,5) .* PANDA.links(7).r;
 
piArray(1,:) = pi0; 

%% CONTROLLER
Kp = 1*diag([200 200 200 20 10]);
Kv = 0.1*diag([200 200 200 10 1]); 
Kd = 0.1*diag([200 200 200 20 1]);

R = diag(repmat([1e1 repmat(1e3,1,3) 1e2 1e7 1e7 1e2 1e7 1e2],1,num_of_joints)); 
P = 0.01*eye(10);
lambda = diag([200, 200, 200, 200, 200])*0.03;

MTilde = zeros(7,7);
CTilde = zeros(7,7);
GTilde = zeros(7,1);

Y = zeros(7,8);

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
    
    e = qd(i-1,:) - q(i-1,:); 
    e_dot = qd_dot(i-1,:) - q_dot(i-1,:); 
    s = (e_dot + e*lambda);
    
    qr_dot(i-1,:) = qd_dot(i-1,:) + e*lambda;
    if (i > 2)
        qr_ddot(i-1,:) = (qr_dot(i-1) - qr_dot(i-2)) / delta_t;
    end
    
    % Substitute estimated parameters
    % for j = 1:num_of_joints 
    %     robot.links(j).m = piArray(i-1,(j-1)*10+1); % elemento 1 di pi
    % end
    % 
    % Mtilde = robot.inertia(q(i-1,:)); 
    % Ctilde = robot.coriolis(q(i-1,:),q_dot(i-1,:)); 
    % Gtilde = robot.gravload(q(i-1,:)); 
    for i=1:7
        Y_sub = Y_pi;
        Y_sub = subs(Y_sub,[r6x r6y r6z r7x r7y r7z],[0 0 0 0 0 0]);
        Y(i,1) = jacobian(Y_sub(i,1),m6);
        Y_sub(i,1) = jacobian(Y_pi(i,1),m6);
        Y(i,2) = jacobian(Y_sub(i,1),r6x);
        Y(i,3) = jacobian(Y_sub(i,1),r6y);
        Y(i,4) = jacobian(Y_sub(i,1),r6z);
        Y(i,5) = jacobian(Y_sub(i,1),m7);
        Y_sub(i,5) = jacobian(Y_pi(i,1),m7);
        Y(i,6) = jacobian(Y_sub(i,5),r7x);
        Y(i,7) = jacobian(Y_sub(i,5),r7y);
        Y(i,8) = jacobian(Y_sub(i,5),r7z);
    end


    
% ADAPTIVE COMPUTED TORQUE    
    tau(i,:) = qd_ddot(i-1,:)*Mtilde' + q_dot(i-1,:)*Ctilde'...
        + Gtilde + e_dot*Kv' + e*Kp';
%   

%  LI SLOTINE
%         tau(i,:) = qr_ddot(i-1,:)*Mtilde'...
%             + qr_dot(i-1,:)*Ctilde' + Gtilde + s*Kd'; 

%  ADAPTIVE BACKSTEPPING
%     tau(i,:) = qr_ddot(i-1,:)*Mtilde' + qr_dot(i-1,:)*Ctilde' + Gtilde + s*Kd' + e*Kp'; 

    
    M = PANDA.inertia(q(i-1,:)); 
    C = PANDA.coriolis(q(i-1,:),q_dot(i-1,:)); 
    G = PANDA.gravload(q(i-1,:)); 
    
    q_ddot = (tau(i,:) - q_dot(i-1,:)*C' - G) * (M')^(-1); 
    
    q_dot(i,:) = q_dot(i-1,:) + delta_t*q_ddot; 
    q(i,:) = q(i-1,:) + delta_t*q_dot(i,:); 
    
    
    q1 = q(i,1); q2 = q(i,2); q3 = q(i,3); q4 = q(i,4); q5 = q(i,5);

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
    
    disp('Calcolo il regressore')
    regressor2
    % regressor_panda_5dof;

    
% COMPUTED TORQUE DYNAMICAL PARAMETERS DYNAMICS
    piArray_dot = ( R^(-1) * Y' * (Mtilde')^(-1) * [zeros(num_of_joints) eye(num_of_joints)] * P * [e e_dot]' )';

    piArray(i,:) = piArray(i-1,:) + delta_t*piArray_dot; 

% BACKSTEPPING DYNAMICAL PARAMETERS DYNAMICS

% 
%     piArray_dot = (R^(-1) * Y' * s')';  
% 
%     piArray(i,:) = piArray(i-1,:) + delta_t*piArray_dot; 
%         
 

    if mod(i,100) == 0
        fprintf('Percent complete: %0.1f%%.',100*i/(length(t)-1));
        hms = fix(mod(toc,[0, 3600, 60])./[3600, 60, 1]);
        fprintf(' Elapsed time: %0.0fh %0.0fm %0.0fs. \n', ...
            hms(1),hms(2),hms(3));
    end
    
end

return


%% PLOT RESULTS
figure
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t(1:10001),q(1:10001,j))
%     legend ()
    hold on
    plot (t,q_des(j,1:length(t)))
    legend ('Computed Torque','Desired angle')
    grid;
end


%% Plot Dynamics parameter
figure
subplot(5,1,1);
plot(t(1:10001),piArray(1:10001,1))
legend ('Mass Link 1')
hold on
subplot(5,1,2);
plot(t(1:10001),piArray(1:10001,11))
legend ('Mass Link 2')
subplot(5,1,3);
plot(t(1:10001),piArray(1:10001,21))
legend ('Mass Link 3')
subplot(5,1,4);
plot(t(1:10001),piArray(1:10001,31))
legend ('Mass Link 4')
subplot(5,1,5);
plot(t(1:10001),piArray(1:10001,41))
legend ('Mass Link 5')
grid;

