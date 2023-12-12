%% Clean ENV

close all
clc
addpath('../../../../Downloads/RVC2-copy/RVC2-copy/rvctools')

%% Choose Trajectory

fprintf('Choose trajectory: \n');

fprintf('1: Circumference \n');

fprintf('2: Helix \n');
fprintf('3: Lissajous \n');


choice = input(' ... ');

%% Setup
 
load('robot.mat')

grey = [0.5, 0.5, 0.5];
orange = [0.8, 0.6, 0];

t_in = 0; % [s]
t_fin = 10; % [s]
delta_t = 0.001; % [s]
timeSpan= 10;

t = t_in:delta_t:t_fin;

num_of_joints = 7;

Q = zeros(num_of_joints,length(t));
dQ = zeros(num_of_joints,length(t));
ddQ = zeros(num_of_joints,length(t));
TAU = zeros(num_of_joints,length(t));

%% Compute Trajectory

switch choice
                    
    case 1 % Circonferenza
        
        q0 = [0 pi/3 0 pi/6 0 0 0];
        q_dot0 = [0 0 0 0 0 0 0];
        
        pos0 = PANDA.fkine(q0).t;

        radius = 0.2; % raggio dell'elica [m]
        center = pos0 - [radius;0;0];
        
%           Standard Trajectory
%         x = center(1) + radius * cos(t/t(end)*2*pi);
%         y = center(2) * ones(size(x));
%         z = center(3) + radius * sin(t/t(end)*2*pi);
%         theta = 0.1*sin(t/5*2*pi);
        
%         Fast Trajectory
        x = center(1) + radius * cos(t/t(end)*2*pi);
        y = center(2) * ones(size(x));
        z = center(3) + radius * sin(t/t(end)*2*pi);
        theta = 0.1*sin(t/3*2*pi);
        
        phi = zeros(size(x));
        psi = zeros(size(x));

        xi = [x; y; z; theta; phi; psi]; % twist
        
        
        q_des= get_ref_from_xi(xi,q0,PANDA)
        dq_des=gradient(q_des)*1000
        ddq_des=gradient(dq_des)*1000
        
        figure
        PANDA.plotopt = {'workspace',[-0.75,0.75,-0.75,0.75,0,1]};
        PANDA.plot(q0,'floorlevel',0,'linkcolor',orange,'jointcolor',grey)
        hold
        plot3(x,y,z,'k','Linewidth',1.5)
        
    case 2 % Traiettoria elicoidale
        
        q0 = [0 pi/3 0 pi/6 0 0 0];
        q_dot0 = [0 0 0 0 0 0 0];
        pos0 = PANDA.fkine(q0).t;
       
        shift = 0.1; % passo dell'elica [m] 
        radius = 0.1; % raggio dell'elica [m]
        num = 2; % numero di giri [#]
        center = pos0 - [radius;0;0];

        x = center(1) + radius * cos(t/t(end)*num*2*pi);
        y = center(2) + t/t(end)*num*shift;
        z = center(3) + radius * sin(t/t(end)*num*2*pi);
        theta = zeros(size(x));
        phi = zeros(size(x));
        psi = zeros(size(x));

        xi = [ x ;y; z; theta ;phi; psi]; % twist
        
        
        q_des= get_ref_from_xi(xi,q0,PANDA)
        dq_des=gradient(q_des)*1000
        ddq_des=gradient(dq_des)*1000

%         figure
%         PANDA.plot(q0)
%         hold
%         plot3(x',y',z','k','Linewidth',1.5, 'Red')

 case 3 % Traiettoria lissajous
        
        q0 = [0 pi/3 0 pi/6 0 0 0];
        q_dot0 = [0 0 0 0 0 0 0];
        pos0 = PANDA.fkine(q0).t;
       
        A = 0.1; % ampiezza [m] 
        a = 1; % frequenza [rad/s]
        B = 0.1; %ampiezza [m]
        b = 2;  % frequenza [rad/s]
        delta = pi/2; %sfasamento [rad]
        num = 2; % numero di giri [#]
        center = pos0 - [A;0;0];

        x = center(1) + A*sin(a*t/t(end)*num*2*pi+delta);
        y = center(2) * ones(size(x));
        z = center(3) + B* sin(b*t/t(end)*num*2*pi);
        theta = zeros(size(x));
        phi = zeros(size(x));
        psi = zeros(size(x));

        xi = [ x ;y; z; theta ;phi; psi]; % twist
        
        
        q_des= get_ref_from_xi(xi,q0,PANDA)
        dq_des=gradient(q_des)*1000
        ddq_des=gradient(dq_des)*1000

%         figure
%         PANDA.plot(q0)
%         hold
%         plot3(x',y',z','k','Linewidth',1.5, 'Red')

end

%% Visualize desired trajectory

figure

plot3(x,y,z,'r','Linewidth',1.5)

grid on
PANDA.plotopt = {'workspace',[-0.75,0.75,-0.75,0.75,0,1]};
hold on
for i=1:100:length(q_des)
    
    PANDA.plot(transpose(q_des(:,i)),'floorlevel',0,'fps',1000,'trail','-k','linkcolor',orange,'jointcolor',grey)

end

%% Plot Joint Trajectories


figure
for j=1:num_of_joints
    
    subplot(4,2,j);
    plot(t,q_des(j,1:length(t)))
    xlabel('time [s]');
    ylabeltext = sprintf('_%i [rad]',j);
    ylabel(['Joint position' ylabeltext]);
    grid;
end

%% Plot xyz trajectory
a=[x;y;z]
figure
for j=1:3
    
    subplot(1,3,j);
    plot(t,a(j,1:length(t)))
    xlabel('time [s]');
    ylabeltext = sprintf('_%i [rad]',j);
    ylabel(['Joint position' ylabeltext]);
    grid;
end


