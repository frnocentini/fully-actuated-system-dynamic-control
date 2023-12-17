close all
clc

addpath('../../rvctools/')
startup_rvc

%% Parameters
deg = pi/180;
mm = 1e-3;
d7 = 107*mm;

d1 = 0.400;
a1 = 0.025;
a2 = 0.315;
a3 = 0.365;
d5 = 0.080;
l1 = d1; 
l2 = a2;
l3 = a3;
l4 = 0;
l5 = d5;


%% Denavit-Hartenberg
L1 = Link('d', d1, 'a', a1, 'alpha', pi/2);

L2 = Link('d', 0, 'a', a2, 'alpha', 0);

L3 = Link('d', 0, 'a', a3, 'alpha', 0);

L4 = Link('d', 0, 'a', 0, 'alpha', pi/2);

L5 = Link('d', d5, 'a', 0, 'alpha', 0);

panda = SerialLink([L1 L2 L3 L4 L5],'name','PANDA_real', 'manufacturer', 'Franka-Emika', 'tool', transl([0 0 d7])); % model of the robot (used in adaptive computed torque control)

m1 = 1; 
m2 = 1;
m3 = 1;
m4 = 1;
m5 = 1;

panda.links(1).m = m1;
panda.links(2).m = m2;
panda.links(3).m = m3;
panda.links(4).m = m4;
panda.links(5).m = m5;

panda.links(1).r = [-a1 -l1/2 0];
panda.links(2).r = [-l2/2 0 0];
panda.links(3).r = [-l3/2 0 0];
panda.links(4).r = [0 0 0];
panda.links(5).r = [0 0 -l5/2];

radius = 0.02; % [m]
panda.links(1).I = diag([m1*l1^2/12, m1*radius^2/2, m1*l1^2/12]); % long. y
panda.links(2).I = diag([m2*radius^2/2, m2*l2^2/12, m2*l2^2/12]); % long. x
panda.links(3).I = diag([m3*radius^2/2, m3*l3^2/12, m3*l3^2/12]); % long. x
panda.links(4).I = diag([0, 0, 0]); % tutto nullo?
panda.links(5).I = diag([m5*l5^2/12, m5*l5^2/12, m5*radius^2/2]); % long. z


%% Code generator

save('real_robot','panda')
