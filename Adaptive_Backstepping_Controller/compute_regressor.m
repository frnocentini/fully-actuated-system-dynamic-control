close all
clear all
clc

sympref('FloatingPointOutput',true);
addpath('../../rvctools')
startup_rvc

% load robot model
mdl_panda

% % Set parameters for symbolic model (panda_sym)
% d1 = 0.400;
% a1 = 0.025;
% a2 = 0.315;
% a3 = 0.365;
% d5 = 0.080;
% l1 = d1; 
% l2 = a2;
% l3 = a3;
% l4 = 0;
% l5 = d5;
% 
% % masses [m]
% m1 = 1; 
% m2 = 1;
% m3 = 1;
% % m4 = 1;
% % m5 = 1;
% syms m4 m5
% 
% panda_sym.links(1).m = m1;
% panda_sym.links(2).m = m2;
% panda_sym.links(3).m = m3;
% panda_sym.links(4).m = m4;
% panda_sym.links(5).m = m5;
% 
% panda_sym.links(1).r = [-a1 -l1/2 0];
% panda_sym.links(2).r = [-l2/2 0 0];
% panda_sym.links(3).r = [-l3/2 0 0];
% panda_sym.links(4).r = [0 0 0];
% panda_sym.links(5).r = [0 0 -l5/2];
% 
% radius = 0.02; % [m]
% panda_sym.links(1).I = diag([m1*l1^2/12, m1*radius^2/2, m1*l1^2/12]); % long. y
% panda_sym.links(2).I = diag([m2*radius^2/2, m2*l2^2/12, m2*l2^2/12]); % long. x
% panda_sym.links(3).I = diag([m3*radius^2/2, m3*l3^2/12, m3*l3^2/12]); % long. x
% panda_sym.links(4).I = diag([0, 0, 0]); % tutto nullo?
% panda_sym.links(5).I = diag([m5*l5^2/12, m5*l5^2/12, m5*radius^2/2]); % long. z

% Masse
m6 = sym('m6','real');
m7 = sym('m7','real');
m1=3.4525; m2=3.4821; 
m3=4.0562; m4=3.4822; m5=2.1633; 
% m6=2.3466; m7=0.31290;

% Posizioni dei centri di massa
syms r6x r6y r6z
syms r7x r7y r7z
r1=transpose([0; -0.03; 0.12]); 
r2=transpose([3.0000e-04; 0.059; 0.042]);
r3=transpose([0; 0.03; 0.13]);
r4=transpose([0; 0.067; 0.034]);
r5=transpose([1.0000e-04; 0.021; 0.076]);
% r6=transpose([0; 6.0000e-04; 4.0000e-04]);
% r7=transpose([0; 0; 0.02]);
r6=transpose([r6x; r6y; r6z]);
r7=transpose([r7x; r7y; r7z]);

% Tensori d'inerzia
I1=[0.0747 0.0085 0;
      0.0085 0.0574 0;
      0 0 0.0239];

I2=[0.0390 -0.0086 -0.0037;
    -0.0086 0.0279 -6.1633e-05;
    -0.0037 -6.1633e-05 0.0199];

I3=[0.006052050623697 0.000000262383560 0.000001120384479;
     0.000000262383560 0.005990028254028 -0.001308542301422;
    0.000001120384479 -0.001308542301422 0.001861529721327];
 
I4=[0.006052050623697 -0.000000262507583 -0.000001120888863;
      -0.000000262507583 0.005990028254028 -0.001308542301422;
     -0.000001120888863 -0.001308542301422 0.001861529721327];

I5=[0.005775526977146 -0.000000448127278 0.000000782342032;
      -0.000000448127278 0.005348473437925 0.001819965983941;
      0.000000782342032 0.001819965983941 0.002181233531810];

I6=[0.001882302441080 0.000000003150206 -0.000000072256604;
      0.000000003150206 0.001889339660303 -0.000012066987492;
     -0.000000072256604 -0.000012066987492 0.002133520179065];

I7=[0.0003390625 0 0;
      0 0.0003390625 0;
      0 0 0.000528125];

% Set link mass, lenght and inertia
panda.links(1).m = m1;
panda.links(2).m = m2;
panda.links(3).m = m3;
panda.links(4).m = m4;
panda.links(5).m = m5;
panda.links(6).m = m6;
panda.links(7).m = m7;

panda.links(1).r = r1;
panda.links(2).r = r2;
panda.links(3).r = r3;
panda.links(4).r = r4;
panda.links(5).r = r5;
panda.links(6).r = r6;
panda.links(7).r = r7;

panda.links(1).I = I1; 
panda.links(2).I = I2;
panda.links(3).I = I3; 
panda.links(4).I = I4;
panda.links(5).I = I5; 
panda.links(6).I = I6; 
panda.links(7).I = I7;


%% CodeGenerator
cg = CodeGenerator(panda);

% Generate code to compute mass matrix
cg.geninertia();
% Generate code to compute gravity vector
cg.gengravload();


%% Compute M,C,G
addpath PANDA
robot = PANDA();

% syms q1 q2 q3 q4 q5 q6 q7
% q = [q1 q2 q3 q4 q5 q6 q7];
% q = [0.1 0.3 0.1 0 0.1 0 0];
q = sym('q',[1,7]);
dq = sym('dq',[1,7]);
ddq = sym('ddq',[1,7]);

% Mass matrix
M = robot.inertia(q);
% Gravity matrix
G = robot.gravload(q);
% Coriolis matrix
C = C_Matrix_Sym(M,q,dq);

% Regressor*pi (pi is the estimated parameters vector)
Y_pi = M * ddq' + C * dq' + G';

%% Save dataset
matrices = [];

matrices.robot = robot;
matrices.M = M;
matrices.G = G;
matrices.C = C;
matrices.Y_pi = Y_pi;

save('matrices','matrices');


