robotModel = loadrobot("kukaIiwa14", DataFormat = 'column', Gravity=[0; 0; -9.81]);

%exportrobot(robotModel,OutputFileName="kukaIiwa14.urdf",ExportMesh=true);
figure;
robotModel.show();

homeConfig = homeConfiguration(robotModel);
randomConfig = randomConfiguration(robotModel);

% wrench = [0.5 0.5 0.5 0 0 0.3];
% fext = externalForce(robotModel,"iiwa_link_ee_kuka",wrench,randomConfig);

%% Define Kp and Kv gain values for PD control

Kp = 0.01*eye(7);
Kv = 0.01*eye(7);

%% Define parameters for the trajectory
% Lissajous trajectory parameters
time_gain = 1.0;
% p0 = [0.85; 0.85; 0.85];
p0 = [0.4; 0; 0.6];
A = 0.4; B = 0.4; C = 0.4;
a0 = 1; b0 = 2; c0 = 1.5;
a = time_gain * a0; b = time_gain * b0; c = time_gain * c0; 
d = pi/2; f = pi/3;

%% SVD for kinematics inversion
lev_mar = false;
Lambda_IK = 1*eye(3);
mu_0 = 1e-1;

%% Initialize jointstate
q0 = [0.25 ; 0.25; 0.25 ; 0.25; 0.25 ; 0.25; 0.25];
dq0 = [0.0 ; 0.0; 0.0 ; 0.0; 0.0 ; 0.0; 0.0];

%% Relative error (uncertainty)
rel_err = 0.0;

%% Gravity matrix 
