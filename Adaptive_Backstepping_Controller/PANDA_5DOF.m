function r = PANDA_5DOF()
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
    
    panda = SerialLink([L1 L2 L3 L4 L5],'name','PANDA_sym', 'manufacturer', 'Franka-Emika', 'tool', transl([0 0 d7])); % model of the robot (used in adaptive computed torque control)
    
    
    qz = [0 0 0 0 0];
    qr = [0 -90 -90 90 0]*deg;
            
    % place the variables into the global workspace
    if nargin == 1
        r = panda;
    elseif nargin == 0
        assignin('caller', 'panda_sym', panda);
        assignin('caller', 'qz', qz); % zero angles
        assignin('caller', 'qr', qr); 
    end
end