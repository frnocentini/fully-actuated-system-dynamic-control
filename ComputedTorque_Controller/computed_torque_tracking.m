addpath('../')
generate_trajectory
%% Trajectory Tracking: Computed Torque Method

if choice == 1
    % Gain circumference parameters matrix
    Kp = 10*diag([5 5 10 5 15 15 300]);
    Kv = 1*diag([9 5 11 8 16 15 200]);
elseif choice == 2
% Good Helix parameters matrix
    Kp = 10*diag([5 5 10 5 15 15 300]);
    Kv = 1*diag([9 5 11 8 16 15 200]);
else
    % Good Lissajous parameters matrix
    Kp = 10*diag([5 10 10 10 15 15 300]);
    Kv = 1*diag([9 10 11 13 16 15 200]);

end

results_computed_torque = q0;
index = 1;
q=q0
dq=q_dot0
ddq=[0 0 0 0 0 0 0]
for i=1:length(t)

   % Error and derivate of the error   
    err = transpose(q_des(:,i)) - q;
    derr = transpose(dq_des(:,i)) - dq;
    
    %Get dynamic matrices
    F = get_FrictionTorque(dq);
    G = get_GravityVector(q);
    C= get_CoriolisVector(q,dq);
    M = get_MassMatrix(q);
    J = get_jacobian(q);

    % Computed Torque Controller
    
    tau = ( M*(ddq_des(:,1) + Kv*(derr') + Kp*(err')) + C + G +F)';
      
    % Robot joint accelerations
    ddq_old = ddq;
    ddq = (pinv(M)*(tau - C'- G'-F')')';
        
    % Tustin integration
    dq_old = dq;
    dq = dq + (ddq_old + ddq) * delta_t / 2;
    q = q + (dq + dq_old) * delta_t /2;
    
    % Store result for the final plot
    results_computed_torque(index,:) = q;
    index = index + 1;

end


%% Plot computed torque results for trajectory tracking

figure
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t(1:10001),results_computed_torque(1:10001,j))
%     legend ()
    hold on
%     plot(t(1:10001),results_computed_torque(1:10001,j))
    plot (t,q_des(j,1:length(t)))
    legend ('Computed Torque','Desired angle')
    grid;
end
%%
q_des_error=(results_computed_torque-q_des(:,1:10001)')'
figure
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t,q_des_error(j,1:length(t)))
%     legend ()
%     hold on
%     plot (t,q_des_error_no_friction(j,1:length(t)))
    legend ('Computed torque Error with no friction model')
    grid;
end

%% Visualize controlled trajectory

figure

plot3(x,y,z,'r','Linewidth',1.5)

grid on
PANDA.plotopt = {'workspace',[-0.75,0.75,-0.75,0.75,0,1]};
hold on
for i=1:100:length(results_computed_torque')
    
    PANDA.plot(results_computed_torque(i,:),'floorlevel',0,'fps',1000,'trail','-k','linkcolor',orange,'jointcolor',grey)

end
