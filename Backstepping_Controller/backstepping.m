addpath('../')
%generate_trajectory

%% Trajectory tracking: Backstepping control

if choice == 1
    % Good Circumference parameters
    %Kp = 1* diag([1 1 1 1 3 1 1]);
    Kp = 1* diag([1 3 3 3 1 1 1]);
else
% Good Helix parameters
    Kp = diag([1 3 1 1 1 1 1]);
end

results_backstepping = q0;
index = 1;
q=q0
dq=q_dot0
ddq=[0 0 0 0 0 0 0]
for i=1:length(t)

   % Error and derivate of the error   
    err = transpose(q_des(:,i)) - q;
    derr = transpose(dq_des(:,i)) - dq;
    
    dqr = transpose(dq_des(:,i)) + err*(Kp);
    ddqr = transpose(ddq_des(:,i)) + derr*(Kp);
    s = derr + err*(Kp');
     
    %Get dynamic matrices
    F = get_FrictionTorque(dq);
    G = get_GravityVector(q);
    C = get_CoriolisMatrix(q,dq);
    M = get_MassMatrix(q);
    


    % Backstepping Controller
    tau = (M*(ddqr') + C*(dqr') ...
        + G + Kp*(s') + err')';      
    
    % Robot joint accelerations
    ddq_old = ddq;
    ddq = (pinv(M)*(tau - transpose(C*(dq'))- G')')';
        
    % Tustin integration
    dq_old = dq;
    dq = dq + (ddq_old + ddq) * delta_t / 2;
    q = q + (dq + dq_old) * delta_t /2;
    
    % Store result for the final plot
    results_backstepping(index,  :) = q;
    index = index + 1;

end

%% Plot backstepping results

figure
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t,results_backstepping(:,j))
    hold on
    plot (t,q_des(j,1:length(t)))
%     hold on
%     plot(t,results_computed_torque(:,j))
    grid;
    legend ('Backstepping Results','Desired angle')
end


%%
q_des_error=(results_backstepping-q_des(:,1:10001)')'
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
for i=1:100:length(results_backstepping')
    
    PANDA.plot(results_backstepping(i,:),'floorlevel',0,'fps',1000,'trail','-k','linkcolor',orange,'jointcolor',grey)

end
