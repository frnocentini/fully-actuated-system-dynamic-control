clc
close all
addpath('./Backstepping_Controller/')
addpath('./ComputedTorque_Controller/')
generate_trajectory
computed_torque_tracking;
backstepping;
%%
figure
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t(1:10000),results_computed_torque(1:10000,j))
    hold on
    plot (t(1:10000),q_des(j,1:10000))
    hold on
    plot(t(1:10000),results_backstepping(1:10000,j))
    hold on
    xlabel('time [s]');
    ylabeltext = sprintf('_%i [rad]',j);
    ylabel(['Joint position' ylabeltext]);
    grid;
    hold on
    legend('CT','Ref', 'BS')
   
end




