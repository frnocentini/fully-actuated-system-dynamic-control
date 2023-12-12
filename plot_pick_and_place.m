clc
close all
addpath('./PD_Controller/')
addpath('./ComputedTorque_Controller/')
PD_without_grav_comp;
PD_with_grav_comp;
computed_torque_pap;
load('results')
%%
figure
for j=1:num_of_joints
    subplot(4,2,j);
    plot(log_results.t(1:15000),log_results.result_PD(1:15000,j))
    hold on
    plot (log_results.t(1:15000),log_results.ref_joint(j,1:15000))
    hold on
    plot(log_results.t(1:15000),log_results.result_PD_g(1:15000,j))
    hold on
    plot(log_results.t(1:15000),log_results.result_CT(1:15000,j))
    hold on
    xlabel('time [s]');
    ylabeltext = sprintf('_%i [rad]',j);
    ylabel(['Joint position' ylabeltext]);
    grid;
    hold on
    legend('PD','Ref', 'PD-g', 'CT')
   
end






