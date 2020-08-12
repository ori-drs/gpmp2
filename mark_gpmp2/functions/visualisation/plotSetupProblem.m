function plotSetupProblem(all_cases)
    figure(fig_num); hold on; 
    set(gcf,'Position',[1350 500 1200 1400]);
    title('3D Environment')
    grid on;
    view(3);
    view(-1.007130428616419e+02, 50.314206527540222);
%     campos([-11.08977871940202,2.4122839520229,16.916622755708733]);
    
    gpmp2.set3DPlotRange(all_cases.datasets(1));
    xlabel('x'); ylabel('y'); zlabel('z');
    plot3DEnvironment(all_cases.datasets(1), X, Y, Z)
    gpmp2.plotArm(all_cases.problem.arm.fk_model(), ...
            problem.start_conf, 'b', 2)
    gpmp2.plotArm(all_cases.problem.arm.fk_model(), ...
            problem.end_conf, 'r', 2)

end


