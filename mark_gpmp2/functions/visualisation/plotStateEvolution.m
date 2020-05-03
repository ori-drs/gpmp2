function plotStateEvolution(result, delta_t, total_time_sec, total_time_step, handle, v_or_x)
%PLOTSTATEEVOLUTION Summary of this function goes here
%   Detailed explanation goes here

time_steps = 0:delta_t:total_time_sec;
    conf_x_series = [];
%     conf_v_series = [];

    for i=0:total_time_step
        x_conf = result.atVector(gtsam.symbol(v_or_x, i))/pi;
        conf_x_series = horzcat(conf_x_series, x_conf);
%         v_conf = result.atVector(gtsam.symbol('v', i));
%         conf_v_series = horzcat(conf_v_series, v_conf);
    end
    
    plot(handle, time_steps, conf_x_series)
    hold on;
%     plot(handle, time_steps, conf_v_series)
%     legend('x1','x2','v1','v2')
    xlabel('Time (s)');
    ylabel('State value (pi)');
end

