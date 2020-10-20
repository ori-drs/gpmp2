
function results_vis(results, delta_t, iters, f)

    f;
    ax = gca;
    sgtitle("Time step 0");
    subplot(1,2,1);
    ax = gca;
    % initial plot
    conf_x_series = [];
    conf_v_series = [];

    num_steps = size(results)-1;
    time_steps = 0:delta_t:delta_t*num_steps;

    for i=0:num_steps
        x_conf = results{1}.atVector(gtsam.symbol('x', i));
        conf_x_series = horzcat(conf_x_series, x_conf);
        v_conf = results{1}.atVector(gtsam.symbol('v', i));
        conf_v_series = horzcat(conf_v_series, v_conf);
    end
    
    x_handles = plot(ax, time_steps, conf_x_series);
    hold on;
    v_handles = plot(ax, time_steps, conf_v_series);
    legend('x1','x2','v1','v2')
    xlabel('Time (s)');
    ylabel('State value');
    xlim([0,6.5]);
    ylim([-8,8]);
    
    uicontrol('Style', 'slider', 'Min', 0, 'Max', iters, ...
       'Value', 0, 'SliderStep',[0.02 , 0.2], 'Position', [400 20 120 20], ...
       'Callback', @react_to_slider);
   
   
  function react_to_slider(source, event)   %nested !!
    val = round(get(source, 'Value'));
    %       disp(val);
    set(source, 'Value', val);
    f;
    ax = gca;
    sgtitle("Time step: " + num2str(val));
    
    subplot(1,2,1);
    ax = gca;
    conf_x_series = [];
    conf_v_series = [];

    for i=0:num_steps
      x_conf = results{val}.atVector(gtsam.symbol('x', i));
      conf_x_series = horzcat(conf_x_series, x_conf);
      v_conf = results{val}.atVector(gtsam.symbol('v', i));
      conf_v_series = horzcat(conf_v_series, v_conf);
    end

    set(x_handles(1), 'YData', conf_x_series(1,:));
    set(x_handles(2), 'YData', conf_x_series(2,:));
    set(v_handles(1), 'YData', conf_v_series(1,:));
    set(v_handles(2), 'YData', conf_v_series(2,:));
    
  end
end