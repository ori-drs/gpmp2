function plotBlockAndConf(block_pos_x,block_pos_y, alpha, c, ...
            conf, arm , origin, cell_size)
%PLOTBLOCKANDCONF Summary of this function goes here
%   Detailed explanation goes here
        patch(block_pos_x, ...
        block_pos_y, ...
        c,...
        'FaceAlpha', alpha);
%         colormap(gray(100));
        hold on
        customPlotPlanarArm(arm.fk_model(), wrapToPi(conf), 'b', 2); 
        xlabel('X/m');
        ylabel('Y/m');
        axis equal
        axis([origin(1), 300*cell_size, ...
        origin(2), 300*cell_size])
end

