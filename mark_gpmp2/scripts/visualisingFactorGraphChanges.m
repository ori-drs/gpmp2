% 
% @author Mark Finean 
% @date April 25, 2020

import gtsam.*
import gpmp2.*

total_steps = 20;
obs_factor_times = 0:total_steps;
obs_interp_factor_times = setdiff(0:0.1:total_steps, obs_factor_times);

factor_times_updated = [1;2;3;4];
all_updated_factors = reshape(repmat(factor_times_updated*10, 1, 10) + [0:9],1, [])/10;

updated_interps = setdiff(all_updated_factors, obs_factor_times);
updated_supports = setdiff(all_updated_factors, updated_interps);

sz1 = 500;
sz2 = 25;
c = 'b';

figure(1); hold on;
title("Factor Graph");
xlabel('Time');
xlim([-1,total_steps + 1]);
ylim([0.98,1.02])
scatter(obs_interp_factor_times, ones(size(obs_interp_factor_times)), sz2, c, 'filled');
scatter(updated_interps, ones(size(updated_interps)), sz2, 'r', 'filled');
scatter(obs_factor_times, ones(size(obs_factor_times)), sz1, c, 'filled');
scatter(updated_supports, ones(size(updated_supports)), sz1, 'r', 'filled');

% axis('tight')
pbaspect([10 1 1])
set(gca,'ytick',[])
set(gca,'yticklabel',[])
set(gca,'Xtick',0:total_steps)
% set(gca,'XtickLabel',obs_interp_factor_times)
text(5.45, 1.01, cellstr(num2str(1)));



