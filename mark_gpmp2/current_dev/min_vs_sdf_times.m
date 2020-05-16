A = zeros(300,300,300);

len_obj = 5:5:100;
size_object = [];
B = zeros(300,300,300);

min_times = zeros(length(5:5:100),1);
mean_times = zeros(length(5:5:100),1);
max_times = zeros(length(5:5:100),1);

for i = 1:length(5:5:100)
    disp(i);
    obj_side_len = len_obj(i);
    
    t_taken = zeros(10,1);

    for j = 1:10
        A = rand([300,300,300]);
        B = rand([obj_side_len, obj_side_len, obj_side_len]);
        tic;
        res = min(A(100:99+obj_side_len,100:99+obj_side_len,100:99+obj_side_len),B);
        t_taken(j) = toc;
        clear res;
    end
    
    max_times(i) = max(max(max(t_taken)))*1000;
    min_times(i) = min(min(min(t_taken)))*1000;
    mean_times(i) = mean(mean(mean(t_taken)))*1000;
    
    
end

%%
close all;

ln_max_times = log(max_times);
ln_min_times = log(min_times);
ln_mean_times = log(mean_times);
    
    
    
figure(1); hold on;
er = errorbar(len_obj,mean_times,mean_times-min_times,max_times-mean_times);    
xlabel('Cube Object Length (Voxels)')
ylabel('Compute Time (ms)')
% er = errorbar(log(len_obj.^3),ln_mean_times,ln_mean_times-ln_min_times,ln_max_times-ln_mean_times);    
% xlabel('Object Size (Voxels)')
% ylabel('Compute Time (ms)')
er.Color = [0 0 0];                            
er.LineStyle = 'none';  
xlabel('Object Size (Voxels)')
ylabel('Compute Time (ms)')
