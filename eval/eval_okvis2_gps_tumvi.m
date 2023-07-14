%% Script for Evaluation of OKVIS 2 Results including GPS measurements
clear all;
close all;
clc;
%addpath('/Users/simonboche/Documents/SRL/okvis2/scripts')

%% configurable paramenters - edit

% tumvi data format flag
tumvi = 1;

% so far: EuRoC datasets used
dataset_names = {'dataset-outdoors4_512_16'};

if tumvi
    gtname = 'mocap0';
else
    gtname = 'state_groundtruth_estimate0';
end

% folders
if tumvi
    dataset_folder = '/srv/Users/Boche/Documents/Datasets/TUMVI';
else
    dataset_folder = '/srv/Users/Boche/Documents/Datasets/EUROC_MAV';
end

% estimator versions

versions = {'run1_noGps-okvis2-vio-final',...
            'okvis2-vio', 'okvis2-vio-final'};
labels = categorical({'VIO', 'GPS-VIO live','GPS-VIO final'});
labels = reordercats(labels,cellstr(labels)');

delay_gt_estimates = [0.0, 0.0, 0.0, 0.0];

colours = {[0.9290, 0.6940, 0.1250],[0, 0.4470, 0.7410],[0, 0.4470, 0.7410]};
lineStyles = {'-',':','-'};
lineWidths = {6,6,6};

%% Plot Trajectories (no error caluclation)2

for i =1:size(dataset_names,2)
    name = dataset_names{i};
    figure('Name', "Pure Trajectories" + name);
    
    ground_truth_file = [dataset_folder '/' name '/mav0/' gtname '/data.csv'];
    fprintf('\tGround truth file: %s\n', ground_truth_file);
    ground_truth = csvread(ground_truth_file,1,0);
    ground_truth = ground_truth-ground_truth(1,:);
    
    for j = 1:size(versions,2)
        data_ij = csvread([dataset_folder '/' name '/mav0/' versions{j} '_trajectory.csv'], 1,0);
        % align to ground-truth
        [~,~,T_align] = align_trajectories_pos_only(data_ij(:,2:4),ground_truth(1:5000,2:4),...
            data_ij(:,1),ground_truth(1:5000,1));
        data_ij_aligned = zeros(size(data_ij,1),3);
        for k = 1:size(data_ij_aligned,1)
            data_ij_aligned(k,:) = (T_align(1:3,1:3)*data_ij(k,2:4)' +T_align(1:3,4))';
        end
        subplot(1,2,1)
        hold on
        plot(data_ij(:,2),data_ij(:,3), 'LineStyle', lineStyles{j}, 'Color', colours{j}, 'LineWidth', lineWidths{j})
        hold off
        subplot(1,2,2)
        hold on
        plot(data_ij(:,2),data_ij(:,3), 'LineStyle', lineStyles{j}, 'Color', colours{j}, 'LineWidth', lineWidths{j})
        hold off
    end
    
    % Plot ground truth
    
    subplot(1,2,1)
    hold on
    plot(ground_truth(:,2),ground_truth(:,3),':', ...
        'LineWidth', 4, 'Color', [0.5, 0.5, 0.5])
    axis equal
    xlabel 'x [m]'
    ylabel 'y [m]'
    legend([labels, 'Ground-Truth'], 'Location', 'NorthEastOutside')
    set(gca, 'FontSize', 30)
    hold off
    
    subplot(1,2,2)
    hold on
    plot(ground_truth(:,2),ground_truth(:,3),':', ...
        'LineWidth', 3, 'Color', [0.5,0.5,0.5])
    axis equal
    xlabel 'x [m]'
    ylabel 'y [m]'
    xlim([-4 2])
    ylim([-3 1])
    set(gca, 'FontSize', 30)
    hold off
end    
% actual plotting

%% Evaluate datasets - absolute

rmse_array = zeros(size(versions,2), size(dataset_names,2));

for i=1:size(dataset_names,2)
    name = dataset_names{i};
    fprintf('Evaluating dataset %s with estimator versions ', name);
    for k=1:size(versions,2)
        fprintf('%s ', versions{k});
    end
    fprintf('\n');

    ground_truth = [dataset_folder '/' name '/mav0/' gtname '/data.csv'];

    fprintf('\tGround truth file: %s\n', ground_truth);

    estimate_files = cell(size(versions));
    for k=1:size(versions,2)
        estimate_files{k} = [dataset_folder '/' name '/mav0/' versions{k} '_trajectory.csv'];
        fprintf('\tFile with estimated states: %s\n', estimate_files{k});
    end


    % absolute evaluation
    [p_g_ts_all, q_g_ts_all, p_e_ts_all, q_e_ts_all, p_eg_ts_all, q_eg_ts_all] = ...
        evaluate_dataset_absolute( ground_truth, ...
        estimate_files, ...
        delay_gt_estimates(k),...
        versions, 0);

    figure('Name', "Comparison GT vs. estimated Trajectories" + name);
    %plot3(p_g_ts_all{1}(:,1), p_g_ts_all{1}(:,2), p_g_ts_all{1}(:,3),'-k')
    plot(p_g_ts_all{1}(:,1), p_g_ts_all{1}(:,2),':', ...
        'LineWidth', 2, 'Color', [0.8,0.8,0.8])
    axis equal;
    hold on;
    for k=1:length(colours)
        %plot3(p_e_ts_all{k}(:,1), p_e_ts_all{k}(:,2), ...
        %    p_e_ts_all{k}(:,3), 'Color', colours{k})
        plot(p_eg_ts_all{k}(:,1), p_eg_ts_all{k}(:,2), 'Color', colours{k})

        % display RMSE of ATE
        err = p_eg_ts_all{k}-p_g_ts_all{k};
        sq = err.*err;
        sq_ate = (sum(sq,2));
        rmse = sqrt(mean(sq_ate))
        rmse_array(k,i) = rmse;
        ate = sqrt(sum(sq,2));
        mate = mean(ate)
        versions_rmse{k+1} = strcat(versions{k},", ", string(rmse), " m");
    end

    versions_rmse{1} = "Ground truth";
    %versions_rmse{k+2} = "GPS signal";
    %plot(gps_measurements(:,2), gps_measurements(:,3), 'o');
    legend(versions_rmse);

end


% Plot RMSEs for versions as bar plots
figure(3)
hold on
bar(labels,rmse_array)
ylabel 'Position RMSE [m]'
grid on
hold off


