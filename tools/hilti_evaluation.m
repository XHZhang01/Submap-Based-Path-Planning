%% VIO Evaluation of HILTI Dataset
% written by Simon Boche (02/2023)

clear all
close all
clc

%% configurable paramenters - edit

% Path to hilti datasets
datapath = '/srv/Users/Boche/Documents/Datasets/HILTI';

% dataset names
dataset_name = 'exp04_construction_upper_level';

% estimator versions
versions = {'okvis2-slam', 'okvis2-slam-final'};

% discretisation
Delta_s = 10.0; % [m]


%% processing


% folders
dataset_folder = [datapath '/' dataset_name '/'];
groundtruth_folder = [datapath '/' dataset_name '/' ];

% groundtruth delay w.r.t. the estimator output
delay_gt_estimates = [0.0, 0.0, 0.0, 0.0, 0.0];

%% Plot Ground Truth
ground_truth = [groundtruth_folder '/' dataset_name '_imu.txt'];
fprintf('\tGround truth file: %s\n', ground_truth);

fileID = fopen(ground_truth,'r');
gt = textscan(fileID, '%f%f%f%f%f%f%f%f');
fclose(fileID);

gt_pos = zeros(length(gt{1}),4);
gt_pos(:,1) = gt{1};
gt_pos(:,2) = gt{2};
gt_pos(:,3) = gt{3};
gt_pos(:,4) = gt{4};

gt_all = zeros(length(gt{1}), 8);
for i = 1:8
    if i == 1
        gt_all(:,i) = gt{i}*1e+09;
    else
        gt_all(:,i) = gt{i};
    end
end
%figure('Name', 'GT Trajectory')
%hold on
%plot(gt{2}, gt{3})
%xlabel 'x[m]'
%ylabel 'y[m]'
%hold off

%% Process ATE
    
for j = 1:size(versions,2)
    estimate_file = [dataset_folder '/' versions{j}...
                     '_trajectory.csv'];
    fprintf('\t Estimate file: %s\n', estimate_file);

    estimate = csvread(estimate_file,1,0);

    % compute length 
    len = 0;
    for ll = 2:size(gt{1},1)
        len = len + norm(gt_all(ll,2:4)-gt_all(ll-1,2:4));
    end
    %len

    % align yaw 
    [estimate_aligned,gt_aligned,~] = align_trajectories_pos_only(...
            estimate(:,2:4), gt_all(:,2:4),...
            estimate(:,1), gt_all(:,1));

    err = estimate_aligned-gt_aligned;
    sq = err.*err;
    sq_ate = (sum(sq,2));
    rmse = sqrt(mean(sq_ate));
    ate = sqrt(sum(sq,2));
    mate = mean(ate)
    
    % Plot
    

end

figure('Name' , 'Trajectories comparison')
hold on
xlabel 'x[m]'
ylabel 'y[m]'
plot(gt_aligned(:,1), gt_aligned(:,2))
plot(estimate_aligned(:,1), estimate_aligned(:,2))
legend('1', '2')
hold off
