%% param_conversion.m
%
% Script to convert camera parameters / extrinsics from BLK2FLY to OKVIS2
%
% Author: Simon Boche
%
close all; clear all; clc
%% Input - edit here

% OKVIS Trajectory CSV
okvis_resultsfile = '/srv/Users/Boche/Projects/Leica/Data/test_submaps/extracted/okvis2-slam_trajectory.csv';
okvis_results = csvread(okvis_resultsfile, 1,0);
okvis_resultsfile2 = '/srv/Users/Boche/Projects/Leica/Data/test_submaps/extracted/test-okvis2-vio_trajectory.csv';
okvis_results2 = csvread(okvis_resultsfile2, 1,0);

% BLK Onboard Trajectory CSV
blk_resultsfile = '/srv/Users/Boche/Projects/Leica/Data/test_submaps/extracted/trajectory.csv';
blk_results = csvread(blk_resultsfile, 1,0);

% OKVIS IMU {S} to BLK Body {B} transformation (can be extracted using
% param_conversion.m)
T_BS = [  -0.999124121180130  -0.041392470270606   0.006136275819278   0.081436602559676
          -0.041295259124995   0.999031327784428   0.015202226122602   0.005653027198102;
          -0.006759589472211   0.014935511714707  -0.999865610189683   0.007586675624732;
           0.                  0.                  0.0                 1.000000000000000];

%% Process OKVIS2 Trajectory
% needs to be aligned to body frame as OKVIS returns traject
%
% Leica Result: T_WB
% OKVIS Result: T_WS
% thus we need t_SB expressed in 

% align z
% Leica: z downwards
% OKVIS: z upwards
align_z = [-pi 0 pi]; %euler angles xyz
R_Wl_Wo = eul2rotm(align_z,'xyz');
T_SB = inv(T_BS);

results_WlS = zeros(size(okvis_results,1),3);
results_WlB = zeros(size(okvis_results,1),3);
results_WlS2 = zeros(size(okvis_results2,1),3);
results_WlB2 = zeros(size(okvis_results2,1),3);


for i = 1:1:size(okvis_results,1)
    t_WS = okvis_results(i,2:4)';
    q_WS = [okvis_results(i,8) okvis_results(i,5:7)];
    R_WS = quat2rotm(q_WS);
    T_WS = [R_WS t_WS ; 0 0 0 1];
    T_WB = T_WS * T_SB;
    
    results_WlB(i,:) = R_Wl_Wo * T_WB(1:3,4);
    results_WlS(i,:) = R_Wl_Wo * okvis_results(i,2:4)';
    
end

for i = 1:1:size(okvis_results2,1)
    t_WS = okvis_results2(i,2:4)';
    q_WS = [okvis_results2(i,8) okvis_results2(i,5:7)];
    R_WS = quat2rotm(q_WS);
    T_WS = [R_WS t_WS ; 0 0 0 1];
    T_WB = T_WS * T_SB;
    
    results_WlB2(i,:) = R_Wl_Wo * T_WB(1:3,4);
    results_WlS2(i,:) = R_Wl_Wo * okvis_results2(i,2:4)';
    
end

%% Plotting
figure('Name', 'okvis trajectory')
hold on
axis equal
%plot(results_WlS(:,1),results_WlS(:,2))
%xlim([-3 10])
%ylim([-3 3])
plot(results_WlB(:,1),results_WlB(:,2), 'LineWidth',2)
plot(results_WlB2(:,1),results_WlB2(:,2), 'LineWidth',2)
%plot(blk_results(:,3), blk_results(:,4), 'LineWidth',2)
legend('OKVIS2 SLAM', 'OKVIS2 VIO', 'BLK2FLY')
set(gca, 'FontSize',24)
hold off

figure('Name', '3D trajectories')
hold on
axis equal
plot3(results_WlB(:,1),results_WlB(:,2),results_WlB(:,3), 'LineWidth',2)
plot3(results_WlB2(:,1),results_WlB2(:,2),results_WlB2(:,3), 'LineWidth',2)
%plot3(blk_results(:,3), blk_results(:,4), blk_results(:,5), 'LineWidth',2)
legend('OKVIS2 SLAM', 'OKVIS2 VIO', 'BLK2FLY')
set(gca, 'FontSize',24)
hold off

