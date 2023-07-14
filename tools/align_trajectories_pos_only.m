function [ p_e,p_g, T_ge] = ... 
    align_trajectories_pos_only( p_e, p_g, t_e, t_g, yaw_only)
%ALIGN_TRAJECTORIES Summary of this function goes here
%   Detailed explanation goes here

if nargin < 5
    yaw_only = 0;
end

if length(t_e)>length(t_g)
    idx = knnsearch(t_e, t_g);
    p_e = p_e(idx,:);
else
    idx = knnsearch(t_g,t_e);
    p_g = p_g(idx,:);
end


% compute centroid for estimate
e_centroid = zeros(1,3);

for i=1:size(p_e,1)
    e_centroid = e_centroid + p_e(i,:);
end
e_centroid = e_centroid / size(p_e,1);

%compute centroid for ground truth
g_centroid = zeros(1,3);

for i=1:size(p_g,1)
    g_centroid = g_centroid + p_g(i,:);
end
g_centroid = g_centroid / size(p_g,1);

% compute new estimate trajectory
estimate_normalized = zeros(size(p_e));
for i=1:size(p_e,1)
    estimate_normalized(i,:) = p_e(i,:) - e_centroid;
end

% compute new ground truth trajectory
groundtruth_normalized = zeros(size(p_g));
for i=1:size(p_g,1)
    groundtruth_normalized(i,:) = p_g(i,:) - g_centroid;
end

% compute matrix H
H = zeros(3);

% no roll/pitch alignment
%estimate_normalized(:,3) = 0*estimate_normalized(:,3);
%groundtruth_normalized(:,3) = 0*groundtruth_normalized(:,3);
for i = 1:size(p_g,1)
    H = H + estimate_normalized(i,:)'*groundtruth_normalized(i,:);
end


if(yaw_only) % to do:rewrite function with flag for pure yaw alignment
    % following https://github.com/uzh-rpg/rpg_trajectory_evaluation/blob/master/src/rpg_trajectory_evaluation/align_trajectory.py
    A = H(1,2)-H(2,1);
    B = H(1,1)+H(2,2);
    
    yaw = pi /2 - atan2(B,A);
    
    R_ge = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0 ; 0 0 1];
    
else
    % Compute SVD
    [U,~,V] = svd(H);

    % build solution
    X = V*U';
    if det(X) == -1
        V(1:3,3) = -V(1:3,3);
        X = V*U';
    end

    R_ge = X;
end

t_ge = g_centroid' - R_ge*e_centroid';

% apply transformation to estimate
for i=1:size(p_e,1)
    p_e(i,:) = (R_ge*p_e(i,:)' + t_ge )';
end


% compose the transformation
T_ge = eye(4);
T_ge(1:3,1:3) = R_ge;
T_ge(1:3,4) = t_ge;


end
