%% param_conversion.m
%
% Script to convert camera parameters / extrinsics from BLK2FLY to OKVIS2
%
% Author: Simon Boche
%
close all; clear all; clc
%% Input Files - edit here

% Dataset Folder
%dataset = '/srv/Users/Boche/Projects/Leica/Data/flight_02/';
%dataset = '/srv/Users/Boche/Projects/Leica/Data/20220209_kemptthal_flight03/';
dataset = '/srv/Users/Boche/Projects/Leica/Data/test_submaps/extracted/';

% Name of parameter files
intrinsics_file = 'cam_intrinsics.json';
extrinsics_file = 'extrinsics.json';
imu_file = 'imus.json';

%% Read Input - do not edit from here

% Read Intrinsics
fid_in = fopen([dataset intrinsics_file]); 
raw_in = fread(fid_in,inf); 
str_in = char(raw_in'); 
fclose(fid_in); 
intrinsics = jsondecode(str_in);

% Read Extrinsics
fid_ex = fopen([dataset extrinsics_file]); 
raw_ex = fread(fid_ex,inf); 
str_ex = char(raw_ex'); 
fclose(fid_ex); 
extrinsics = jsondecode(str_ex);

% Read IMUs

fid_imu = fopen([dataset imu_file]); 
raw_imu = fread(fid_imu,inf); 
str_imu = char(raw_imu'); 
fclose(fid_imu); 
imus = jsondecode(str_imu);

clear fid* raw* str* % Remove unnecessary variables from workspace

%% Process Data
% Assumption: Use bottom imu as this one is fixed
% => all relative transformations need to be determined to this IMU
% 
% we need: per camera:
% - T_SC : camera to IMU
% - image_dimensions => take from raw data
% - distortion_coefficients
% - distortion_type
% - intrinsic parameters (focal lengths, principal point)
%
% Leica quat definition: [qx qy qz qw] <-> Matlab: [qw, qx, qy, qz]

% Bottom 
q_sc_bottom = extrinsics.bottom.q_ic;
q_sc_bottom = [q_sc_bottom(4) q_sc_bottom(1) q_sc_bottom(2) q_sc_bottom(3)];
t_sc_bottom = extrinsics.bottom.t_ic;
T_sc_bottom = [quat2rotm(q_sc_bottom) t_sc_bottom; 0 0 0 1];
q_bc_bottom = extrinsics.bottom.q_bc;
q_bc_bottom = [q_bc_bottom(4) q_bc_bottom(1) q_bc_bottom(2) q_bc_bottom(3)];
t_bc_bottom = extrinsics.bottom.t_bc;
T_bc_bottom = [quat2rotm(q_bc_bottom) t_bc_bottom; 0 0 0 1];
q_ic_bottom = extrinsics.bottom.q_ic;
q_ic_bottom = [q_ic_bottom(4) q_ic_bottom(1) q_ic_bottom(2) q_ic_bottom(3)];
t_ic_bottom = extrinsics.bottom.t_ic;
T_ic_bottom = [quat2rotm(q_ic_bottom) t_ic_bottom; 0 0 0 1];

% Front
q_bc_front = extrinsics.front.q_bc;
q_bc_front = [q_bc_front(4) q_bc_front(1) q_bc_front(2) q_bc_front(3)];
t_bc_front = extrinsics.front.t_bc;
T_bc_front = [quat2rotm(q_bc_front) t_bc_front; 0 0 0 1];

% For OKVIS : camera to bottom IMU
T_sc_front = T_sc_bottom * inv(T_bc_bottom) * T_bc_front;


% Left
q_bc_left = extrinsics.left.q_bc;
q_bc_left = [q_bc_left(4) q_bc_left(1) q_bc_left(2) q_bc_left(3)];
t_bc_left = extrinsics.left.t_bc;
T_bc_left = [quat2rotm(q_bc_left) t_bc_left; 0 0 0 1];

% For OKVIS : camera to bottom IMU
T_sc_left = T_sc_bottom * inv(T_bc_bottom) * T_bc_left;

% Right
q_bc_right = extrinsics.right.q_bc;
q_bc_right = [q_bc_right(4) q_bc_right(1) q_bc_right(2) q_bc_right(3)];
t_bc_right = extrinsics.right.t_bc;
T_bc_right = [quat2rotm(q_bc_right) t_bc_right; 0 0 0 1];

% For OKVIS : camera to bottom IMU
T_sc_right = T_sc_bottom * inv(T_bc_bottom) * T_bc_right;

% Top
q_bc_top = extrinsics.top.q_bc;
q_bc_top = [q_bc_top(4) q_bc_top(1) q_bc_top(2) q_bc_top(3)];
t_bc_top = extrinsics.top.t_bc;
T_bc_top = [quat2rotm(q_bc_top) t_bc_top; 0 0 0 1];

% For OKVIS : camera to bottom IMU
T_sc_top = T_sc_bottom * inv(T_bc_bottom) * T_bc_top;

% Lidar
q_cbottomL = extrinsics.bottom.q_cl;
q_cbottomL = [q_cbottomL(4) q_cbottomL(1) q_cbottomL(2) q_cbottomL(3)];
t_cbottomL = extrinsics.bottom.t_cl;
T_cbottomL = [quat2rotm(q_cbottomL) t_cbottomL; 0 0 0 1];

% For Supereight: Lidar to body (if onboard trajectory is used)
T_BL = T_bc_bottom * T_cbottomL;

% For OKVIS + supereight : Lidar to bottom IMU {S} needed
T_SB = T_sc_bottom*T_cbottomL*inv(T_BL);
T_SL = T_SB * T_BL;

% If GPS is used : translation IMU {S} to GPS antenna {A}
T_SB = T_sc_bottom*T_cbottomL*inv(T_BL);
r_SA = T_SB(1:3,1:3) * extrinsics.t_b_gnss + T_SB(1:3,4);

clear q_* t_* % Remove unnecessary variables from workspace

%% Print Required Outputs

display("----- SE2 : Lidar {L} to Body {B} -----")
display(T_BL)

display("----- OKVIS : Lidar {L} to IMU bottom {S} -----")
display(T_SL)

display("----- GPS : displacement IMU {S} to GPS antenna {A} -----")
display(r_SA')

display("----- Camera - IMU extrinsics -----")
display("bottom:")
display(T_sc_bottom)
display("front:")
display(T_sc_front)
display("left:")
display(T_sc_left)
display("right:")
display(T_sc_right)
display("top:")
display(T_sc_top)
