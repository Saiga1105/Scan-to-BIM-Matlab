%MB_CreateTrainingData Process raw .ply, computes feature values and exports a .mat file that can be used for training MB_ParameterEstimation
clear;close;
addpath(genpath('D:\Google Drive\Research\2017-05 point cloud segmentation\Matlab\Mijn code'));
addpath(genpath('D:\Google Drive\Research\2017-05 point cloud segmentation\Matlab\UGM'));
filename='F:\Data\Lidar_17.Stanford Benchmark\MATLAB\Area1.ply';
load('F:\Data\Lidar_17.Stanford Benchmark\MATLAB\Area1_labels.mat');
load('F:\Data\Lidar_17.Stanford Benchmark\MATLAB\Area1_cluster.mat');
%filename='F:\Data\Lidar_15.Sint Jacobs Kerk Leuven\PTG\test1b_culled.ply';
%% load data
fprintf('loading point cloud...');
tic
pc=pcread(filename);
fprintf('... %d points found...',length(pc.Location));
toc

%% Grow Clusters
ThresValN = 30; % max angle deviation normals
ThresValC= 30; % max color deviation over all channels
MaxDist=0.10;   % if not indicated, dynamic 
Minsize=2000; % minimum pixel size of a region
Offset=0.02; % offset of computed plane
Tilesize=1000000 ; % the data is seperated in N tiles that are dynamically assigned to each core.


% region growing based on normals and color
[ cluster,pc ] = F_RegionGrowingNC( pc,ThresValN,ThresValC,MaxDist,Minsize,Offset, Tilesize );
%load('F:\Data\Lidar_7.N-block\Point clouds\pcN1.mat');

%% Compute Cluster Info
y=cluster;
tic
fprintf('Compute cluster info...');
[ Clusters ] = F_ClusterInfo( pc,cluster,y);
toc

n=15; % evaluation pool
k=4; % neighbours
TLarge=4; % threshold large surface 4m²
Thori=pi/4; % angle threshold horizontal surf 45°
tic
fprintf('Compute Neighbours...');
[ Clusters ] = F_Neighbours( Clusters,n,k,TLarge,Thori );
toc
%% Xnode Feature extraction 
% compute unary potentials
% genereer 1 tabel met features voor elke node
% bv node1
% Area1 NormalZ1 Height1 Width1 Surfacetype1 
% Area2 NormalZ2 Height2 Width2 Surfacetype2
tic
fprintf('Extract NodeFeatures...');
[Area, NormalZ, Height, Width, Surfacetype ]=F_FeatureNodeExtraction(Clusters);
NodeFeatures=[abs(Area), NormalZ, abs(Height), abs(Width), Surfacetype ];
toc
%% Xedge Feature extraction 
% compute pairwise potentials
% genereer 1 tabel met features voor elke edge (zoals nodes)
tic
fprintf('Extract Edge Features...');
[Parallelity, NormalZSim, Coplanarity, SurfacetypeSim,Dij ]=F_FeatureEdgeExtraction(Clusters,NodeFeatures);
EdgeFeatures=[Parallelity, NormalZSim, Coplanarity, SurfacetypeSim,(Dij/max(Dij)) ];
toc

%% Export training data
% NodeFeatures, EdgeFeatures, Clusters, labels


