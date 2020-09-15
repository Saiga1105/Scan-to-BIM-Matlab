%this dataset contains all 6 areas
clear;close;
addpath(genpath('D:\Google Drive\Research\2017-05 point cloud segmentation\Matlab\Mijn code'));
addpath(genpath('D:\Google Drive\Research\2017-05 point cloud segmentation\Matlab\UGM'));

%% import data
load('F:\Data\Lidar_17.Stanford Benchmark\MATLAB\Stanford3dDataset_v1.2_Aligned_Version.mat')
filename='F:\Data\Lidar_17.Stanford Benchmark\Stanford3dDataset_v1.2\Area_1\labels.xlsx';
labels=importfile(filename);

xyz=[];
rgb=[];
y=[];
nr=5;
%isolate area 1
for i=1:length(AlignedArea(nr).Disjoint_Space)
    for j=1:length(AlignedArea(nr).Disjoint_Space(i).object)
        xyz=[xyz; AlignedArea(nr).Disjoint_Space(i).object(j).points];  
        rgb=[rgb; AlignedArea(nr).Disjoint_Space(i).object(j).RGB_color];
        %temp=zeros(length(AlignedArea(nr).Disjoint_Space(i).object(j).points),1)+table2array(labels(j,i));
        %y=[y ;temp];
    end
end
test=uint8(rgb);
pc=pointCloud(xyz,'Color',test);
pcwrite(pc,'Area4','PLYFormat','binary');
% % save y and pc to .mat file
% load('F:\Data\Lidar_17.Stanford Benchmark\MATLAB\Area1_cluster.mat');
% load('F:\Data\Lidar_17.Stanford Benchmark\MATLAB\Area1_training.mat');
% % 
% ThresValN = 30; % max angle deviation normals
% ThresValC= 30; % max color deviation over all channels
% MaxDist=0.10;   % if not indicated, dynamic 
% Minsize=2000; % minimum pixel size of a region
% Offset=0.02; % offset of computed plane
% Tilesize=1000000 ; % the data is seperated in N tiles that are dynamically assigned to each core.
% 
% 
% % region growing based on normals and color
% [ cluster,test ] = F_RegionGrowingNC( test,ThresValN,ThresValC,MaxDist,Minsize,Offset, Tilesize );
% 
% %cluster info
% [ Clusters ] = F_ClusterInfo( pc,cluster,y );