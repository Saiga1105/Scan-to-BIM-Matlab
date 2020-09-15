clear;close;addpath('D:\Google Drive\Research\2017-05 point cloud segmentation\Matlab\CRFall');
addpath('D:\Google Drive\Research\2017-05 point cloud segmentation\Matlab\Mijn Code');

fprintf('loading point cloud...');
tic
pc=pcread('F:\Data\Lidar_15.Sint Jacobs Kerk Leuven\PTG\test1a_culled.ply');
fprintf('... %d points found...',length(pc.Location));
pcshow(pc)
toc

% region growing
ThresValN = 50; % max angle deviation normals
ThresValC= 30; % max color deviation over all channels
MaxDist=0.10;   % if not indicated, dynamic 
Minsize=2000; % minimum pixel size of a region
Offset=0.02; % offset of computed plane
Tilesize=1000000 ; % the data is seperated in N tiles that are dynamically assigned to each core.

tic
[cluster] = F_RegionGrowingNC( pc,ThresValN,ThresValC,MaxDist,Minsize,Offset, Tilesize );
toc

tic
[ centre,normals,normalsstd,colors,type ] = F_ClusterInfo( pc,cluster );
toc