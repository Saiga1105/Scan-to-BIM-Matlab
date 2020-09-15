clear;close;addpath('regionGrowing 2D3D');addpath('pointcloudtoolsfunctions');

fprintf('loading point cloud...');
tic
%filename='F:\Data\Lidar_17.Stanford Benchmark\MATLAB\Area1_training.mat';
pc=pcread('F:\Data\Lidar_7.N-block\Point clouds\piece1_3M.ply');
%load(filename2);
%pc=test;
%pc=pcread('F:\Data\Lidar_15.Sint Jacobs Kerk Leuven\PTG\test1a_culled.ply');
fprintf('... %d points found...',length(pc.Location));
pcshow(pc)
toc

%% region growing
ThresValN = 50; % max angle deviation normals
ThresValC= 30; % max color deviation over all channels
MaxDist=0.10;   % if not indicated, dynamic 
Minsize=2000; % minimum pixel size of a region
Offset=0.02; % offset of computed plane
Tilesize=1000000 ; % the data is seperated in N tiles that are dynamically assigned to each core.

tic
[cluster] = F_RegionGrowingNC( pc,ThresValN,ThresValC,MaxDist,Minsize,Offset, Tilesize );
toc

% player=pcplayer(pc.XLimits,pc.YLimits,pc.ZLimits);
% for i=1:max(cluster)
%     view(player,select(pc,find(cluster==i)));
% end
% show unclustered points
%pcshow(select(pc,find(cluster==0)));

