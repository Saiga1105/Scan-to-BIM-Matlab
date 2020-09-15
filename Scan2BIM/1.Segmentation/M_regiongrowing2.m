clear;close;addpath('regionGrowing 2D3D');addpath('pointcloudtoolsfunctions');

fprintf('loading point cloud...');
tic
pc=pcread('F:\Data\Lidar_15.Sint Jacobs Kerk Leuven\PTG\test1a_culled.ply');
fprintf('... %d points found...',length(pc.Location));
pcshow(pc)
toc

% region growing
ThresVal = 30; % max angle deviation normals
MaxDist=0.10;   % if not indicated, dynamic 
Minsize=1000; % minimum pixel size of a region
Offset=0.01; % offset of computed plane
Tilesize=1000000 ; % the data is seperated in N tiles that are dynamically assigned to each core.

tic
[cluster] = F_RegionGrowingN( pc,ThresVal,MaxDist,Minsize,Offset,Tilesize);
toc

% player=pcplayer(pc.XLimits,pc.YLimits,pc.ZLimits);
% for i=1:max(cluster)
%     view(player,select(pc,find(cluster==i)));
% end
% show unclustered points
%pcshow(select(pc,find(cluster==0)));

