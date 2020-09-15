clear;close;

fprintf('loading point cloud...');
tic
%filename='K:\Projects\V4Design\Data\PUC4sc1 Gendarmenmarkt\Mesh\market1\Gendarmenmarkt_simplification.ply';
filename='K:\Projects\V4Design\Data\PUC4sc1 Gendarmenmarkt\PCD\markt_piece1.ply';
%filename='K:\Projects\Consultancy\2019-02 Sint-jacobs vaults Monument\PCD vault monument\scheur2.ply';
%pc=pcread('F:\Data\Lidar_7.N-block\Point clouds\piece1_3M.ply');
pc=pcread(filename);
fprintf('... %d points found...',length(pc.Location));
pcshow(pc)
toc

%% region growing
ThresValN = 30; % default=50deg : max angle deviation normals
ThresValC= 30; % default=50: max color deviation over all channels
MaxDist=0.5;   % default=0.10m :if not indicated, dynamic 
Minsize=200; % default=2000 :minimum pixel size of a region
Offset=0.05; % default=0.02m :offset of computed plane
Tilesize=1000000 ; % default=1000000 : the data is seperated in N tiles that are dynamically assigned to each core.


%[cluster] = F_RegionGrowingNC( pc,ThresValN,ThresValC,MaxDist,Minsize,Offset, Tilesize );
%toc

points= pc.Location;
colors=[];
normals=pc.Normal;

tic
%[ clusters] = G_RegionGrowingNC4(points,normals,colors);
[ cluster,pctotal] = F_RegionGrowingNC( pc,ThresValN,ThresValC,MaxDist,Minsize,Offset, Tilesize )
%cluster = G_RegionGrowingNC2( points,normals,colors,ThresValN,ThresValC,MaxDist,Minsize,Offset, Tilesize );
toc
%% 