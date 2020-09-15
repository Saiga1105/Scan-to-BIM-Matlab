clear;close;addpath('regionGrowing 2D3D');addpath('pointcloudtoolsfunctions');
tic
fprintf('loading point cloud...');
pc=pcread('F:\Data\Lidar_15.Sint Jacobs Kerk Leuven\PTG\test1a_full.ply');
toc

M=402;
N=406;

% create raster image point cloud
[ pc ] = F_pointCloudtoRasterIm( pc,M,N);

% region growing
ThresVal = 0.1; % deviation 
MaxDist=0.05;   % max distance [m] between neighbouring pixels beware of resolution
Resolution=10;      % u,v uniform seed resolution  
Minsize=500; % minimum pixel size of a region

I=rgb2gray(mat2gray(pc.Normal));
test=rgb2gray(mat2gray(pc.Color));
test=imgaussfilt(test,2); % gaussian smooth
test2=rgb2gray(mat2gray(pc.Location));

tic
[J,Bn] = F_RegionGrowingImage( pc,I,ThresVal,MaxDist,Resolution,Minsize );
toc

imshow(pc.Color);
hold on
for k = 1:length(Bn)
   boundary = Bn{k};
   plot(boundary(:,2), boundary(:,1), 'LineWidth', 2);
end
