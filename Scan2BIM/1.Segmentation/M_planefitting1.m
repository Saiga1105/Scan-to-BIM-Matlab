clear;close;%addpath('regionGrowing 2D3D');addpath('pointcloudtoolsfunctions');
tic
fprintf('loading point cloud...');
pc=pcread('F:\Data\Lidar_15.Sint Jacobs Kerk Leuven\PTG\test1a_culled.ply');
toc

xyz=pc.Location;
n=pc.Normal;
rgb=pc.Color;
pc1=pointCloud(xyz,'Color',rgb,'Normal',n);
maxDistance=0.01;
i=1;

% when point cloud> 10% initial point cloud, isolate inlier point cloud of
% plane fit
while size(pc1.Location,1)>=0.1*size(pc.Location,1)
tic
fprintf('computing plane...');
[model,inlierIndices,outlierIndices] = pcfitplane(pc1,maxDistance);
pcsub{i}=pointCloud(pc1.Location(inlierIndices,:),'Color',pc1.Color(inlierIndices,:),'Normal',pc1.Normal(inlierIndices,:));
pcsub{i+1}=pointCloud(pc1.Location(outlierIndices,:),'Color',pc1.Color(outlierIndices,:),'Normal',pc1.Normal(outlierIndices,:));
pc1=pointCloud(pc1.Location(outlierIndices,:),'Color',pc1.Color(outlierIndices,:),'Normal',pc1.Normal(outlierIndices,:));
planes{i}=model;
i = i + 1;
toc
end 
fprintf('Number of groups found:..%d..\n',i-1);
fprintf('Number of remaining points:..%d..',size(pc1.Location,1));


% pc.Location(inlierIndices,:)=[];