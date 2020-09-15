%MB_CreateTrainingData Process raw .ply, computes feature values and exports a .mat file that can be used for training MB_ParameterEstimation
clear;close;
addpath(genpath('D:\Google Drive\Research\2017-05 point cloud segmentation\Matlab\Mijn code'));
addpath(genpath('D:\Google Drive\Research\2017-05 point cloud segmentation\Matlab\UGM'));

%% load data
fprintf('loading data...');
tic
% see MB_createtrainingsdata to make these files
%load('F:\Data\Lidar_17.Stanford Benchmark\MATLAB\Area1_labels.mat');
%load('F:\Data\Lidar_17.Stanford Benchmark\MATLAB\Area1_cluster.mat');
% load('F:\Data\Lidar_17.Stanford Benchmark\MATLAB\Area1_Clusters.mat');
% %load('F:\Data\Lidar_17.Stanford Benchmark\MATLAB\Area1_pcd.mat');
% load('F:\Data\Lidar_17.Stanford Benchmark\MATLAB\Area1_EdgeFeatures.mat');
% load('F:\Data\Lidar_17.Stanford Benchmark\MATLAB\Area1_NodeFeatures2.mat');
% % pc=pcread(filename);
%fprintf('... %d points found...',length(pc.Location));
toc
%% load data
fprintf('loading point cloud...');
tic
filename='F:\Data\Lidar_17.Stanford Benchmark\PCD\Area4.ply';
pc=pcread(filename);
fprintf('... %d points found...',length(pc.Location));
toc
pcshow(pc)
%% Grow Clusters
ThresValN = 30; % max angle deviation normals
ThresValC= 30; % max color deviation over all channels
MaxDist=0.10;   % if not indicated, dynamic 
Minsize=2000; % minimum pixel size of a region
Offset=0.02; % offset of computed plane
Tilesize=1000000 ; % the data is seperated in N tiles that are dynamically assigned to each core.


% region growing based on normals and color
tic
fprintf('Growing Clusters...');
[ cluster,pc ] = F_RegionGrowingNC( pc,ThresValN,ThresValC,MaxDist,Minsize,Offset, Tilesize );
toc

%% Compute Cluster Info
tic
fprintf('Compute cluster info...');
[ Clusters ] = F_ClusterInfo( pc,cluster);
toc

n=20; % evaluation pool
k=10; % neighbours
TLarge=4; % threshold large surface 4m²
Thori=pi/4; % angle threshold horizontal surf 45°
tic
fprintf('Compute Neighbours...');
[ Clusters ] = F_Neighbours( Clusters,n,k,TLarge,Thori );
toc


%% Graph description
adj = F_AdjancencyMatrix( Clusters );
nStates=2;
edgeStruct = UGM_makeEdgeStruct(adj,nStates);
nEdges=edgeStruct.nEdges;
nNodes = length(Clusters);

%% features
tic
fprintf('Feature extraction...');
[ Area, NormalZ, Height, Width, Surfacetype ] = F_FeatureNodeExtraction( Clusters);
NodeFeatures=[Area,Surfacetype];
NodeFeatures=UGM_standardizeCols(NodeFeatures,1);
EdgeFeatures=F_FeatureEdgeExtraction2(edgeStruct,Clusters);
nNodeFeatures=size(NodeFeatures,2); % Area, Surfacetype
nEdgeFeatures= size(EdgeFeatures,2); % parallel, surfacetype, distance
nInstances=1;

Xnode=zeros(nInstances,nNodeFeatures,nNodes);
Xnode(1,1,:)=NodeFeatures(:,1);
Xnode(1,2,:)=NodeFeatures(:,2);

Xedge=zeros(nInstances,nEdgeFeatures,nEdges);
Xedge(1,1,:)=EdgeFeatures(:,1);
Xedge(1,2,:)=EdgeFeatures(:,2);
Xedge(1,3,:)=EdgeFeatures(:,3);
toc

%% map parameters
% Map parameters over NodePot and EdgePot
% set parameters identifiers for the different states. each feature gets a
% parameter
nodeMap = zeros(nNodes,nStates,nNodeFeatures,'int32'); % s*f = 5w
for f=1:nNodeFeatures
    for state=1:(nStates)
        nodeMap(:,state,f)=nStates*(f-1)+state;
    end
    
end
edgeMap = zeros(nStates,nStates,nEdges,nEdgeFeatures,'int32');
f=nStates*f;
for edgeFeat = 1:nEdgeFeatures % 25 per feature (3) => +75w
    for s1 = 1:nStates
        for s2 = 1:nStates
                f = f+1;
                edgeMap(s1,s2,:,edgeFeat) = f;
%                 if edgeMap(nStates,nStates,:,edgeFeat) == f;
%                 edgeMap(nStates,nStates,:,edgeFeat)=0;
%                 f=f-1;
%                 end
        end
    end
end

%% parameter vector w
% Initialize weights
% nParams = max([nodeMap(:);edgeMap(:)]);
% w = ones(nParams,1);
 w= [0.520733113557813;1.09445475416208;-1.33907572299355;-0.706258142090149;-0.295255412397591;-0.295719873166258;-0.580045494893548;-0.262428254771018;0.724673730409234;0.265827509696869;0.393378002002510;0.385913801421776;0.728688610830049;0.779993005387577;0.453012541268464;0.909443554277232];

%% Potentials
% %nodePot(n,s) with n: nodes and s: states
% edgePot(s,s,e) with e: nEdges
% in eerste instantie gaan we de gewichten niet trained maar zelf kiezen.
instance = 1;
[nodePot,edgePot] = UGM_CRF_makePotentials(w,Xnode,Xedge,nodeMap,edgeMap,edgeStruct,instance);

%% decoding & inference
% hier krijgen we de meest waarschijnlijke state van elke cluster
tic
fprintf('Decoding Graph...');
[nodeBel, edgeBel, logZ] = UGM_Infer_LBP(nodePot,edgePot,edgeStruct);
y = UGM_Decode_LBP(nodePot,edgePot,edgeStruct);
%tabulate(y);
% 1= geisoleerde cluster
% 2= cluster die wil mergen
%inliers=tabulate(y-label)
toc
% a=0;c=0;
% for i =1:length(y)
%     if y(i)==2 && label(i)==2
%         a=a+1;
%     end
%     if y(i)==1 && label(i)==1
%         c=c+1;
%     end
% end
% a
% c

%% cluster groups
tic
fprintf('Clustering...');
angulart=0.5;
probt=0.5;

[ clustering ] = F_Clustering( y,nNodes,edgeStruct,edgeBel,Clusters,angulart,probt );
clustering=clustering(:,2);
toc
%% clustering results
% % initial
% % tic
% % fprintf('merging results...');
% xyz=Clusters{1}.PointCloud.Location;
% rgb=Clusters{1}.PointCloud.Color;
% for i=1:length(Clusters);
%      Clusters{i}.Color=randi([0 255],1,3);
%      RGBlocal=repmat(Clusters{i}.Color,size(Clusters{i}.PointCloud.Color,1),1);
%     
%      if i~=1
%      xyz=[xyz ;Clusters{i}.PointCloud.Location];
%      rgb=[rgb ;RGBlocal];
%     end
% end
% % pcsegmented=pointCloud(xyz,'Color',rgb);
% % figure
% % pcshow(pcsegmented);
% 
% xyz=Clusters{1}.PointCloud.Location;
% rgb=Clusters{1}.PointCloud.Color;
% for i=1:length(Clusters);
%      Clusters{i}.Color=Clusters{clustering(i)}.Color;
%      RGBlocal=repmat(Clusters{i}.Color,size(Clusters{i}.PointCloud.Color,1),1);
%      %Clusters{i}.PointCloud.Color=uint8(RGBlocal);
% %     Clusters{i}.label=Clusters{clustering(i)}.Label;
%     if i~=1
%      xyz=[xyz ;Clusters{i}.PointCloud.Location];
%      rgb=[rgb ;RGBlocal];
%      %pctest=pcmerge(pctest,Clusters{i}.PointCloud,5);
%     end
% end
% pcclustered=pointCloud(xyz,'Color',rgb);
% figure
% pcshow(pcclustered);
% toc
Reduction=1-(length(unique(clustering))/nNodes)
% specific cluster 
% %% test
% nr=179;
% xyz=Clusters{nr}.PointCloud.Location;
% rgb=Clusters{nr}.PointCloud.Color;
% for i=1:length(Clusters);
% 
%     if clustering(i)==nr
%      %test1=Clusters{i}.Label;
%      %test2=test1(find(clustering==129))
%      xyz=[xyz ;Clusters{i}.PointCloud.Location];
%      rgb=[rgb ;Clusters{i}.PointCloud.Color];
%      %pctest=pcmerge(pctest,Clusters{i}.PointCloud,5);
%     end
%     
% end
% pctest=pointCloud(xyz,'Color',rgb);
% pcshow(pctest);

% % pcshow(Clusters{1}.PointCloud)
% for i=1:length(Clusters);
%     Clusters{i}.Color=randi([0 255],1,3);
%     pcshowpair(pctest,Clusters{i}.PointCloud)
%     pause
% end
