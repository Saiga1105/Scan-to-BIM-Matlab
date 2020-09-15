function [ Clusters ] = F_Neighbours( Clusters,n,k,Tlarge,Thori )
%% Compute Neighbours

% input
%   Cluster{1:p}: cell array of 1xM structures containing the information
%                 of each cluster
%   n: evaluation pool (15)
%   k: # neighbours (4)
%   TLarge: threshold large surface 4m²
%   Thori: angle threshold horizontal surf 45°

% output
%   Cluster.K
%   Cluster.KHLarge
%   Cluster.KHLargeAbove
%   Cluster.KHLargeBelow
%   Cluster.KVLarge

 %% preallocate arrays

C=zeros(length(Clusters),3);
% Count=zeros(1,length(Clusters));
% A=false(length(Clusters));

%% Create point cloud of Centres
for i=1:length(Clusters)
    C(i,:)=Clusters{i}.Centre;
end
pc=pointCloud(C);

%% compute k neighbours
for i=1:length(Clusters)
    Dij=zeros(1,n);
    % compute neirest neighbours    
    [indices,~] = findNearestNeighbors(pc,C(i,:),n);
    indices(1)=[];
    % compute edge to edge distance for each k
    for j=1:length(indices)
        [edgej] = findNearestNeighbors(Clusters{indices(j)}.PointCloud,C(i,:),1);
        [edgei] = findNearestNeighbors(Clusters{i}.PointCloud,Clusters{indices(j)}.Centre,1);
        edgej=Clusters{indices(j)}.PointCloud.Location(edgej,:);
        edgei=Clusters{i}.PointCloud.Location(edgei,:);
        Dij(j)=pdist([edgej ;edgei]);
    end
    
    % remove invalid instances
    indices=indices(Dij~=0);
    Dij=Dij(Dij~=0);
    
    % compute 4 closest neighbours
    [dist,order] = sort(Dij);
    K=indices(order);
    if length(K)>=k
        Clusters{i}.K=K(1:k);
        Clusters{i}.Kdist=dist(1:k);
    else
        Clusters{i}.K=K;
        Clusters{i}.Kdist=dist;
    end
    
    % compute large surfaces
%     A=zeros(length(indices),1);
%     for j=1:length(indices)
%         x=Clusters{indices(j)}.PointCloud.XLimits(2)-Clusters{indices(j)}.PointCloud.XLimits(1);
%         y=Clusters{indices(j)}.PointCloud.YLimits(2)-Clusters{indices(j)}.PointCloud.YLimits(1);
%         xy=sqrt(x^2+y^2);
%         z=Clusters{indices(j)}.PointCloud.ZLimits(2)-Clusters{indices(j)}.PointCloud.ZLimits(1);
%         A(j)=z*xy;
%     end
%     indicesLarge=indices(A>=Tlarge);
%     Dij=Dij(A>=Tlarge);
%     
%     if Clusters{j}
    
end

end

