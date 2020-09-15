function [ EdgeFeatures] = F_FeatureEdgeExtraction( edgeStruct,Clusters )
%F_FEATUREEDGEEXTRACTION Compute Edge features between nodes. Computes
%parallelity,SurfacetypeSim and Distance

%% parameters

nEdges=edgeStruct.nEdges;
Parallelity=zeros(nEdges,1);
Surfacetypsimilarity=zeros(nEdges,1);
Dij=zeros(nEdges,1);

for i=1:edgeStruct.nEdges
    begin=edgeStruct.edgeEnds(i,1);
    ending=edgeStruct.edgeEnds(i,2);
    % surface similarity
    if Clusters{begin}.SurfaceType == Clusters{ending}.SurfaceType
        Surfacetypsimilarity(i)=1;
        % coplanarity
        par=abs(dot(Clusters{begin}.Normal,Clusters{ending}.Normal));
        if par >cos(pi/12)
        centroidvector=normr(Clusters{begin}.Centre-Clusters{ending}.Centre);
        Parallelity(i)=log(1/abs(dot(Clusters{begin}.Normal,centroidvector)));
        end
    end
   
    % distance
    test=find(Clusters{begin}.K==ending,1);
    if isempty(test)
        [edgej] = findNearestNeighbors(Clusters{ending}.PointCloud,Clusters{begin}.Centre,1);
        [edgei] = findNearestNeighbors(Clusters{begin}.PointCloud,Clusters{ending}.Centre,1);
        edgej=Clusters{ending}.PointCloud.Location(edgej,:);
        edgei=Clusters{begin}.PointCloud.Location(edgei,:);
        Dij(i)=pdist([edgej ;edgei]);
    else
        Dij(i)=Clusters{begin}.Kdist(test);
    end
    Dij(i)=log(1/Dij(i));
end

EdgeFeatures= [Parallelity Surfacetypsimilarity Dij];
EdgeFeatures=UGM_standardizeCols(EdgeFeatures,1);
end

