function [ Parallelity, NormalZSim, Coplanarity, SurfacetypeSim,Dij] = F_FeatureEdgeExtraction( Clusters,NodeFeatures )
%F_FEATUREEDGEEXTRACTION Compute Edge features between nodes. Computes AreaSim, NormalZSim, NormalXYSim, SurfacetypeSim

% input
%   Clusters
%   edgeStruct
%   NodeFeatures

% output
%   AreaSim
%   NormalZSim
%   NormalXYSim
%   SurfacetypeSim

%% parameters
en=length(Clusters)*length(Clusters{1}.K);
Coplanarity= zeros(en,1);
NormalZSim=zeros(en,1);
Parallelity=zeros(en,1);
SurfacetypeSim=zeros(en,1);
Dij=zeros(en,1);
NormalThreshold=pi/6; %30 degrees

for e=1:length(Clusters)
    K=Clusters{e}.K;
    s1Normal=Clusters{e}.Normal;
    
    for k=1:length(K)
        %% Dij
        Dij(length(K)*(e-1)+k)=log(1/Clusters{e}.Kdist(k));
        
        %% Parallelity
        s2Normal=Clusters{K(k)}.Normal;
        Parallelity(length(K)*(e-1)+k)=abs(dot(s1Normal,s2Normal));
        
        %% NormalZSim
        NormalZSim(length(K)*(e-1)+k)=1-abs(abs(s2Normal(3))-abs(s1Normal(3)));
        
        %% coplanarity
        if Parallelity(length(K)*(e-1)+k)>NormalThreshold
            centroidvector=normr(Clusters{K(k)}.Centre-Clusters{e}.Centre);
            Coplanarity(length(K)*(e-1)+k)=1-abs(dot(s1Normal,centroidvector));
        end
        
        %% SurfacetypeSim
        if Clusters{K(k)}.SurfaceType == Clusters{e}.SurfaceType
        SurfacetypeSim(length(K)*(e-1)+k)=1;
        end
        
    end
   
end
end

