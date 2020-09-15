function [ edgePot ] = F_EdgePotGraphCut( Clusters,edgeStruct)
%F_ADJANCENCYMATRIX Compute Adjancecy Matrix (Clusters)
%   Detailed explanation goes here

edgePot=ones(edgeStruct.nStates(1),edgeStruct.nStates(1),edgeStruct.nEdges);
edgePot(:,1,:)=5;
edgePot(1,:,:)=5;
edgePot(1,1,:)=10;
% give neighbouring nodePots 2
for i=1:length(Clusters)
    
    for j=1:length(Clusters{i}.K)
        edgePot(i,Clusters{i}.K(j),:)=1;
        
        if Clusters{i}.K(j)==i
            edgePot(i,Clusters{i}.K(j),:)=0;
        end
        
    end
end
edgePot = edgePot+permute(edgePot,[2 1 3]);
edgePot(edgePot~=0)=2;

% set own nodePot to 3
ii = diag(true(length(Clusters),1),0);
ii=repmat(ii,[1 1 edgeStruct.nEdges]);
edgePot(ii)=3;

% set others to 1
edgePot(edgePot==0)=1;
end

