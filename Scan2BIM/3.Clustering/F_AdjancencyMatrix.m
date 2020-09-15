function [ adj ] = F_AdjancencyMatrix( Clusters)
%F_ADJANCENCYMATRIX Compute Adjancecy Matrix (Clusters)
%   Detailed explanation goes here

adj=zeros(length(Clusters));
for i=1:length(Clusters)
    
    for j=1:length(Clusters{i}.K)
        adj(i,Clusters{i}.K(j))=1;
        if Clusters{i}.K(j)==i
            adj(i,Clusters{i}.K(j))=0;
        end
        
    end
end

adj = adj+adj';
adj(adj~=0)=1;

end

