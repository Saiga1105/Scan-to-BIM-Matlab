function [ nodePot ] = F_NodePotGraphCut( Clusters,edgeStruct)
%F_ADJANCENCYMATRIX Compute Adjancecy Matrix (Clusters)
%   Detailed explanation goes here

nodePot=ones(edgeStruct.nNodes,edgeStruct.nStates(1));
nodePot(:,1)=2;
% give neighbouring nodePots 2
% for i=1:length(Clusters)
%     
%     for j=1:length(Clusters{i}.K)
%         nodePot(i,Clusters{i}.K(j))=1;
%         
%         if Clusters{i}.K(j)==i
%             nodePot(i,Clusters{i}.K(j))=0;
%         end
%         
%     end
% end
% nodePot = nodePot+nodePot';
% nodePot(nodePot~=0)=2;
% 
% % set own nodePot to 3
% ii = diag(true(length(Clusters),1),0);
% nodePot(ii)=3;

% % set others to 1
% nodePot(nodePot==0)=1;

end

