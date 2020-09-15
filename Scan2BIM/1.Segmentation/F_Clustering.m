function [ clustering ] = F_Clustering( y,nNodes,edgeStruct,edgeBel,Clusters,angulart,probt )
%F_CLUSTERING Summary Grouping of clusters
% identificeer wijzigende cluster
% identificeer hun beste link
% check of link 1 of 2 is
% indien 2, zoek naar volgende beste link tot je 1 tegenkomt.
% weizig label
%% clustering
clustering= [(1:nNodes)' y];
clustering(clustering(y==1),2)=clustering(y==1);
clustering(clustering(y==2),2)=0;
clusteringa=clustering;

mergingcluster=find(clustering(:,2)==0);
    %a=a+1;
    for i=1:length(mergingcluster)
        %node (2) filter
        candidateedges=UGM_getEdges(mergingcluster(i),edgeStruct);
        candidatenode = edgeStruct.edgeEnds(candidateedges,:); % search al neighbouring nodes
        candidatenode=candidatenode(candidatenode ~= mergingcluster(i));
        nodefilter=clusteringa(candidatenode,2) == 0;
        %probability filter
        candidateedgeProb=(edgeBel(2,2,candidateedges));
        candidateedgeProb=reshape(candidateedgeProb,[size(candidateedgeProb,3) 1]);
        probfilter=candidateedgeProb>probt;
        % normal filter
        normals=zeros(length(candidatenode),1);
        for j=1:length(candidatenode)
            normals(j)=abs(dot(Clusters{mergingcluster(i)}.Normal,Clusters{candidatenode(j)}.Normal));
        end
        normalfilter=normals>angulart;%30°
        
        % nodes
        filter=nodefilter.*probfilter.*normalfilter;
        if any(filter);
            nodes=candidatenode(filter==1);
            if any(nodes)
                clustering(nodes,2)=mergingcluster(i);
            end
            candidatelabels=clustering(nodes,2);
            labels=candidatelabels(candidatelabels~=0);
            if isempty(labels)
                clustering(nodes,2)=mergingcluster(i);
            end
            if any(labels)
                    for k=1:length(labels)
                    othernodes=find(clustering(:,2)==labels(k));
                    clustering(othernodes,2)=mergingcluster(i);
                    end
            end
        end
        clustering(mergingcluster(i),2)=mergingcluster(i);
    end    

%% merge Clusters
% clustering=clustering(:,2);
% labels=unique(clustering);
% test=Clusters;
% for i=1:length(labels)
%     % find which clusters have changed
%     indices=find(clustering==labels(i));
%     if length(indices)>1;
%         A=Clusters{indices(1)}.PointCloud;
%         for j=1:length(indices)-1
%             %merge pc's
%             B=Clusters{indices(j+1)}.PointCloud;
%             A= pcmerge(A,B,1);
%             
% 
%         end
%         
%     end
%     Clusters{indices(i)}.PointCloud.Color=
%     Clusters{indices(i)}.Color=
% end


end

