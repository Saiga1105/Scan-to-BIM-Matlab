
adj=

edgestruct 

%% training data
nStates = 10;% aantal seeds

% [ adj ] = F_AdjancencyMatrix( Clusters);
adj= zeros(215,215);

for i=1:665
    adj(si(i),sj(i))=1;
    adj(sj(i),si(i))=1;
end

 edgeStruct = UGM_makeEdgeStruct(adj,nStates);
 
  for s1 = 1:maxState
        %s=s+1;
        for s2 = 1:maxState
            if s1==s2
                edgeMap(s1,s2,:) = 3;
%             if edgeMap(maxState,maxState,:,edgeFeat) == f;
%                 edgeMap(maxState,maxState,:,edgeFeat)=0;
%                 f=f-1;
             end
        end
    end