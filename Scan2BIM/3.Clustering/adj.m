adj=zeros(216,216);
for i=1:724

        adj(si(i),sj(i))=1;
        adj(sj(i),si(i))=1;

end

nStates=9;
edgeStruct = UGM_makeEdgeStruct(adj,nStates);
