function Decoding = G_CRF_wall_decoding( nStates,fn1,fn2,fe1,fe2,si,sj,w)
%#function UGM_Decode_ICM UGM_makeEdgeStruct

% Decode CRF graph (Nodes, Edges) based on set of node/edge features and a
% set of pretrained weights
%   fn1= first node feature (wall thickness to seeds), [node*seeds,1]
%   fn2= second node feature (distance to seeds), [node*seeds,1]
%   fe1= first edge feature (wall thickness to nieghbours), [edges*seeds,1]
%   fe2= second edge feature (distance to neighbours), [edges*seeds,1]
%   si= index of node at start of edge [edges,1]
%   sj= index of node at end of edge [edges,1]
%   parameters= parameters [w,1]

% Copyright Maarten Bassier, PhD Candidate of the KU Leuven University
% Belgium. contact: maarten.bassier@kuleuven.be
    
% above function pragma's are important for CS code. It invokes the
% functions so they can be used.

% Data input = rows of doubles equal to the amount of
% observations

% Data output = row of doubles 0-n depending on the number of seeds nStates
%% Overview procedure
%   0. create adjancency matrix (edgestruct)
%   1. restructure Xnode
%   2. restructure Xedge
%   3. Create nodeMap & edgeMap
%   4. Create parameter vector
%   5. create nodePot and edgePot
%   6. decode graph

%% 0.Create EdgeStruct 1
% set parameters
nStates = int32(nStates);%
nNodes= int32(length(fn1)/nStates);
nEdges= length(fe1);
maxState = max(nStates);
si=int32(si);
sj=int32(sj);
useMex=1;

% adjancecy matrix
A=zeros(nNodes,nNodes);
for i=1:nEdges
    A(si(i),sj(i))=1;
    A(sj(i),si(i))=1;
end
edgeStruct = UGM_makeEdgeStruct(A,nStates);

%% 1.Compute Xnode 
temp= [ reshape(fn1,nStates,nNodes)' reshape(fn2,nStates,nNodes)'];

for i=1:nNodes
    NodeFeatures(1,:,i)=temp(i,:);
end

% when solving the graph, there is only 1 instance of each node
% with training, there are many
[nNInstances,nNodeFeatures,nNodes] = size(NodeFeatures);
Xnode=NodeFeatures;
%Xnode=UGM_standardizeCols(NodeFeatures,1); % values should be already
%normalized in rhino


%% 2.Compute Xedge
temp= [ fe1 fe2];

for i=1:nEdges
    EdgeFeatures(1,:,i)=temp(i,:);
end

% when solving the graph, there is only 1 instance of each edge
% with training, there are many
[nEInstances,nEdgeFeatures,nEdges] = size(EdgeFeatures);
Xedge= EdgeFeatures;
%Xedge=UGM_standardizeCols(EdgeFeatures,1);% values should be already
%normalized in rhino

%% 3. Create nodeMap %and EdgeMap
% Map parameters over NodePot and EdgePot
% %nodePot(n,s) with n: nodes and s: states
% edgePot(s,s,e) with e: nEdges

% set parameters identifiers for the different states. each feature gets a
% parameter
% current hypothesis is that all thickness and distance features get the
% same parameter
nodeMap = zeros(nNodes,maxState,nNodeFeatures,'int32');
a=1;
for f=1:(nNodeFeatures/2)
        nodeMap(:,:,f)=a;
end
a=a+1;
for f=(nNodeFeatures/2):nNodeFeatures
        nodeMap(:,:,f)=a;
end

% give each combo a parameter for each feature
edgeMap = zeros(maxState,maxState,nEdges,nEdgeFeatures,'int32');
for edgeFeat = 1:nEdgeFeatures
    for s1 = 1:maxState
        edgeMap(s1,s1,:,edgeFeat)=edgeFeat+a;
        
    end
end

%% 6. create nodePot and edgePot (distributions)
% nodePot=[nNodes, nStates]
% edgePot=[nStates,nStates,nEdges]
%[nodePot,edgePot] = UGM_CRF_makePotentials(w,Xnode,Xedge,nodeMap,edgeMap,edgeStruct);
% watch out! only certain features may contribute to certain potentials!
% => wall thickness with respect to seed1 should only apply to state1!
    i=1;   
    nNodes = size(nodeMap,1);
    maxState = size(nodeMap,2);
    nNodeFeatures = size(Xnode,2);
    nEdgeFeatures = size(Xedge,2);
    nEdges = edgeStruct.nEdges;
    edgeEnds = edgeStruct.edgeEnds;
    
nodePot = zeros(nNodes,maxState);
    for n = 1:nNodes
        for s = 1:nStates
            for f = s:nStates:nNodeFeatures
                if nodeMap(n,s,f) > 0
                    nodePot(n,s) = nodePot(n,s) + w(nodeMap(n,s,f))*Xnode(i,f,n);
                end
            end
            nodePot(n,s) = exp(nodePot(n,s));
        end
    end
edgePot = zeros(maxState,maxState,nEdges);
        for e = 1:nEdges
            n1 = edgeEnds(e,1);
            n2 = edgeEnds(e,2);
            for s1 = 1:nStates
                for s2 = 1:nStates
                    for f = 1:nEdgeFeatures
                        if edgeMap(s1,s2,e,f) > 0
                            edgePot(s1,s2,e) = edgePot(s1,s2,e) + w(edgeMap(s1,s2,e,f))*Xedge(i,f,e);
                        end
                    end
                    edgePot(s1,s2,e) = exp(edgePot(s1,s2,e));
                end
            end
        end

%% decoding
Decoding = UGM_Decode_ICM(nodePot,edgePot,edgeStruct);


