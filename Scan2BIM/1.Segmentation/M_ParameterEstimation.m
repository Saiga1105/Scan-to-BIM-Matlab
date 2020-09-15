%MB_ParameterEstimation load .mat file with edge features and estimate paramters w of EdgePot
% enkel edgepot aangezien nodepot door NN wordt berekend
clear;close;
addpath(genpath('D:\Google Drive\Research\2017-05 point cloud segmentation\Matlab\Mijn code'));
addpath(genpath('D:\Google Drive\Research\2017-05 point cloud segmentation\Matlab\UGM'));

%% Load Training data (edgeStruct,Tn,Te,y)
fprintf('loading Training data...');
tic
load('D:\Google Drive\Research\2017-05 point cloud segmentation\Matlab\Mijn code\Data\npc1.mat');
toc

%% Create labels
    y=Label;
    for i=1:length(Label)
        y(i,2)=Clusters{Clusters{i}.K(1)}.label;
        y(i,3)=Clusters{Clusters{i}.K(2)}.label;
        y(i,4)=Clusters{Clusters{i}.K(3)}.label;
        y(i,5)=Clusters{Clusters{i}.K(4)}.label;
    end
    y=int32(y+1);

    [nInstances,nNodes] = size(y);

%% Create EdgeStruct
nStates = max(y);%1= clutter, 2= hori, 3= verti
% crf graph
A=[ 0 1 1 1 1
    1 0 0 0 0
    1 0 0 0 0
    1 0 0 0 0
    1 0 0 0 0];
edgeStruct = UGM_makeEdgeStruct(A,nStates);
nEdges = edgeStruct.nEdges;
maxState = max(nStates);

%% Compute Xnode 
[nNInstances,nNodeFeatures] = size(NodeFeatures);

Xnode=zeros(nNInstances,nNodeFeatures,nNodes); % feature variable=nInstances(1) x nNodeFeatures (5) x nNodes
% all nodes have the same trainingsdata
for n=1:nNodes
    Xnode(:,:,n)=NodeFeatures;
end
% add bias feature =1 so see how it reacts to the weights.
Xnode = [ones(nNInstances,1,nNodes) UGM_standardizeCols(Xnode,1)];
nNodeFeatures = size(Xnode,2);

%% Compute Xedge
[nEInstances,nEdgeFeatures] = size(EdgeFeatures);
Xedge=zeros(nEInstances,nEdgeFeatures,nEdges); 
% all edges have the same trainingsdata
for e=1:nEdges
    Xedge(:,:,e)=EdgeFeatures;
end

% sharedFeatures = ones(nNodeFeatures,1);
% Xedge = UGM_makeEdgeFeatures(Xnode,edgeStruct.edgeEnds,sharedFeatures);
% nEdgeFeatures = size(Xedge,2);

%% Create nodeMap %and EdgeMap
% Map parameters over NodePot and EdgePot
% %nodePot(n,s) with n: nodes and s: states
% edgePot(s,s,e) with e: nEdges

% set parameters identifiers for the different states. each feature gets a
% parameter
nodeMap = zeros(nNodes,maxState,nNodeFeatures,'int32');
for f=1:nNodeFeatures
    for state=1:maxState
        nodeMap(:,state,f)=maxState*(f-1)+state;
    end
    
end

% 9= clutter-clutter, 10=floor-floor, 11=ceilings-ceiling, 12=roof-roof, 13=wall, 13=door, 14=window, 15=beam
% give each combo a parameter for each feature
edgeMap = zeros(maxState,maxState,nEdges,nEdgeFeatures,'int32');
f=maxState*f;
for edgeFeat = 1:nEdgeFeatures
    for s1 = 1:maxState
        %s=s+1;
        for s2 = 1:maxState
                f = f+1;
                edgeMap(s1,s2,:,edgeFeat) = f;
            if edgeMap(maxState,maxState,:,edgeFeat) == f;
                edgeMap(maxState,maxState,:,edgeFeat)=0;
                f=f-1;
            end
        end
    end
end

%% create parameter vector
% Initialize weights
nParams = max([nodeMap(:);edgeMap(:)]);
w = zeros(nParams,1);

%% Create regularization parameter
% Set up regularization parameters
lambda = 10*ones(size(w));
lambda(1) = 0; % Don't penalize node bias variable
regFunObj = @(w)penalizedL2(w,@UGM_CRF_NLL,lambda,Xnode,Xedge,y,nodeMap,edgeMap,edgeStruct,@UGM_Infer_Chain);

%% training
%Associative CRFs training

UB=inf(length(w),1); % No upper bound on elements of w 
LB=[-inf(max(max(max(nodeMap))),1);zeros(max(max(max(max(edgeMap))))-max(max(max(nodeMap))),1)]; %no lower bounds on node elements of w, no lower than 0 for edge w 

w = minConf_TMP(regFunObj ,w,LB,UB);


%% create nodePot and edgePot (distributions)
%instance = 1;
[nodePot,edgePot] = UGM_CRF_makePotentials(w,Xnode,Xedge,nodeMap,edgeMap,edgeStruct);
NodePot=nodePot(1,:)
EdgePot=mean(edgePot,3)

%% decoding
optimalDecoding = UGM_Decode_GraphCut(nodePot,edgePot,edgeStruct);
optimalDecoding = UGM_Decode_ICM(nodePot,edgePot,edgeStruct);
%% Export model
% notePot, edgePot,

