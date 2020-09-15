%MB_ParameterEstimation load .mat file with edge features and estimate paramters w of EdgePot
% enkel edgepot aangezien nodepot door NN wordt berekend
clear;close;
addpath(genpath('D:\Google Drive\Research\2017-05 point cloud segmentation\Matlab\Mijn code'));
addpath(genpath('D:\Google Drive\Research\2017-05 point cloud segmentation\Matlab\UGM'));

%% Load Training data (edgeStruct,Tn,Te,y)
fprintf('loading Training data...');
tic
load('F:\Data\Lidar_7.N-block\Matlab2\Corner_labelsclustering.mat');
load('F:\Data\Lidar_7.N-block\Matlab2\Corner_NodeFeatures.mat');
load('F:\Data\Lidar_7.N-block\Matlab2\Corner_NodeFeatures2.mat');
load('F:\Data\Lidar_7.N-block\Matlab2\Corner_pc.mat');
load('F:\Data\Lidar_7.N-block\Matlab2\Corner_w.mat');
load('F:\Data\Lidar_7.N-block\Matlab2\Corner_cluster.mat');
load('F:\Data\Lidar_7.N-block\Matlab2\Corner_Clusters.mat');
load('F:\Data\Lidar_7.N-block\Matlab2\Corner_EdgeFeatures.mat');
toc

%% training data
nStates = 2;%
%nInstances=size(NodeFeatures,1);
 [ adj ] = F_AdjancencyMatrix( Clusters);
 edgeStruct = UGM_makeEdgeStruct(adj,nStates);
nEdges = edgeStruct.nEdges;
temp=edgeStruct.edgeEnds;
temp=reshape(temp, size(temp,1)*2,1);
for i=1:length(temp)
    %Clusters{i}.Label=label(i);
    test(i)=Clusters{temp(i)}.Label;
end
labels=int32(reshape(test, nEdges,2));
[nInstances,nNodes] = size(labels);
%labels=repmat(labels,5,1);
%Xnode=repmat(NodeFeatures,5,1,2);
%Xedge=repmat(EdgeFeatures,2,1,3);

%% Create EdgeStruct 1
nStates = 2;%
% crf graph
A=[ 0 1  
    1 0 ];
edgeStruct = UGM_makeEdgeStruct(A,nStates);
nEdges = edgeStruct.nEdges;
maxState = max(nStates);

%% Compute Xnode 
[nNInstances,nNodeFeatures] = size(NodeFeatures);

% Xnode=ones(nInstances,nNodeFeatures,nNodes); % feature variable=nInstances(1) x nNodeFeatures (5) x nNodes
% all nodes have the same trainingsdata

%Xnode=NodeFeatures(temp,:);
NodeFeatures=UGM_standardizeCols(NodeFeatures,1);
t1=NodeFeatures(temp,1);
t2=NodeFeatures(temp,2);
t1=reshape(t1,nInstances,1,nNodes);
t2=reshape(t2,nInstances,1,nNodes);
Xnode=[t1 t2];
%Xnode=repmat(Xnode,1,1,nNodes);
% for n=1:nNodes
%     Xnode(:,:,n)=NodeFeatures;
% end
% add bias feature =1 so see how it reacts to the weights.
% Xnode = [ones(nNInstances,1,nNodes) UGM_standardizeCols(Xnode,1)];
% nNodeFeatures = size(Xnode,2);

%% Compute Xedge
[nEInstances,nEdgeFeatures] = size(EdgeFeatures);
% Xedge=ones(nInstances,nEdgeFeatures,nEdges); 
% % all edges have the same trainingsdata
% 
% Xedge=repmat(Xedge,1,1,nEdges);

Xedge=EdgeFeatures;
Xedge=UGM_standardizeCols(Xedge,1);
% for e=1:nEdges
%     Xedge(:,:,e)=EdgeFeatures;
% end

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
%             if edgeMap(maxState,maxState,:,edgeFeat) == f;
%                 edgeMap(maxState,maxState,:,edgeFeat)=0;
%                 f=f-1;
%             end
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
lambda = ones(size(w));
% lambda(1) = 0; % Don't penalize node bias variable
 regFunObj = @(w)penalizedL2(w,@UGM_CRF_NLL,lambda,Xnode,Xedge,labels,nodeMap,edgeMap,edgeStruct,@UGM_Infer_Chain);
nParams = max([nodeMap(:);edgeMap(:)]);
w = zeros(nParams,1);
% %% training
% %Associative CRFs training
% 
% UB=inf(length(w),1); % No upper bound on elements of w 
% LB=[-inf(max(max(max(nodeMap))),1);zeros(max(max(max(max(edgeMap))))-max(max(max(nodeMap))),1)]; %no lower bounds on node elements of w, no lower than 0 for edge w 
% 
% w = minConf_TMP(regFunObj ,w,LB,UB);
% w = minFunc(@UGM_CRF_NLL,randn(size(w)),[],Xnode,Xedge,labels,nodeMap,edgeMap,edgeStruct,@UGM_Infer_Chain)
% 
% funObj = @(w)UGM_CRF_PseudoNLL(w,Xnode,Xedge,labels,nodeMap,edgeMap,edgeStruct);
% w = minFunc(funObj,w);

w = minFunc(@UGM_CRF_NLL,randn(size(w)),[],Xnode,Xedge,labels,nodeMap,edgeMap,edgeStruct,@UGM_Infer_Chain)

w = minFunc(regFunObj,w);
NLL = UGM_CRF_NLL(w,Xnode,Xedge,labels,nodeMap,edgeMap,edgeStruct,@UGM_Infer_Chain)

%% create nodePot and edgePot (distributions)
%instance = 1;
instance = 1;
[nodePot,edgePot] = UGM_CRF_makePotentials(test(:,3),Xnode,Xedge,nodeMap,edgeMap,edgeStruct,instance);
nodePot(1,:)
edgePot(:,:,1)

%% decoding
optimalDecoding = UGM_Decode_GraphCut(nodePot,edgePot,edgeStruct);
optimalDecoding = UGM_Decode_ICM(nodePot,edgePot,edgeStruct);
%% Export model
% notePot, edgePot,

