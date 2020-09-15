%MB_ParameterEstimation load .mat file with edge features and estimate paramters w of EdgePot
% enkel edgepot aangezien nodepot door NN wordt berekend
clear;close;
addpath(genpath('D:\Google Drive\Research\2020-11 Grasshopper plug ins\Clustering\Matlab\Mijn code'));
addpath(genpath('D:\Google Drive\Research\2020-11 Grasshopper plug ins\Clustering\Matlab\UGM'));

%% Load Training data (edgeStruct,Tn,Te,y)
fprintf('loading Training data...');
tic
% load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\test3_binary\y.mat');
% load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\Xnode.mat');
% load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\adj.mat');
% load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\edgeStruct.mat');
% load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\edgeMap.mat');
% load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\edgeStruct.mat');
% load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\nodeMap.mat');
% load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\Xedge_bias.mat');

load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\test3_binary\y3.mat');
load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\Xnode_BB.mat');
load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\Xnode_edge.mat');
load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\test3_binary\Xnode2.mat');
load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\test3_binary\adj3.mat');
load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\test4_binary\edgeStruct4.mat');
load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\test4_binary\edgeMap4.mat');
load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\test3_binary\nodeMap3.mat');
load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\test3_binary\Xedge_bias2.mat');
load('K:\Projects\ScanToBIM\Data\2016-05 Dubo Campus\Matlab_Clustering\test4_binary\trainedMode.mat');


nStates = edgeStruct.nStates;%
nEdges = edgeStruct.nEdges;
nNodes = edgeStruct.nNodes;
maxState = max(nStates);
%w = zeros(nParams,1);
nodePot=Xnode_BB+Xnode_edge;
w=[0;0;1;0];


tied = 1;
temp = GM_standardizeCols(Xnode,tied);

[nInstances,nNodeFeatures] = size(Xnode);
w=[2;2;0.66;0.33]; % associativity
y= int32(y);

toc

%% xNode, nodeMap, nodePot
temp=zeros(216,9,18);
for i=1:9
    temp(:,i,i)=1;
    temp(:,i,i+9)=2;

end

nodeMap=int32(temp);

%% xEdge, edgeMap, edgePot

temp2=zeros(1,18,724);
for i=1:724
    temp2(1,:,i)=(xNodeCRF(1,:,edgeStruct.edgeEnds(i,1))).* (xNodeCRF(1,:,edgeStruct.edgeEnds(i,2)));
end
xEdgeCRF=temp2;

temp3=zeros(9,9,724,18);
for i= 1:9
    for j=1:9
       if i==j
            temp3(i,i,:,i)=3;
            temp3(i,i,:,i+9)=3;
       end
    end
end

edgeMap=int32(temp3);
%% xNode training
% yfit2=zeros(216,9);
% for i =1:9
%     t=[xNode_BB(:,i) xNode_edge(:,i)];
%     yfit2(:,i) = trainedModel.predictFcn(t);
% end
% 
% xNode=yfit;
% 
% %training
% funObj = @(w)UGM_CRF_NLL(w,Xnode,Xedge,y,nodeMap,edgeMap,edgeStruct,@UGM_Infer_LBP);
% UB = [inf;inf;inf;inf]; % No upper bound on parameters
% LB = [-inf;-inf;0;0]; % No lower bound on node parameters, edge parameters must be non-negative 
% w = minConf_TMP(funObj,w,LB,UB);

%% nodePot, edgePot
nParams = max([nodeMap(:);edgeMap(:)]);
w=ones(nParams,1);
w=[8;5;1];
[nodePot,edgePot] = UGM_CRF_makePotentials(w,xNodeCRF,xEdgeCRF,nodeMap,edgeMap,edgeStruct);

%decoding
%[nodePot,edgePot] = UGM_CRF_makePotentials(w,Xnode,Xedge,nodeMap,edgeMap,edgeStruct);
nRestarts = 1000;
yDecode =  UGM_Decode_ICM(nodePot,edgePot,edgeStruct);

%% validate
a=0;
for i=1:216
    if y(i)==yDecode(i);
        a=a+1;
    end
end

%% training data
nStates = 10;%
%nInstances=size(NodeFeatures,1);

edgeStruct = UGM_makeEdgeStruct(adj,nStates);
nEdges = edgeStruct.nEdges;
% temp=edgeStruct.edgeEnds;
% temp=reshape(temp, size(temp,1)*2,1);
% for i=1:length(temp)
%     Clusters{i}.Label=label(i);
%     test(i)=Clusters{temp(i)}.Label;
% end
% labels=int32(reshape(test, nEdges,2));
% [nInstances,nNodes] = size(labels);
%labels=repmat(labels,5,1);
%Xnode=repmat(NodeFeatures,5,1,2);
%Xedge=repmat(EdgeFeatures,2,1,3);

%% Create EdgeStruct 1

% crf graph
% A=[ 0 1  
%     1 0 ];
% edgeStruct = UGM_makeEdgeStruct(A,nStates);
nEdges = edgeStruct.nEdges;
maxState = max(nStates);

%% Compute Xnode 
[nNInstances,nNodeFeatures] = size(Xnode);

% Xnode=ones(nInstances,nNodeFeatures,nNodes); % feature variable=nInstances(1) x nNodeFeatures (5) x nNodes
% all nodes have the same trainingsdata

%Xnode=NodeFeatures(temp,:);
% NodeFeatures=UGM_standardizeCols(NodeFeatures,1);
% t1=NodeFeatures(temp,1);
% t2=NodeFeatures(temp,2);
% t1=reshape(t1,nInstances,1,nNodes);
% t2=reshape(t2,nInstances,1,nNodes);
% Xnode=[t1 t2];
%Xnode=repmat(Xnode,1,1,nNodes);
% for n=1:nNodes
%     Xnode(:,:,n)=NodeFeatures;
% end
% add bias feature =1 so see how it reacts to the weights.
% Xnode = [ones(nNInstances,1,nNodes) UGM_standardizeCols(Xnode,1)];
% nNodeFeatures = size(Xnode,2);

%% Compute Xedge
[nEInstances,nEdgeFeatures] = size(Xedge);
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
nNodes=edgeStruct.nNodes;

nodeMap = zeros(nNodes,maxState,nNodeFeatures,'int32');
for f=11:20
    for state=1:maxState
        nodeMap(:,state,f)=1;
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

