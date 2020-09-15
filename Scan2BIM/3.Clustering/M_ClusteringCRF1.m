%MB_Classification1 Load parameter model (w.mat), point cloud (.ply)process new data and decode the model. The resulting ICM decode paramater holds the most likely configuration for the test data
% requirements
%   graphviz
%   graphViz4Matlab
%   Undirected Graphical Models (UGM) toolbox

clear;close;
addpath(genpath('D:\Google Drive\Research\2020-11 Grasshopper plug ins\Clustering\Matlab\Mijn code'));
addpath(genpath('D:\Google Drive\Research\2020-11 Grasshopper plug ins\Clustering\Matlab\UGM'));

%% Load test data


%% Load model parameters
load('w.mat'); % model parameters

%% Create EdgeStruct
% compute adjancency matrix
k=10;
Threshold=0.5; % max0.5m between edges
nStates=int32(8); % 8 mogelijk classen

[ A,Clusters ] = F_AdjacencyMatrix( Clusters,k,Threshold);
%graphViz4Matlab(A,'-undirected',1,'-layout',GridLayout);
% creer edge struct
edgeStruct = UGM_makeEdgeStruct(A,nStates);
nNodes=edgeStruct.nNodes;
nEdges=edgeStruct.nEdges;
maxState=max(edgeStruct.nStates);

%% Feature Xnode extraction 
% compute unary potentials
nNodeFeatures=4;
[Area, NormalZ, Height, Width, Surfacetype ]=F_FeatureNodeExtraction(Clusters,edgeStruct);
Xnode=[Area, NormalZ, Height, Width, Surfacetype ];

%% Feature Xedge extraction 
% compute pairwise potentials
nEdgeFeatures=4;
[AreaSim, NormalZSim, NormalXYSim, SurfacetypeSim ]=F_FeatureEdgeExtraction(Clusters,edgeStruct,Xnode);
Xedge=[AreaSim, NormalZSim, NormalXYSim, SurfacetypeSim ];

%% Compute NodeMap 
nodeMap = zeros(nNodes,maxState,nNodeFeatures,'int32');
for f = 1:nNodeFeatures
    nodeMap(:,:,f) = w(f);
end

%% Compute EdgeMap
edgeMap = zeros(maxState,maxState,nEdges,nEdgeFeatures,'int32');

for edgeFeat = 1:nEdgeFeatures
    for s1 = 1:maxState
        for s2 = 1:maxState
            f = f+1;
            edgeMap(s1,s2,:,edgeFeat) = w(f);
        end
    end
end

%% Compute NodePot & EdgePot
load('trainedClassifier.mat');
% compute NodePot with pretrained neural network
% X= nxf node feature vector
%[NodePot,Xf,Af] = F_NeuralNetworkFunction(X);

[nodePot,edgePot] = UGM_CRF_makePotentials(w,Xnode,Xedge,nodeMap,edgeMap,edgeStruct);

% 
% %% Creeer NodePot en EdgePot uit de ingeladen parameters w en features
% %   nodePot(n,s) = exp(w(nodeMap(n,s))).
% %   We set edgePot(s1,s2,e)= exp(w(edgeMap(s1,s2,e)).
% 
% % creer Nodepot = prob distr: nNodes x nStates matrix uit initiele machine
% % learning. De kans mag vermenigvuldigt zijn met een scalair (wordt
% % gecompenseerd door 1/Z)
% % neem voorlopig dat de kans op state j uniform verdeelt is (alles 1)
% 
% % de probabiliteit wordt weergegeven door een sigmoid function
% % nodeplot=1./(1+(exp(-x)));
% nodePot=ones(max(edgeStruct.nNodes),max(edgeStruct.nStates));
% nodePot(:,2)=2;
% 
% % creer edgePot= prob distr van iedere edge voor iedere combinatie van
% % states.
% % neem voorlopig dat kans nStates=nStates gelijk is aan ~=
% edgePot=ones(max(edgeStruct.nStates),max(edgeStruct.nStates),edgeStruct.nEdges);

 %% decode model
% %(find most likely configuration)
% %optimalDecoding = UGM_Decode_Exact(nodePot,edgePot,edgeStruct);
% ICMDecoding = UGM_Decode_ICM(nodePot,edgePot,edgeStruct);
% % inference (compute normalizing Z and most likely state of each node)
% 
