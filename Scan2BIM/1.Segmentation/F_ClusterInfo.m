function [ Clusters] = F_ClusterInfo( pc,cluster)
%% Compute basic cluster attributes

% Requirements:
%   Computer Vision Toolbox (pointCloud)
%   Neural Network Toolbox (normr)
%   Parallell computing toolbox

% input
%   pc= pointCloud e.g. Mx3
%   cluster= Mx1 integer array with the cluster number of each point
%     clear;close;addpath('regionGrowing 2D3D');addpath('pointcloudtoolsfunctions');
%     pc=pcread('F:\Data\Lidar_15.Sint Jacobs Kerk Leuven\PTG\test1a_culled.ply');
%     load('F:\Data\Lidar_15.Sint Jacobs Kerk Leuven\PTG\test1a.mat');
%     pcshow(select(pc,find(cluster==1)));
% output: struct with following attributes
%   unary features
%         centre: centerpoint of cluster
%         Normal: mean normal of cluster
%         Height of cluster
%         Width of cluster 
%         surface type
%         Area: area of cluster 
%         boundary length
%   pairwise
%       Neighbours
%       Proximity: distance from border to border
%       length of overlapping edge
%       angle of overlapping edge
%       eval edges: shape similarity of edges of neighbouring clusters
%       Vertical Distance to large hori surface above
%       Vertical Distance to large hori surface below                   
%       Diff in Normals of neighbours

%% variables
    % SurfaceFit
    MaxDistancePlanefit=0.01; % 0.02m
    MaxDistanceCylfit=0.01; %0.01m
    MaxDistanceSpherefit=0.01; %0.01m
    MaxRadius=1; % 1.5m
    inliersratio=0.8; % 80% inlierIndices/outlierIndices

%% initialise
     n=max(cluster);
%% surfaces
for p=1:n
     label=0;
     centre= [0 0 0];
     normals= [0 0 0];
     normalsstd= [0 0 0];
     colors= [0 0 0];
     type=0; %0= complex, 1= plane, 2=cylinder, 3= sphere;
     K=[ 0 0 0 0];
     Kdist= [0 0 0 0];
     KHLarge=0;
     KHLargeAbove=0;
     KHLargeBelow=0;
     KVLarge=0;

     
    % select points
    indices=find(cluster==p);
    points=select(pc,indices);
    %label= mode(y(indices));
    % compute  centre clusters
    centre=mean(points.Location());
    
    % compute cluster normals
    normals=normr(mean(points.Normal()));
    normalsstd(:)=std(points.Normal());
    
    % compute cluster colors
    colors=round(mean(points.Color()));
    
    % compute hull
    %hullIndexes = convhull(double(points.Location(:,1)), double(points.Location(:,2)),double(points.Location(:,3)));

    
    %% compute best fit type for cluster
    % M-estimator SAmple Consensus (MSAC)
    % Torr, P. H. S., and A. Zisserman. "MLESAC: A New Robust Estimator with Application to Estimating Image Geometry." Computer Vision and Image Understanding. 2000.
    planemodel=[];
    [planemodel,inlierIndices,~] = pcfitplane(points,MaxDistancePlanefit); % ax+by+cz+d=0
    if isempty(planemodel)==0
        if (length(inlierIndices)/points.Count)>inliersratio
            type=1;
           
        else  
        % compute best fit cylinder for cluster
        % Torr, P. H. S., and A. Zisserman. "MLESAC: A New Robust Estimator with Application to Estimating Image Geometry." Computer Vision and Image Understanding. 2000.
        % [x1,y1,z1] and [x2,y2,z2] are the centers of each end-cap surface of the cylinder.
        % r is the radius of the cylinder.
        cylindermodel=[];
        [cylindermodel,inlierIndices,outlierIndices] = pcfitcylinder(points,MaxDistanceCylfit);
        if isempty(cylindermodel)==0
            if cylindermodel.Radius <= MaxRadius  &&...
                (length(inlierIndices)/(length(outlierIndices)+length(inlierIndices)))>inliersratio
                type=2;
            
            else
            spheremodel=[];    
            [spheremodel,inlierIndices,outlierIndices] = pcfitsphere(points,MaxDistanceSpherefit);
            if isempty(spheremodel)==0
                if spheremodel.Radius <= MaxRadius  &&...
                    (length(inlierIndices)/(length(outlierIndices)+length(inlierIndices)))>inliersratio
                    type=3;
                end
            end
            
        end
        end
        end
        
    end
    
    %% create structure
    Clusters{p} = struct('ID',n,...
                         'PointCloud',points,...% unary
                         'Centre',centre,...
                         'Normal',normals,...
                         'NormalStd',normalsstd,...
                         'Color',colors,...
                         'Label',label,...
                         'SurfaceType',type,...
                         'K',K,...
                         'Kdist',Kdist,...
                         'KHLarge',KHLarge,...
                         'KHLargeAbove',KHLargeAbove,...
                         'KHLargeBelow',KHLargeBelow,...
                         'KVLarge',KVLarge);    

end

end

