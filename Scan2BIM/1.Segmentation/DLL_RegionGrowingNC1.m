function cluster = G_RegionGrowingNC1( points,ThresValN,ThresValC,MaxDist,Minsize,Offset )
%% Explenation
%   this function takes as input any point cloud and returns point
%   groups of similar properties. The main goal is point cloud segmentation

% Requirements:
%   Computer Vision Toolbox (pointCloud)
%   Neural Network Toolbox (normr)
%   Parallell computing toolbox

% input
%   pc= pointCloud e.g. Mx3
%   maxDist= max distance between neighbouring pixels
%   thresVal: max normal deviation
%   Minszise: minimum size of the cluster
%   Offset: max offset of computed plane
%   Tilesize: the data is seperated in N tiles that are dynamically assigned to each core.

% output 
%   clusters: Mx1 list where grouped points share a common interger

% example
%   MB_regiongrowing2
%   MB_regiongrowing3
%   MB_mesh_segmentation1

%% error catching & default values
if ~exist('MaxDist', 'var') || isempty(MaxDist)
    MaxDist = 0.10; %10cm default
end 
if ~exist('Minsize', 'var') || isempty(Minsize)
    Minsize = 1000; % minimum 500 points per cluster
end
if ~exist('ThresValN', 'var') || isempty(ThresValN)
    ThresValN = 50; % 30° max deviation
end
if ~exist('ThresValC', 'var') || isempty(ThresValC)
    ThresValC = 30; % 30° max deviation
end
if ~exist('Offset', 'var') || isempty(Offset)
    Offset = 0.02;% 3cm max offset between planes 
end


%% construct point cloud
%% construct point cloud
pc=pointCloud(points);
color_criteria=1;
if ~exist('normals', 'var') || isempty(normals)
    normals = pcnormals(pc); %10cm default
    if ~exist('colors', 'var') || isempty(colors)
        color_criteria=0;
        pc=pointCloud(points,'Normal',normals);
    else
        pc=pointCloud(points,'Normal',normals,'Color',colors);
    end 
else
    if ~exist('colors', 'var') || isempty(colors)
        color_criteria=0;
        pc=pointCloud(points,'Normal',normals);
    else
        pc=pointCloud(points,'Normal',normals,'Color',colors);
    end
end

%% compute initial resolution
      Resolution=(sqrt( sum(((pc.Location(1:100:end-1,:) - pc.Location(2:100:end,:)).^2),2)));
      Resolution=mean(Resolution(Resolution<mean(Resolution)));
      Resolution=round(pi*MaxDist^2*(1/Resolution)^2);
      
% set variables and preallocate arrays
    clusters=1;
    cluster=zeros(pc.Count,1); % binary cluster array
 
%% Seeding
    % compute uniform seeds, preallocate seeding vector
     seeds=10:50:(length(pc.Location)-10);
     
%% loop region grow
while any(seeds) &&... % seeding loop
      length(find(cluster==0)) >= 0.1*length(pc.Location); % & >10% unlabelled points
      
    % update seed
    queue=seeds(1);
    seeds(1)=[];
    previous=queue;
    % initialise dist and color
    D=MaxDist;
    color=randi([1 255],1,3);
  
     % reset mean normal
     meannormal=pc.Normal(queue,:);
     normals=meannormal;
     
     % reset mean color
     if color_criteria==1
        meancolor=pc.Color(queue,:);
        colors=meancolor;
     end
     
    while any(queue)    % region growing loop
   
    % the first queue position determines the new values
    point = queue(1);
    

    % search neirest neighbours
    [j ,~] = findNeighborsInRadius(pc,pc.Location(point,:),D);
   
%     if length(j)<0.7*Resolution
%         D=D*1/(length(j)/Resolution);
%         %K(p)=K(p)*1/(length(j)/K(p));
%     elseif length(j)>1.3*Resolution
%         D=D*1/(length(j)/Resolution);
%         %K(p)=K(p)*1/(length(j)/K(p));
%     end
    
    % filter any queue from j
    [~,index]=intersect(queue,j);
    queue(index)=[];  
    
    % filter previously selected points
    j=j(cluster(j)==0);

    % filter seeds i
    if any(j)
        i=j;
        % keep only outer ring
        %i=i(dists>0.7*D); 
        i=i(round(0.8*length(i)):end);
        % check dot product normals <threshold
        normalvectorpoint=bsxfun(@minus,meannormal,zeros(length(i),3));
        i=i(abs(dot(normalvectorpoint',pc.Normal(i,:)')')>=cos(ThresValN/180*pi));
        % check offset< treshhold based on dot product normal point and centroidvector 
        normalvectorpoint=bsxfun(@minus,meannormal,zeros(length(i),3));
        centroidvector=bsxfun(@minus,pc.Location(i,:),pc.Location(point,:));
        i=i(abs(dot(normalvectorpoint',centroidvector')')<=Offset); % driehoeksstelling
        % color should approximatly match
        if color_criteria==1;
            i=i(mean(bsxfun(@minus,meancolor,pc.Color(i,:)),2)<=ThresValC);
        end

        
    % determine j points that should be added to selection
    % punten in zowel i als j worden dubbel getest. dit moet eruit.
        normalvectorpoint=bsxfun(@minus,meannormal,zeros(length(j),3));
        centroidvector=bsxfun(@minus,pc.Location(j,:),pc.Location(point,:));
        j=j(abs(dot(centroidvector',normalvectorpoint')')<=1.3*Offset); % driehoeksstelling
        normalvectorpoint=bsxfun(@minus,meannormal,zeros(length(j),3));
        j=j(abs(dot(pc.Normal(j,:)',normalvectorpoint')')>=0.7*cos(ThresValN/180*pi));
        %j=j(mean(bsxfun(@minus,meancolor,pc.Color(j,:)),2)<=1.3*ThresValC);
        
        % update meannormal based on i (those are closest)
        normals = [normals;pc.Normal(i,:)];
        meannormal=normr(mean(normals,1));
        
        % update meancolor based on i (those are closest)
        if color_criteria==1;
            colors = [colors;pc.Color(i,:)];
            meancolor=uint8(round(mean(colors,1)));
        end
        
        % add j to indices
        cluster(j)=clusters;
        
        % add queue
        queue=[queue; i];       
    end

    end
       
%% validate cluster
    if any(indices(indices==clusters))
        if length(indices(indices==clusters))<Minsize
            % reset invalid clusters
            indices(indices==clusters)=0;
        else
            clusters=clusters+1;
            color=randi([1 255],1,3);
        end
    end
    clusters=clusters+1;
  %    end
      
%% update seeds
    % cull seeds
    seeds=seeds(cluster(seeds)==0);
    
    % sort seeds
    sorting=dot(pc.Normal(seeds,:)',bsxfun(@minus,pc.Normal(previous,:),zeros(length(seeds),3))')';
    [~, order] = sort(abs(sorting),'ascend');
    seeds = seeds(order);   
 end
%% classify unclustered points
%     
%  unclusteredpoints=find(indices==0)';
%  clusteredpoints=find(indices~=0);
% pctemp=select(pc,clusteredpoints);
%     for i=1:length(unclusteredpoints)
%         [j,dists] = findNearestNeighbors(pctemp,pc.Location(unclusteredpoints(i),:),1);
%         if dists <0.03
%             indices(unclusteredpoints(i))=indices(j);
%             localcolor(unclusteredpoints(i),:)=localcolor(j,:);
%         end
%      end
    
cluster=cluster';  
 end 


