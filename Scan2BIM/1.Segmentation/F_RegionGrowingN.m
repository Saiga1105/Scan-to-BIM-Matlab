function [ cluster] = F_RegionGrowingN( pctotal,ThresVal,MaxDist,Minsize,Offset,Tilesize)
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
%   cores:  number of parallell threads

% output 
%   clusters: Mx1 list where grouped points share a common interger

% example
%   MB_regiongrowing2

%% error catching & default values
if ~exist('MaxDist', 'var') || isempty(MaxDist)
    MaxDist = 0.10; %10cm default
end 
if ~exist('Minsize', 'var') || isempty(Minsize)
    Minsize = 1000; % minimum 500 points per cluster
end
if ~exist('ThresVal', 'var') || isempty(ThresVal)
    ThresVal = 30; % 30° max deviation
end
if ~exist('Offset', 'var') || isempty(Offset)
    Offset = 0.02;% 3cm max offset between planes 
end
if ~exist('Tilesize', 'var') || isempty(Tilesize)
    Tilesize = 500000; % default tile size for each core
end

%% compute initial resolution
      Resolution=(sqrt( sum(((pctotal.Location(1:100:end-1,:) - pctotal.Location(2:100:end,:)).^2),2)));
      Resolution=mean(Resolution(Resolution<mean(Resolution)));
      Resolution=round(pi*MaxDist^2*(1/Resolution)^2);
      
%% set up parallell loop
% divide point cloud in p parts
    
    %set Tilesize
    Tile=ceil(length(pctotal.Location)/Tilesize);
    Tilesize=bsxfun(@minus,Tilesize,zeros(1,Tile));
    Tilesize(Tile)=length(pctotal.Location)-Tilesize(1)*(Tile-1);
    part=cell(1,Tile);
    globalcolor=cell(1,Tile);

tic
fprintf('Connecting to all cores...');
gcp;
toc
tic

parfor p=1:Tile
  
% subselect point cloud

    % set variables and preallocate arrays
    idx=single(Tilesize(1)*(p-1)+(1:Tilesize(p)));
    pc=select(pctotal,idx);
    clusters=1;
    indices=zeros(Tilesize(p),1); % binary cluster array
    localcolor=zeros(Tilesize(p),3); % color array
    part{p}=zeros(Tilesize(p),1);
        
    % compute normals if not there
    if isempty(pctotal.Normal)
        fprintf('Computing Normals...');
        tic
            pc.Normal(:,:)=pcnormals(pc);
        toc
    end

%% Seeding
    % compute uniform seeds, preallocate seeding vector
     seeds=10:50:(length(pc.Location)-10);
    % validate seeds => takes too long
%       tic
%       test=zeros(length(seeds),5);
%       for i=1:5
%             t1=(sqrt( sum(((pc1.Location(seeds,:) - pc1.Location(seeds-i,:)).^2),2)));
%             t2=(sqrt( sum(((pc1.Location(seeds,:) - pc1.Location(seeds+i,:)).^2),2)));
%             test(:,i)=abs(t1-t2);
%       end
%       seeds=seeds((std(t,0,2)<0.003)');
%       toc
%       K=round(pi*MaxDist^2*(1/Resolution)^2);

%% loop region grow
    
while any(seeds) &&... % seeding loop
      length(find(indices==0)) >= 0.1*length(pc.Location); % & >10% unlabelled points
      
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

    while any(queue)    % region growing loop
    
    % the first queue position determines the new values
    point = queue(1);
    
%     % update normal 1st time
%     meannormal=mean([pc.Normal(queue(1),:) ;meannormal],1);
%     
    % search neirest neighbours
    [j ,~] = findNeighborsInRadius(pc,pc.Location(point,:),D);

    if length(j)<0.7*Resolution
        D=D*1/(length(j)/Resolution);
        %K(p)=K(p)*1/(length(j)/K(p));
    elseif length(j)>1.3*Resolution
        D=D*1/(length(j)/Resolution);
        %K(p)=K(p)*1/(length(j)/K(p));
    end
    
    % filter any queue from j
    %[~,index]=intersect(queue,single(j));
    [~,index]=intersect(queue,j);
    queue(index)=[];  
    
    % filter previously selected points
%     j=[single(j) dists];
%     j=j(indices(j(:,1))==0,:);
%     dists=j(:,2);
%     j=j(:,1);
    j=j(indices(j)==0);
    % filter seeds i
    if any(j)
        i=j;
        % keep only outer ring
        %i=i(dists>0.7*D); 
        i=i(round(0.8*length(i)):end);
        % check dot product normals <threshold
        normalvectorpoint=bsxfun(@minus,meannormal,zeros(length(i),3));
        i=i(abs(dot(normalvectorpoint',pc.Normal(i,:)')')>=cos(ThresVal/180*pi));
        % check offset< treshhold based on dot product normal point and centroidvector 
        normalvectorpoint=bsxfun(@minus,meannormal,zeros(length(i),3));
        centroidvector=bsxfun(@minus,pc.Location(i,:),pc.Location(point,:));
        i=i(abs(dot(normalvectorpoint',centroidvector')')<=Offset); % driehoeksstelling
        
    % determine j points that should be added to selection
    % punten in zowel i als j worden dubbel getest. dit moet eruit.
        normalvectorpoint=bsxfun(@minus,meannormal,zeros(length(j),3));
        centroidvector=bsxfun(@minus,pc.Location(j,:),pc.Location(point,:));
        j=j(abs(dot(centroidvector',normalvectorpoint')')<=1.3*Offset); % driehoeksstelling
        normalvectorpoint=bsxfun(@minus,meannormal,zeros(length(j),3));
        j=j(abs(dot(pc.Normal(j,:)',normalvectorpoint')')>=0.7*cos(ThresVal/180*pi));

        % update meannormal based on i (those are closest)
        normals = [normals;pc.Normal(i,:)];
        meannormal=mean(normals,1);
        
        % add j to indices
        indices(j)=clusters;
        localcolor(j,:)=bsxfun(@plus,zeros(length(j),1),color);
        
        % add queue
        queue=[queue; i];       
    end

    end
       
%% validate cluster
    if any(indices(indices==clusters))
        if length(indices(indices==clusters))<Minsize
            % reset invalid clusters
            indices(indices==clusters)=0;
            localcolor((indices==clusters),:)=zeros(length(find(indices==clusters)),3);
        else
            clusters=clusters+1;
            color=randi([1 255],1,3);
        end
    end
  %    end
      
%% update seeds
    % cull seeds
    seeds=seeds(indices(seeds)==0);
    
    % sort seeds
    sorting=dot(pc.Normal(seeds,:)',bsxfun(@minus,pc.Normal(previous,:),zeros(length(seeds),3))')';
    [values, order] = sort(abs(sorting),'ascend');
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
    
%% finalise
    % write clusters 
    c(p)=clusters-1;
    part{p}=indices;
    globalcolor{p}=localcolor;
    % post end results
    disp(['Clustering Results: ',num2str(round(100*((length(pc.Location)-length(find(indices==0)))/length(pc.Location)))),'% of points was clustered in ',num2str((clusters-1)),' clusters.']);

 end 

toc

%% colorizing point cloud & constructing cluster
cluster=zeros(length(pctotal.Location),1);
for p=1:Tile
    idx=single(Tilesize(1)*(p-1)+(1:Tilesize(p)));
    %colorizing
    pctotal.Color(idx,:)=globalcolor{p};
    test=(part{p}==0);
    test=test*(-sum(c(1:p-1)));
    part{p}=part{p}+test;
    cluster(idx)=part{p}+sum(c(1:p-1));
    pctotal.Color(find(cluster==0),:)=zeros(length(find(cluster==0)),3);
end
figure
pcshow(pctotal);

%% construct cluster list
% cluster=zeros(length(pctotal.Location),1);
% for p=1:Tile
%     idx=single(Tilesize*(p-1)+1:Tilesize*(p));
%     if p==Tile
%         idx=single(Tilesize*(p-1)+(1:LastTilesize));
%     end
%     test=(part{p}==0);
%     test=test*(-sum(c(1:p-1)));
%     part{p}=part{p}+test;
%     cluster(idx)=part{p}+sum(c(1:p-1));
% end

end