function [ J,Bn] = F_RegionGrowingImage( pc,I,ThresVal,MaxDist,Resolution,Minsize )
%% Explenation
%   this function takes as input a rasterized point cloud and returns pixel
%   groups of similar properties. The main goal is point cloud segmentation

% Requirements:
%   Computer Vision Toolbox

% input
%   pc=rasterized pointCloud e.g. pc.Normal => MxNx3 e.g. 512x460x3
%   I: normal image
%   MaxDist= max distance between neighbouring pixels
%   ThresVal: max normal deviation
%   Minsize: minimum cluster size
%   Resolution: kernel size for region growing

% output
%   J= segmented image 
%   Bn= boundary points of clusters

% example
%   MB_regiongrowing1

%% initialise

% create image Normals 
%I=rgb2gray(mat2gray(pc.Normal));
%I=imgaussfilt(I,2); % gaussian smooth

% create location image
points=pc.Location;

% define rows and collumns
M=size(pc.Location,1);
N=size(pc.Location,2);

%% error catching & default values
% if nargin ~= 1
%     error('Wrong number of input arguments!')
% end
if ~exist('MaxDist', 'var') || isempty(MaxDist)
    MaxDist = 0.05;
end 
if ~exist('Minsize', 'var') || isempty(Minsize)
    Minsize = 500;
end
if ~exist('ThresVal', 'var') || isempty(ThresVal)
    ThresVal = double((max(I(:)) - min(I(:)))) * 0.05; % 5% deviation
end
if ~exist('Resolution', 'var') || isempty(Resolution)
    Resolution = 10;
end

%% Seeding
% compute uniform seeds, preallocate seeding vector
a=1;seeds=zeros(round((M*N/Resolution^2),0),2);
for u=4:Resolution:(M-4);
    for v=4:Resolution:(N-4);
        seeds(a,:)=[u v];
        a=a+1;
    end
end
seeds=seeds(find(seeds(:,1).*seeds(:,2)),:); % clear zero elements
fprintf('Number of potential seeds:..%d..\n',size(seeds,1));

%% loop region grow
% set variables and preallocate image array
A=1;rest= M*N; J = zeros(size(I,1),size(I,2));

% run loop
while size(seeds, 1) &&... %run while there are still seeds   
        rest>=0.1*size(I(:),1); % & >10% unlabelled points
    
    queue=seeds(1,:);
    prev=queue;
    seeds(1,:) = [];
    regVal = I(queue(1),queue(2));

    % grow region
    while size(queue, 1)     
        % the first queue position determines the new values
        xv = queue(1,1);
        yv = queue(1,2);
        a=1; % reset counter for mean update
        queue(1,:) = []; % and delete the first queue position

        % check the neighbors for the current position
        for i = -2:1:2
            for j = -2:1:2
               if   xv+i >= 2  &&  xv+i <= size(I,1)-1 &&... % within the x-bounds?
                    yv+j >= 2  &&  yv+j <= size(I,2)-1 &&... % within the y-bounds?
                    ~J(xv+i, yv+j) &&...          % draw subset of J
                    sqrt( (points(xv+i,yv+j,1)-points(xv,yv,1)^2) +...
                          (points(xv+i,yv+j,2)-points(xv,yv,2)^2) +...
                          (points(xv+i,yv+j,3)-points(xv,yv,3)^2)) < MaxDist &&...   % within distance?
                    I(xv+i, yv+j) <= (regVal + ThresVal) &&...% within range
                    I(xv+i, yv+j) >= (regVal - ThresVal) &&...;  
                    J(xv+i, yv+j)==0; % only eval pixels that are not in the mask
                       % mark pixels (-1:1) if conditions are met
                        J(xv+i+(-1:1), yv+j+(-1:1)) = A; 
                       % add pixel to the computation queue (recursive)
                       queue(end+1,:) = [xv+i, yv+j];
               end
            end
        end
        % increment counter
        a=a+1;
        % check if shape is still ok
        if size(J,1)~=M || size(J,2)~=N
            J=J(1:M,1:N);
        end
        % update mean normal every 100th pixel
        if rem(a,100) ==0
           regVal = mean(mean(I(J == A))); % --> slow!
        end 
    end

    % reject small clusters
    if length(find(J==A))>=Minsize 
        %optimize region
        K=J;
        K(K~=A)=0;
        K=imfill(K);                     % fill gaps in mask
        B=bwboundaries(K,'noholes'); % compute boundary
        Bn{A}=B{1};                      % add boundary to cell array

        fprintf('Cluster found:..%d..\n',length(find(K==A)))

        % delete seeds in ROI
        temp=impixel(J, seeds(:,1),seeds(:,2));
        temp=temp(:,1);
        seeds=seeds(find(temp~=A),:);

        % update rest points
        rest=length(find(~J));

        %update A cluster count
        A=A+1;

    elseif  isempty(find(J==A))

    else 
            J(find(J==A))=0; % reset pixels if cluster is <Minsize
    end

    % sort seeds so 1st is furthest away from previous seed
    sorting=[abs(seeds(:,1)-prev(1)) abs(seeds(:,2)-prev(2))];
    [values, order] = sort((sum(sorting,2)),'descend');
    seeds = seeds(order,:);
end

% post end results
disp(['Clustering Results: ',num2str(round(100*(M*N-rest)/(M*N),1)),'% of points was clustered in ',num2str(length(Bn)),' clusters.']);

end

