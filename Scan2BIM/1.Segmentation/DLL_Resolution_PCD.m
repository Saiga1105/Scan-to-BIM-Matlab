function resolution = G_Resolution_PCD( points,samples )
%% Explenation
%   this function takes as input any point cloud and returns teh average
%   point spacing based on a number of samples

% Requirements:
%   /

% input
%   points = array [mx3] with XYZ coordinates of point cloud
%   samples= double with number of samples to take from point cloud

% output 
%   resolution =  double with average point spacing

% example
%   /

%% construct point cloud
pc=pointCloud(points);
random= randi(length(points),samples,1);

% search neirest neighbours
for i=1:length(samples)    
[~,D] = findNearestNeighbors(pc,pc.Location(random(i),:),6);
temp(i) = mean(D);
end

resolution = mean(temp);


end