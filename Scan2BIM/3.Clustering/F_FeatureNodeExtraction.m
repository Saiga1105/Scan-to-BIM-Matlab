function [ Area, NormalZ, Height, Width, Surfacetype ] = F_FeatureNodeExtraction( Clusters)
%F_FEATUREEXTRACTION Computes unary feature values for the following attributes: Area, NormalZ, Height, Width, Surfacetype
% input
%    Clusters

% output
%   Area(x)= x1= 1/(1+exp^-x) sigmoid function
%   NormalZ= vec(0,0,nz)
%   Height= pointCloud.ZLimit
%   Width= sqrt(sum(XLimit.^2+YLimit.^2,2));

%% parameters
n=length(Clusters);

% preallocate arrays
Area=zeros(n,1);
NormalZ= zeros(n,1);
Height=zeros(n,1);
Width=zeros(n,1);
Surfacetype=zeros(n,1); %0= complex, 1= plane, 2=cylinder, 3= sphere;

for p=1:n
%% Area
  Resolution=(sqrt( sum(((Clusters{p}.PointCloud.Location(1:50:end-1,:) - Clusters{p}.PointCloud.Location(2:50:end,:)).^2),2)));
  Resolution=mean(Resolution(Resolution<mean(Resolution)));
  Area(p)=Resolution*sqrt(length(Clusters{p}.PointCloud.Location));

%% NormalZ
NormalZ(p)=abs(Clusters{p}.Normal(3));

%% Height
Height(p)=Clusters{p}.PointCloud.ZLimits(2)-Clusters{p}.PointCloud.ZLimits(1);
%% Width
xl=Clusters{p}.PointCloud.XLimits(2)-Clusters{p}.PointCloud.XLimits(1);
yl=Clusters{p}.PointCloud.YLimits(2)-Clusters{p}.PointCloud.YLimits(1);
Width(p)=sqrt(xl.^2+yl.^2);
%% Surfacetype
Surfacetype(p)=Clusters{p}.SurfaceType;
end
tied=1;
Area=UGM_standardizeCols(Area,tied);
Height=UGM_standardizeCols(Height,tied);
Width=UGM_standardizeCols(Width,tied);

end

