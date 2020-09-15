function [ Kn, Krgb, Klocation ] = F_DetermineKnn( pc )
%F_DETERMINEKNN Summary of this function goes here
%   Detailed explanation goes here
%   het doel van deze functie is het correct bepalen van K (aantal
%   clusters) op basis van histogrammen
% input
%   pc= rasterized point cloud MxNx3 with Location,Normal and COlor
%   Kn= K clusters voor de normals
%   Krgb= K clusters voor de kleuren
%   Kxyz= K voor de plaats

%% K for normals
%normal to grayscale
%N=reshape(pc.Normal,size(pc.Normal,1)*size(pc.Normal,2),3);
N=rgb2gray(mat2gray(pc.Normal));%N=N(:,1);

%find & filter peaks
[counts,binLocations]=imhist(N);
[X,Nval]=findpeaks(counts,binLocations);
index=find(X<=mean(counts));
Nval(index)=[];
Kn=size(Nval,1);

%% find K for Color
I=rgb2gray(mat2gray(pc.Color));
sigma = 2;
%I = imgaussfilt(I,sigma);
%I = imgradient(I,'CentralDifference');

[counts,binLocations]=imhist(I);
[X,Cval]=findpeaks(counts,binLocations,...
    'SortStr','descend',...
    'MinPeakHeight',mean(counts),...
    'Threshold',0.01*size(I,1)*size(I,2),...
    'MinPeakDistance',0.05);
index=find(X<=mean(counts));
Cval(index)=[];
Kc=size(Cval,1);


% polar=xyz2polar(pc.Location);

end

