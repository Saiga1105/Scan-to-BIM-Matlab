clear;close;addpath('pointcloudtoolsfunctions');
tic
fprintf('loading point cloud...');
pc=pcread('F:\Data\Lidar_15.Sint Jacobs Kerk Leuven\PTG\test1a_culled.ply');
toc
% ab = reshape(pc.Normal,size(pc.Normal,1)*3,1);
% N=rgb2gray(mat2gray(pc.Normal));
% N=N(:,1);
% I=rgb2gray(mat2gray(pc.Color));
% I=I(:,1);
% polar=xyz2polar(pc.Location);
k=3;

tic
[clusterindex1,cluster_center1] = kmeans(pc.Normal,k);
fprintf('k means normals computation...');
toc

tic
fprintf('k means location computation...');
[clusterindex2,cluster_center2] = kmeans(double(pc.Location),k);
toc

tic 
fprintf('k means location+color+normal computation...');
[clusterindex3,cluster_center3] = kmeans([pc.Location pc.Normal double(pc.Color)],k);
%pc2=pointCloud(cluster_center3(:,1:3),'Color',round(cluster_center3(:,4:6),0),'Normal',cluster_center3(:,7:9));
toc

%% k means generalised color computation...
% tic
% fprintf('k means generalised color computation...');
% % vorm kleur om naar L*a*b code. dit is belichting invariant l = belichting, a=rood en b is blauw
% cform=makecform('srgb2lab');
% lab = applycform(pc.Color,cform);
% % smijt L weg + smijt image information weg (we parsen gewoon een array)
% ab = double(lab(:,2:3));
% [clusterindex3,cluster_center3] = kmeans(ab,k);
% toc

% tracht pc om te zetten naar een image voor de canny detector op los te
% laten. eenmaal gebeurt vertaal je dit terug naar de 3D puntenwolk.
% polar=xyz2polar(pc.Location);
% reshape(pc.Color,polar(:,1),polar(:,2));
% pointCloud(
%% visualisation
player0 = pcplayer(pc.XLimits,pc.YLimits,pc.ZLimits);
view(player0,pc.Location,pc.Color);
title(player0.Axes, 'Initial point Cloud');
player1 = pcplayer(pc.XLimits,pc.YLimits,pc.ZLimits);
view(player1,pc.Location,clusterindex1);
title(player1.Axes, 'k means clustered point cloud based on normals');
player2 = pcplayer(pc.XLimits,pc.YLimits,pc.ZLimits);
view(player2,pc.Location,clusterindex2);
title(player2.Axes, 'k means clustered point cloud based on color');
player3 = pcplayer(pc.XLimits,pc.YLimits,pc.ZLimits);
view(player3,pc.Location,clusterindex3);
title(player3.Axes, 'k means clustered point cloud based on color+normal');