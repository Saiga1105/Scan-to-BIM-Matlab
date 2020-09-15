%%Read point cloud
% use computer vision toolbox to load point clouds in matlab. DO NOT use
% "point cloud tools"
% if cloud is already loaded, do not load again
% read .ply or .pcd file
% it does not read the intensity

if exist('ptCloud')==0;
    tic
    fprintf('loading point cloud...');
    ptCloud = pcread('F:\Data\Lidar_Sint Jacobs Kerk Leuven 2017\PTG\test1a.ply');
    %[X, A] = pcread('D:\Code\Point cloud tools for Matlab\pglira-Point_cloud_tools_for_Matlab-0cad900\demodata\Dino.xyz');
    toc
    % ptCloudA.info('ExtInfo',true)
    pcshow(ptCloud);
    xlabel('X(m)');
    ylabel('Y(m)');
    zlabel('Z(m)');
    title('Original Point Cloud');
else fprintf('Point cloud already loaded');
end   
%% rotate point cloud slowly
% define R (1°)
% x = pi/180;
% R = [ cos(x) sin(x) 0 0
%      -sin(x) cos(x) 0 0
%       0         0   1 0
%       0         0   0 1];
% R = affine3d(R);
% 
% translate point cloud to centre
Tx = mean(ptCloud.XLimits);
Ty = mean(ptCloud.YLimits);
Tz = mean(ptCloud.ZLimits);
T = [ 1 0 0 0
      0 1 0 0
      0 0 1 0
      -Tx -Ty -Tz 1];
 T = affine3d(T);
ptCloud = pctransform(ptCloud,T);  
% 
% %define display box
% lower = min([ptCloud.XLimits ptCloud.YLimits]);
% upper = max([ptCloud.XLimits ptCloud.YLimits]);
% xlimits = [lower upper];
% ylimits = [lower upper];
% zlimits = ptCloud.ZLimits;
% player = pcplayer(xlimits,ylimits,zlimits);
% xlabel(player.Axes,'X (m)');
% ylabel(player.Axes,'Y (m)');
% zlabel(player.Axes,'Z (m)');
% title('Rotating Point Cloud');
% 
% for i = 1:360
%     ptCloud = pctransform(ptCloud,R);
%     view(player,ptCloud);
% end
%% Segment by normals
% fprintf('\n computing eigenvalues normals...');
% tic
% %pca moet je niet runnen want je kent de normals al;
% %[coeff,score,latent,tsquared,explained,mu] = pca(ptCloud.Normal);
% % gradientmap berekenen
% %gradientmap = ac_gradient_map(ptCloud.Normal);
% %ptCloudA=pointCloud(ptCloud.Location,'Color',gradientmap);
% 
% %k = boundary(ptCloud.Location(:,1),ptCloud.Location(:,2));
% 
% toc
%% segment point cloud based on bounding box
objects=plyread('D:\Code\Point cloud tools for Matlab\pglira-Point_cloud_tools_for_Matlab-0cad900\demodata\objects1.ply');
% lower = min([ptCloud.XLimits ptCloud.YLimits]);
% upper = max([ptCloud.XLimits ptCloud.YLimits]);
% xlimits = [lower upper];
% ylimits = [lower upper];
% zlimits = ptCloud.ZLimits;

roi=[min([;  ;  ];









