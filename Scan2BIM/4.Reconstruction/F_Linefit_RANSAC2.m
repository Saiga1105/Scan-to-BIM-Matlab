function [mx, my,inlierNum, error,xy] = F_Linefit_RANSAC2(Cx,Cy,Px,Py,k,threshDist,dw,n)
%F_Arcfit_RANSAC Computes best fit 2-point line through RANSAC
 % Cx = 1 x n matrix with x coordinates of data points
 % Cy = 1 x n matrix with y coordinates of data points
 % Px = x-coordinates eval points
 % Py = y-coordinates eval points
 % k: the number of iterations
 % threshDist: the threshold of the distances between points and the fitting line
 % omega: the threshold of the number of inliers 
 % dw = width wall
 
%#function distance2curve 

% set parameters
data= [Cx Cy];
mx=linspace(min([Px;Cx]),max([Px;Cx]),20)';
my=zeros(length(mx),k);
error=zeros(k,1);
inlrNum=zeros(k,1);

for i = 1:k
    % 1. fit using 3 random points
%     sampleIdx1 = randi([1 ceil(ptNum/2*1)-1],1);
% 	sampleIdx2 = randi([ceil(ptNum/2*1) ceil(ptNum/2*2)],1);
%     sampleIdx= [sampleIdx1 sampleIdx2];
%     
% 	ptSample = pts(:,sampleIdx)';
    
    sample= data(randperm( length(Cx),2),:);
    
    % compute model
    modelLeastSquares = polyfit(sample(:,1),sample(:,2),1);
    my(:,i) = modelLeastSquares(1)*mx + modelLeastSquares(2);
    
    % compute perpendicular distance
    curvexy=[mx my(:,i)];
    mapxy= [Px Py];
    [xy,distance,~] = distance2curve(curvexy,mapxy,'linear');
    error(i)=mean(abs(abs(distance)-dw/2));
    
    % compute number of inliers
    inlierIdx = find(dw/2-threshDist<=abs(distance) & abs(distance)<=dw/2+threshDist);
    inlrNum(i) = length(inlierIdx);

end
%score=inlrNum./(2*error);
score=inlrNum;
[~,index]=max(score);
inlierNum=inlrNum(index);
error = error(index);
my= my(:,index);

%% recompute model
    % compute perpendicular distance to C
    curvexy=[mx my];
    mapxy= [Cx Cy];
    [xy,distance,~] = distance2curve(curvexy,mapxy,'linear');
    
    % compute number of inliers
    inlierIdx = find(abs(distance)<=threshDist);

    % compute model
    modelLeastSquares = polyfit(Cx(inlierIdx,1),Cy(inlierIdx,1),1);
    my = modelLeastSquares(1)*mx + modelLeastSquares(2);
    

data =[mx my];
data=data(data(:,1)>=min([Px;Cx]) & data(:,2)>=min([Py;Cy]),:);
data=data(data(:,1)<=max([Px;Cx]) & data(:,2)<=max([Py;Cy]),:);

mx=num2cell(data(:,1));
my=num2cell(data(:,2));
end


