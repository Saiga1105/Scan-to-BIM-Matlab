function [indices] = G_mesh_segmentation_test1(points)
%F_MESH_SEGMENTATION1 Summary of this function goes here
%   Detailed explanation goes here
%#function TreeBagger classificationEnsemble ClassificationECOC ClassificationBaggedEnsemble predict
indices= points(:,1)+points(:,2)+points(:,3);
end

