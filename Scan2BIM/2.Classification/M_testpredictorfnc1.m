clear;close;
addpath('D:\Google Drive\Research\Grasshopper Plugin Scan-to-BIM\Classification\Predictors');
addpath('D:\Google Drive\Research\Grasshopper Plugin Scan-to-BIM\Classification\Matlab');

% predictor location
filename=('D:\Google Drive\Research\Grasshopper Plugin Scan-to-BIM\Classification\Predictors\total.csv');
[Label,Area,Normalsimilarity,NormalZ,DiagonalXY,Height,Coplanarity,Proximity,Connections,Wallinlier,DvBottom,DvTop,ColAbove,ColBelow,ColFarAbove,Vbot,Vtop,Raytrace] = F_importfile(filename);
% model location
filename2= ('D:\Google Drive\Research\Grasshopper Plugin Scan-to-BIM\Classification\Matlab\Bagged_Trees_Model.mat');

Area=Area';
Normalsimilarity=Normalsimilarity';
NormalZ=NormalZ';
DiagonalXY=DiagonalXY';
Height=Height';
Coplanarity=Coplanarity';
Proximity=Proximity';
Connections=Connections';
Wallinlier=Wallinlier';
DvBottom=DvBottom';
DvTop=DvTop';
ColAbove=ColAbove';
ColBelow=ColBelow';
ColFarAbove=ColFarAbove';
Vbot=Vbot';
Vtop=Vtop';
Raytrace=Raytrace';

output = G_predictfunction(Area, Normalsimilarity, NormalZ, DiagonalXY, Height, Coplanarity, Proximity, Connections, Wallinlier, DvBottom, DvTop, ColAbove, ColBelow,ColFarAbove,Vbot,Vtop,Raytrace);