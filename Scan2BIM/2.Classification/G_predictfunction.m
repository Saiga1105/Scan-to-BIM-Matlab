function output = G_predictfunction(Area, Normalsimilarity, NormalZ, DiagonalXY, Height, Coplanarity, Proximity, Connections, Wallinlier, DvBottom, DvTop, ColAbove, ColBelow,ColFarAbove,Vbot,Vtop,Raytrace)
    %#function TreeBagger classificationEnsemble ClassificationECOC ClassificationBaggedEnsemble predict

% Copyright Maarten Bassier, PhD Candidate of the KU Leuven University
% Belgium. contact: maarten.bassier@kuleuven.be
    
% above function pragma's are important for CS code. It invokes the
% functions so they can be used.

% Data input = rows of doubles equal to the amount of
% observations

% Data output = row of doubles 0-5 depending on the class
% (0=floor,1=ceiling,2=roof,3=walls,4=beams,5=clutter)

% Prediction model = the trainedclassifer is trained with a dataset of 10
% different buildings
% 
% Area=Area';
% Normalsimilarity=Normalsimilarity';
% NormalZ=NormalZ';
% DiagonalXY=DiagonalXY';
% Height=Height';
% Coplanarity=Coplanarity';
% Proximity=Proximity';
% Connections=Connections';
% Wallinlier=Wallinlier';
% DvBottom=DvBottom';
% DvTop=DvTop';
% ColAbove=ColAbove';
% ColBelow=ColBelow';
% ColFarAbove=ColFarAbove';
% Vbot=Vbot';
% Vtop=Vtop';
% Raytrace=Raytrace';
%  
T=table(Area, Normalsimilarity, NormalZ, DiagonalXY, Height, Coplanarity, Proximity, Connections, Wallinlier, DvBottom, DvTop,ColAbove, ColBelow,ColFarAbove,Vbot,Vtop,Raytrace);
load('Bagged_Trees_Model.mat','-mat');
classificationEnsemble=trainedClassifier.ClassificationEnsemble;
yfit= predict(classificationEnsemble, T);

temp=zeros(length(yfit),1);
for i = 1:length(temp)
    if strcmp((yfit(i)),'Floors')==1
        temp(i)=0;
    else 
    if strcmp((yfit(i)),'Ceilings')==1
        temp(i)=1;
    else
    if strcmp((yfit(i)),'Roofs')==1
        temp(i)=2;
    else
    if strcmp((yfit(i)),'Walls')==1
       temp(i)=3;
    else
    if strcmp((yfit(i)),'Beams')==1
       temp(i)=4;
    else
    if strcmp((yfit(i)),'Clutter')==1
        temp(i)=5;
    end
    end
    end
    end
    end
    end
end
output=temp';

