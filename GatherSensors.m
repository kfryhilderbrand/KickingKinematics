%% Function to gather data together 
%This function will take a cell matrix X and gather the cells of each
%X{i}.  So, the first member of each X{i} into the first cell of
%GatheredSensors and so on for length of X{i}.  Currently used to gather
%Accelerometer and gyroscope data from various Metawear sensors.
%% Inputs: 
%X:       Data to remap
%NumMeas: Number of measurements per sensor (i.e. 2 for Accel and Gyro)
%% Outputs:
%GatheredSensors:   Filtered Data cell array

function [GatheredSensors]=GatherSensors(X, NumMeas)

GatheredSensors = cell(1,NumMeas);
for i=1:NumMeas
    GatheredSensors{i}=cell(1,length(X));
    for j=1:length(X)
    GatheredSensors{i}{j}=X{j}{i};
    end
end

end