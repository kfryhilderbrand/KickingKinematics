%% Calculate Sensor bias
%%%%%Currently only demeans by mean of gyroscope data, accelerometer left
%%%%%unchanged
%% Inputs: 
%X:            meaned data 
%% Outputs:
%sensorbias:   sensor bias

function [sensorbias]=Debias(X)

sensorbias = cell(1,3);
for i=1:3
    sensorbias{i}=cell(1,2);
    sensorbias{i}{1} = [0,0,0];
    sensorbias{i}{2} = [mean(X{i}{2}(:,2)),mean(X{i}{2}(:,3)),mean(X{i}{2}(:,4))];
end