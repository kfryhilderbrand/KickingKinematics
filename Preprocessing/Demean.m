%% Function to demean data
%%%%%Currently only demeans by mean of data (will need to incorporate way
%%%%%to demean by resting bias
%% Inputs: 
%X:       meaned data 
%sensor_bias: bias of channel when sitting still
%% Outputs:
%Xdemean:   demeaned data

function [Xdemean]=Demean(X,sensor_bias)

Xdemean=X;
for i=1:length(X)
    for j=1:length(X{i})
        [~,n] = size(X{i}{j});
        for k=2:n
            Xdemean{i}{j}(:,k) = X{i}{j}(:,k)-sensor_bias{i}{j}(k-1);
        end
    end
end

disp('Done Demeaning')
end