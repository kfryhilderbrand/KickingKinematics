%% Function to create classifier labels from a give feature vector using a threshold
%%%This function will use a threshold to determine if members of a feature
%%%vector are classified as 1 or 0 (i.e. if they exceed the threshold or
%%%not).
%% Inputs: 
%%%Features:  Vector of features (e.g. SHOD features used for activity
%%%           detection)
%%%thresh:    cutoff in Feature vector that separates the two classes of
%%%           Truth (i.e. activity vs not)
%% Outputs:
%%%Labels:    Vector of classifier labels

function [Labels] = ThreshPred(Features, thresh)

Labels = zeros(length(Features),1);
for i=1:length(Features)
    if Features(i)>=thresh
        Labels(i)=1;
    end
end

end