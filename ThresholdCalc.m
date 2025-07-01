%% Function to calculate threshold using ROC curve
%%%This function will calculate a threshold (cutoff) separating two classes.
%%%This cutoff is calcualted using an ROC curve and is the cross point of
%%%sensitivity and specificity curves (i.e. maximizes both sensitivity and
%%%specificity)
%% Inputs: 
%%%Features:  Vector of features (e.g. SHOD features used for activity
%%%           detection)
%%%Truth:     Vector corresponding to truth for the items in Features
%% Outputs:
%%%thresh:    cutoff in Feature vector that separates the two classes of
%%%           Truth (i.e. activity vs not)

function [thresh]=ThresholdCalc(Features,Truth)
Matr = [Features,Truth];
ROCout=roc(Matr,0,0.01,0);

%%%Cost effective (cross point of sensitivity and specificity curves)
thresh=ROCout.co(3);

end
