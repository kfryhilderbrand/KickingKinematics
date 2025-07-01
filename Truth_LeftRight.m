%% Function to create left and right leg truth
% Will form universal truth vector for left and right leg and save plots
% for later use
%% Inputs:
%%%     TruthResamp: vector of truth labels
%%%     plotFlag: flag indicating whether to show truth part plots
%% Outputs:
%%%     TruthLeft: left part of resampled truth
%%%     TruthRight: right part of resampled truth

function [TruthLeft,TruthRight] = Truth_LeftRight(TruthResamp, plotFlag)

tempLeft=[];
tempRight=[];

timeLeft=TruthResamp{1}(:,1);
timeRight=TruthResamp{1+length(TruthResamp)/2}(:,1);

for i=1:length(TruthResamp)/2
    tempLeft=[tempLeft,TruthResamp{i}(:,2)];
    tempRight=[tempRight,TruthResamp{i+length(TruthResamp)/2}(:,2)];
end

TruthLeft = [timeLeft,mode(tempLeft,2)];
TruthRight = [timeRight,mode(tempRight,2)];

%% Codifier Plots

if plotFlag==1
    a=figure();
    subplot(3,1,1)
    plot(TruthResamp{1}(:,1)-TruthResamp{1}(1,1),TruthResamp{1}(:,2),'-b')
    subplot(3,1,2)
    plot(TruthResamp{2}(:,1)-TruthResamp{1}(1,1),TruthResamp{2}(:,2),'-b')
    subplot(3,1,3)
    plot(TruthResamp{3}(:,1)-TruthResamp{1}(1,1),TruthResamp{3}(:,2),'-b')
    savefig(a,'CodifierTruthLeft.fig','compact')
    close(a)
    
    b=figure();
    subplot(3,1,1)
    plot(TruthResamp{4}(:,1)-TruthResamp{4}(1,1),TruthResamp{4}(:,2),'-b')
    subplot(3,1,2)
    plot(TruthResamp{5}(:,1)-TruthResamp{4}(1,1),TruthResamp{5}(:,2),'-b')
    subplot(3,1,3)
    plot(TruthResamp{6}(:,1)-TruthResamp{4}(1,1),TruthResamp{6}(:,2),'-b')
    savefig(b,'CodifierTruthRight.fig','compact')
    close(b)
end

end
