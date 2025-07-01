%% Create Training and Threshold Activity Detection
%%% Note: Nothing in this script will need to be updated.

%% Create Training Data 
%%%Creates training data and saves to the ActData struct
TrainData = CreateTraining(SensData, ActData, Truth);

%% Threshold calculation for SHOD activity detector
%%%Trains on the SHOD data stored in ActData struct. Does this for the
%%%overall activity and for each sensor. Finds the threshold which is
%%%stored in the ActData as well. 
num_train = length(TrainData.datasegment);
for i=1:num_train
    %%%Overall
    ActData.datasegment(i).Left.Thresh = ThresholdCalc(TrainData.datasegment(i).Left.SHOD3D,...
        TrainData.datasegment(i).Left.WinTruth);
    ActData.datasegment(i).Right.Thresh = ThresholdCalc(TrainData.datasegment(i).Right.SHOD3D,...
        TrainData.datasegment(i).Right.WinTruth);
end

%% Threshold activity detector
%%%Classifier SHOD data for each datasegment stored in ActData. Does this 
%%%for overall.
for i=1:num_train
    %%%Overall
   ActData.datasegment(i).Left.PredLabelsT = medfilt1(ThreshPred(ActData.datasegment(i).Left.SHOD3D,...
        ActData.datasegment(i).Left.Thresh),3);
   ActData.datasegment(i).Left.cpT = classperf(Truth.datasegment(i).Left.WinTruth,ActData.datasegment(i).Left.PredLabelsT);
    ActData.datasegment(i).Right.PredLabelsT =  medfilt1(ThreshPred(ActData.datasegment(i).Right.SHOD3D,...
        ActData.datasegment(i).Right.Thresh),3);
    ActData.datasegment(i).Right.cpT = classperf(Truth.datasegment(i).Right.WinTruth,ActData.datasegment(i).Right.PredLabelsT);    
   
    %%%Foot
    ActData.datasegment(i).Left.Foot.PredLabelsT = ThreshPred(ActData.datasegment(i).Left.Foot.SHOD,...
        ActData.datasegment(i).Left.Thresh);
    ActData.datasegment(i).Right.Foot.PredLabelsT =  Smooth_StepSignal(ThreshPred(ActData.datasegment(i).Right.Foot.SHOD,...
        ActData.datasegment(i).Right.Thresh),3);

    %%%Shin
    ActData.datasegment(i).Left.Shin.PredLabelsT = ThreshPred(ActData.datasegment(i).Left.Shin.SHOD,...
        ActData.datasegment(i).Left.Thresh);
    ActData.datasegment(i).Right.Shin.PredLabelsT =  Smooth_StepSignal(ThreshPred(ActData.datasegment(i).Right.Shin.SHOD,...
        ActData.datasegment(i).Right.Thresh),3);
   
    %%%Thigh
    ActData.datasegment(i).Left.Thigh.PredLabelsT = ThreshPred(ActData.datasegment(i).Left.Thigh.SHOD,...
        ActData.datasegment(i).Left.Thresh);
    ActData.datasegment(i).Right.Thigh.PredLabelsT =  Smooth_StepSignal(ThreshPred(ActData.datasegment(i).Right.Thigh.SHOD,...
        ActData.datasegment(i).Right.Thresh),3);
end
%% Resample vector of predictions
num_dataseg = length(ActData.datasegment);
for i=1:num_dataseg
    ActData.datasegment(i).Left.PredLabelsT_Resamp = round(interp1(ActData.datasegment(i).Left.WinTime, ActData.datasegment(i).Left.PredLabelsT,ActData.datasegment(i).Left.Time,'spline'));
    ActData.datasegment(i).Left.Foot.PredLabelsT_Resamp = round(interp1(ActData.datasegment(i).Left.WinTime, ActData.datasegment(i).Left.Foot.PredLabelsT,ActData.datasegment(i).Left.Time,'spline'));
    ActData.datasegment(i).Left.Shin.PredLabelsT_Resamp = round(interp1(ActData.datasegment(i).Left.WinTime, ActData.datasegment(i).Left.Shin.PredLabelsT,ActData.datasegment(i).Left.Time,'spline'));
    ActData.datasegment(i).Left.Thigh.PredLabelsT_Resamp = round(interp1(ActData.datasegment(i).Left.WinTime, ActData.datasegment(i).Left.Thigh.PredLabelsT,ActData.datasegment(i).Left.Time,'spline'));
    
    ActData.datasegment(i).Right.PredLabelsT_Resamp = round(interp1(ActData.datasegment(i).Right.WinTime, ActData.datasegment(i).Right.PredLabelsT,ActData.datasegment(i).Right.Time,'spline'));
    ActData.datasegment(i).Right.Foot.PredLabelsT_Resamp = round(interp1(ActData.datasegment(i).Right.WinTime, ActData.datasegment(i).Right.Foot.PredLabelsT,ActData.datasegment(i).Right.Time,'spline'));
    ActData.datasegment(i).Right.Shin.PredLabelsT_Resamp = round(interp1(ActData.datasegment(i).Right.WinTime, ActData.datasegment(i).Right.Shin.PredLabelsT,ActData.datasegment(i).Right.Time,'spline'));
    ActData.datasegment(i).Right.Thigh.PredLabelsT_Resamp = round(interp1(ActData.datasegment(i).Right.WinTime, ActData.datasegment(i).Right.Thigh.PredLabelsT,ActData.datasegment(i).Right.Time,'spline'));
end

%% Save structs

save('Truth.mat','Truth')
save('SensorData.mat','SensData')
save('ActivityData.mat','ActData')
save('TrainingData.mat','TrainData')

clearvars -except SensData ActData Truth Constants Flags