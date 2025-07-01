%% Function to split data into predefined lengths
%%% This function is used to split sensor data, truth data, and activity
%%% detection data into data segments of specified length.
%% Inputs: 
%%% X:         Data to segment
%%% SegLength: Desired length of data segments
%%% fs:        Sampling frequency
%%% NumMeas:   number of sensor measurements
%% Outputs:
%SegData:   Segmented Data

function [SegData, TruthData]=Segment_Data(SensData, Truth, ActData, SegLength, SegStep, fs)

NumSamp = floor(SegLength/(1/fs));
NumStep = floor(SegStep/(1/fs));
n = length(SensData.datasegment);
k = 1;

for i=1:n
    [m,~] = size(SensData.datasegment(i).Left.Foot.Accel);
    temp = floor(m/NumSamp);
    for j = 1:NumStep:temp*NumSamp
    %%% Left
        %%%Foot
        SegData.datasegment(k).Left.Foot.Accel = SensData.datasegment(i).Left.Foot.Accel(j:j+NumSamp,:);
        SegData.datasegment(k).Left.Foot.Gyro = SensData.datasegment(i).Left.Foot.Gyro(j:j+NumSamp,:);
        SegData.datasegment(k).Left.Foot.SHOD = ActData.datasegment(i).Left.Foot.SHOD_Resamp(j:j+NumSamp);
        
        %%%Shin
        SegData.datasegment(k).Left.Shin.Accel = SensData.datasegment(i).Left.Shin.Accel(j:j+NumSamp,:);
        SegData.datasegment(k).Left.Shin.Gyro = SensData.datasegment(i).Left.Shin.Gyro(j:j+NumSamp,:);
        SegData.datasegment(k).Left.Shin.SHOD = ActData.datasegment(i).Left.Shin.SHOD_Resamp(j:j+NumSamp);
               
        %%%Thigh
        SegData.datasegment(k).Left.Thigh.Accel = SensData.datasegment(i).Left.Thigh.Accel(j:j+NumSamp,:);
        SegData.datasegment(k).Left.Thigh.Gyro = SensData.datasegment(i).Left.Thigh.Gyro(j:j+NumSamp,:);
        SegData.datasegment(k).Left.Thigh.SHOD = ActData.datasegment(i).Left.Thigh.SHOD_Resamp(j:j+NumSamp);
                
        %%%All
        SegData.datasegment(k).Left.SHOD3D = ActData.datasegment(i).Left.SHOD3D_Resamp(j:j+NumSamp);
        SegData.datasegment(k).Left.SHOD3D = ActData.datasegment(i).Left.Time(j:j+NumSamp); 
        
        %%%Truth
        TruthData.datasegment(k).Left = Truth.datasegment(i).Left.TruthResamp(j:j+NumSamp,:);
        
        %%%Time
        SegData.datasegment(k).Left.Time = ActData.datasegment(i).Left.Time(j:j+NumSamp); 
        
        %%%Predicted Labels
        SegData.datasegment(k).Left.PredLabelsT = ActData.datasegment(i).Left.PredLabelsT_Resamp(j:j+NumSamp);
        SegData.datasegment(k).Left.Foot.PredLabelsT = ActData.datasegment(i).Left.Foot.PredLabelsT_Resamp(j:j+NumSamp);
        SegData.datasegment(k).Left.Shin.PredLabelsT = ActData.datasegment(i).Left.Shin.PredLabelsT_Resamp(j:j+NumSamp);
        SegData.datasegment(k).Left.Thigh.PredLabelsT = ActData.datasegment(i).Left.Thigh.PredLabelsT_Resamp(j:j+NumSamp);

    %%% Right
        %%%Foot
        SegData.datasegment(k).Right.Foot.Accel = SensData.datasegment(i).Right.Foot.Accel(j:j+NumSamp,:);
        SegData.datasegment(k).Right.Foot.Gyro = SensData.datasegment(i).Right.Foot.Gyro(j:j+NumSamp,:);
        SegData.datasegment(k).Right.Foot.SHOD = ActData.datasegment(i).Right.Foot.SHOD_Resamp(j:j+NumSamp);
        
        %%%Shin
        SegData.datasegment(k).Right.Shin.Accel = SensData.datasegment(i).Right.Shin.Accel(j:j+NumSamp,:);
        SegData.datasegment(k).Right.Shin.Gyro = SensData.datasegment(i).Right.Shin.Gyro(j:j+NumSamp,:);
        SegData.datasegment(k).Right.Shin.SHOD = ActData.datasegment(i).Right.Shin.SHOD_Resamp(j:j+NumSamp);
               
        %%%Thigh
        SegData.datasegment(k).Right.Thigh.Accel = SensData.datasegment(i).Right.Thigh.Accel(j:j+NumSamp,:);
        SegData.datasegment(k).Right.Thigh.Gyro = SensData.datasegment(i).Right.Thigh.Gyro(j:j+NumSamp,:);
        SegData.datasegment(k).Right.Thigh.SHOD = ActData.datasegment(i).Right.Thigh.SHOD_Resamp(j:j+NumSamp);
                
        %%%All
        SegData.datasegment(k).Right.SHOD3D = ActData.datasegment(i).Right.SHOD3D_Resamp(j:j+NumSamp);
        SegData.datasegment(k).Right.SHOD3D = ActData.datasegment(i).Right.Time(j:j+NumSamp); 
        
        %%%Truth
        TruthData.datasegment(k).Right = Truth.datasegment(i).Right.TruthResamp(j:j+NumSamp,:);
        
        %%%Time
        SegData.datasegment(k).Right.Time = ActData.datasegment(i).Right.Time(j:j+NumSamp); 
        
        %%%Predicted Labels
        SegData.datasegment(k).Right.PredLabelsT = ActData.datasegment(i).Right.PredLabelsT_Resamp(j:j+NumSamp);
        SegData.datasegment(k).Right.Foot.PredLabelsT = ActData.datasegment(i).Right.Foot.PredLabelsT_Resamp(j:j+NumSamp);
        SegData.datasegment(k).Right.Shin.PredLabelsT = ActData.datasegment(i).Right.Shin.PredLabelsT_Resamp(j:j+NumSamp);
        SegData.datasegment(k).Right.Thigh.PredLabelsT = ActData.datasegment(i).Right.Thigh.PredLabelsT_Resamp(j:j+NumSamp);
    
        k = k+1;
    end
end

end