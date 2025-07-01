%% Function to calculate SHOD features for all data
%%%This function will use a stance hypothesis optimal detector (SHOD) to
%%%analyze both acceleration and angular velocity. Will create a struct to
%%%store a 3D SHOD feature vector for one leg of sensors. Will also store 
%%%window data at each timestamp that the SHOD value is calculated.
%% Inputs:
%%%X:           Data to classify, ActData.datasegment(i).Right/Left
%%%SensData:    SensData.datasegment(i).Right/Left
%%%Time:        time vector
%% Outputs:
%%%SensData:    Struct containing window data and timestamps for center of
%%%             windows

function [SHOD]=Calculate3DSHOD_CLASS(X, SensData, Time)

SHOD = [];
SHOD_RMS = [];
for i = 1:length(Time)
   indT = find(abs(SensData.Thigh.WinTime - Time(i))<0.001);
   indS = find(abs(SensData.Shin.WinTime - Time(i))<0.001);
   indF = find(abs(SensData.Foot.WinTime - Time(i))<0.001); 
   if (indT ~= 0)&&(indS ~= 0)&&(indF ~= 0)
       temp = [X.Foot.SHOD(indF),X.Shin.SHOD(indS),X.Thigh.SHOD(indF)];
       SHOD = [SHOD; norm(temp)];
       SHOD_RMS = [SHOD_RMS; rms(temp)];
   end
end

end