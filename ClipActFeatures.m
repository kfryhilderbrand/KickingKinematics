%% Function to clip overall SHOD feature vector
%%%This function will take a time vector associated with truth data and
%%%will find corresponding windows in the SHOD data. Any windows that do
%%%not have truth data associated with them will be clipped.
%% Inputs:
%%%TruthTimes:   Vector corresponding to the times at the center of each
%%%              window with associated truth
%%%Act:          Struct containing overall SHOD features for windows

%% Outputs:
%%%Leg:      Struct containint vectors corresponding to the times at center 
%%%          of each truth window and Shod features for each window.

function [Leg]=ClipActFeatures(TruthTimes, Act)
    %Extracts fields from Data struct
    ActTimes = Act.WinTime;
    DataSHOD = Act.SHOD3D;
    Foot = Act.Foot.SHOD;
    Shin = Act.Shin.SHOD;    
    Thigh = Act.Thigh.SHOD;
     
    %Searches through window data and clips windows with no associated
    %truth
    WinTime = [];
    SHOD = [];
    FootSHOD = [];
    ShinSHOD = [];
    ThighSHOD = [];
    
    ind = find(abs(ActTimes - TruthTimes(1))<0.001);
    if ind ~= 0
        WinTime = [WinTime; ActTimes(ind)];
        SHOD = [SHOD; DataSHOD(ind)];
        FootSHOD = [FootSHOD; Foot(ind)];
        ShinSHOD = [ShinSHOD; Shin(ind)];
        ThighSHOD = [ThighSHOD; Thigh(ind)];
        prev = ActTimes(ind);
    end
    
    for i=2:length(TruthTimes)
        ind = find(abs(ActTimes - TruthTimes(i))<0.001);
        if ((ind ~= 0) && (ActTimes(ind)~=prev))
            WinTime = [WinTime; ActTimes(ind)];
            SHOD = [SHOD; DataSHOD(ind)];
            FootSHOD = [FootSHOD; Foot(ind)];
            ShinSHOD = [ShinSHOD; Shin(ind)];
            ThighSHOD = [ThighSHOD; Thigh(ind)];
            prev = ActTimes(ind);
        else
            WinTime = [WinTime; ActTimes(ind)+0.005];
            SHOD = [SHOD; DataSHOD(ind)];
            FootSHOD = [FootSHOD; Foot(ind)];
            ShinSHOD = [ShinSHOD; Shin(ind)];
            ThighSHOD = [ThighSHOD; Thigh(ind)];
            prev = ActTimes(ind);            
        end
    end
    
    Leg.Foot.SHOD = FootSHOD;
    Leg.Shin.SHOD = ShinSHOD;
    Leg.Thigh.SHOD = ThighSHOD;
    Leg.WinTime = WinTime;
    Leg.SHOD3D = SHOD;

end