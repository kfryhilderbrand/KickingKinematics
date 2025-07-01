%% Determine truth and sensor offset
%%%Will plot truth and sensor data against each other to determine the
%%%offset. Once the offset, if any, has been determined, will adjust time
%%%of sensor data. Finally, will trim sensor time vector and truth time
%%%vector to uniform start and stop time. Adjusted truth and data will be saved to
%%%Truth struct and SensData struct respectively.
%% Inputs:
%%%Thighoffset:        offset between truth and thigh sensor data
%%%Shinoffset:         offset between truth and shin sensor data
%%%Footoffset:         offset between truth and foot sensor data
%%%Data:               ActData.datasegment(i).Left/Right
%%%Truth:              TruthLabels.TruthLeft/Right
%% Outputs:

function []=TruthSensOffset(Thighoffset, Shinoffset, Footoffset, Data, Truth)
   
    Thigh = Data.Thigh.SHOD;
    Shin = Data.Shin.SHOD;
    Foot = Data.Foot.SHOD;
    Time = Data.WinTime-Truth(1,1);
    
    Upperlim = max([Thigh;Shin;Foot]);
    if isnan(Upperlim)||Upperlim==0
        Upperlim = 1;
    end
    Lowerlim = 0;
    
    a=figure(); 
    subplot(3,1,1)
    hold on
    for j=1:length(Truth(:,1))
        if Truth(j,2)==1
            line([Truth(j,1)-Truth(1,1) Truth(j,1)-Truth(1,1)],...
                [Lowerlim Upperlim],'Color',[0.8 0.8 0.8]);
        end
    end
    plot(Time+Thighoffset,Thigh,'-b');
    title('Codifier Truth vs Thigh SHOD Values')
    axis([min(0,Time(1)+Thighoffset) Time(end)+Thighoffset Lowerlim Upperlim])
    hold off
    
    subplot(3,1,2)
    hold on
    for j=1:length(Truth(:,1))
        if Truth(j,2)==1
            line([Truth(j,1)-Truth(1,1) Truth(j,1)-Truth(1,1)],...
                [Lowerlim Upperlim],'Color',[0.8 0.8 0.8]);
        end
    end
    plot(Time+Shinoffset,Shin,'-b');
    title('Codifier Truth vs Shin SHOD Values')
    axis([min(0,Time(1)+Shinoffset) Time(end)+Shinoffset Lowerlim Upperlim])
    hold off
    
    subplot(3,1,3)
    hold on
    for j=1:length(Truth(:,1))
        if Truth(j,2)==1
            line([Truth(j,1)-Truth(1,1) Truth(j,1)-Truth(1,1)],...
                [Lowerlim Upperlim],'Color',[0.8 0.8 0.8]);
        end
    end
    plot(Time+Footoffset,Foot,'-b');
    title('Codifier Truth vs Foot SHOD Values')
    axis([min(0,Time(1)+Footoffset) Time(end)+Footoffset Lowerlim Upperlim])
    hold off
    
    clear TruthLabels

clear j a Lowerlim Upperlim Thigh Shin Foot Time
end
