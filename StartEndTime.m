%% Function to find the start and end time of the uniform time vector. 
%%%Will find the minimum and maximum time from the sensor data to create a
%%%overall time vector.
%% Inputs:
%XRaw: 	  Unfiltered data. Cell array where each cell represents data from
%         a different sensor (and each cell could have multiple
%         measurements like Accel and Gyro)

%% Outputs:
%starttime: maximum starttime from all sensors
%endtime:   minimum endtime from all sensors

function [starttime, endtime] = StartEndTime(XRaw)

%Finds maximum start time and minimum end time from sensor data 
starttime = XRaw{1}{1}(1,1);
endtime = XRaw{1}{1}(end,1);
for i=1:length(XRaw)
    for j=1:length(XRaw{i})
        if XRaw{i}{j}(1,1) > starttime
            starttime = XRaw{i}{j}(1,1);
        elseif XRaw{i}{j}(end,1) < endtime
            endtime = XRaw{i}{j}(end,1);
        end
    end
end

%Round to nearest hundreth
starttime = str2double(sprintf('%0.2f',starttime));
endtime = str2double(sprintf('%0.2f',endtime));

end