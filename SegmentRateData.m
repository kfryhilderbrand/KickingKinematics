%% Function to split joint rate data into predefined lengths
%%% This function is used to split joint rate data into data segments of 
%%% specified length.
%% Inputs:
%%%rates:      Data to segment
%%%SegLength:  Desired length of data segments
%%%SegStep:    Desired step between segments
%%%fs:         Sampling frequency

%% Outputs:
%SegData:   Segmented Data

function SegRates = SegmentRateData(rates, SegLength, SegStep, fs)

NumSamp = floor(SegLength/(1/fs));
NumStep = floor(SegStep/(1/fs));
num_sets = length(rates.dataset);

num_legs = length(fieldnames(rates.dataset(1)));
legs_name = fieldnames(rates.dataset(1));

for leg=1:num_legs
    m = 1;
    for set=1:num_sets
        [p,~] = size(rates.dataset(set).(legs_name{leg}).HF);
        temp = floor(p/NumSamp);
        for j = 1:NumStep:temp*NumSamp
            axs_name = fieldnames(rates.dataset(set).(legs_name{leg}));
            axs = length(fieldnames(rates.dataset(set).(legs_name{leg})));
            for k=1:axs
                SegRates.dataset(m).(legs_name{leg}).(axs_name{k}) = rates.dataset(set).(legs_name{leg}).(axs_name{k})(j:j+NumSamp,1);
            end
            m = m+1;
        end
    end
end

end