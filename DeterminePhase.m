%% Function to Determine Phases of each DOF
%%% 3-DOF hip, 1-DOF knee, 2-DOF ankle

%% Inputs: 
%%%rates: structure with joint rates for all DOF's
%%%thresh = #: Considered NM if rate results in joint angle change less than # deg for 1 seconds of movement

%% Outputs: 
%%%phase: matrix containing phase vectors

function [phase] = DeterminePhase(rates, thresh)

sets = length(rates.dataset); %%num datasets
num_neigh = 5;

for i=1:sets
    legs = length(fieldnames(rates.dataset(i)));
    legs_name = fieldnames(rates.dataset(i));
    for j=1:legs
        temp = [];
        axs_name = fieldnames(rates.dataset(i).(legs_name{j}));
        axs = length(fieldnames(rates.dataset(i).(legs_name{j})));
        for num_DOF = 1:axs     
            Phase = double(rates.dataset(i).(legs_name{j}).(axs_name{num_DOF})>thresh)-(double(rates.dataset(i).(legs_name{j}).(axs_name{num_DOF}))<-thresh); %1 for one direction,-1 for other direction 0, for NM
            Phase_Smooth = medfilt1(Phase, num_neigh);
            phase.dataset(i).(legs_name{j}).(axs_name{num_DOF})=Phase_Smooth;
            temp = [temp; Phase_Smooth'];
            clear Phase
        end
        phase.dataset(i).(legs_name{j}).full = temp;
    end
end

end