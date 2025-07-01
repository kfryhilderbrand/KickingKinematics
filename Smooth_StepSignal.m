%% Function to smooth classifier output
%%% Takes a vector of classifier labels and smooths the signal to eliminate
%%% potential movement artifacts.
%% Inputs: 
%%%Labels:           Vector of classifier labels
%%%mem:              number of members to smooth by
%% Outputs:
%%%Labels_Smooth:    Vector of classifier labels smoothed

function [Labels_Smooth] = Smooth_StepSignal(Labels, mem)
    Labels_Smooth = Labels;

%     %Handle boundary conditions
%     Labels_Smooth(1)=Labels(1);
%     if Labels(1)~=Labels(2)
%         if Labels(2)~=Labels(3)
%             Labels_Smooth(2)=Labels(1);
%         else
%             Labels_Smooth(1)=Labels(2);
%         end
%     elseif Labels(end-1)~=Labels(end)
%         Labels_Smooth(end-1)=Labels(end);
%     end
    
    changedIndexes = (diff(Labels_Smooth)~=0);
    groups = Labels_Smooth(1);
    %Smooth signal such that a group of size mem is needed    
    for i=mem:length(Labels)-mem
        if (changedIndexes(i-1))&&(changedIndexes(i)||changedIndexes(i+1))
            Labels_Smooth(i) = groups;
        elseif (~changedIndexes(i-1))&&(changedIndexes(i-2)||changedIndexes(i))
            Labels_Smooth(i) = groups;
        end
        groups = Labels_Smooth(i);
    end

end