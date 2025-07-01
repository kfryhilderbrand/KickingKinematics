%% Function to Determine Phases of each DOF (Names)
%%% 3-DOF hip, 1-DOF knee, 2-DOF ankle

%% Inputs: 
%%%Phase: structure with phases for all legs,DOFs, and datasets

%% Outputs:
%%%phase_names: structure with phase names for all legs, DOFs, and datasets
%%% With our sensor placement   y<---Xz      z point down to limb surface
%                                    |       X point down limb shank
%                                    v x
%%%  Left Leg:  Positive joint rates
%%%  HF (extension), HA (adduction, toward midline), HR (internal
%%%  rotation), KF (extension), AF (flexion), AI (eversion)

%%%  Right Leg: Positive joint rates
%%%  HF (extension), HA (abduction, away from midline), HR (external
%%%  rotation), KF (extension), AF (flexion), AI (inversion)

function [phase_names] = DeterminePhase_Names(Phase)

%%% Left leg lookup table with numerical phase code equivalent (1, -1, 0)
Lookup.Left.HF = {'flexion','extension','NM'};
Lookup.Left.HA = {'adduction','abduction','NM'};
Lookup.Left.HR = {'internal_rot','external_rot','NM'};
Lookup.Left.KF = {'extension','flexion','NM'};
Lookup.Left.AF = {'flexion','extension','NM'};
Lookup.Left.AI = {'eversion','inversion','NM'};

%%% Right leg lookup table with numerical phase code equivalent (1, -1, 0)
Lookup.Right.HF = {'flexion','extension','NM'};
Lookup.Right.HA = {'abduction','adduction','NM'};
Lookup.Right.HR = {'external_rot','internal_rot','NM'};
Lookup.Right.KF = {'extension','flexion','NM'};
Lookup.Right.AF = {'flexion','extension','NM'};
Lookup.Right.AI = {'inversion','eversion','NM'};

sets = length(Phase.dataset); %%num datasets

for i=1:sets
    legs = length(fieldnames(Phase.dataset(i)));
    legs_name = fieldnames(Phase.dataset(i));
    for j=1:legs
        axs_name = fieldnames(Phase.dataset(i).(legs_name{j}));
        axs = length(fieldnames(Phase.dataset(i).(legs_name{j})))-1;
        temp_full = cell(axs,length(Phase.dataset(i).(legs_name{j}).(axs_name{axs})));
        for num_DOF = 1:axs
            temp=cell(length(Phase.dataset(i).(legs_name{j}).(axs_name{num_DOF})),1);
            for k=1:length(Phase.dataset(i).(legs_name{j}).(axs_name{num_DOF}))
                if Phase.dataset(i).(legs_name{j}).(axs_name{num_DOF})(k)==1
                    temp{k,1}= Lookup.(legs_name{j}).(axs_name{num_DOF}){1};
                    temp{k,2}=1;
                elseif Phase.dataset(i).(legs_name{j}).(axs_name{num_DOF})(k)==-1
                    temp{k,1}= Lookup.(legs_name{j}).(axs_name{num_DOF}){2};
                    temp{k,2}=-1;
                else
                    temp{k,1}= Lookup.(legs_name{j}).(axs_name{num_DOF}){3};
                    temp{k,2}=0;
                end
                temp_full{num_DOF,k} = temp{k,1};
            end
            phase_names.dataset(i).(legs_name{j}).(axs_name{num_DOF})=temp;
        end 
        phase_names.dataset(i).(legs_name{j}).full = temp_full;
    end  
end

end