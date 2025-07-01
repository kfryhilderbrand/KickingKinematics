%%%%%%%%%Metrics
%%Movement Sequence = Period of activity
%%Movement = Period of bilateral movement, period of unilateral movement,
%%etc.

%% Auxillary Data (only run first time)
%%% Segment joint rate data, calculate magnitudes of acceleration,
%%% determine phase sequences

clearvars
load('SegData.mat')
load('SegTruth.mat')
load('JointRates.mat')
ConstantsFlags

%%%% Segment joint rate data
SegRates = SegmentRateData(rates, Constants.SegLength, Constants.SegStep, Constants.fs);
save('SegRates.mat','SegRates');

%%%% Magnitudes of Acceleration (Saved in SegData)
for set=1:length(SegData.datasegment)
    legs_name = {'Left','Right'};
    seg_name = {'Foot','Shin','Thigh'};
    for leg=1:length(legs_name)
        for cnt_seg = 1:length(seg_name)
            SegData.datasegment(set).(legs_name{leg}).(seg_name{cnt_seg}).MagAccel = vecnorm(SegData.datasegment(set).(legs_name{leg}).(seg_name{cnt_seg}).Accel(:,2:4)')'; 
        end
    end
end
save('SegData.mat','SegData');

%%%% Define phase sequences for movements
phases = DeterminePhase(SegRates,5);   %Tune Thresh
phase_names = DeterminePhase_Names(phases);
save('PhaseData.mat','phases','phase_names');

%%% Clear Variables and Load in Data
clearvars 

load('SegData.mat')
load('SegTruth.mat')
%%Read in joint rate data
load('SegRates.mat')
%%Read in phase data
load('PhaseData.mat')
ConstantsFlags

%%% Creation of Movement Variable
%%% Determine: Movement Sequences (MVMT or NM), Coordination
%%% Patterns/Movements (NM, UL, UR, BI), Phases (6D Vector with states of
%%% individual DOFs), Kick Cycles (Full flexion followed by Full
%%% extension), Joint Excursions

%%%% Kick Cycles
        %%% Parse out Kick Cycles: full flexions and extensions
        %%% Full flexion if: 
        %%%      a) infant's foot movd at least 5 consecutive frames (10 ms per frame), i.e. move in space 
        %%%      b) either hip or knee joint excursion exceeded 11.5 deg. 
        %%%      c) hip and knee flexing (not necessarily ankle)
        %%% Full extension if:
        %%%      d) following full flexion

for set=1:length(SegData.datasegment)
    
%%%% Movement Sequences: 0 = NM, 1 = MVMT
    Movement.datasegment(set).MovementSequences = double(or(SegData.datasegment(set).Left.PredLabelsT, SegData.datasegment(set).Right.PredLabelsT));
    
    %%% Indeces of endpoints of movement sequences. 
    Movement.datasegment(set).MovementSequencesLogicalIndeces = abs(diff(Movement.datasegment(set).MovementSequences));
    Movement.datasegment(set).MovementSequencesLogicalIndeces(1) = 1;
    Movement.datasegment(set).MovementSequencesIndeces = find(Movement.datasegment(set).MovementSequencesLogicalIndeces==1);

%%%% Coordination Patterns (movements): 0 = NM, 1 = UL, 2 = UR, 3 = BI
    Movement.datasegment(set).Movements = SegData.datasegment(set).Left.PredLabelsT + 2*SegData.datasegment(set).Right.PredLabelsT;
   
    %%% Indeces of endpoints of movements. Movement goes from one endpoint
    %%% to the next endpoint-1
    Movement.datasegment(set).MovementsLogicalIndeces = abs(diff(Movement.datasegment(set).Movements));
    Movement.datasegment(set).MovementsLogicalIndeces(1) = 0;
    Movement.datasegment(set).MovementsIndeces = find(Movement.datasegment(set).MovementsLogicalIndeces~=0);

%%%% Phases: [startind,endind,phase_state,phase_name]
    Movement.datasegment(set).Phase_Names = phase_names.dataset(set);
    Movement.datasegment(set).Phase_States = phases.dataset(set);
    
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);   
    for leg=1:legs_num
        axs_name = fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg}));
        axs = length(fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg})))-1;
        temp_phase = zeros(length(Movement.datasegment(set).Phase_States.(legs_name{leg}).(axs_name{axs}))-1,1);
        temp_kickcycle = temp_phase;
        
        %%% Indeces of endpoints of phase changes.
        for num_DOF = 1:axs
            Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF})=abs(diff(Movement.datasegment(set).Phase_States.(legs_name{leg}).(axs_name{num_DOF})));
            Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF})(1) = 1;
            temp_phase = temp_phase + Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF});
            %%^ if any DOF is changing at a sample, then indicates a phase change
            %%| either 0 or some positive integer for these values.
            if strcmp(axs_name{num_DOF},'HF')||strcmp(axs_name{num_DOF},'KF')%strcmp(axs_name{num_DOF}(2),'F')
                temp_kickcycle = temp_kickcycle + Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF});
            end
        end
        
        %%% Indeces of endpoints of phases. Phases go from one endpoint
        %%% to the next endpoint-1
        Movement.datasegment(set).PhaseIndeces.(legs_name{leg}) = find(temp_phase~=0);
        
        %%% Indeces of endpoints of phases (just flexion axes). Phases go
        %%% from one endpoint to the next endpoint-1
        Movement.datasegment(set).PhaseFlexExtIndeces.(legs_name{leg}) = find(temp_kickcycle~=0);
        
        %%% Determine Phases [startind,endind,phase_state,phase_name].
        num_phs = length(Movement.datasegment(set).PhaseIndeces.(legs_name{leg}));
        Movement.datasegment(set).Phases.(legs_name{leg})=cell(1,4);%length(Movement.datasegment(i).PhaseIndeces.(legs_name{leg})),4);
        cnt_phs = 1;
        for phs=1:num_phs-1
            startind = Movement.datasegment(set).PhaseIndeces.(legs_name{leg})(phs)+1;
            endind = Movement.datasegment(set).PhaseIndeces.(legs_name{leg})(phs+1);
            phase_state = Movement.datasegment(set).Phase_States.(legs_name{leg}).full(:,startind);
            phase_name = Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(:,startind);
            if endind-startind>1%startind~=endind  %%Filter out noise in phases
                Movement.datasegment(set).Phases.(legs_name{leg})(cnt_phs,:)={startind, endind, phase_state,phase_name};
                cnt_phs=cnt_phs+1;
            end
        end
        startind = Movement.datasegment(set).PhaseIndeces.(legs_name{leg})(num_phs);
        [~,endind] = size(Movement.datasegment(set).Phase_States.(legs_name{leg}).full);
        phase_state = Movement.datasegment(set).Phase_States.(legs_name{leg}).full(:,startind);
        phase_name = Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(:,startind);
        Movement.datasegment(set).Phases.(legs_name{leg})(cnt_phs,:)=...
            {startind, endind, phase_state,phase_name};
        
        %%% Flexions and Extensions
        num_phs=length(Movement.datasegment(set).PhaseFlexExtIndeces.(legs_name{leg}));
        Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})=cell(1,4);
        cnt_phs = 1;
        for phs=1:num_phs-1
            startind = Movement.datasegment(set).PhaseFlexExtIndeces.(legs_name{leg})(phs)+1;
            endind = Movement.datasegment(set).PhaseFlexExtIndeces.(legs_name{leg})(phs+1);
            phase_state = [Movement.datasegment(set).Phase_States.(legs_name{leg}).full(1,startind);...
                Movement.datasegment(set).Phase_States.(legs_name{leg}).full(4,startind)];%;Movement.datasegment(set).Phase_States.(legs_name{leg}).full(5,startind)];
            phase_name = {Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(1,startind);...
                Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(4,startind)};%;Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(5,startind)};
            if endind-startind>1%startind~=endind  %%Filter out noise in phases
                Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})(cnt_phs,:)={startind, endind, phase_state,phase_name};
                cnt_phs=cnt_phs+1;
            end
        end
        startind = Movement.datasegment(set).PhaseFlexExtIndeces.(legs_name{leg})(num_phs);
        [~,endind] = size(Movement.datasegment(set).Phase_States.(legs_name{leg}).full);
        phase_state = [Movement.datasegment(set).Phase_States.(legs_name{leg}).full(1,startind);...
            Movement.datasegment(set).Phase_States.(legs_name{leg}).full(4,startind);Movement.datasegment(set).Phase_States.(legs_name{leg}).full(5,startind)];
        phase_name = {Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(1,startind);...
            Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(4,startind);Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(5,startind)};
        Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})(cnt_phs,:)={startind, endind, phase_state,phase_name};
    end
    
%%%% Kick Cycles
        %%% Parse out Kick Cycles: full flexions and extensions
        %%% Full flexion if: 
        %%%      a) infant's foot movd at least 5 consecutive frames (10 ms per frame), i.e. move in space 
        %%%      b) either hip or knee joint excursion exceeded 11.5 deg. 
        %%%      c) hip and knee flexing (not necessarily ankle)
        %%% Full extension if:
        %%%      d) following full flexion
    for leg=1:legs_num
        Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})=cell(1,4);
        Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg})=zeros(1,3);
        cnt_FullFlexExt = 1;
        [cnt_phs,~]=size(Movement.datasegment(set).Phases_FlexExt.(legs_name{leg}));
        for phs=1:cnt_phs
            startind = cell2mat(Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})(phs,1));
            endind = cell2mat(Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})(phs,2));
            
            %%% Condition a (movement is at least 50 ms)
            duration = (endind - startind)/Constants.fs;
            if duration >= 0.05
                
                %%% Condition c (hip and knee flexing or extending)
                temp_state = Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})(phs,4);
                if (strcmp(temp_state{1}{1},'flexion')&&strcmp(temp_state{1}{2},'flexion'))||(strcmp(temp_state{1}{1},'extension')&&strcmp(temp_state{1}{2},'extension'))
                    
                    %%% Condition b (hip or knee joint excursion exceeded 11.5 deg.)
                    Joint_exc = zeros(1,3);
                    cnt = 1;
                    for num_DOF = 1:axs
                        if strcmp(axs_name{num_DOF}(2),'F')
                            temp_exc=0;
                            for samp = 1:(endind-startind+1)
                                temp_exc = temp_exc+SegRates.dataset(set).(legs_name{leg}).(axs_name{num_DOF})(startind+samp-1)/Constants.fs;
                            end
                            Joint_exc(cnt) = temp_exc;
                            cnt = cnt + 1;
                        end
                    end
                    
                    if abs(Joint_exc(1))>=11.5||abs(Joint_exc(2))>=11.5
                        Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(cnt_FullFlexExt,:)=Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})(phs,:);
                        Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg})(cnt_FullFlexExt,:)=Joint_exc;
                        
                        cnt_FullFlexExt = cnt_FullFlexExt + 1;
                    end
                    
                end
            end
        end
        clearvars cnt_FullFlexExt cnt Joint_exc samp temp_exc temp_state duration startind endind
        
        %%% Condition d (full flexion followed by full extension)
        [cnt_kick, ~] = size(Movement.datasegment(set).Phases_KickCycle.(legs_name{leg}));
        Temp_KickCycle = Movement.datasegment(set).Phases_KickCycle.(legs_name{leg});
        Temp_JointExc =  Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg});
        temp_state = Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(1,4);
        if (~isempty(temp_state{1}))
            kick = 1;
            while ~(strcmp(temp_state{1}{1},'flexion'))&&(kick<=cnt_kick)&&(~isempty(Temp_KickCycle))
                Temp_KickCycle(1,:)=[];
                Temp_JointExc(1,:)=[];
                kick = kick + 1;
                if isempty(Temp_KickCycle)
                    Temp_KickCycle = cell(1,4);
                    break
                else
                    temp_state = Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(kick,4);
                end
            end
        end
        if isempty(Temp_KickCycle{1})
            Movement.datasegment(set).Phases_KickCycle.(legs_name{leg}) = {0,0,NaN,NaN};
            Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg}) = [NaN, NaN, NaN];%zeros(1,3);
            clearvars Temp_KickCycle Temp_JointExc cnt_kick temp_state kick
        else
            Movement.datasegment(set).Phases_KickCycle.(legs_name{leg}) = Temp_KickCycle;
            Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg}) = Temp_JointExc;
            clearvars Temp_KickCycle Temp_JointExc cnt_kick temp_state kick
            
            [cnt_kick, ~] = size(Movement.datasegment(set).Phases_KickCycle.(legs_name{leg}));
            Temp_KickCycle = [];%Movement.datasegment(set).Phases_KickCycle.(legs_name{leg});
            Temp_JointExc = [];
            for kick = 1:cnt_kick-1
                temp_state = Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(kick,4);
                temp_next = Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(kick+1,4);
                if (strcmp(temp_state{1}{1},'flexion'))&&(strcmp(temp_next{1}{1},'extension'))
                    Temp_KickCycle = [Temp_KickCycle;Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(kick,:);Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(kick+1,:)];
                    Temp_JointExc = [Temp_JointExc; Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg})(kick,:); Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg})(kick+1,:)];
                end
            end
            if isempty(Temp_KickCycle)
                Movement.datasegment(set).Phases_KickCycle.(legs_name{leg}) = {0,0,NaN,NaN};
                Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg}) = [NaN, NaN, NaN];%zeros(1,3);
                clearvars Temp_KickCycle Temp_JointExc cnt_kick temp_state kick
            else
                Movement.datasegment(set).Phases_KickCycle.(legs_name{leg}) = Temp_KickCycle;
                Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg}) = Temp_JointExc;
                clearvars Temp_KickCycle Temp_JointExc cnt_kick temp_state temp_next kick
            end
        end
    end
    
%%%% Joint Excursions (for each DOF, considered 0 at start of each
%%%% individual joint phase).
    for leg=1:legs_num
        for num_DOF = 1:axs
            Movement.datasegment(set).DOF_JointExc.(legs_name{leg}).(axs_name{num_DOF})=cell(1,3);
            Movement.datasegment(set).DOF_PhaseIndeces.(legs_name{leg}).(axs_name{num_DOF}) = find(Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF})~=0);
            num_phs = length(Movement.datasegment(set).DOF_PhaseIndeces.(legs_name{leg}).(axs_name{num_DOF}));
            cnt_phs = 1;
            for phs=2:num_phs-1%1:num_phs-1  %%clip first and last phase (because may be cut off)
                startind = Movement.datasegment(set).DOF_PhaseIndeces.(legs_name{leg}).(axs_name{num_DOF})(phs)+1;
                endind = Movement.datasegment(set).DOF_PhaseIndeces.(legs_name{leg}).(axs_name{num_DOF})(phs+1);
                if (Movement.datasegment(set).Phase_States.(legs_name{leg}).(axs_name{num_DOF})(startind)~=0)
                    temp_exc = 0;
                    for samp = 1:(endind-startind+1)
                        temp_exc = temp_exc+SegRates.dataset(set).(legs_name{leg}).(axs_name{num_DOF})(startind+samp-1)/Constants.fs;
                    end
                    Movement.datasegment(set).DOF_JointExc.(legs_name{leg}).(axs_name{num_DOF})(cnt_phs,:)={startind,endind,temp_exc};
                    cnt_phs = cnt_phs+1;
                end
            end
            %             startind = Movement.datasegment(set).DOF_PhaseIndeces.(legs_name{leg}).(axs_name{num_DOF})(end)+1;
            %             endind = length(Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF}));
            %             if (Movement.datasegment(set).Phase_States.(legs_name{leg}).(axs_name{num_DOF})(startind)~=0)
            %                 temp_exc = 0;
            %                 for samp = 1:(endind-startind+1)
            %                     temp_exc = temp_exc+SegRates.dataset(set).(legs_name{leg}).(axs_name{num_DOF})(startind+samp-1)/Constants.fs;
            %                 end
            %                 Movement.datasegment(set).DOF_JointExc.(legs_name{leg}).(axs_name{num_DOF})(cnt_phs,:)={startind,endind,temp_exc};
            %                 cnt_phs = cnt_phs+1;
            %             end
        end
    end
    
%%%% Inter-Joint Correlations (for each pair of flexion DOF)
    fields_flexCombo = {'HFKF','HFAF','KFAF'};
    for leg=1:legs_num
        [num_Kicks,~] = size(Movement.datasegment(set).Phases_KickCycle.(legs_name{leg}));
        for num_Combo = 1:length(fields_flexCombo)
            Movement.datasegment(set).InterCoor.(legs_name{leg}).(fields_flexCombo{num_Combo})=zeros(num_Kicks,1);
            for kick = 1:num_Kicks
                
                startind = Movement.datasegment(set).Phases_KickCycle.(legs_name{leg}){kick,1};
                endind = Movement.datasegment(set).Phases_KickCycle.(legs_name{leg}){kick,2};
                if ((startind~=0)&&(endind~=0))
                    time_vec = 0:1/Constants.fs:(endind-startind)/Constants.fs;
                    
                    HF_rates = SegRates.dataset(set).(legs_name{leg}).HF(startind:endind);
                    KF_rates = SegRates.dataset(set).(legs_name{leg}).KF(startind:endind);
                    AF_rates = SegRates.dataset(set).(legs_name{leg}).AF(startind:endind);
                    
                    HF_angles = cumtrapz(time_vec, HF_rates);
                    KF_angles = cumtrapz(time_vec, KF_rates);
                    AF_angles = cumtrapz(time_vec, AF_rates);
                    
                    if (num_Combo==1)
                        temp_Corr = xcorr(HF_angles,KF_angles,1,'normalized');
                    elseif (num_Combo==2)
                        temp_Corr = xcorr(HF_angles,AF_angles,1,'normalized');
                    else
                        temp_Corr = xcorr(KF_angles,AF_angles,1,'normalized');
                    end
                    Movement.datasegment(set).InterCoor.(legs_name{leg}).(fields_flexCombo{num_Combo})(kick) = temp_Corr(2);
                else
                    Movement.datasegment(set).InterCoor.(legs_name{leg}).(fields_flexCombo{num_Combo})(kick) = NaN;
                end
            end
        end
    end
end

fields = {'DOFPhaseLogicalIndeces','PhaseIndeces','PhaseFlexExtIndeces','DOF_PhaseIndeces',...
    'Phases_FlexExt','MovementSequencesLogicalIndeces','MovementsLogicalIndeces'};
Movement.datasegment=rmfield(Movement.datasegment,fields);
clearvars -except SegData SegRates SegTruth Movement Constants 
%%% Calculation of Metrics
%%% Frequencies, Durations, Accelerations, Joint Rates, Joint Excursions,
%%% Kick Amplitude, Inter-Joint Coordination

%%%%%%%%%%%%%Frequencies:
for set=1:length(SegData.datasegment)
%%%%Movement Frequency: how long is the baby active within a datasegment
    Metrics.Frequencies.MovementFrequency(set,1) = ...
        sum(Movement.datasegment(set).MovementSequences)/length(Movement.datasegment(set).MovementSequences);
    
%%%%Individual Movement Frequency: how long is baby performing bilateral, 
%%%%UL, UR, NM
    leg_seg = {'NM','UL','UR','BI'};
    for cnt_field = 1:length(leg_seg)
        Metrics.Frequencies.(append(leg_seg{cnt_field},'Frequency'))(set,1) = ...
            sum(Movement.datasegment(set).Movements(:) == (cnt_field - 1))/length(Movement.datasegment(set).Movements);
    end
end
Frequency_Metrics = struct2table(Metrics.Frequencies);

%%%%%%%%%%%%%Durations: 
for set=1:length(SegData.datasegment)
%%%%Kicking Duration: Duration of each movement sequence
    %Initialize variables
    Metrics.Durations.KickingDuration(1) = 0;
    
    if (~isempty(Movement.datasegment(set).MovementSequencesIndeces))
        cnt_k = 1;
        for j=2:length(Movement.datasegment(set).MovementSequencesIndeces)
            startind = Movement.datasegment(set).MovementSequencesIndeces(j-1)+1;
            endind = Movement.datasegment(set).MovementSequencesIndeces(j);
            if Movement.datasegment(set).MovementSequences(startind)==1
                Metrics.Durations.datasegment(set).KickingDuration(cnt_k) = ...
                    SegData.datasegment(set).Left.Time(endind)-SegData.datasegment(set).Left.Time(startind);
                cnt_k = cnt_k + 1;
            end
        end
        clear cnt_k
    end
    
%%%%Movement Duration: Duration of each movement
    %Initialize variables
    leg_seg = {'NM','UL','UR','BI'};
    for cnt_fields=1:length(leg_seg)
        Metrics.Durations.datasegment(set).(append('MovementDuration',leg_seg{cnt_fields}))(1) = 0;
    end
    
    nm=1; ul=1; ur=1; bi=1;
    if (~isempty(Movement.datasegment(set).MovementsIndeces))
        for j=2:length(Movement.datasegment(set).MovementsIndeces)
            startind = Movement.datasegment(set).MovementsIndeces(j-1)+1;
            endind = Movement.datasegment(set).MovementsIndeces(j);
            if Movement.datasegment(set).Movements(startind)==0
                Metrics.Durations.datasegment(set).MovementDurationNM(nm) = ...
                    SegData.datasegment(set).Left.Time(endind)-SegData.datasegment(set).Left.Time(startind);
                nm = nm + 1;
            elseif Movement.datasegment(set).Movements(startind)==1
                Metrics.Durations.datasegment(set).MovementDurationUL(ul) = ...
                    SegData.datasegment(set).Left.Time(endind)-SegData.datasegment(set).Left.Time(startind);
                ul = ul + 1;
            elseif Movement.datasegment(set).Movements(startind)==2
                Metrics.Durations.datasegment(set).MovementDurationUR(ur) = ...
                    SegData.datasegment(set).Left.Time(endind)-SegData.datasegment(set).Left.Time(startind);
                ur = ur + 1;
            else
                Metrics.Durations.datasegment(set).MovementDurationBI(bi) = ...
                    SegData.datasegment(set).Left.Time(endind)-SegData.datasegment(set).Left.Time(startind);
                bi = bi + 1;
            end
        end
    end
    clear nm ul ur bi

%%%%%% Overall Durations
    leg_seg = {'KickingDuration','MovementDurationNM','MovementDurationUL',...
        'MovementDurationUR','MovementDurationBI'};
    for cnt_field = 1:length(leg_seg)
        %%%%Average Durations
        Metrics.Overall_Dur.(append('Avg',leg_seg{cnt_field}))(set,1) = ...
            mean(Metrics.Durations.datasegment(set).(leg_seg{cnt_field}));
    end
    for cnt_field = 1:length(leg_seg)
        %%%%Max Durations
        if(isempty(Metrics.Durations.datasegment(set).(leg_seg{cnt_field})))
            Metrics.Overall_Dur.(append('Max',leg_seg{cnt_field}))(set,1) = 0;
        else
            Metrics.Overall_Dur.(append('Max',leg_seg{cnt_field}))(set,1) = ...
                max(Metrics.Durations.datasegment(set).(leg_seg{cnt_field}));
        end
    end
end
Duration_Metrics = struct2table(Metrics.Overall_Dur);

%%%%%%%%%%%%%Accelerations: 
for set=1:length(SegData.datasegment)
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);
    seg_name = {'Foot','Shin','Thigh'};
    leg_seg = {'LF','LS','LT','RF','RS','RT'};
    coor_name = {'U','BI'};

%%%%Average Kicking Accelerations: Average acceleration of each movement sequence
    %Initialize variables
    for cnt_fields = 1:length(leg_seg)
        Metrics.Accelerations.datasegment(set).(append('AvgKickingAcc',leg_seg{cnt_fields}))(1) = 0;
    end
     
    for leg = 1:legs_num
        cnt_k=1;
        for j=2:length(Movement.datasegment(set).MovementSequencesIndeces)
            startind = Movement.datasegment(set).MovementSequencesIndeces(j-1)+1;
            endind = Movement.datasegment(set).MovementSequencesIndeces(j);
            if Movement.datasegment(set).MovementSequences(startind)==1
                Metrics.Accelerations.datasegment(set).(append('AvgKickingAcc',legs_name{leg}(1),seg_name{1}(1)))(cnt_k) = ...
                    mean(SegData.datasegment(set).(legs_name{leg}).(seg_name{1}).MagAccel(startind:endind));
                Metrics.Accelerations.datasegment(set).(append('AvgKickingAcc',legs_name{leg}(1),seg_name{2}(1)))(cnt_k) = ...
                    mean(SegData.datasegment(set).(legs_name{leg}).(seg_name{2}).MagAccel(startind:endind));
                Metrics.Accelerations.datasegment(set).(append('AvgKickingAcc',legs_name{leg}(1),seg_name{3}(1)))(cnt_k) = ...
                    mean(SegData.datasegment(set).(legs_name{leg}).(seg_name{3}).MagAccel(startind:endind));
                cnt_k = cnt_k + 1;
            end
            clear startind endind
        end
    end
    clear cnt_k
    
%%%%Peak Kicking Accelerations: Peak acceleration of each movement sequence
    %Initialize variables
    for cnt_fields = 1:length(leg_seg)
        Metrics.Accelerations.datasegment(set).(append('PeakKickingAcc',leg_seg{cnt_fields}))(1) = 0;
    end
     
    for leg = 1:legs_num
        cnt_k=1;
        for j=2:length(Movement.datasegment(set).MovementSequencesIndeces)
            startind = Movement.datasegment(set).MovementSequencesIndeces(j-1)+1;
            endind = Movement.datasegment(set).MovementSequencesIndeces(j);
            if Movement.datasegment(set).MovementSequences(startind)==1
                Metrics.Accelerations.datasegment(set).(append('PeakKickingAcc',legs_name{leg}(1),seg_name{1}(1)))(cnt_k) = ...
                    max(SegData.datasegment(set).(legs_name{leg}).(seg_name{1}).MagAccel(startind:endind));
                Metrics.Accelerations.datasegment(set).(append('PeakKickingAcc',legs_name{leg}(1),seg_name{2}(1)))(cnt_k) = ...
                    max(SegData.datasegment(set).(legs_name{leg}).(seg_name{2}).MagAccel(startind:endind));
                Metrics.Accelerations.datasegment(set).(append('PeakKickingAcc',legs_name{leg}(1),seg_name{3}(1)))(cnt_k) = ...
                    max(SegData.datasegment(set).(legs_name{leg}).(seg_name{3}).MagAccel(startind:endind));
                cnt_k = cnt_k + 1;
            end
            clear startind endind
        end
    end
    clear cnt_k
    
%%%%Average Movement Accelerations: Average acceleration of each movement
    ul=1; ur=1; bi=1;
    %Initialize variables
    for cnt_fields = 1:length(leg_seg)
        Metrics.Accelerations.datasegment(set).(append('AvgAcc',coor_name{1},leg_seg{cnt_fields}(1),'_',leg_seg{cnt_fields}))(1) = 0;
        Metrics.Accelerations.datasegment(set).(append('AvgAcc',coor_name{2},'_',leg_seg{cnt_fields}))(1) = 0;        
    end

    for j=2:length(Movement.datasegment(set).MovementsIndeces)
        startind = Movement.datasegment(set).MovementsIndeces(j-1)+1;
        endind = Movement.datasegment(set).MovementsIndeces(j);
        if Movement.datasegment(set).Movements(startind)==1
            for cnt_fields = 1:length(seg_name)
                Metrics.Accelerations.datasegment(set).(append('AvgAccUL_L',seg_name{cnt_fields}(1)))(ul) = ...
                    mean(SegData.datasegment(set).Left.(seg_name{cnt_fields}).MagAccel(startind:endind));
            end
            ul = ul + 1;
        
        elseif Movement.datasegment(set).Movements(startind)==2
            for cnt_fields = 1:length(seg_name)
                Metrics.Accelerations.datasegment(set).(append('AvgAccUR_R',seg_name{cnt_fields}(1)))(ur) = ...
                    mean(SegData.datasegment(set).Right.(seg_name{cnt_fields}).MagAccel(startind:endind));
            end
            ur = ur + 1;
        
        elseif Movement.datasegment(set).Movements(startind)==3
            for cnt_fields = 1:length(seg_name)
                Metrics.Accelerations.datasegment(set).(append('AvgAccBI_L',seg_name{cnt_fields}(1)))(bi) = ...
                    mean(SegData.datasegment(set).Left.(seg_name{cnt_fields}).MagAccel(startind:endind));
                Metrics.Accelerations.datasegment(set).(append('AvgAccBI_R',seg_name{cnt_fields}(1)))(bi) = ...
                    mean(SegData.datasegment(set).Right.(seg_name{cnt_fields}).MagAccel(startind:endind));
            end
            bi = bi + 1;            
        end
        clear startind endind
    end
    
%%%%Peak Movement Accelerations: Peak acceleration of each movement
    ul=1; ur=1; bi=1;
    %Initialize variables
    for cnt_fields = 1:length(leg_seg)
        Metrics.Accelerations.datasegment(set).(append('PeakAcc',coor_name{1},leg_seg{cnt_fields}(1),'_',leg_seg{cnt_fields}))(1) = 0;
        Metrics.Accelerations.datasegment(set).(append('PeakAcc',coor_name{2},'_',leg_seg{cnt_fields}))(1) = 0;
    end
    
    for j=2:length(Movement.datasegment(set).MovementsIndeces)
        startind = Movement.datasegment(set).MovementsIndeces(j-1)+1;
        endind = Movement.datasegment(set).MovementsIndeces(j);
        if Movement.datasegment(set).Movements(startind)==1
            for cnt_fields = 1:length(seg_name)
                Metrics.Accelerations.datasegment(set).(append('PeakAccUL_L',seg_name{cnt_fields}(1)))(ul) = ...
                    max(SegData.datasegment(set).Left.(seg_name{cnt_fields}).MagAccel(startind:endind));
            end
            ul = ul + 1;
            
        elseif Movement.datasegment(set).Movements(startind)==2
            for cnt_fields = 1:length(seg_name)
                Metrics.Accelerations.datasegment(set).(append('PeakAccUR_R',seg_name{cnt_fields}(1)))(ur) = ...
                    max(SegData.datasegment(set).Right.(seg_name{cnt_fields}).MagAccel(startind:endind));
            end
            ur = ur + 1;
            
        elseif Movement.datasegment(set).Movements(startind)==3
            for cnt_fields = 1:length(seg_name)
                Metrics.Accelerations.datasegment(set).(append('PeakAccBI_L',seg_name{cnt_fields}(1)))(bi) = ...
                    max(SegData.datasegment(set).Left.(seg_name{cnt_fields}).MagAccel(startind:endind));
                Metrics.Accelerations.datasegment(set).(append('PeakAccBI_R',seg_name{cnt_fields}(1)))(bi) = ...
                    max(SegData.datasegment(set).Right.(seg_name{cnt_fields}).MagAccel(startind:endind));
            end
            bi = bi + 1;
        end
        clear startind endind
    end
    clear ul ur bi j
    
%%%%Overall Avg Kicking Accelerations: Overall Average Acceleration of movement sequences
    for cnt_fields = 1:length(leg_seg)
        Metrics.Overall_Accel.(append('AvgKickingAcc',leg_seg{cnt_fields}))(set,1) = ...
            mean(Metrics.Accelerations.datasegment(set).(append('AvgKickingAcc',leg_seg{cnt_fields})));
    end

%%%%Overall Peak Kicking Accelerations: Peak acceleration of movement sequences
    for cnt_fields = 1:length(leg_seg)
        Metrics.Overall_Accel.(append('PeakKickingAcc',leg_seg{cnt_fields}))(set,1) = ...
            max(Metrics.Accelerations.datasegment(set).(append('PeakKickingAcc',leg_seg{cnt_fields})));
    end

%%%%Overall Average movement Accelerations: Average Acceleration of movements
    for cnt_fields = 1:length(leg_seg)
        Metrics.Overall_Accel.(append('AvgAcc',coor_name{1},leg_seg{cnt_fields}(1),'_',leg_seg{cnt_fields}))(set,1) = ...
            mean(Metrics.Accelerations.datasegment(set).(append('AvgAcc',coor_name{1},leg_seg{cnt_fields}(1),'_',leg_seg{cnt_fields})));
    end
    for cnt_fields = 1:length(leg_seg)
        Metrics.Overall_Accel.(append('AvgAcc',coor_name{2},'_',leg_seg{cnt_fields}))(set,1) = ...
            mean(Metrics.Accelerations.datasegment(set).(append('AvgAcc',coor_name{2},'_',leg_seg{cnt_fields})));        
    end
      
%%%%Overall Peak movement accelerations: peak acceleration of each movement 
    for cnt_fields = 1:length(leg_seg)
        Metrics.Overall_Accel.(append('PeakAcc',coor_name{1},leg_seg{cnt_fields}(1),'_',leg_seg{cnt_fields}))(set,1) = ...
            max(Metrics.Accelerations.datasegment(set).(append('PeakAcc',coor_name{1},leg_seg{cnt_fields}(1),'_',leg_seg{cnt_fields})));
    end
    for cnt_fields = 1:length(leg_seg)
        Metrics.Overall_Accel.(append('PeakAcc',coor_name{2},'_',leg_seg{cnt_fields}))(set,1) = ...
            max(Metrics.Accelerations.datasegment(set).(append('PeakAcc',coor_name{2},'_',leg_seg{cnt_fields})));        
    end
end
Acceleration_Metrics = struct2table(Metrics.Overall_Accel);

%%%%%%%%%%%%%Joint Rates:
for set=1:length(SegData.datasegment)    
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);
    
%%%%Max Joint Rates: Maximum joint rate for each DOF for all time (+/-
%%%%direction) [max/min of joint rate data for each DOF]
    for leg=1:legs_num
        axs_name = fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg}));
        axs = length(fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg})))-1;
%     %Initialize variables:
%         for num_DOF = 1:axs
%             pos_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(1);
%             neg_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(2);
%             Metrics.Joint_Rates.(append('MaxJointRate','_',legs_name{leg}(1),axs_name{num_DOF},'_',pos_dir_name{1}))= 0;
%             Metrics.Joint_Rates.(append('MaxJointRate','_',legs_name{leg}(1),axs_name{num_DOF},'_',neg_dir_name{1}))= 0;
%         end
        for num_DOF = 1:axs
            pos_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(1);
            neg_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(2);

            temp = cell2mat(Movement.datasegment(set).Phase_Names.(legs_name{leg}).(axs_name{num_DOF})(:,2));
            pos_ind=find(temp==1, 1);
            neg_ind=find(temp==-1, 1);
            if ~isempty(pos_ind)
                Metrics.Joint_Rates.(append('MaxJointRate','_',legs_name{leg}(1),axs_name{num_DOF},'_',pos_dir_name{1}))(set,1)= ...
                    max(SegRates.dataset(set).(legs_name{leg}).(axs_name{num_DOF}));
            end
            if ~isempty(neg_ind)
                Metrics.Joint_Rates.(append('MaxJointRate','_',legs_name{leg}(1),axs_name{num_DOF},'_',neg_dir_name{1}))(set,1)= ...
                    min(SegRates.dataset(set).(legs_name{leg}).(axs_name{num_DOF}));
            end
        end
    end
end
Joint_Rate_Metrics = struct2table(Metrics.Joint_Rates);

%%%%%%%%%%%%%Joint Excursions:
for set=1:length(SegData.datasegment)
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);
    
%%%%Average Joint Excursion: The average excursion for each DOF during 
%%%%individual joint phases (+/- direction)
    for leg=1:legs_num
        axs_name = fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg}));
        axs = length(fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg})))-1;
        for num_DOF = 1:axs
            pos_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(1);
            neg_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(2);
            
            temp = cell2mat(Movement.datasegment(set).DOF_JointExc.(legs_name{leg}).(axs_name{num_DOF})(:,3));
            pos_ind = find(temp>0);
            neg_ind = find(temp<0);
            if ~isempty(pos_ind)
                Metrics.Joint_Excs.(append('AvgJointExc','_',legs_name{leg}(1),axs_name{num_DOF},'_',pos_dir_name{1}))(set,1)= mean(temp(pos_ind));
            end
            if ~isempty(neg_ind)
                Metrics.Joint_Excs.(append('AvgJointExc','_',legs_name{leg}(1),axs_name{num_DOF},'_',neg_dir_name{1}))(set,1)= mean(temp(neg_ind));
            end
        end
    end
    
%%%%Maximum Joint Excursion: The maximum excursion for each DOF during 
%%%%individual joint phases (+/- direction)
    for leg=1:legs_num
        axs_name = fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg}));
        axs = length(fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg})))-1;
        for num_DOF = 1:axs
            pos_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(1);
            neg_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(2);
            
            temp = cell2mat(Movement.datasegment(set).DOF_JointExc.(legs_name{leg}).(axs_name{num_DOF})(:,3));
            pos_ind = find(temp>0);
            neg_ind = find(temp<0);
            if ~isempty(pos_ind)
                Metrics.Joint_Excs.(append('MaxJointExc','_',legs_name{leg}(1),axs_name{num_DOF},'_',pos_dir_name{1}))(set,1)= max(temp(pos_ind));
            end
            if ~isempty(neg_ind)
                Metrics.Joint_Excs.(append('MaxJointExc','_',legs_name{leg}(1),axs_name{num_DOF},'_',neg_dir_name{1}))(set,1)= min(temp(neg_ind));
            end
        end
    end
end
Joint_Exc_Metrics = struct2table(Metrics.Joint_Excs);

%%%%%%%%%%%%%Kick Amplitude:
fields_flex = {'HF','KF','AF'};
for set=1:length(SegData.datasegment)
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);
    
%%%%Average Kick Amplitude: The average excursion during the full flexion
%%%%and full extension phase of a kick cycle for each flexion DOF (+/- direction)
    for leg=1:legs_num
        %Gather flexion joint excursions into array (rows 1, 3, etc.)
        temp_flex = Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg})(1:2:end,:);
        %Gather extension joint excursions into array (rows 2, 4, etc.)
        temp_ext = Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg})(2:2:end,:);
        [num_kicks,~] = size(temp_flex);
        
        for cnt_axs = 1:length(fields_flex)
            if ~isempty(temp_flex)
                Metrics.Kick_Amps.(append('AvgKickAmp_',legs_name{leg}(1),'Flex_',fields_flex{cnt_axs}))(set,1) = ...
                    mean(temp_flex(:,cnt_axs));
            else
                Metrics.Kick_Amps.(append('AvgKickAmp_',legs_name{leg}(1),'Flex_',fields_flex{cnt_axs}))(set,1) = NaN;
            end
            if ~isempty(temp_ext)
                Metrics.Kick_Amps.(append('AvgKickAmp_',legs_name{leg}(1),'Ext_',fields_flex{cnt_axs}))(set,1) = ...
                    mean(temp_ext(:,cnt_axs));
            else
                Metrics.Kick_Amps.(append('AvgKickAmp_',legs_name{leg}(1),'Ext_',fields_flex{cnt_axs}))(set,1) = NaN;
            end
        end
    end
%%%%Maximum Kick Amplitude: The maximum excursion during the full flexion
%%%%and full extension phase of a kick cycle for each flexion DOF (+/- direction)
    for leg=1:legs_num
        %Gather flexion joint excursions into array (rows 1, 3, etc.)
        temp_flex = Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg})(1:2:end,:);
        %Gather extension joint excursions into array (rows 2, 4, etc.)
        temp_ext = Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg})(2:2:end,:);
        [num_kicks,~] = size(temp_flex);
        
        for cnt_axs = 1:length(fields_flex)
            if ~isempty(temp_flex)
                Metrics.Kick_Amps.(append('MaxKickAmp_',legs_name{leg}(1),'Flex_',fields_flex{cnt_axs}))(set,1) = ...
                    max(temp_flex(:,cnt_axs));
            else
                Metrics.Kick_Amps.(append('MaxKickAmp_',legs_name{leg}(1),'Flex_',fields_flex{cnt_axs}))(set,1) = NaN;
            end
            if ~isempty(temp_ext)
                Metrics.Kick_Amps.(append('MaxKickAmp_',legs_name{leg}(1),'Ext_',fields_flex{cnt_axs}))(set,1) = ...
                    max(temp_ext(:,cnt_axs));
            else
                Metrics.Kick_Amps.(append('MaxKickAmp_',legs_name{leg}(1),'Ext_',fields_flex{cnt_axs}))(set,1) = NaN;
            end
        end
    end
end
Kick_Amp_Metrics = struct2table(Metrics.Kick_Amps);

%%%%%%%%%%%%%Inter-Joint Correlation:
fields_flexCombo = {'HFKF','HFAF','KFAF'};
for set=1:length(SegData.datasegment)
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);
    for leg=1:legs_num
        for num_Combo = 1:length(fields_flexCombo) %%Inphase and out phase for AF?
            idx = Movement.datasegment(set).InterCoor.(legs_name{leg}).(fields_flexCombo{num_Combo})<0;
            neg = Movement.datasegment(set).InterCoor.(legs_name{leg}).(fields_flexCombo{num_Combo})(idx);
            pos = Movement.datasegment(set).InterCoor.(legs_name{leg}).(fields_flexCombo{num_Combo})(~idx);
            
            if (num_Combo == 1) %%HFKF
                Metrics.InterJoint.(append('AvgInterJoint_',legs_name{leg}(1),fields_flexCombo{num_Combo},'_inPhase'))(set,1) = ...
                    mean(neg); %%HF and KF in phase when corr is neg
            elseif (num_Combo == 2) %%HFAF
                Metrics.InterJoint.(append('AvgInterJoint_',legs_name{leg}(1),fields_flexCombo{num_Combo},'_inPhase'))(set,1) = ...
                    mean(pos); %%HF and AF in phase when corr is pos
                Metrics.InterJoint.(append('AvgInterJoint_',legs_name{leg}(1),fields_flexCombo{num_Combo},'_outPhase'))(set,1) = ...
                    mean(neg);
            else
                Metrics.InterJoint.(append('AvgInterJoint_',legs_name{leg}(1),fields_flexCombo{num_Combo},'_inPhase'))(set,1) = ...
                    mean(neg); %KF and AF in phase when coor is neg
                Metrics.InterJoint.(append('AvgInterJoint_',legs_name{leg}(1),fields_flexCombo{num_Combo},'_outPhase'))(set,1) = ...
                    mean(pos);                
            end
        end
    end
end
Inter_Joint_Metrics = struct2table(Metrics.InterJoint);

clearvars -except SegData Metrics Movement Frequency_Metrics Duration_Metrics Acceleration_Metrics Joint_Rate_Metrics Joint_Exc_Metrics Kick_Amp_Metrics Inter_Joint_Metrics
%%%

save('Metrics.mat','Metrics','Movement')



%% Joint Rate Metrics
%%%Inter-joint coordination: pair-wise cross-correlations of joint angles at the  hip, knee and ankle. 
%%%-These correlations tend to be high in the newborn period and decrease as the infant ages. 
%%%-This metric is typically calculated during a kick cycle.

%%%Joint Rates and Joint Excursions:

%%%Max Joint Rates: Maximum joint rate for each DOF for all time (positive
%%%and negative direction): max/min of joint rate data for each DOF

%%%Max/Avg Joint Excursion: For each DOF, the maximum and average excursion
%%%during individual joint phases (positive and negative direction): look
%%%at joint excursion for each change of individual joint phase (i.e. phase
%%%of HF, etc.) then take maximum and average of these values.

%%%Max/Avg Kick Amplitude: For each flexion DOF, the maximum and average
%%%excursion during the full flexion and full extension phase of a kick
%%%cycle(positive and negative direction): look at full flexions and full
%%%extensions and find max and average excursions for each direction



%%%Initialize Variables
for set=1:length(SegData.datasegment)
    %%%%%%%%%%%%%Joint Excursions: 

    %%%%Maximum Joint Excursion: Maximum excursion of a joint for all
    %%%%movements (Positive and Negative)
    %%%%Kick Amplitude: Maximum excursion of hip joint during
    %%%%flexion/extension during a kick cycle (flexion followed by
    %%%%extension)(increase with age)
      
    %Initialize variables: Kick Amplitude,
    %Inter-joint Coordination
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);
    for leg=1:legs_num
        axs_name = fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg}));
        axs = length(fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg})))-1;
        %%%Max Joint Excursion
        for num_DOF = 1:axs
            pos_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(1);
            neg_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(2);
            Metrics.Joint_Excs.(append('MaxJointExc','_',legs_name{leg}(1),axs_name{num_DOF},'_',pos_dir_name{1}))= 0;
            Metrics.Joint_Excs.(append('MaxJointExc','_',legs_name{leg}(1),axs_name{num_DOF},'_',neg_dir_name{1}))= 0;
        end
        %%%Avg Joint Excursion
        for num_DOF = 1:axs
            pos_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(1);
            neg_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(2);
            Metrics.Joint_Excs.(append('AvgJointExc','_',legs_name{leg}(1),axs_name{num_DOF},'_',pos_dir_name{1}))= 0;
            Metrics.Joint_Excs.(append('AvgJointExc','_',legs_name{leg}(1),axs_name{num_DOF},'_',neg_dir_name{1}))= 0;
        end
        %%%Max Joint Rate
        for num_DOF = 1:axs
            pos_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(1);
            neg_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(2);
            Metrics.Joint_Rates.(append('MaxJointRate','_',legs_name{leg}(1),axs_name{num_DOF},'_',pos_dir_name{1}))= 0;
            Metrics.Joint_Rates.(append('MaxJointRate','_',legs_name{leg}(1),axs_name{num_DOF},'_',neg_dir_name{1}))= 0;
        end
    end
    
    for leg=1:legs_num
        %%%Max/Avg Kick Amplitude (flexion/extension excursion) of Hip joint during kick cycle(full flexion
        %%%followed by full extension)
        Metrics.Kick_Amp.(append('MaxKickAmp','_',legs_name{leg}(1),'HF')) = 0;
        Metrics.Kick_Amp.(append('AvgKickAmp','_',legs_name{leg}(1),'HF')) = 0;
        Metrics.Kick_Amp.(append('MaxKickAmp','_',legs_name{leg}(1),'HE')) = 0;
        Metrics.Kick_Amp.(append('AvgKickAmp','_',legs_name{leg}(1),'HE')) = 0;
        
        %%%Inter-joint coordination:
        %%%pair-wise cross-correlations of joint angles at the hip, knee and ankle during kick cycle.
        Metrics.InterJointCord.(legs_name{leg}).HipKnee = 0;
        Metrics.InterJointCord.(legs_name{leg}).HipAnkle = 0;
        Metrics.InterJointCord.(legs_name{leg}).KneeAnkle = 0;
    end
end

for set=1:length(SegData.datasegment)
    %%%%%%%%%%%%%Joint Rates and Joint Excursions:
    %%%Maximum Joint Rate of a joint for all movements(positive and negative)
    %%%Maximum and average joint excursions for all movements (positive and
    %%%negative)
    for leg=1:legs_num
        axs_name = fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg}));
        axs = length(fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg})))-1;
        for num_DOF = 1:axs
            pos_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(1);
            neg_dir_name = Constants.Lookup.(legs_name{leg}).(axs_name{num_DOF})(2);
            
            %%% Max joint rate
            temp = cell2mat(Movement.datasegment(set).Phase_Names.(legs_name{leg}).(axs_name{num_DOF})(:,2));
            pos_ind=find(temp==1, 1);%%%
            neg_ind=find(temp==-1, 1);%%%
            if ~isempty(pos_ind)
                Metrics.Joint_Rates.(append('MaxJointRate','_',legs_name{leg}(1),axs_name{num_DOF},'_',pos_dir_name{1}))(set,1)= max(SegRates.dataset(set).(legs_name{leg}).(axs_name{num_DOF}));
            end
            if ~isempty(neg_ind)
                Metrics.Joint_Rates.(append('MaxJointRate','_',legs_name{leg}(1),axs_name{num_DOF},'_',neg_dir_name{1}))(set,1)= min(SegRates.dataset(set).(legs_name{leg}).(axs_name{num_DOF}));
            end
            
            %%% Max and average joint excursion
            temp = cell2mat(Movement.datasegment(set).DOF_JointExc.(legs_name{leg}).(axs_name{num_DOF})(:,3));
            pos_ind = find(temp>0);
            neg_ind = find(temp<0);
            if ~isempty(pos_ind)
                Metrics.Joint_Excs.(append('MaxJointExc','_',legs_name{leg}(1),axs_name{num_DOF},'_',pos_dir_name{1}))(set,1)= max(temp(pos_ind));
                Metrics.Joint_Excs.(append('AvgJointExc','_',legs_name{leg}(1),axs_name{num_DOF},'_',pos_dir_name{1}))(set,1)= mean(temp(pos_ind));
            end
            if ~isempty(neg_ind)
                Metrics.Joint_Excs.(append('MaxJointExc','_',legs_name{leg}(1),axs_name{num_DOF},'_',neg_dir_name{1}))(set,1)= min(temp(neg_ind));
                Metrics.Joint_Excs.(append('AvgJointExc','_',legs_name{leg}(1),axs_name{num_DOF},'_',neg_dir_name{1}))(set,1)= mean(temp(neg_ind));
            end
        end
    end
end



%             pos_dir_name = Movement.datasegment(set).Phase_Names.(legs_name{leg}).(axs_name{num_DOF})(find(temp==1,1),1);
%             neg_dir_name = Movement.datasegment(set).Phase_Names.(legs_name{leg}).(axs_name{num_DOF})(find(temp==-1,1),1);
%            Metrics.datasegment(set).MaxJointExc.(legs_name{leg}).(axs_name{num_DOF}).pos_dir_name = {pos_dir_name};
           % Metrics.datasegment(set).MaxJointExc.(legs_name{leg}).(axs_name{num_DOF}).(pos_dir_name) = .01;
           % Metrics.datasegment(set).MaxJointExc.(legs_name{leg}).(axs_name{num_DOF}).neg_dir_name = {neg_dir_name};
           % Metrics.datasegment(set).MaxJointExc.(legs_name{leg}).(axs_name{num_DOF}).(neg_dir_name) = {neg_dir_name};
%%
for set=1:length(SegData.datasegment)
    
    %%% Phases
    Movement.datasegment(set).Phase_Names = phase_names.dataset(set);
    Movement.datasegment(set).Phase_States = phases.dataset(set);
    
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);
    
    for leg=1:legs_num
        axs_name = fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg}));
        axs = length(fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg})))-1;
        temp_phase = zeros(length(Movement.datasegment(set).Phase_States.(legs_name{leg}).(axs_name{axs}))-1,1);
        temp_kickcycle = temp_phase;
        %%% Indeces of endpoints of phase changes.
        for num_DOF = 1:axs
            Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF})=abs(diff(Movement.datasegment(set).Phase_States.(legs_name{leg}).(axs_name{num_DOF})));
            Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF})(1) = 1;
            temp_phase = temp_phase + Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF});
            %%^ if any DOF is changing at a sample, then indicates a phase change
            %%| either 0 or some positive integer for these values.
            if strcmp(axs_name{num_DOF},'HF')||strcmp(axs_name{num_DOF},'KF')%strcmp(axs_name{num_DOF}(2),'F')
                temp_kickcycle = temp_kickcycle + Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF});
            end
        end
        %%% Indeces of endpoints of phases. Phases go from one endpoint
        %%% to the next endpoint-1
        Movement.datasegment(set).PhaseIndeces.(legs_name{leg}) = find(temp_phase~=0);
        
        %%% Indeces of endpoints of phases (just flexion axes). Phases go
        %%% from one endpoint to the next endpoint-1
        Movement.datasegment(set).PhaseFlexExtIndeces.(legs_name{leg}) = find(temp_kickcycle~=0);
        
        %%%Create variable for what phases are happening when [startind,
        %%%endind,phase_state,phase_name].
        num_phs = length(Movement.datasegment(set).PhaseIndeces.(legs_name{leg}));
        Movement.datasegment(set).Phases.(legs_name{leg})=cell(1,4);%length(Movement.datasegment(i).PhaseIndeces.(legs_name{leg})),4);
        cnt_phs = 1;
        for phs=1:num_phs-1
            startind = Movement.datasegment(set).PhaseIndeces.(legs_name{leg})(phs)+1;
            endind = Movement.datasegment(set).PhaseIndeces.(legs_name{leg})(phs+1);
            phase_state = Movement.datasegment(set).Phase_States.(legs_name{leg}).full(:,startind);
            phase_name = Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(:,startind);
            if endind-startind>1%startind~=endind  %%Filter out noise in phases
                Movement.datasegment(set).Phases.(legs_name{leg})(cnt_phs,:)={startind, endind, phase_state,phase_name};
                cnt_phs=cnt_phs+1;
            end
        end
        startind = Movement.datasegment(set).PhaseIndeces.(legs_name{leg})(num_phs);
        [~,endind] = size(Movement.datasegment(set).Phase_States.(legs_name{leg}).full);
        phase_state = Movement.datasegment(set).Phase_States.(legs_name{leg}).full(:,startind);
        phase_name = Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(:,startind);
        Movement.datasegment(set).Phases.(legs_name{leg})(cnt_phs,:)={startind, endind, phase_state,phase_name};
        
        %%% Flexions and Extensions
        num_phs=length(Movement.datasegment(set).PhaseFlexExtIndeces.(legs_name{leg}));
        Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})=cell(1,4);
        cnt_phs = 1;
        for phs=1:num_phs-1
            startind = Movement.datasegment(set).PhaseFlexExtIndeces.(legs_name{leg})(phs)+1;
            endind = Movement.datasegment(set).PhaseFlexExtIndeces.(legs_name{leg})(phs+1);
            phase_state = [Movement.datasegment(set).Phase_States.(legs_name{leg}).full(1,startind);...
                Movement.datasegment(set).Phase_States.(legs_name{leg}).full(4,startind)];%;Movement.datasegment(set).Phase_States.(legs_name{leg}).full(5,startind)];
            phase_name = {Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(1,startind);...
                Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(4,startind)};%;Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(5,startind)};
            if endind-startind>1%startind~=endind  %%Filter out noise in phases
                Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})(cnt_phs,:)={startind, endind, phase_state,phase_name};
                cnt_phs=cnt_phs+1;
            end
        end
        startind = Movement.datasegment(set).PhaseFlexExtIndeces.(legs_name{leg})(num_phs);
        [~,endind] = size(Movement.datasegment(set).Phase_States.(legs_name{leg}).full);
        phase_state = [Movement.datasegment(set).Phase_States.(legs_name{leg}).full(1,startind);...
            Movement.datasegment(set).Phase_States.(legs_name{leg}).full(4,startind);Movement.datasegment(set).Phase_States.(legs_name{leg}).full(5,startind)];
        phase_name = {Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(1,startind);...
            Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(4,startind);Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(5,startind)};
        Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})(cnt_phs,:)={startind, endind, phase_state,phase_name};
        
        
        %%% Parse out Kick Cycles: full flexions and extensions
        %%% Full flexion if: 
        %%%      a) infant's foot movd at least 5 consecutive frames (10 ms per frame), i.e. move in space 
        %%%      b) either hip or knee joint excursion exceeded 11.5 deg. 
        %%%      c) hip and knee flexing (not necessarily ankle)
        %%% Full extension if:
        %%%      d) following full flexion
        Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})=cell(1,4);
        Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg})=zeros(1,3);
        cnt_FullFlexExt = 1;
        for phs=1:cnt_phs
            startind = cell2mat(Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})(phs,1));
            endind = cell2mat(Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})(phs,2));
            
            %%% Condition a (movement is at least 50 ms)
            duration = (endind - startind)/Constants.fs;
            if duration >= 0.05
                
                %%% Condition c (hip and knee flexing or extending)
                temp_state = Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})(phs,4);
                if (strcmp(temp_state{1}{1},'flexion')&&strcmp(temp_state{1}{2},'flexion'))||(strcmp(temp_state{1}{1},'extension')&&strcmp(temp_state{1}{2},'extension'))
                    
                    %%% Condition b (hip or knee joint excursion exceeded 11.5 deg.)
                    Joint_exc = zeros(1,3);
                    cnt = 1;
                    for num_DOF = 1:axs
                        if strcmp(axs_name{num_DOF}(2),'F')
                            temp_exc=0;
                            for samp = 1:(endind-startind+1)
                                temp_exc = temp_exc+SegRates.dataset(set).(legs_name{leg}).(axs_name{num_DOF})(startind+samp-1)/Constants.fs;
                            end
                            Joint_exc(cnt) = temp_exc;
                            cnt = cnt + 1;
                        end
                    end
                    
                    if abs(Joint_exc(1))>=11.5||abs(Joint_exc(2))>=11.5
                        Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(cnt_FullFlexExt,:)=Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})(phs,:);
                        Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg})(cnt_FullFlexExt,:)=Joint_exc;

                        cnt_FullFlexExt = cnt_FullFlexExt + 1;
                    end
                                   
                end
            end
        end
        clearvars cnt_FullFlexExt cnt Joint_exc samp temp_exc temp_state duration startind endind 
        
        %%% Condition d (full flexion followed by full extension)
        [cnt_kick, ~] = size(Movement.datasegment(set).Phases_KickCycle.(legs_name{leg}));
        Temp_KickCycle = Movement.datasegment(set).Phases_KickCycle.(legs_name{leg});
        temp_state = Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(1,4);
        kick = 1;
        while ~(strcmp(temp_state{1}{1},'flexion'))&&(kick<=cnt_kick)
            Temp_KickCycle(kick,:)=[];
            kick = kick + 1;
            temp_state = Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(kick,4);
        end
        Movement.datasegment(set).Phases_KickCycle.(legs_name{leg}) = Temp_KickCycle;
        clearvars Temp_KickCycle cnt_kick temp_state kick
        
        [cnt_kick, ~] = size(Movement.datasegment(set).Phases_KickCycle.(legs_name{leg}));
        Temp_KickCycle = [];%Movement.datasegment(set).Phases_KickCycle.(legs_name{leg});
        for kick = 1:cnt_kick-1
            temp_state = Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(kick,4);
            temp_next = Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(kick+1,4);
            if (strcmp(temp_state{1}{1},'flexion'))&&(strcmp(temp_next{1}{1},'extension'))
                Temp_KickCycle = [Temp_KickCycle;Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(kick,:);Movement.datasegment(set).Phases_KickCycle.(legs_name{leg})(kick+1,:)];
            end
        end
        Movement.datasegment(set).Phases_KickCycle.(legs_name{leg}) = Temp_KickCycle;
        clearvars Temp_KickCycle cnt_kick temp_state temp_next kick
        
        %%% Joint Excursions (for each DOF, considered 0 at start of each
        %%% individual joint phase). Phases go from one endpoint
        %%% to the next endpoint-1
        for num_DOF = 1:axs
            Movement.datasegment(set).DOF_JointExc.(legs_name{leg}).(axs_name{num_DOF})=cell(1,3);
            Movement.datasegment(set).DOF_PhaseIndeces.(legs_name{leg}).(axs_name{num_DOF}) = find(Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF})~=0);
            num_phs = length(Movement.datasegment(set).DOF_PhaseIndeces.(legs_name{leg}).(axs_name{num_DOF}));
            cnt_phs = 1;
            for phs=2:num_phs-1%1:num_phs-1  %%clip first and last phase (because may be cut off)
                startind = Movement.datasegment(set).DOF_PhaseIndeces.(legs_name{leg}).(axs_name{num_DOF})(phs)+1;
                endind = Movement.datasegment(set).DOF_PhaseIndeces.(legs_name{leg}).(axs_name{num_DOF})(phs+1);
                if (Movement.datasegment(set).Phase_States.(legs_name{leg}).(axs_name{num_DOF})(startind)~=0)
                    temp_exc = 0;
                    for samp = 1:(endind-startind+1)
                        temp_exc = temp_exc+SegRates.dataset(set).(legs_name{leg}).(axs_name{num_DOF})(startind+samp-1)/Constants.fs;
                    end
                    Movement.datasegment(set).DOF_JointExc.(legs_name{leg}).(axs_name{num_DOF})(cnt_phs,:)={startind,endind,temp_exc};
                    cnt_phs = cnt_phs+1;
                end
            end
            
            %             startind = Movement.datasegment(set).DOF_PhaseIndeces.(legs_name{leg}).(axs_name{num_DOF})(end)+1;
            %             endind = length(Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF}));
            %             if (Movement.datasegment(set).Phase_States.(legs_name{leg}).(axs_name{num_DOF})(startind)~=0)
            %                 temp_exc = 0;
            %                 for samp = 1:(endind-startind+1)
            %                     temp_exc = temp_exc+SegRates.dataset(set).(legs_name{leg}).(axs_name{num_DOF})(startind+samp-1)/Constants.fs;
            %                 end
            %                 Movement.datasegment(set).DOF_JointExc.(legs_name{leg}).(axs_name{num_DOF})(cnt_phs,:)={startind,endind,temp_exc};
            %                 cnt_phs = cnt_phs+1;
            %             end
        end        
    end
end
    
clearvars -except SegData SegRates SegTruth Movement Constants phases phase_names
leg_seg = {'DOFPhaseLogicalIndeces','PhaseIndeces','PhaseFlexExtIndeces','DOF_PhaseIndeces','Phases_FlexExt'};
Movement.datasegment=rmfield(Movement.datasegment,leg_seg);
%%
%%%%%%%%%%%%###
%     %%% Phases
%     Movement.datasegment(set).Phase_Names = phase_names.dataset(set);
%     Movement.datasegment(set).Phase_States = phases.dataset(set);    
%     
%     %%% Indeces of endpoints of phase changes. 
%     legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
%     legs_name = fieldnames(Movement.datasegment(set).Phase_States);
%     for leg=1:legs_num
%         axs_name = fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg}));
%         axs = length(fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg})))-1;
%         temp_phase = zeros(length(Movement.datasegment(set).Phase_States.(legs_name{leg}).(axs_name{axs}))-1,1);
%         for num_DOF = 1:axs
%             Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF})=abs(diff(Movement.datasegment(set).Phase_States.(legs_name{leg}).(axs_name{num_DOF})));
%             Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF})(1) = 1;
%             temp_phase = temp_phase + Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF}); 
%             %%^ if any DOF is changing at a sample, then indicates a phase change                                                                                                      
%             %%| either 0 or some positive integer for these values.
%         end
%         %%% Indeces of endpoints of phases. Phases go from one endpoint
%         %%% to the next endpoint-1
%         Movement.datasegment(set).PhaseIndeces.(legs_name{leg}) = find(temp_phase~=0);
%         
%         %%%Create variable for what phases are happening when [startind,endind,phase_state,phase_name]
%         num_phs = length(Movement.datasegment(set).PhaseIndeces.(legs_name{leg}));
%         Movement.datasegment(set).Phases.(legs_name{leg})=cell(1,4);%length(Movement.datasegment(i).PhaseIndeces.(legs_name{leg})),4);
%         cnt_phs = 1;
%         for phs=1:num_phs-1
%             startind = Movement.datasegment(set).PhaseIndeces.(legs_name{leg})(phs)+1;
%             endind = Movement.datasegment(set).PhaseIndeces.(legs_name{leg})(phs+1);
%             phase_state = Movement.datasegment(set).Phase_States.(legs_name{leg}).full(:,startind);
%             phase_name = Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(:,startind);
%             if endind-startind>1%startind~=endind  %%Filter out noise in phases
%                 Movement.datasegment(set).Phases.(legs_name{leg})(cnt_phs,:)={startind, endind, phase_state,phase_name};
%                 cnt_phs=cnt_phs+1;
%             end
%         end
%         startind = Movement.datasegment(set).PhaseIndeces.(legs_name{leg})(num_phs);
%         [~,endind] = size(Movement.datasegment(set).Phase_States.(legs_name{leg}).full);
%         phase_state = Movement.datasegment(set).Phase_States.(legs_name{leg}).full(:,startind);
%         phase_name = Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(:,startind);
%         Movement.datasegment(set).Phases.(legs_name{leg})(cnt_phs,:)={startind, endind, phase_state,phase_name};        
%     end
%     
%     %%% Flexions and Extensions
%  
%     %%% Separate out variable with just HF, KF, and AF states (NEED to
%     %%% combine states that are the same sequentially into one state)
%     %legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
%     %legs_name = fieldnames(Movement.datasegment(set).Phase_States);
%     for leg=1:legs_num
%         axs_name = fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg}));
%         axs = length(fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg})))-1;
%         temp_phase = zeros(length(Movement.datasegment(set).Phase_States.(legs_name{leg}).(axs_name{axs}))-1,1);
%       
%         Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})=cell(1,4);%length(Movement.datasegment(i).PhaseIndeces.(legs_name{leg})),4);
%         cnt_FlexExt = 1;
%         [num_phs,~]=size(Movement.datasegment(set).Phases.(legs_name{leg}));
%         temp_state = zeros(3,1);
%         temp_names = cell(3,1);
%         for phs=1:num_phs
%             startind=Movement.datasegment(set).Phases.(legs_name{leg}){phs,1};
%             endind=Movement.datasegment(set).Phases.(legs_name{leg}){phs,2};
%             temp_state = [Movement.datasegment(set).Phases.(legs_name{leg}){phs,3}(1);...
%                 Movement.datasegment(set).Phases.(legs_name{leg}){phs,3}(4);Movement.datasegment(set).Phases.(legs_name{leg}){phs,3}(5)];
%             temp_names = {Movement.datasegment(set).Phases.(legs_name{leg}){phs,4}(1);...
%                 Movement.datasegment(set).Phases.(legs_name{leg}){phs,4}(4);Movement.datasegment(set).Phases.(legs_name{leg}){phs,4}(5)};
%             Movement.datasegment(set).Phases_FlexExt.(legs_name{leg})(phs,:)={startind, endind, temp_state, temp_names};
%         end
%     end
%   

%%

%%%%%%%%%%%%%Frequencies:
for set=1:length(SegData.datasegment)
    %%%%Movement Frequency: how long is the baby active within a
    %%%%datasegment
    %Metrics.Frequencies.datasegment(set).MovementFrequency = sum(Movement.datasegment(set).MovementSequences)/length(Movement.datasegment(set).MovementSequences);
    Metrics.Frequencies.MovementFrequency(set,1) = sum(Movement.datasegment(set).MovementSequences)/length(Movement.datasegment(set).MovementSequences);
    
    %%%%Individual Movement Frequency: how long is baby performing bilateral, UL,
    %%%%UR, NM  
%     Metrics.Frequencies.datasegment(set).NMFrequency = sum(Movement.datasegment(set).Movements(:) == 0)/length(Movement.datasegment(set).Movements);
%     Metrics.Frequencies.datasegment(set).ULFrequency = sum(Movement.datasegment(set).Movements(:) == 1)/length(Movement.datasegment(set).Movements);
%     Metrics.Frequencies.datasegment(set).URFrequency = sum(Movement.datasegment(set).Movements(:) == 2)/length(Movement.datasegment(set).Movements);
%     Metrics.Frequencies.datasegment(set).BIFrequency = sum(Movement.datasegment(set).Movements(:) == 3)/length(Movement.datasegment(set).Movements);
    Metrics.Frequencies.NMFrequency(set,1) = sum(Movement.datasegment(set).Movements(:) == 0)/length(Movement.datasegment(set).Movements);
    Metrics.Frequencies.ULFrequency(set,1) = sum(Movement.datasegment(set).Movements(:) == 1)/length(Movement.datasegment(set).Movements);
    Metrics.Frequencies.URFrequency(set,1) = sum(Movement.datasegment(set).Movements(:) == 2)/length(Movement.datasegment(set).Movements);
    Metrics.Frequencies.BIFrequency(set,1) = sum(Movement.datasegment(set).Movements(:) == 3)/length(Movement.datasegment(set).Movements);
end

%%%%%%%%%%%%%Durations: 
for set=1:length(SegData.datasegment)
    %%%%Kicking Duration: Duration of each movement sequence
    k=1;
    %Initialize variables
    for init=1:1
        Metrics.Durations.datasegment(set).KickingDuration(1) = 0;
    end
    
    for j=2:length(Movement.datasegment(set).MovementSequencesIndeces)
        startind = Movement.datasegment(set).MovementSequencesIndeces(j-1)+1;
        endind = Movement.datasegment(set).MovementSequencesIndeces(j);
        if Movement.datasegment(set).MovementSequences(startind)==1  
            Metrics.Durations.datasegment(set).KickingDuration(k) = SegData.datasegment(set).Left.Time(endind)-SegData.datasegment(set).Left.Time(startind); 
            k = k + 1;
        end
    end
    clear k
    
    %%%%Movement Duration: Duration of each movement
    nm=1;
    ul=1;
    ur=1;
    bi=1;
    %Initialize variables
    for init=1:1
        Metrics.Durations.datasegment(set).MovementDurationNM(1) = 0;
        Metrics.Durations.datasegment(set).MovementDurationUL(1) = 0;
        Metrics.Durations.datasegment(set).MovementDurationUR(1) = 0;
        Metrics.Durations.datasegment(set).MovementDurationBI(1) = 0;
    end
    
    for j=2:length(Movement.datasegment(set).MovementsIndeces)
        startind = Movement.datasegment(set).MovementsIndeces(j-1)+1;
        endind = Movement.datasegment(set).MovementsIndeces(j);
        if Movement.datasegment(set).Movements(startind)==0
            Metrics.Durations.datasegment(set).MovementDurationNM(nm) = SegData.datasegment(set).Left.Time(endind)-SegData.datasegment(set).Left.Time(startind); 
            nm = nm + 1;
        elseif Movement.datasegment(set).Movements(startind)==1  
            Metrics.Durations.datasegment(set).MovementDurationUL(ul) = SegData.datasegment(set).Left.Time(endind)-SegData.datasegment(set).Left.Time(startind); 
            ul = ul + 1;
        elseif Movement.datasegment(set).Movements(startind)==2
            Metrics.Durations.datasegment(set).MovementDurationUR(ur) = SegData.datasegment(set).Left.Time(endind)-SegData.datasegment(set).Left.Time(startind); 
            ur = ur + 1;
        else 
            Metrics.Durations.datasegment(set).MovementDurationBI(bi) = SegData.datasegment(set).Left.Time(endind)-SegData.datasegment(set).Left.Time(startind); 
            bi = bi + 1;            
        end
    end
    clear nm ul ur bi

    %%%%Average Durations
    Metrics.Overall_Dur.datasegment(set).AvgKickingDuration = mean(Metrics.Durations.datasegment(set).KickingDuration);
    Metrics.Overall_Dur.datasegment(set).AvgMovementDurationNM = mean(Metrics.Durations.datasegment(set).MovementDurationNM);    
    Metrics.Overall_Dur.datasegment(set).AvgMovementDurationUL = mean(Metrics.Durations.datasegment(set).MovementDurationUL);
    Metrics.Overall_Dur.datasegment(set).AvgMovementDurationUR = mean(Metrics.Durations.datasegment(set).MovementDurationUR);
    Metrics.Overall_Dur.datasegment(set).AvgMovementDurationBI = mean(Metrics.Durations.datasegment(set).MovementDurationBI);

    %%%%Max Durations
    Metrics.Overall_Dur.datasegment(set).MaxKickingDuration = max(Metrics.Durations.datasegment(set).KickingDuration);
    Metrics.Overall_Dur.datasegment(set).MaxMovementDurationNM = max(Metrics.Durations.datasegment(set).MovementDurationNM);    
    Metrics.Overall_Dur.datasegment(set).MaxMovementDurationUL = max(Metrics.Durations.datasegment(set).MovementDurationUL);
    Metrics.Overall_Dur.datasegment(set).MaxMovementDurationUR = max(Metrics.Durations.datasegment(set).MovementDurationUR);
    Metrics.Overall_Dur.datasegment(set).MaxMovementDurationBI = max(Metrics.Durations.datasegment(set).MovementDurationBI);

%%%%%%%%%%%%%Accelerations: 

    %%%%Average Kicking Accelerations: Average Acceleration of each movement
    %%%%sequence
    %%%%Peak Kicking Accelerations: Peak acceleration of each movement
    %%%%sequence
    k=1;   
    %Initialize variables
    for init=1:1
        Metrics.Accelerations.datasegment(set).AvgKickingAccLF(1) = 0;
        Metrics.Accelerations.datasegment(set).AvgKickingAccLS(1) = 0;
        Metrics.Accelerations.datasegment(set).AvgKickingAccLT(1) = 0;
        
        Metrics.Accelerations.datasegment(set).AvgKickingAccRF(1) = 0;
        Metrics.Accelerations.datasegment(set).AvgKickingAccRS(1) = 0;
        Metrics.Accelerations.datasegment(set).AvgKickingAccRT(1) = 0;
        
        Metrics.Accelerations.datasegment(set).PeakKickingAccLF(1) = 0;
        Metrics.Accelerations.datasegment(set).PeakKickingAccLS(1) = 0;
        Metrics.Accelerations.datasegment(set).PeakKickingAccLT(1) = 0;
        
        Metrics.Accelerations.datasegment(set).PeakKickingAccRF(1) = 0;
        Metrics.Accelerations.datasegment(set).PeakKickingAccRS(1) = 0;
        Metrics.Accelerations.datasegment(set).PeakKickingAccRT(1) = 0;
    end
    
    for j=2:length(Movement.datasegment(set).MovementSequencesIndeces)
        startind = Movement.datasegment(set).MovementSequencesIndeces(j-1)+1;
        endind = Movement.datasegment(set).MovementSequencesIndeces(j);
        if Movement.datasegment(set).MovementSequences(startind)==1  
           Metrics.Accelerations.datasegment(set).AvgKickingAccLF(k) = mean(SegData.datasegment(set).Left.Foot.MagAccel(startind:endind));
           Metrics.Accelerations.datasegment(set).AvgKickingAccLS(k) = mean(SegData.datasegment(set).Left.Shin.MagAccel(startind:endind));
           Metrics.Accelerations.datasegment(set).AvgKickingAccLT(k) = mean(SegData.datasegment(set).Left.Thigh.MagAccel(startind:endind));

           Metrics.Accelerations.datasegment(set).AvgKickingAccRF(k) = mean(SegData.datasegment(set).Right.Foot.MagAccel(startind:endind));
           Metrics.Accelerations.datasegment(set).AvgKickingAccRS(k) = mean(SegData.datasegment(set).Right.Shin.MagAccel(startind:endind));
           Metrics.Accelerations.datasegment(set).AvgKickingAccRT(k) = mean(SegData.datasegment(set).Right.Thigh.MagAccel(startind:endind));

           Metrics.Accelerations.datasegment(set).PeakKickingAccLF(k) = max(SegData.datasegment(set).Left.Foot.MagAccel(startind:endind));
           Metrics.Accelerations.datasegment(set).PeakKickingAccLS(k) = max(SegData.datasegment(set).Left.Shin.MagAccel(startind:endind));
           Metrics.Accelerations.datasegment(set).PeakKickingAccLT(k) = max(SegData.datasegment(set).Left.Thigh.MagAccel(startind:endind));

           Metrics.Accelerations.datasegment(set).PeakKickingAccRF(k) = max(SegData.datasegment(set).Right.Foot.MagAccel(startind:endind));
           Metrics.Accelerations.datasegment(set).PeakKickingAccRS(k) = max(SegData.datasegment(set).Right.Shin.MagAccel(startind:endind));
           Metrics.Accelerations.datasegment(set).PeakKickingAccRT(k) = max(SegData.datasegment(set).Right.Thigh.MagAccel(startind:endind));

           k = k + 1;
        end
        clear startind endind
    end
    clear k
    
    %%%%Average movement Accelerations: Average Acceleration of each movement
    %%%%Peak movement accelerations: peak acceleration of each movement
    ul=1;
    ur=1;
    bi=1;
    %Initialize variables
    for init = 1:1
        Metrics.Accelerations.datasegment(set).AvgAccUL_LF(1) = 0;
        Metrics.Accelerations.datasegment(set).AvgAccUL_LS(1) = 0;
        Metrics.Accelerations.datasegment(set).AvgAccUL_LT(1) = 0;
        
        Metrics.Accelerations.datasegment(set).PeakAccUL_LF(1) = 0;
        Metrics.Accelerations.datasegment(set).PeakAccUL_LS(1) = 0;
        Metrics.Accelerations.datasegment(set).PeakAccUL_LT(1) = 0;
        
        Metrics.Accelerations.datasegment(set).AvgAccUR_RF(1) = 0;
        Metrics.Accelerations.datasegment(set).AvgAccUR_RS(1) = 0;
        Metrics.Accelerations.datasegment(set).AvgAccUR_RT(1) = 0;
        
        Metrics.Accelerations.datasegment(set).PeakAccUR_RF(1) = 0;
        Metrics.Accelerations.datasegment(set).PeakAccUR_RS(1) = 0;
        Metrics.Accelerations.datasegment(set).PeakAccUR_RT(1) = 0;
        
        Metrics.Accelerations.datasegment(set).AvgAccBI_LF(1) = 0;
        Metrics.Accelerations.datasegment(set).AvgAccBI_LS(1) = 0;
        Metrics.Accelerations.datasegment(set).AvgAccBI_LT(1) = 0;
        
        Metrics.Accelerations.datasegment(set).PeakAccBI_LF(1) = 0;
        Metrics.Accelerations.datasegment(set).PeakAccBI_LS(1) = 0;
        Metrics.Accelerations.datasegment(set).PeakAccBI_LT(1) = 0;
        
        Metrics.Accelerations.datasegment(set).AvgAccBI_RF(1) = 0;
        Metrics.Accelerations.datasegment(set).AvgAccBI_RS(1) = 0;
        Metrics.Accelerations.datasegment(set).AvgAccBI_RT(1) = 0;
        
        Metrics.Accelerations.datasegment(set).PeakAccBI_RF(1) = 0;
        Metrics.Accelerations.datasegment(set).PeakAccBI_RS(1) = 0;
        Metrics.Accelerations.datasegment(set).PeakAccBI_RT(1) = 0;
    end

    for j=2:length(Movement.datasegment(set).MovementsIndeces)
        startind = Movement.datasegment(set).MovementsIndeces(j-1)+1;
        endind = Movement.datasegment(set).MovementsIndeces(j);
        if Movement.datasegment(set).Movements(startind)==1  
            Metrics.Accelerations.datasegment(set).AvgAccUL_LF(ul) = mean(SegData.datasegment(set).Left.Foot.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).AvgAccUL_LS(ul) = mean(SegData.datasegment(set).Left.Shin.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).AvgAccUL_LT(ul) = mean(SegData.datasegment(set).Left.Thigh.MagAccel(startind:endind));
           
            Metrics.Accelerations.datasegment(set).PeakAccUL_LF(ul) = max(SegData.datasegment(set).Left.Foot.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).PeakAccUL_LS(ul) = max(SegData.datasegment(set).Left.Shin.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).PeakAccUL_LT(ul) = max(SegData.datasegment(set).Left.Thigh.MagAccel(startind:endind));
           
            ul = ul + 1;
        
        elseif Movement.datasegment(set).Movements(startind)==2
            Metrics.Accelerations.datasegment(set).AvgAccUR_RF(ur) = mean(SegData.datasegment(set).Right.Foot.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).AvgAccUR_RS(ur) = mean(SegData.datasegment(set).Right.Shin.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).AvgAccUR_RT(ur) = mean(SegData.datasegment(set).Right.Thigh.MagAccel(startind:endind));
           
            Metrics.Accelerations.datasegment(set).PeakAccUR_RF(ur) = max(SegData.datasegment(set).Right.Foot.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).PeakAccUR_RS(ur) = max(SegData.datasegment(set).Right.Shin.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).PeakAccUR_RT(ur) = max(SegData.datasegment(set).Right.Thigh.MagAccel(startind:endind));
    
            ur = ur + 1;
        
        elseif Movement.datasegment(set).Movements(startind)==3 
            Metrics.Accelerations.datasegment(set).AvgAccBI_LF(bi) = mean(SegData.datasegment(set).Left.Foot.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).AvgAccBI_LS(bi) = mean(SegData.datasegment(set).Left.Shin.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).AvgAccBI_LT(bi) = mean(SegData.datasegment(set).Left.Thigh.MagAccel(startind:endind));
           
            Metrics.Accelerations.datasegment(set).PeakAccBI_LF(bi) = max(SegData.datasegment(set).Left.Foot.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).PeakAccBI_LS(bi) = max(SegData.datasegment(set).Left.Shin.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).PeakAccBI_LT(bi) = max(SegData.datasegment(set).Left.Thigh.MagAccel(startind:endind));

            Metrics.Accelerations.datasegment(set).AvgAccBI_RF(bi) = mean(SegData.datasegment(set).Right.Foot.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).AvgAccBI_RS(bi) = mean(SegData.datasegment(set).Right.Shin.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).AvgAccBI_RT(bi) = mean(SegData.datasegment(set).Right.Thigh.MagAccel(startind:endind));
           
            Metrics.Accelerations.datasegment(set).PeakAccBI_RF(bi) = max(SegData.datasegment(set).Right.Foot.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).PeakAccBI_RS(bi) = max(SegData.datasegment(set).Right.Shin.MagAccel(startind:endind));
            Metrics.Accelerations.datasegment(set).PeakAccBI_RT(bi) = max(SegData.datasegment(set).Right.Thigh.MagAccel(startind:endind));
    
            bi = bi + 1;            
        end
        clear startind endind
    end
    clear ul ur bi

    %%%%Overall Avg Kicking Accelerations: Overall Average Acceleration of movement sequences 
    Metrics.Overall_Accel.datasegment(set).OverallAvgKickingAccLF = mean(Metrics.Accelerations.datasegment(set).AvgKickingAccLF);
    Metrics.Overall_Accel.datasegment(set).OverallAvgKickingAccLS = mean(Metrics.Accelerations.datasegment(set).AvgKickingAccLS);
    Metrics.Overall_Accel.datasegment(set).OverallAvgKickingAccLT = mean(Metrics.Accelerations.datasegment(set).AvgKickingAccLT);

    Metrics.Overall_Accel.datasegment(set).OverallAvgKickingAccRF = mean(Metrics.Accelerations.datasegment(set).AvgKickingAccRF);
    Metrics.Overall_Accel.datasegment(set).OverallAvgKickingAccRS = mean(Metrics.Accelerations.datasegment(set).AvgKickingAccRS);
    Metrics.Overall_Accel.datasegment(set).OverallAvgKickingAccRT = mean(Metrics.Accelerations.datasegment(set).AvgKickingAccRT);

    %%%%Overall Peak Kicking Accelerations: Peak acceleration of movement sequences    
    Metrics.Overall_Accel.datasegment(set).OverallPeakKickingAccLF = max(Metrics.Accelerations.datasegment(set).PeakKickingAccLF);
    Metrics.Overall_Accel.datasegment(set).OverallPeakKickingAccLS = max(Metrics.Accelerations.datasegment(set).PeakKickingAccLS);
    Metrics.Overall_Accel.datasegment(set).OverallPeakKickingAccLT = max(Metrics.Accelerations.datasegment(set).PeakKickingAccLT);

    Metrics.Overall_Accel.datasegment(set).OverallPeakKickingAccRF = max(Metrics.Accelerations.datasegment(set).PeakKickingAccRF);
    Metrics.Overall_Accel.datasegment(set).OverallPeakKickingAccRS = max(Metrics.Accelerations.datasegment(set).PeakKickingAccRS);
    Metrics.Overall_Accel.datasegment(set).OverallPeakKickingAccRT = max(Metrics.Accelerations.datasegment(set).PeakKickingAccRT);
    
    %%%%Overall Average movement Accelerations: Average Acceleration of movements
    Metrics.Overall_Accel.datasegment(set).OverallAvgAccUL_LF = mean(Metrics.Accelerations.datasegment(set).AvgAccUL_LF);
    Metrics.Overall_Accel.datasegment(set).OverallAvgAccUL_LS = mean(Metrics.Accelerations.datasegment(set).AvgAccUL_LS);
    Metrics.Overall_Accel.datasegment(set).OverallAvgAccUL_LT = mean(Metrics.Accelerations.datasegment(set).AvgAccUL_LT);
    
    Metrics.Overall_Accel.datasegment(set).OverallAvgAccUR_RF = mean(Metrics.Accelerations.datasegment(set).AvgAccUR_RF);
    Metrics.Overall_Accel.datasegment(set).OverallAvgAccUR_RS = mean(Metrics.Accelerations.datasegment(set).AvgAccUR_RS);
    Metrics.Overall_Accel.datasegment(set).OverallAvgAccUR_RT = mean(Metrics.Accelerations.datasegment(set).AvgAccUR_RT);
    
    Metrics.Overall_Accel.datasegment(set).OverallAvgAccBI_LF = mean(Metrics.Accelerations.datasegment(set).AvgAccBI_LF);
    Metrics.Overall_Accel.datasegment(set).OverallAvgAccBI_LS = mean(Metrics.Accelerations.datasegment(set).AvgAccBI_LS);
    Metrics.Overall_Accel.datasegment(set).OverallAvgAccBI_LT = mean(Metrics.Accelerations.datasegment(set).AvgAccBI_LT);
    
    Metrics.Overall_Accel.datasegment(set).OverallAvgAccBI_RF = mean(Metrics.Accelerations.datasegment(set).AvgAccBI_RF);
    Metrics.Overall_Accel.datasegment(set).OverallAvgAccBI_RS = mean(Metrics.Accelerations.datasegment(set).AvgAccBI_RS);
    Metrics.Overall_Accel.datasegment(set).OverallAvgAccBI_RT = mean(Metrics.Accelerations.datasegment(set).AvgAccBI_RT);
        
    %%%%Peak movement accelerations: peak acceleration of each movement   
    Metrics.Overall_Accel.datasegment(set).OverallPeakAccUL_LF = max(Metrics.Accelerations.datasegment(set).PeakAccUL_LF);
    Metrics.Overall_Accel.datasegment(set).OverallPeakAccUL_LS = max(Metrics.Accelerations.datasegment(set).PeakAccUL_LS);
    Metrics.Overall_Accel.datasegment(set).OverallPeakAccUL_LT = max(Metrics.Accelerations.datasegment(set).PeakAccUL_LT); 
    
    Metrics.Overall_Accel.datasegment(set).OverallPeakAccUR_RF = max(Metrics.Accelerations.datasegment(set).PeakAccUR_RF);
    Metrics.Overall_Accel.datasegment(set).OverallPeakAccUR_RS = max(Metrics.Accelerations.datasegment(set).PeakAccUR_RS);
    Metrics.Overall_Accel.datasegment(set).OverallPeakAccUR_RT = max(Metrics.Accelerations.datasegment(set).PeakAccUR_RT);
    
    Metrics.Overall_Accel.datasegment(set).OverallPeakAccBI_LF = max(Metrics.Accelerations.datasegment(set).PeakAccBI_LF);
    Metrics.Overall_Accel.datasegment(set).OverallPeakAccBI_LS = max(Metrics.Accelerations.datasegment(set).PeakAccBI_LS);
    Metrics.Overall_Accel.datasegment(set).OverallPeakAccBI_LT = max(Metrics.Accelerations.datasegment(set).PeakAccBI_LT);
        
    Metrics.Overall_Accel.datasegment(set).OverallPeakAccBI_RF = max(Metrics.Accelerations.datasegment(set).PeakAccBI_RF);
    Metrics.Overall_Accel.datasegment(set).OverallPeakAccBI_RS = max(Metrics.Accelerations.datasegment(set).PeakAccBI_RS);
    Metrics.Overall_Accel.datasegment(set).OverallPeakAccBI_RT = max(Metrics.Accelerations.datasegment(set).PeakAccBI_RT);
end
clear i j

save('Metrics.mat','Metrics','Movement')

Frequencies = struct2table(Metrics.Frequencies);