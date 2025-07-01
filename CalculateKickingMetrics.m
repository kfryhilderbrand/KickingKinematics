%% Calculate Kinematic Metrics
%%% Note: Nothing in this script will need to be updated.

%% Calculate Auxillary Variables

angle_thresh = 5; %angle change less than <angle_thresh> deg for 1 seconds considered a period of NM
num_datasegs = length(SegData.datasegment);

%%% Magnitudes of Acceleration
for set=1:num_datasegs
    legs_name = {'Left','Right'};
    seg_name = {'Foot','Shin','Thigh'};
    for leg=1:length(legs_name)
        for cnt_seg = 1:length(seg_name)
            SegData.datasegment(set).(legs_name{leg}).(seg_name{cnt_seg}).MagAccel = vecnorm(SegData.datasegment(set).(legs_name{leg}).(seg_name{cnt_seg}).Accel(:,2:4)')'; 
        end
    end
end
save('SegData.mat','SegData');

%%% Phase Sequences
%%% With our sensor placement   y<---Xz      z point down to limb surface
%                                    |       X point down limb shank
%                                    v x
%%%  Left Leg:  Positive joint rates
%%%  HF (extension), HA (adduction, toward midline), HR (internal
%%%  rotation), KF (extension), AF (flexion), AI (eversion)

%%%  Right Leg: Positive joint rates
%%%  HF (extension), HA (abduction, away from midline), HR (external
%%%  rotation), KF (extension), AF (flexion), AI (inversion)
phases = DeterminePhase(SegRates, angle_thresh);  
phase_names = DeterminePhase_Names(phases);
save('PhaseData.mat','phases','phase_names');

%% Creation of Movement Variable
%%% Determine: 
%   Movement Sequences (0 = NM, 1 = MVMT)
%   Coordination Patterns/Movements (0 = NM, 1 = UL, 2 = UR, 3 = BI)
%   Phases (6D Vector with states of individual DOFs)
%   Joint Excursions
%   Kick Cycles (Full flexion followed by Full extension)        
        %%% Full flexion if: 
        %%%      a) infant's foot movd at least 5 consecutive frames (10 ms per frame), i.e. move in space 
        %%%      b) either hip or knee joint excursion exceeded 11.5 deg. 
        %%%      c) hip and knee flexing (not necessarily ankle)
        %%% Full extension if:
        %%%      d) following full flexion

for set=1:num_datasegs
    
%%% Movement Sequences
%   0 = NM, 1 = MVMT
    Movement.datasegment(set).MovementSequences = double(or(SegData.datasegment(set).Left.PredLabelsT, SegData.datasegment(set).Right.PredLabelsT));
    
    %%% Indeces of endpoints of movement sequences. 
    Movement.datasegment(set).MovementSequencesLogicalIndeces = abs(diff(Movement.datasegment(set).MovementSequences));
    Movement.datasegment(set).MovementSequencesLogicalIndeces(1) = 1;
    Movement.datasegment(set).MovementSequencesIndeces = find(Movement.datasegment(set).MovementSequencesLogicalIndeces==1);

%%% Coordination Patterns (movements)
%   0 = NM, 1 = UL, 2 = UR, 3 = BI
    Movement.datasegment(set).Movements = SegData.datasegment(set).Left.PredLabelsT + 2*SegData.datasegment(set).Right.PredLabelsT;
   
    %%% Indeces of Endpoints of Movements
    %   Movement goes from one endpoint to the next endpoint-1
    Movement.datasegment(set).MovementsLogicalIndeces = abs(diff(Movement.datasegment(set).Movements));
    Movement.datasegment(set).MovementsLogicalIndeces(1) = 0;
    Movement.datasegment(set).MovementsIndeces = find(Movement.datasegment(set).MovementsLogicalIndeces~=0);

%%% Phases
%   [startind,endind,phase_state,phase_name]
    Movement.datasegment(set).Phase_Names = phase_names.dataset(set);
    Movement.datasegment(set).Phase_States = phases.dataset(set);
    
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);   
    for leg=1:legs_num
        axs_name = fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg}));
        axs = length(fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg})))-1;
        temp_phase = zeros(length(Movement.datasegment(set).Phase_States.(legs_name{leg}).(axs_name{axs}))-1,1);
        temp_kickcycle = temp_phase;
        
        %%% Indeces of Endpoints of Phase Changes
        for num_DOF = 1:axs
            Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF})=abs(diff(Movement.datasegment(set).Phase_States.(legs_name{leg}).(axs_name{num_DOF})));
            Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF})(1) = 1;
            temp_phase = temp_phase + Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF});
            %%^ if any DOF is changing at a sample, then indicates a phase change
            %%| either 0 or some positive integer for these values.
            if strcmp(axs_name{num_DOF},'HF')||strcmp(axs_name{num_DOF},'KF')
                temp_kickcycle = temp_kickcycle + Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF});
            end
        end
        
        %%% Indeces of Endpoints of Phases 
        %   Phases go from one endpoint to the next endpoint-1
        Movement.datasegment(set).PhaseIndeces.(legs_name{leg}) = find(temp_phase~=0);
        
        %%% Indeces of Endpoints of Phases (just flexion axes). 
        %   Phases go from one endpoint to the next endpoint-1
        Movement.datasegment(set).PhaseFlexExtIndeces.(legs_name{leg}) = find(temp_kickcycle~=0);
        
        %%% Determine Phases 
        %   [startind,endind,phase_state,phase_name].
        num_phs = length(Movement.datasegment(set).PhaseIndeces.(legs_name{leg}));
        Movement.datasegment(set).Phases.(legs_name{leg})=cell(1,4);%length(Movement.datasegment(i).PhaseIndeces.(legs_name{leg})),4);
        cnt_phs = 1;
        for phs=1:num_phs-1
            startind = Movement.datasegment(set).PhaseIndeces.(legs_name{leg})(phs)+1;
            endind = Movement.datasegment(set).PhaseIndeces.(legs_name{leg})(phs+1);
            phase_state = Movement.datasegment(set).Phase_States.(legs_name{leg}).full(:,startind);
            phase_name = Movement.datasegment(set).Phase_Names.(legs_name{leg}).full(:,startind);
            if endind-startind>1  % Filter out noise in phases
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
            if endind-startind>1 % Filter out noise in phases
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
    
%%% Kick Cycles
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
            Temp_KickCycle = [];
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
    
%%% Joint Excursions 
%   For each DOF, considered 0 at start of each individual joint phase.
    for leg=1:legs_num
        for num_DOF = 1:axs
            Movement.datasegment(set).DOF_JointExc.(legs_name{leg}).(axs_name{num_DOF})=cell(1,3);
            Movement.datasegment(set).DOF_PhaseIndeces.(legs_name{leg}).(axs_name{num_DOF}) = find(Movement.datasegment(set).DOFPhaseLogicalIndeces.(legs_name{leg}).(axs_name{num_DOF})~=0);
            num_phs = length(Movement.datasegment(set).DOF_PhaseIndeces.(legs_name{leg}).(axs_name{num_DOF}));
            cnt_phs = 1;
            for phs=2:num_phs-1 % clip first and last phase (because may be cut off)
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
        end
    end
    
%%% Inter-Joint Correlations 
%   For each pair of flexion DOF
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

clearvars -except SegData SegRates SegTruth Movement Constants  num_datasegs

%% Calculation of Metrics
%%% Definitions: Movement Sequence = Period of activity, Movement = Period 
%%% of bilateral movement, period of unilateral movement, etc.

%%% Calculates the following kinematic metrics of infant kicking
%   Frequencies:
         %%% Movement Frequency: how long is the baby active within a datasegment
         %%% Individual Movement Frequency: how long is baby performing BI, UL, UR, NM
%   Durations:
         %%% Kicking Duration: Duration of each movement sequence (vector)
         %%% Movement Duration: Duration of each movement (vector)
         %%% Avg/Max Overall Durations: Average and maximum durations of movements and movement sequences
%   Accelerations:
         %%% Average Kicking Accelerations: Average acceleration of each movement sequence (vector)
         %%% Peak Kicking Accelerations: Peak acceleration of each movement sequence (vector)
         %%% Average Movement Accelerations: Average acceleration of each movement (vector)
         %%% Peak Movement Accelerations: Peak acceleration of each movement (vector)
         %%% Avg Overall Kicking Accelerations: Overall Average Acceleration of movement sequences
         %%% Overall Peak Kicking Accelerations: Peak acceleration of movement sequences
         %%% Overall Average movement Accelerations: Average Acceleration of movements
         %%% Overall Peak movement accelerations: peak acceleration of each movement 
%   Joint Rates:
         %%% Max Joint Rates: Maximum joint rate for each DOF for all time (+/- direction) [max/min of joint rate data for each DOF]
%   Joint Excursions:
         %%% Average Joint Excursion: The average excursion for each DOF during individual joint phases (+/- direction)
         %%% Maximum Joint Excursion: The maximum excursion for each DOF during individual joint phases (+/- direction)
%   Kick Amplitude:
         %%% Average Kick Amplitude: The average excursion during the full flexion and full extension phase of a kick cycle for each flexion DOF (+/- direction)
         %%% Maximum Kick Amplitude: The maximum excursion during the full flexion and full extension phase of a kick cycle for each flexion DOF (+/- direction)
%   Inter-Joint Coordination:
         %%% Inter-Joint Coordination: correlation between joint rates during a kick cycle (hip-knee, hip-ankle (in/out phase), knee-ankle(in/out phase))

%%% Frequencies:
for set=1:length(SegData.datasegment)
%%% Movement Frequency: how long is the baby active within a datasegment
    Metrics.Frequencies.MovementFrequency(set,1) = ...
        sum(Movement.datasegment(set).MovementSequences)/length(Movement.datasegment(set).MovementSequences);
    
%%% Individual Movement Frequency: how long is baby performing BI, UL, UR, NM
    leg_seg = {'NM','UL','UR','BI'};
    for cnt_field = 1:length(leg_seg)
        Metrics.Frequencies.(append(leg_seg{cnt_field},'Frequency'))(set,1) = ...
            sum(Movement.datasegment(set).Movements(:) == (cnt_field - 1))/length(Movement.datasegment(set).Movements);
    end
end
Frequency_Metrics = struct2table(Metrics.Frequencies);

%%% Durations: 
for set=1:num_datasegs
%%% Kicking Duration: Duration of each movement sequence
    %%% Initialize variables
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
    
%%% Movement Duration: Duration of each movement
    %%% Initialize variables
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

%%% Overall Durations
    leg_seg = {'KickingDuration','MovementDurationNM','MovementDurationUL',...
        'MovementDurationUR','MovementDurationBI'};
    for cnt_field = 1:length(leg_seg)
        %%% Average Durations
        Metrics.Overall_Dur.(append('Avg',leg_seg{cnt_field}))(set,1) = ...
            mean(Metrics.Durations.datasegment(set).(leg_seg{cnt_field}));
    end
    for cnt_field = 1:length(leg_seg)
        %%% Max Durations
        if(isempty(Metrics.Durations.datasegment(set).(leg_seg{cnt_field})))
            Metrics.Overall_Dur.(append('Max',leg_seg{cnt_field}))(set,1) = 0;
        else
            Metrics.Overall_Dur.(append('Max',leg_seg{cnt_field}))(set,1) = ...
                max(Metrics.Durations.datasegment(set).(leg_seg{cnt_field}));
        end
    end
end
Duration_Metrics = struct2table(Metrics.Overall_Dur);

%%% Accelerations: 
for set=1:num_datasegs
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);
    seg_name = {'Foot','Shin','Thigh'};
    leg_seg = {'LF','LS','LT','RF','RS','RT'};
    coor_name = {'U','BI'};

%%% Average Kicking Accelerations: Average acceleration of each movement sequence
    %%% Initialize variables
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
    
%%% Peak Kicking Accelerations: Peak acceleration of each movement sequence
    %%% Initialize variables
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
    
%%% Average Movement Accelerations: Average acceleration of each movement
    ul=1; ur=1; bi=1;
    %%% Initialize variables
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
    
%%% Peak Movement Accelerations: Peak acceleration of each movement
    ul=1; ur=1; bi=1;
    %%% Initialize variables
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
    
%%% Overall Avg Kicking Accelerations: Overall Average Acceleration of movement sequences
    for cnt_fields = 1:length(leg_seg)
        Metrics.Overall_Accel.(append('AvgKickingAcc',leg_seg{cnt_fields}))(set,1) = ...
            mean(Metrics.Accelerations.datasegment(set).(append('AvgKickingAcc',leg_seg{cnt_fields})));
    end

%%% Overall Peak Kicking Accelerations: Peak acceleration of movement sequences
    for cnt_fields = 1:length(leg_seg)
        Metrics.Overall_Accel.(append('PeakKickingAcc',leg_seg{cnt_fields}))(set,1) = ...
            max(Metrics.Accelerations.datasegment(set).(append('PeakKickingAcc',leg_seg{cnt_fields})));
    end

%%% Overall Average movement Accelerations: Average Acceleration of movements
    for cnt_fields = 1:length(leg_seg)
        Metrics.Overall_Accel.(append('AvgAcc',coor_name{1},leg_seg{cnt_fields}(1),'_',leg_seg{cnt_fields}))(set,1) = ...
            mean(Metrics.Accelerations.datasegment(set).(append('AvgAcc',coor_name{1},leg_seg{cnt_fields}(1),'_',leg_seg{cnt_fields})));
    end
    for cnt_fields = 1:length(leg_seg)
        Metrics.Overall_Accel.(append('AvgAcc',coor_name{2},'_',leg_seg{cnt_fields}))(set,1) = ...
            mean(Metrics.Accelerations.datasegment(set).(append('AvgAcc',coor_name{2},'_',leg_seg{cnt_fields})));        
    end
      
%%% Overall Peak movement accelerations: peak acceleration of each movement 
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

%%% Joint Rates:
for set=1:num_datasegs
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);
    
%%% Max Joint Rates: Maximum joint rate for each DOF for all time (+/-
%%% direction) [max/min of joint rate data for each DOF]
    for leg=1:legs_num
        axs_name = fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg}));
        axs = length(fieldnames(Movement.datasegment(set).Phase_States.(legs_name{leg})))-1;
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

%%% Joint Excursions:
for set=1:num_datasegs
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);
    
%%% Average Joint Excursion: The average excursion for each DOF during 
%%% individual joint phases (+/- direction)
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
    
%%% Maximum Joint Excursion: The maximum excursion for each DOF during 
%%% individual joint phases (+/- direction)
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

%%% Kick Amplitude:
fields_flex = {'HF','KF','AF'};
for set=1:num_datasegs
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);
    
%%% Average Kick Amplitude: The average excursion during the full flexion
%%% and full extension phase of a kick cycle for each flexion DOF (+/- direction)
    for leg=1:legs_num
        %%% Gather flexion joint excursions into array (rows 1, 3, etc.)
        temp_flex = Movement.datasegment(set).JointExc_KickCycle.(legs_name{leg})(1:2:end,:);
        %%% Gather extension joint excursions into array (rows 2, 4, etc.)
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
%%% Maximum Kick Amplitude: The maximum excursion during the full flexion
%%% and full extension phase of a kick cycle for each flexion DOF (+/- direction)
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

%%% Inter-Joint Correlation:
fields_flexCombo = {'HFKF','HFAF','KFAF'};
for set=1:num_datasegs
    legs_num = length(fieldnames(Movement.datasegment(set).Phase_States));
    legs_name = fieldnames(Movement.datasegment(set).Phase_States);
    for leg=1:legs_num
        for num_Combo = 1:length(fields_flexCombo) 
            idx = Movement.datasegment(set).InterCoor.(legs_name{leg}).(fields_flexCombo{num_Combo})<0;
            neg = Movement.datasegment(set).InterCoor.(legs_name{leg}).(fields_flexCombo{num_Combo})(idx);
            pos = Movement.datasegment(set).InterCoor.(legs_name{leg}).(fields_flexCombo{num_Combo})(~idx);
            
            if (num_Combo == 1) % HFKF
                Metrics.InterJoint.(append('AvgInterJoint_',legs_name{leg}(1),fields_flexCombo{num_Combo},'_inPhase'))(set,1) = ...
                    mean(neg); % HF and KF in phase when corr is neg
            elseif (num_Combo == 2) % HFAF
                Metrics.InterJoint.(append('AvgInterJoint_',legs_name{leg}(1),fields_flexCombo{num_Combo},'_inPhase'))(set,1) = ...
                    mean(pos); % HF and AF in phase when corr is pos
                Metrics.InterJoint.(append('AvgInterJoint_',legs_name{leg}(1),fields_flexCombo{num_Combo},'_outPhase'))(set,1) = ...
                    mean(neg);
            else
                Metrics.InterJoint.(append('AvgInterJoint_',legs_name{leg}(1),fields_flexCombo{num_Combo},'_inPhase'))(set,1) = ...
                    mean(neg); % KF and AF in phase when coor is neg
                Metrics.InterJoint.(append('AvgInterJoint_',legs_name{leg}(1),fields_flexCombo{num_Combo},'_outPhase'))(set,1) = ...
                    mean(pos);                
            end
        end
    end
end
Inter_Joint_Metrics = struct2table(Metrics.InterJoint);

%%
save('Metrics.mat','Metrics','Movement')
clearvars -except Frequency_Metrics Duration_Metrics Acceleration_Metrics Joint_Rate_Metrics Joint_Exc_Metrics Kick_Amp_Metrics Inter_Joint_Metrics
