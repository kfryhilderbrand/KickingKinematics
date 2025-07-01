clear 
load('SensorData.mat','SensData')
load('ActivityData.mat')
load('Axes_Full.mat')
Axes.HR = cross(Axes.HF,Axes.HA);
sample_offset = 0;

% %%%%LEFT
% for set=1:length(SensData.datasegment)
% 
%     GyroThighTemp = SensData.datasegment(set).Left.Thigh.Gyro;
%     GyroShinTemp = SensData.datasegment(set).Left.Shin.Gyro;
%     GyroFootTemp = SensData.datasegment(set).Left.Foot.Gyro;
%     [n, ~] = size(GyroThighTemp);
%     
%     gyro_T = GyroThighTemp(1+sample_offset:end,:);
%     gyro_S = GyroShinTemp(1:end-sample_offset,:);
% %%% Hip Rates
%     for k=2:n
%         rates.dataset(set).Left.HF(k,1) = dot(Axes.HF,gyro_T(k,2:4)');
%         rates.dataset(set).Left.HA(k,1) = dot(Axes.HA,gyro_T(k,2:4)');
%         rates.dataset(set).Left.HR(k,1) = dot(Axes.HR,gyro_T(k,2:4)');
%     end
% 
% %%% Knee Rates
% 
%     %%%Gather Gravity vectors when at rest (equal weight between gravity
%     %%%vectors and axes of rotation
%     Thigh = [Axes.HF';SensData.datasegment(set).LeftMeaned.Thigh.Accel(1,2:4)];
%     Shin = [Axes.KF';SensData.datasegment(set).LeftMeaned.Shin.Accel(1+sample_offset,2:4)];
%     for i=3:length(ActData.datasegment(set).Left.PredLabelsT)-2
%         if (ActData.datasegment(set).Left.PredLabelsT(i)==0&&ActData.datasegment(set).Left.PredLabelsT(i+1)==0 ...
%                 &&ActData.datasegment(set).Left.PredLabelsT(i-1)==0&&ActData.datasegment(set).Left.PredLabelsT(i+2)==0 ...
%                 &&ActData.datasegment(set).Left.PredLabelsT(i-2)==0)
%             Thigh = [Thigh;Axes.HF';SensData.datasegment(set).LeftMeaned.Thigh.WinAccel{i}(:,2:4)];  %SensData.datasegment(set).Left.Shin.Accel(i,2:4)];
%             Shin = [Shin;Axes.KF';SensData.datasegment(set).LeftMeaned.Shin.WinAccel{i}(:,2:4)];  %SensData.datasegment(set).Left.Foot.Accel(i+sample_offset,2:4)];
%         end
%     end
% 
%     %%%Initial Rotation
%     H = Thigh'*Shin;
%     [U,~,V] = svd(H);
%     d = det(V*U');
%     R_ST_init = V*[1,0,0;0,1,0;0,0,d]*U';
% 
%     % theta_KF_prev = 0;
%     % t_prev = gyro_S(1,1);
%     % angle_KF = 0;
%     % rate_KF = 0;
%     for k=2:n
%         gyro_S_rem(k,1:4) = [gyro_S(k,1),(R_ST_init*(R_ST_init'*gyro_S(k,2:4)'-gyro_T(k,2:4)'))'];
%         rates.dataset(set).Left.KF(k,1) = dot(Axes.KF,gyro_S_rem(k,2:4)');
%         %     theta_dot_KF = rates.dataset(set).Left.KF(k);
%         %     theta_KF = theta_KF_prev + theta_dot_KF*(t-t_prev);
%         %
%         %     angle_KF = [angle_KF; theta_KF];
%         %     theta_KF_prev = theta_KF;
%         %     t_prev = t;
%     end
% 
% %%% Ankle Rates
% 
%     %%%Gather Gravity vectors when at rest (equal weight between gravity
%     %%%vectors and axes of rotation
%      Shin = [Axes.KF';SensData.datasegment(set).LeftMeaned.Shin.Accel(1,2:4)];
%      Foot = [Axes.AF';SensData.datasegment(set).LeftMeaned.Foot.Accel(1+sample_offset,2:4)];
%     for i=4:length(ActData.datasegment(set).Left.PredLabelsT)-3
%         if (ActData.datasegment(set).Left.PredLabelsT(i)==0&&ActData.datasegment(set).Left.PredLabelsT(i+1)==0 ...
%             &&ActData.datasegment(set).Left.PredLabelsT(i-1)==0&&ActData.datasegment(set).Left.PredLabelsT(i+2)==0 ...
%             &&ActData.datasegment(set).Left.PredLabelsT(i-2)==0&&ActData.datasegment(set).Left.PredLabelsT(i+3)==0 ...
%             &&ActData.datasegment(set).Left.PredLabelsT(i-3))
%             Shin = [Shin;Axes.KF';SensData.datasegment(set).LeftMeaned.Shin.WinAccel{i}(:,2:4)];  %SensData.datasegment(set).Left.Shin.Accel(i,2:4)];
%             Foot = [Foot;Axes.AF';SensData.datasegment(set).LeftMeaned.Foot.WinAccel{i}(:,2:4)];  %SensData.datasegment(set).Left.Foot.Accel(i+sample_offset,2:4)];
%         end
%     end
%     
%     %%%Initial Rotation
%     H = (Shin'*Foot);
%     [U,~,V] = svd(H);
%     d = det(V*U');
%     R_FS_init = V*[1,0,0;0,1,0;0,0,d]*U';
% 
%     gyro_S = GyroShinTemp(1+sample_offset:end,:);
%     gyro_F = GyroFootTemp(1:end-sample_offset,:);
% 
% 
%     v_AF = Axes.AF;
%     v_AI = Axes.AI;
%     t_prev = gyro_F(1,1);
% 
%     theta_AF_prev = 0;
%     angle_AF = [];
%     theta_AI_prev = 0;
%     angle_AI = [];
%     
%     for k=2:length(gyro_F(:,1))
%         t = gyro_F(k,1);
%         gyro_F_rem(k,1:4) = [gyro_F(k,1),(R_FS_init*(R_FS_init'*gyro_F(k,2:4)'-gyro_S(k,2:4)'))'];
% 
%         rates.dataset(set).Left.AF(k,1) = dot(v_AF,gyro_F_rem(k,2:4)');
%         theta_dot_AF = rates.dataset(set).Left.AF(k);
%         theta_AF = theta_AF_prev + theta_dot_AF*(t-t_prev);
%         Rv_AF = cosd(theta_AF)*eye(3) + (1-cosd(theta_AF))*(v_AF*v_AF')+sind(theta_AF)*[0,-v_AF(3),v_AF(2);v_AF(3),0,-v_AF(1);-v_AF(2),v_AF(1),0];
% 
%         rates.dataset(set).Left.AI(k,1) = dot(v_AI,gyro_F_rem(k,2:4)');
%         theta_dot_AI = rates.dataset(set).Left.AI(k);
%         theta_AI = theta_AI_prev + theta_dot_AI*(t-t_prev);
%         Rv_AI = cosd(theta_AI)*eye(3) + (1-cosd(theta_AI))*(v_AI*v_AI')+sind(theta_AI)*[0,-v_AI(3),v_AI(2);v_AI(3),0,-v_AI(1);-v_AI(2),v_AI(1),0];
% 
%         Rv =  (Rv_AI*Rv_AF)';
% 
%         %Iterate
%         angle_AF = [angle_AF; theta_AF];
%         theta_AF_prev = theta_AF;
%         angle_AI = [angle_AI; theta_AI];
%         theta_AI_prev = theta_AI;    
%         t_prev = t;
%     end
% end
  
%%%%Right
for set=1:length(SensData.datasegment)

    GyroThighTemp = SensData.datasegment(set).Right.Thigh.Gyro;
    GyroShinTemp = SensData.datasegment(set).Right.Shin.Gyro;
    GyroFootTemp = SensData.datasegment(set).Right.Foot.Gyro;
    [n, ~] = size(GyroThighTemp);
    
    gyro_T = GyroThighTemp(1+sample_offset:end,:);
    gyro_S = GyroShinTemp(1:end-sample_offset,:);

%%% Hip Rates
    for k=2:n
        rates.dataset(set).Right.HF(k,1) = dot(Axes.HF,gyro_T(k,2:4)');
        rates.dataset(set).Right.HA(k,1) = dot(Axes.HA,gyro_T(k,2:4)');
        rates.dataset(set).Right.HR(k,1) = dot(Axes.HR,gyro_T(k,2:4)');
    end

%%% Knee Rates

    %%%Gather Gravity vectors when at rest (equal weight between gravity
    %%%vectors and axes of rotation
    Thigh = [Axes.HF';SensData.datasegment(set).RightMeaned.Thigh.Accel(1,2:4)];
    Shin = [Axes.KF';SensData.datasegment(set).RightMeaned.Shin.Accel(1+sample_offset,2:4)];
    for i=3:length(ActData.datasegment(set).Right.PredLabelsT)-2
        if (ActData.datasegment(set).Right.PredLabelsT(i)==0&&ActData.datasegment(set).Right.PredLabelsT(i+1)==0 ...
                &&ActData.datasegment(set).Right.PredLabelsT(i-1)==0&&ActData.datasegment(set).Right.PredLabelsT(i+2)==0 ...
                &&ActData.datasegment(set).Right.PredLabelsT(i-2)==0)
            Thigh = [Thigh;Axes.HF';SensData.datasegment(set).RightMeaned.Thigh.WinAccel{i}(:,2:4)];  %SensData.datasegment(set).Right.Shin.Accel(i,2:4)];
            Shin = [Shin;Axes.KF';SensData.datasegment(set).RightMeaned.Shin.WinAccel{i}(:,2:4)];  %SensData.datasegment(set).Right.Foot.Accel(i+sample_offset,2:4)];
        end
    end

    %%%Initial Rotation
    H = Thigh'*Shin;
    [U,~,V] = svd(H);
    d = det(V*U');
    R_ST_init = V*[1,0,0;0,1,0;0,0,d]*U';

    % theta_KF_prev = 0;
    % t_prev = gyro_S(1,1);
    % angle_KF = 0;
    % rate_KF = 0;
    for k=2:n
        gyro_S_rem(k,1:4) = [gyro_S(k,1),(R_ST_init*(R_ST_init'*gyro_S(k,2:4)'-gyro_T(k,2:4)'))'];
        rates.dataset(set).Right.KF(k,1) = dot(Axes.KF,gyro_S_rem(k,2:4)');
        %     theta_dot_KF = rates.dataset(set).Right.KF(k);
        %     theta_KF = theta_KF_prev + theta_dot_KF*(t-t_prev);
        %
        %     angle_KF = [angle_KF; theta_KF];
        %     theta_KF_prev = theta_KF;
        %     t_prev = t;
    end

%%% Ankle Rates

    %%%Gather Gravity vectors when at rest (equal weight between gravity
    %%%vectors and axes of rotation
     Shin = [Axes.KF';SensData.datasegment(set).RightMeaned.Shin.Accel(1,2:4)];
     Foot = [Axes.AF';SensData.datasegment(set).RightMeaned.Foot.Accel(1+sample_offset,2:4)];
    for i=4:length(ActData.datasegment(set).Right.PredLabelsT)-3
        if (ActData.datasegment(set).Right.PredLabelsT(i)==0&&ActData.datasegment(set).Right.PredLabelsT(i+1)==0 ...
            &&ActData.datasegment(set).Right.PredLabelsT(i-1)==0&&ActData.datasegment(set).Right.PredLabelsT(i+2)==0 ...
            &&ActData.datasegment(set).Right.PredLabelsT(i-2)==0&&ActData.datasegment(set).Right.PredLabelsT(i+3)==0 ...
            &&ActData.datasegment(set).Right.PredLabelsT(i-3))
            Shin = [Shin;Axes.KF';SensData.datasegment(set).RightMeaned.Shin.WinAccel{i}(:,2:4)];  %SensData.datasegment(set).Right.Shin.Accel(i,2:4)];
            Foot = [Foot;Axes.AF';SensData.datasegment(set).RightMeaned.Foot.WinAccel{i}(:,2:4)];  %SensData.datasegment(set).Right.Foot.Accel(i+sample_offset,2:4)];
        end
    end
    
    %%%Initial Rotation
    H = (Shin'*Foot);
    [U,~,V] = svd(H);
    d = det(V*U');
    R_FS_init = V*[1,0,0;0,1,0;0,0,d]*U';

    gyro_S = GyroShinTemp(1+sample_offset:end,:);
    gyro_F = GyroFootTemp(1:end-sample_offset,:);


    v_AF = Axes.AF;
    v_AI = Axes.AI;
    t_prev = gyro_F(1,1);

    theta_AF_prev = 0;
    angle_AF = [];
    theta_AI_prev = 0;
    angle_AI = [];
    
    for k=2:length(gyro_F(:,1))
        t = gyro_F(k,1);
        gyro_F_rem(k,1:4) = [gyro_F(k,1),(R_FS_init*(R_FS_init'*gyro_F(k,2:4)'-gyro_S(k,2:4)'))'];

        rates.dataset(set).Right.AF(k,1) = dot(v_AF,gyro_F_rem(k,2:4)');
        theta_dot_AF = rates.dataset(set).Right.AF(k);
        theta_AF = theta_AF_prev + theta_dot_AF*(t-t_prev);
        Rv_AF = cosd(theta_AF)*eye(3) + (1-cosd(theta_AF))*(v_AF*v_AF')+sind(theta_AF)*[0,-v_AF(3),v_AF(2);v_AF(3),0,-v_AF(1);-v_AF(2),v_AF(1),0];

        rates.dataset(set).Right.AI(k,1) = dot(v_AI,gyro_F_rem(k,2:4)');
        theta_dot_AI = rates.dataset(set).Right.AI(k);
        theta_AI = theta_AI_prev + theta_dot_AI*(t-t_prev);
        Rv_AI = cosd(theta_AI)*eye(3) + (1-cosd(theta_AI))*(v_AI*v_AI')+sind(theta_AI)*[0,-v_AI(3),v_AI(2);v_AI(3),0,-v_AI(1);-v_AI(2),v_AI(1),0];

        Rv =  (Rv_AI*Rv_AF)';

        %Iterate
        angle_AF = [angle_AF; theta_AF];
        theta_AF_prev = theta_AF;
        angle_AI = [angle_AI; theta_AI];
        theta_AI_prev = theta_AI;    
        t_prev = t;
    end
end    

save('JointRates.mat','rates');
clear