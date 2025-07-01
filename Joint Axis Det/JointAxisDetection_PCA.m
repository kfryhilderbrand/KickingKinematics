%% Joint axis detection with different combinations of sensor activity
%%% All 3 active, Shin and Foot active, Foot active
%%% Other combinations unlikely: Thigh and Shin active, Thigh and foot active
%%% Thigh active, Shin active

%%% 1 DOF movements (this method assumes that the axis of motion falls
%%% within certain quadrants....i.e. flexion axis will have a positive y
%%% component)
%% Load in ActData and SensData and set constants
load('ActivityData.mat')
load('SensorData.mat')
load('Truth.mat')

ConstantsFlags
windowsize = Constants.windowsize;
%%%Check for even windowsize
if mod(windowsize,2)
    windowsize = windowsize+1;
end
iterations = 150;
init = 0;
%init = rand;
Flag_flip = 0;

n = 1; %datasegment
%% Concatenate data
%%% Creates concatenation of all sensor data within each sensor when sensor
%%% is active
close all
GyroFootR = [];
GyroShinR = [];
GyroThighR = [];
for i=1:length(SensData.datasegment) %%%create vector of gyro data when active
   for j=1:length(ActData.datasegment(i).Right.Thigh.PredLabels)%j=1:length(SensData.datasegment(i).Right.Foot.WinTime)
        if ActData.datasegment(i).Right.Thigh.PredLabels(j)==1
            GyroThighR = [GyroThighR; [SensData.datasegment(i).Right.Thigh.WinTime(j),...
                SensData.datasegment(i).Right.Thigh.WinGyro{j}(1+windowsize/2,2:4)]];
        end
    end 
    for j=1:length(ActData.datasegment(i).Right.Foot.PredLabels)%j=1:length(SensData.datasegment(i).Right.Foot.WinTime)
        if ActData.datasegment(i).Right.Foot.PredLabels(j)==1
            GyroFootR = [GyroFootR; [SensData.datasegment(i).Right.Foot.WinTime(j),...
                SensData.datasegment(i).Right.Foot.WinGyro{j}(1+windowsize/2,2:4)]];
        end
    end
    for j=1:length(ActData.datasegment(i).Right.Shin.PredLabels)%j=1:length(SensData.datasegment(i).Right.Foot.WinTime)
        if ActData.datasegment(i).Right.Shin.PredLabels(j)==1
            GyroShinR = [GyroShinR; [SensData.datasegment(i).Right.Shin.WinTime(j),...
                SensData.datasegment(i).Right.Shin.WinGyro{j}(1+windowsize/2,2:4)]];
        end
    end    
end


%% PCA method
%%%Uses gyroscope data at the center of all active windows. PCA is run on
%%%that data and a component is considered to be a joint axis if the
%%%variance explained exceeds 95%. 

%%%%%%%%%%%%%%Add in error function (needs to be able to determine which
%%%%%%%%%%%%%%axis to compare to...

%%% Hip axes
[n, ~] = size(GyroThighR);
if n<100 %%%%%%%%%%%%%%%%%%%%%%%%This number may need to change
    disp('Not enough active thigh data available!')
end
for i=1:n
    [J1_pca,~,~,~,explained1,~] = pca(GyroThighR(:,2:4));
    if explained1(1) > 95
        SensAxes.Thigh.Primary = J1_pca(:,1);
    elseif explained1(1) + explained1(2) > 95
        SensAxes.Thigh.Primary = J1_pca(:,1);
        SensAxes.Thigh.Secondary = J1_pca(:,2);
    else
        SensAxes.Thigh.Primary = J1_pca(:,1);
        SensAxes.Thigh.Secondary = J1_pca(:,2);
        SensAxes.Thigh.Tertiary = J1_pca(:,3);
    end
end

%%% Knee axes
[n, ~] = size(GyroShinR);
if n==0
    disp('No active shin data available!')
end
for i=1:n
    [J2_pca,~,~,~,explained2,~] = pca(GyroShinR(:,2:4));
    if explained2(1) > 95
        SensAxes.Shin.Primary = J2_pca(:,1);
    else
        disp('Error! Something is weird with the knee.')
    end
end

%%% Ankle axes
[n, ~] = size(GyroFootR);
if n==0
    disp('No active foot data available!')
end
for i=1:n
    [J3_pca,~,~,~,explained3,~] = pca(GyroFootR(:,2:4));
    if explained3(1) > 95
        SensAxes.Foot.Primary = J3_pca(:,1);
    elseif explained3(1) + explained3(2) > 95
        SensAxes.Foot.Primary = J3_pca(:,1);
        SensAxes.Foot.Secondary = J3_pca(:,2);
    else
        disp('Error! Something is weird with the foot.')
    end
end
%% Ankle Flexion
[J3_pca,~,~,~,explained3,~] = pca(GyroFootR(:,2:4));
j3_pca = J3_pca(:,1);
error3_PCA = norm(j3_pca-CalAxes.Foot.AFlex');%/norm(CalAxes.Foot.AFlex);
angle3_PCA = atan2d(norm(cross(j3_pca,CalAxes.Foot.AFlex')), dot(j3_pca,CalAxes.Foot.AFlex'));

figure()
hold on
grid on
title('Ankle Flexion Axis')
quiver3(0,0,0, CalAxes.Foot.AFlex(1),CalAxes.Foot.AFlex(2),CalAxes.Foot.AFlex(3))
quiver3(0,0,0, j3_pca(1),j3_pca(2),j3_pca(3))
legend('Actual','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off
%% Ankle Inversion
[J3_pca,~,~,~,explained3,~] = pca(GyroFootR(:,2:4));
j3_pca = J3_pca(:,1);
error3_PCA = norm(j3_pca-CalAxes.Foot.AInv');%/norm(CalAxes.Foot.AInv);
angle3_PCA = atan2d(norm(cross(j3_pca,CalAxes.Foot.AInv')), dot(j3_pca,CalAxes.Foot.AInv'));

figure()
hold on
grid on
title('Ankle Inversion Axis')
quiver3(0,0,0, CalAxes.Foot.AInv(1),CalAxes.Foot.AInv(2),CalAxes.Foot.AInv(3))
quiver3(0,0,0, j3_act(1),j3_act(2),j3_act(3))
quiver3(0,0,0, j3_pca(1),j3_pca(2),j3_pca(3))
legend('Actual','EM','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off

%% Ankle Flexion + Inversion

%%%Need to add code to run EM until a certain percent decrease in variance
%%%is reached
% V1 = sum(var(GyroFootR(:,2:4)));
% V2 = sum(var(GyroFootR_res(:,2:4)));
% %dec=(V1-V2)./V1*100;
j31_act = JointAxis1Sens(iterations, init, GyroFootR, 1, CalAxes.Foot.AFlex);
B31 = zeros(length(GyroFootR(:,1)),1);
for k = 1:length(GyroFootR(:,1))
    g3 = GyroFootR(k,2:4)';
    
    %%% Find angular rate about gi axis.
    B31(k)=dot(j31_act,g3);
end
GyroFootR_res = [GyroFootR(:,1),GyroFootR(:,2:4)-B31.*j31_act']; %%Subtract off gyro around first converged axis
j32_act = JointAxis1Sens(iterations, init, GyroFootR_res, 1, CalAxes.Foot.AInv);
B32 = zeros(length(GyroFootR_res(:,1)),1);
for k = 1:length(GyroFootR_res(:,1))
    g3 = GyroFootR_res(k,2:4)';
    
    %%% Find angular rate about gi axis.
    B32(k)=dot(j32_act,g3);
end
GyroFootR_res2 = [GyroFootR_res(:,1),GyroFootR(:,2:4)-B31.*j31_act'-B32.*j32_act']; %%Subtract off gyro around first converged axis

%%%Checks for alignment
if j31_act(2)<0
    j31_act = -j31_act;
end
if j32_act(1)<0
    j32_act = -j32_act;
end

error31_EM = norm(j31_act-CalAxes.Foot.AFlex');%/norm(CalAxes.Foot.AFlex);
angle31_EM = atan2d(norm(cross(j31_act,CalAxes.Foot.AFlex')), dot(j31_act,CalAxes.Foot.AFlex'));
error32_EM = norm(j32_act-CalAxes.Foot.AInv');%/norm(CalAxes.Foot.AInv);
angle32_EM = atan2d(norm(cross(j32_act,CalAxes.Foot.AInv')), dot(j32_act,CalAxes.Foot.AInv'));


[J3_pca,~,~,~,explained3,~] = pca(GyroFootR(:,2:4));
j31_pca = J3_pca(:,1);
j32_pca = J3_pca(:,2);
error31_PCA = norm(j31_pca-CalAxes.Foot.AFlex');%/norm(CalAxes.Foot.AFlex);
angle31_PCA = atan2d(norm(cross(j31_pca,CalAxes.Foot.AFlex')), dot(j31_pca,CalAxes.Foot.AFlex'));
error32_PCA = norm(j32_pca-CalAxes.Foot.AInv');%/norm(CalAxes.Foot.AFlex);
angle32_PCA = atan2d(norm(cross(j32_pca,CalAxes.Foot.AInv')), dot(j32_pca,CalAxes.Foot.AInv'));


figure()
hold on
grid on
title('Ankle Flexion Axis')
quiver3(0,0,0, CalAxes.Foot.AFlex(1),CalAxes.Foot.AFlex(2),CalAxes.Foot.AFlex(3))
quiver3(0,0,0, j31_act(1),j31_act(2),j31_act(3))
quiver3(0,0,0, j31_pca(1),j31_pca(2),j31_pca(3))
legend('Actual','EM','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off

figure()
hold on
grid on
title('Ankle Inversion Axis')
quiver3(0,0,0, CalAxes.Foot.AInv(1),CalAxes.Foot.AInv(2),CalAxes.Foot.AInv(3))
quiver3(0,0,0, j32_act(1),j32_act(2),j32_act(3))
quiver3(0,0,0, j32_pca(1),j32_pca(2),j32_pca(3))
legend('Actual','EM','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off

%% Knee Flexion
%%%May want to add code to calculate error of knee flexion as seen by foot
%%%sensor by comparing Knee flexion axis conversion in foot sensor to the
%%%calibrated ankle axis.
%%%Alternatively, you could just use your error metrics to compare how
%%%different ankle flexion calibration axis and knee flexion calibration
%%%axis as seen by the foot sensor are...
j2_act = JointAxis1Sens(iterations, init, GyroShinR, 1, CalAxes.Shin.KFlex);
B2 = zeros(length(GyroShinR(:,1)),1);
for k = 1:length(GyroShinR(:,1))
    g2 = GyroShinR(k,2:4)';
    
    %%% Find angular rate about gi axis.
    B2(k)=dot(j2_act,g2);
end
GyroShinR_res = [GyroShinR(:,1),GyroShinR(:,2:4)-B2.*j2_act']; %%Subtract off gyro around first converged axis

%%%Checks for alignment
if j2_act(2)<0
    j2_act = -j2_act;
end

error2_EM = norm(j2_act-CalAxes.Shin.KFlex');%/norm(CalAxes.Foot.AFlex);
angle2_EM = atan2d(norm(cross(j2_act,CalAxes.Shin.KFlex')), dot(j2_act,CalAxes.Shin.KFlex'));

[J2_pca,~,~,~,explained2,~] = pca(GyroShinR(:,2:4));
j2_pca = J2_pca(:,1);
error2_PCA = norm(j2_pca-CalAxes.Shin.KFlex');%/norm(CalAxes.Foot.AFlex);
angle2_PCA = atan2d(norm(cross(j2_pca,CalAxes.Shin.KFlex')), dot(j2_pca,CalAxes.Shin.KFlex'));

figure()
hold on
grid on
title('Knee Flexion Axis')
quiver3(0,0,0, CalAxes.Shin.KFlex(1),CalAxes.Shin.KFlex(2),CalAxes.Shin.KFlex(3))
quiver3(0,0,0, j2_act(1),j2_act(2),j2_act(3))
quiver3(0,0,0, j2_pca(1),j2_pca(2),j2_pca(3))
legend('Actual','EM','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off

%%% Check if ankle flexion is approx knee flexion
j3_act = JointAxis1Sens(iterations, init, GyroFootR, 1, CalAxes.Foot.KFlex);
B3 = zeros(length(GyroFootR(:,1)),1);
for k = 1:length(GyroFootR(:,1))
    g3 = GyroFootR(k,2:4)';
    
    %%% Find angular rate about gi axis.
    B3(k)=dot(j3_act,g3);
end
GyroFootR_res = [GyroFootR(:,1),GyroFootR(:,2:4)-B3.*j3_act']; %%Subtract off gyro around first converged axis

%%%Checks for alignment
if j3_act(2)<0
    j3_act = -j3_act;
end

error3_EM = norm(j3_act-CalAxes.Foot.KFlex');%/norm(CalAxes.Foot.AFlex);
angle3_EM = atan2d(norm(cross(j3_act,CalAxes.Foot.KFlex')), dot(j3_act,CalAxes.Foot.KFlex'));

[J3_pca,~,~,~,explained3,~] = pca(GyroFootR(:,2:4));
j3_pca = J3_pca(:,1);
error3_PCA = norm(j3_pca-CalAxes.Foot.KFlex');%/norm(CalAxes.Foot.AFlex);
angle3_PCA = atan2d(norm(cross(j3_pca,CalAxes.Foot.KFlex')), dot(j3_pca,CalAxes.Foot.KFlex'));

figure()
hold on
grid on
title('Knee Flexion Axis (Foot Sensor)')
quiver3(0,0,0, CalAxes.Foot.KFlex(1),CalAxes.Foot.KFlex(2),CalAxes.Foot.KFlex(3))
quiver3(0,0,0, j3_act(1),j3_act(2),j3_act(3))
quiver3(0,0,0, j3_pca(1),j3_pca(2),j3_pca(3))
legend('Actual','EM','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off
%% Knee Flexion and Ankle Flexion
%%%Should be approximately the same axis...
%%%No PCA for combined joint movements...
j2_act = JointAxis1Sens(iterations, init, GyroShinR, 0, CalAxes.Shin.KFlex);
B2 = zeros(length(GyroShinR(:,1)),1);
for k = 1:length(GyroShinR(:,1))
    g2 = GyroShinR(k,2:4)';
    
    %%% Find angular rate about gi axis.
    B2(k)=dot(j2_act,g2);
end
GyroShinR_res = [GyroShinR(:,1),GyroShinR(:,2:4)-B2.*j2_act']; %%Subtract off gyro around first converged axis

%%%Checks for alignment
if j2_act(2)<0
    j2_act = -j2_act;
end

error2_EM = norm(j2_act-CalAxes.Shin.KFlex');%/norm(CalAxes.Foot.AFlex);
angle2_EM = atan2d(norm(cross(j2_act,CalAxes.Shin.KFlex')), dot(j2_act,CalAxes.Shin.KFlex'));

[J2_pca,~,~,~,explained2,~] = pca(GyroShinR(:,2:4));
j2_pca = J2_pca(:,1);
error2_PCA = norm(j2_pca-CalAxes.Shin.KFlex');%/norm(CalAxes.Foot.AFlex);
angle2_PCA = atan2d(norm(cross(j2_pca,CalAxes.Shin.KFlex')), dot(j2_pca,CalAxes.Shin.KFlex'));

figure()
hold on
grid on
title('Knee Flexion Axis')
quiver3(0,0,0, CalAxes.Shin.KFlex(1),CalAxes.Shin.KFlex(2),CalAxes.Shin.KFlex(3))
quiver3(0,0,0, j2_act(1),j2_act(2),j2_act(3))
quiver3(0,0,0, j2_pca(1),j2_pca(2),j2_pca(3))
legend('Actual','EM','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off

j3_act = JointAxis1Sens(iterations, init, GyroFootR, 0, CalAxes.Foot.KFlex);
B3 = zeros(length(GyroFootR(:,1)),1);
for k = 1:length(GyroFootR(:,1))
    g3 = GyroFootR(k,2:4)';
    
    %%% Find angular rate about gi axis.
    B3(k)=dot(j3_act,g3);
end
GyroFootR_res = [GyroFootR(:,1),GyroFootR(:,2:4)-B3.*j3_act']; %%Subtract off gyro around first converged axis

%%%Checks for alignment
if j3_act(2)<0
    j3_act = -j3_act;
end

error3_EM = norm(j3_act-CalAxes.Foot.KFlex');%/norm(CalAxes.Foot.AFlex);
angle3_EM = atan2d(norm(cross(j3_act,CalAxes.Foot.KFlex')), dot(j3_act,CalAxes.Foot.KFlex'));

[J3_pca,~,~,~,explained3,~] = pca(GyroFootR(:,2:4));
j3_pca = J3_pca(:,1);
error3_PCA = norm(j3_pca-CalAxes.Foot.KFlex');%/norm(CalAxes.Foot.AFlex);
angle3_PCA = atan2d(norm(cross(j3_pca,CalAxes.Foot.KFlex')), dot(j3_pca,CalAxes.Foot.KFlex'));

figure()
hold on
grid on
title('Knee Flexion Axis (Foot Sensor)')
quiver3(0,0,0, CalAxes.Foot.KFlex(1),CalAxes.Foot.KFlex(2),CalAxes.Foot.KFlex(3))
quiver3(0,0,0, j3_act(1),j3_act(2),j3_act(3))
quiver3(0,0,0, j3_pca(1),j3_pca(2),j3_pca(3))
legend('Actual','EM','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off

%% Knee Flexion and Ankle Inversion
%%%No PCA for combined joint movements...
j2_act = JointAxis1Sens(iterations, init, GyroShinR, 0, CalAxes.Shin.KFlex);
B2 = zeros(length(GyroShinR(:,1)),1);
for k = 1:length(GyroShinR(:,1))
    g2 = GyroShinR(k,2:4)';
    
    %%% Find angular rate about gi axis.
    B2(k)=dot(j2_act,g2);
end
GyroShinR_res = [GyroShinR(:,1),GyroShinR(:,2:4)-B2.*j2_act']; %%Subtract off gyro around first converged axis

%%%Checks for alignment
if j2_act(2)<0
    j2_act = -j2_act;
end

error2_EM = norm(j2_act-CalAxes.Shin.KFlex');%/norm(CalAxes.Foot.AFlex);
angle2_EM = atan2d(norm(cross(j2_act,CalAxes.Shin.KFlex')), dot(j2_act,CalAxes.Shin.KFlex'));

[J2_pca,~,~,~,explained2,~] = pca(GyroShinR(:,2:4));
j2_pca = J2_pca(:,1);
error2_PCA = norm(j2_pca-CalAxes.Shin.KFlex');%/norm(CalAxes.Foot.AFlex);
angle2_PCA = atan2d(norm(cross(j2_pca,CalAxes.Shin.KFlex')), dot(j2_pca,CalAxes.Shin.KFlex'));

figure()
hold on
grid on
title('Knee Flexion Axis')
quiver3(0,0,0, CalAxes.Shin.KFlex(1),CalAxes.Shin.KFlex(2),CalAxes.Shin.KFlex(3))
quiver3(0,0,0, j2_act(1),j2_act(2),j2_act(3))
quiver3(0,0,0, j2_pca(1),j2_pca(2),j2_pca(3))
legend('Actual','EM','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off

%%%
j31_act = JointAxis1Sens(iterations, init, GyroFootR, 1, CalAxes.Foot.AInv);
B31 = zeros(length(GyroFootR(:,1)),1);
for k = 1:length(GyroFootR(:,1))
    g3 = GyroFootR(k,2:4)';
    
    %%% Find angular rate about gi axis.
    B31(k)=dot(j31_act,g3);
end
GyroFootR_res = [GyroFootR(:,1),GyroFootR(:,2:4)-B31.*j31_act']; %%Subtract off gyro around first converged axis
j32_act = JointAxis1Sens(iterations, init, GyroFootR_res, 0, CalAxes.Foot.KFlex);
B32 = zeros(length(GyroFootR_res(:,1)),1);
for k = 1:length(GyroFootR_res(:,1))
    g3 = GyroFootR_res(k,2:4)';
    
    %%% Find angular rate about gi axis.
    B32(k)=dot(j32_act,g3);
end
GyroFootR_res2 = [GyroFootR_res(:,1),GyroFootR(:,2:4)-B31.*j31_act'-B32.*j32_act']; %%Subtract off gyro around first converged axis

%%%Checks for alignment
% if j31_act(2)<0
%     j31_act = -j31_act;
% end
% if j32_act(1)<0
%     j32_act = -j32_act;
% end

error31_EM = norm(j31_act-CalAxes.Foot.AInv');%/norm(CalAxes.Foot.AInv);
angle31_EM = atan2d(norm(cross(j31_act,CalAxes.Foot.AInv')), dot(j31_act,CalAxes.Foot.AInv'));
error32_EM = norm(j32_act-CalAxes.Foot.KFlex');%/norm(CalAxes.Foot.KFlex);
angle32_EM = atan2d(norm(cross(j32_act,CalAxes.Foot.KFlex')), dot(j32_act,CalAxes.Foot.KFlex'));

[J3_pca,~,~,~,explained3,~] = pca(GyroFootR(:,2:4));
j31_pca = J3_pca(:,1);
j32_pca = J3_pca(:,2);
error31_PCA = norm(j31_pca-CalAxes.Foot.AInv');%/norm(CalAxes.Foot.AFlex);
angle31_PCA = atan2d(norm(cross(j31_pca,CalAxes.Foot.AInv')), dot(j31_pca,CalAxes.Foot.AInv'));
error32_PCA = norm(j32_pca-CalAxes.Foot.AFlex');%/norm(CalAxes.Foot.AFlex);
angle32_PCA = atan2d(norm(cross(j32_pca,CalAxes.Foot.AFlex')), dot(j32_pca,CalAxes.Foot.AFlex'));

figure()
hold on
grid on
title('Ankle Inversion')
quiver3(0,0,0, CalAxes.Foot.AInv(1),CalAxes.Foot.AInv(2),CalAxes.Foot.AInv(3))
quiver3(0,0,0, j31_act(1),j31_act(2),j31_act(3))
quiver3(0,0,0, j31_pca(1),j31_pca(2),j31_pca(3))
legend('Actual','EM','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off

figure()
hold on
grid on
title('Knee Flexion Axis (Foot Sensor)')
quiver3(0,0,0, CalAxes.Foot.KFlex(1),CalAxes.Foot.KFlex(2),CalAxes.Foot.KFlex(3))
quiver3(0,0,0, j32_act(1),j32_act(2),j32_act(3))
quiver3(0,0,0, j32_pca(1),j32_pca(2),j32_pca(3))
legend('Actual','EM','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off
%% PCA method (only for 1 DOF movement per joint...need to adjust foot and thigh code)

%%%%%Foot 2 DOF potentially (this calculates 1 per window...maybe
%%%%%concatenate all data together and run pca on that
J3_pca = [];
Explained3 = [];
for j=1:length(ActData.datasegment(n).Right.Foot.PredLabels)
    if ActData.datasegment(n).Right.Foot.PredLabels(j)==1
        Gyro3 = SensData.datasegment(n).Right.Foot.WinGyro{j};
        %[j3_pca,rates_pca,~] = pca(Gyro(:,2:4));
        [j3_pca,~,~,~,explained3,~] = pca(Gyro3(:,2:4));
        if explained3(1) > 95
            J3_pca = [J3_pca; j3_pca(:,1)'];
            Explained3 = [Explained3; explained3'];
        end
    end
end
J3_pca_avg = mean(J3_pca)/norm(mean(J3_pca));
error3_PCA = norm(J3_pca_avg-Axes.Foot(2,:));%/norm(Axes.Foot(2,:));
clear j3 j3_pca Gyro3 explained3 rates_pca J3_pca

%%%%%Shin
J2_pca = [];
for j=1:length(ActData.datasegment(n).Right.Shin.PredLabels)
    if ActData.datasegment(n).Right.Shin.PredLabels(j)==1
        Gyro2 = SensData.datasegment(n).Right.Shin.WinGyro{j};
        %[j3_pca,rates_pca,~] = pca(Gyro(:,2:4));
        [j2_pca,~,~,~,explained2,~] = pca(Gyro2(:,2:4));
        if explained2(1) > 95
            J2_pca = [J2_pca; j2_pca(:,1)'];
        end
    end
end
J2_pca_avg = mean(J2_pca)/norm(mean(J2_pca));
error2_PCA = norm(J2_pca_avg-Axes.Shin(1,:));%/norm(Axes.Shin(1,:));

clear j2 j2_pca J2_pca Gyro2 explained2 rates_pca

%%%%%Thigh
J1_pca = [];
for j=1:length(ActData.datasegment(n).Right.Thigh.PredLabels)
    if ActData.datasegment(n).Right.Thigh.PredLabels(j)==1
        Gyro1 = SensData.datasegment(n).Right.Thigh.WinGyro{j};
        %[j3_pca,rates_pca,~] = pca(Gyro(:,2:4));
        [j1_pca,~,~,~,explained1,~] = pca(Gyro1(:,2:4));
        if explained1(1) > 95
            J1_pca = [J1_pca; j1_pca(:,1)'];
        end
    end
end
J1_pca_avg = mean(J1_pca)/norm(mean(J1_pca));
error1_PCA = norm(J1_pca_avg-Axes.Thigh(1,:));%/norm(Axes.Thigh(1,:));

clear j1 j1_pca J1_pca Gyro1 explained1 rates_pca

%% 1 sensor method
%%% When activity detected on foot. Unlikely that shin or thigh will be
%%% moving on their own.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Most of this will need to change
J3=[];
for j=1:length(ActData.datasegment(n).Right.Foot.PredLabels)
    if ActData.datasegment(n).Right.Foot.PredLabels(j)==1
        Gyro3 = SensData.datasegment(n).Right.Foot.WinGyro{j};
        j3 = JointAxis1Sens(iterations, init, Gyro3, 0, CalAxes.Foot.AFlex);%%%%%%%%%
        J3 = [J3; j3'];
    end
end
clear j3 Gyro3


%%
temp = [];
%%%%enforces constraint that axes of motion are limited to quandrants with
%%%%positive y component.
for i=1:length(J3(:,1))
    if norm(J3(i,:))
        if J3(i,2)<0
            J3(i,:)=-J3(i,:);
        end
    temp = [temp; J3(i,:)];
    end
end
J3_avg = mean(temp)/norm(mean(temp));
error3 = norm(J3_avg-Axes.Foot(2,:));%/norm(Axes.Foot(2,:));

clear i j temp

figure()
hold on
grid on
title('Foot Axis')
quiver3(0,0,0, Axes.Foot(2,1),Axes.Foot(2,2),Axes.Foot(2,3))
quiver3(0,0,0, J3_avg(1),J3_avg(2),J3_avg(3))
quiver3(0,0,0, J3_pca_avg(1),J3_pca_avg(2),J3_pca_avg(3))
legend('Actual','EM','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off

%% 1 sensor 2 DOF
J3=[];
J2=[];
for j=1:length(ActData.datasegment(n).Right.Foot.PredLabels)
    if ActData.datasegment(n).Right.Foot.PredLabels(j)==1
        Gyro3 = SensData.datasegment(n).Right.Foot.WinGyro{j};
        %j3 = JointAxis1Sens(iterations, init, Gyro3, 0, Axes.Foot(1,:));
        [j2, j3] = JointAxis2Sens(iterations, init, Gyro3, Gyro3);
        J3 = [J3; j3'];
        J2 = [J2; j2'];
    end
end
clear j3 Gyro3


% %% 1 sensor concat activity data
% 
% j3_act = JointAxis1Sens(iterations, init, GyroFootR, 0, Axes.Foot(1,:));
% [j3_pca_act,rates_pca_act,~] = pca(GyroFootR(:,2:4));
% 
% for k = 1:length(GyroFootR(:,1))
%     g3 = GyroFootR(k,2:4)';
%     
%     %%% Find angular rate about gi axis.
%     b3(k)=dot(j3_act(:,1),g3);
%     b3pca(k)=dot(j3_pca_act(:,1),g3);
% end
% b3=b3';
% b3pca=b3pca';
% % % % % %% Other
% % % % % i=1;
% % % % % J3=[];
% % % % % J2=[];
% % % % % J3_pca = [];
% % % % % J2_pca = [];
% % % % % for j=1:length(ActData.datasegment(i).Right.Foot.PredLabels)
% % % % %     if ActData.datasegment(i).Right.Foot.PredLabels(j)==1
% % % % %         Gyro = SensData.datasegment(i).Right.Foot.WinGyro{j};
% % % % %         %[j3_pca,rates_pca,~] = pca(Gyro(:,2:4));
% % % % %         [j3_pca,rates_pca,~,~,explained,~] = pca(Gyro(:,2:4));
% % % % %         if explained(1) > 95
% % % % %             J3_pca = [J3_pca; j3_pca(:,1)'];
% % % % %         end
% % % % %         [j2, j3] = JointAxis2Sens(iterations, init, Gyro, Gyro);
% % % % %         J3 = [J3; j3'];
% % % % %         J2 = [J2; j2'];
% % % % %     end
% % % % % end
% % % % % temp = [];
% % % % % %%%%enforces constraint that axes of motion are limited to quandrants with
% % % % % %%%%positive y component.
% % % % % for i=1:length(J2(:,1))
% % % % %     if norm(J2(i,:))
% % % % %         if J2(i,2)<0
% % % % %             J2(i,:)=-J2(i,:);
% % % % %         end
% % % % %     temp = [temp; J2(i,:)];
% % % % %     end
% % % % % end
% % % % % J2_avg = mean(temp)/norm(mean(temp));
% % % % % error2 = norm(J2_avg-Axes.Foot(:,2))/norm(Axes.Foot(:,2));
% % % % % 
% % % % % temp = [];
% % % % % %%%%enforces constraint that axes of motion are limited to quandrants with
% % % % % %%%%positive y component.
% % % % % for i=1:length(J3(:,1))
% % % % %     if norm(J3(i,:))
% % % % %         if J3(i,2)<0
% % % % %             J3(i,:)=-J3(i,:);
% % % % %         end
% % % % %     temp = [temp; J3(i,:)];
% % % % %     end
% % % % % end
% % % % % J3_avg = mean(temp)/norm(mean(temp));
% % % % % error3 = norm(J3_avg-Axes.Foot(:,2))/norm(Axes.Foot(:,2));


%% 2 sensor method SHIN and FOOT
%%% When activity detected on foot. thigh and foot or thigh and shin will
%%% be moving on their own

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%EM on shin data alone to find knee flexion
J2=[];
for j=1:length(ActData.datasegment(n).Right.Shin.PredLabels)
    if ActData.datasegment(n).Right.Shin.PredLabels(j)==1
        Gyro2 = SensData.datasegment(n).Right.Shin.WinGyro{j};
        j2 = JointAxis1Sens(iterations, init, Gyro2, 0, Axes.Shin(1,:));
        J2 = [J2; j2'];
    end
end
clear j2 Gyro2

temp = [];
%%%%enforces constraint that axes of motion are limited to quandrants with
%%%%positive y component.
for i=1:length(J2(:,1))
    if norm(J2(i,:))
        if J2(i,2)<0
            J2(i,:)=-J2(i,:);
        end
    temp = [temp; J2(i,:)];
    end
end
J2_avg = mean(temp)/norm(mean(temp));
error2 = norm(J2_avg-Axes.Shin(1,:));%/norm(Axes.Shin(1,:));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%EM on foot and shin
J3=[];
for j=1:length(ActData.datasegment(n).Right.Shin.PredLabels)
    if ActData.datasegment(n).Right.Shin.PredLabels(j)==1&&ActData.datasegment(n).Right.Foot.PredLabels(j)==1
        Gyro2 = SensData.datasegment(n).Right.Shin.WinGyro{j};
        Gyro3 = SensData.datasegment(n).Right.Foot.WinGyro{j};        
        j3 = JointAxis2SensKnown(iterations, init, Gyro2, Gyro3, J2_avg);
        J3 = [J3; j3'];
    end
end
clear j3 Gyro2 Gyro3

temp = [];
%%%%enforces constraint that axes of motion are limited to quandrants with
%%%%positive y component.
for i=1:length(J3(:,1))
    if norm(J3(i,:))
        if J3(i,2)<0
            J3(i,:)=-J3(i,:);
        end
    temp = [temp; J3(i,:)];
    end
end
J3_avg = mean(temp)/norm(mean(temp));
error3 = norm(J3_avg-Axes.Foot(2,:));%/norm(Axes.Foot(2,:));

clear i j temp

figure()
hold on
grid on
title('Shin Axis')
quiver3(0,0,0, Axes.Shin(1,1),Axes.Shin(1,2),Axes.Shin(1,3))
quiver3(0,0,0, J2_avg(1),J2_avg(2),J2_avg(3))
quiver3(0,0,0, J2_pca_avg(1),J2_pca_avg(2),J2_pca_avg(3))
legend('Actual','EM','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off

figure()
hold on
grid on
title('Foot Axis')
quiver3(0,0,0, Axes.Foot(2,1),Axes.Foot(2,2),Axes.Foot(2,3))
quiver3(0,0,0, J3_avg(1),J3_avg(2),J3_avg(3))
quiver3(0,0,0, J3_pca_avg(1),J3_pca_avg(2),J3_pca_avg(3))
legend('Actual','EM','PCA')
xlabel('x')
ylabel('y')
zlabel('z')
%axis([ -1 1 -1 1 -1 1])
hold off

%% 2 sensor method
%%% When activity detected on foot and shin. Unlikely that thigh and shin 
%%% or thigh and foot will be moving on their own (without foot).
i=1;
for j=1:length(ActData.datasegment(i).Left.Foot.PredLabels)
    if (ActData.datasegment(i).Left.Foot.PredLabels(j)==1)&&...
            (ActData.datasegment(i).Left.Shin.PredLabels(j)==1)
        Gyro2 = SensData.datasegment(i).Left.Shin.WinGyro{j};
        Gyro3 = SensData.datasegment(i).Left.Foot.WinGyro{j};
        [j2, j3] = JointAxis2Sens(100, init, Gyro2, Gyro3);
    end
end


%%%%Test the above with the NAO data.  Need to create truth for the
%%%%HipFlexion, KneeFlexion, and AnkleFlexion data. Then run the Main_Init
%%%%to create files. Finally, test the above.


%% 3 sensor method hierarchy
%%% When activity detected on thigh, shin and foot. This assumes the
%%% alignment of hip sensor is "correct." I.e. the shin and foot sensor
%%% will align to this.

%%% Expected Conv
thigh_conv = [0.99,-0.02,0];
shin_conv = [0.59,0.81,-0.05];
foot_conv = [-0.16,0.99,0.01];

%%% Initialization
phi_1 = 0;
theta_1 = 0;
phi_2 = 0;
theta_2 = 0;
phi_3 = 0;
theta_3 = 0;

% phi_1 = rand;
% theta_1 = rand;
% phi_2 = rand;
% theta_2 = rand;
% phi_3 = rand;
% theta_3 = rand;

init = [phi_1, theta_1, phi_2, theta_2, phi_3, theta_2];

%%% Convergence for thigh and shin
x = [phi_1, theta_1, phi_2, theta_2]'; %%% define state vector x

%1 define joing axes wrt frames
j1 = [cos(phi_1)*cos(theta_1), cos(phi_1)*sin(theta_1), sin(phi_1)]';
j2 = [cos(phi_2)*cos(theta_2), cos(phi_2)*sin(theta_2), sin(phi_2)]';
j1_conv = j1';
j2_conv = j2';
conv_iter = 0;
for iter=1:iterations
    conv_iter = [conv_iter; iter];

    %2 Calculate error vector
    e = zeros(length(KickingData.Resamp{1}{2}(1:end,1)),1); 
    J = [];
    for k=1:length(KickingData.Resamp{1}{2}(1:end,1)) %First cell index of Resamp corresponds to which sensor, second cell index indicates accel or gyro (1 or 2)
        g1 = KickingData.Resamp{3}{2}(k,2:4)';
        g2 = KickingData.Resamp{2}{2}(k,2:4)';
        e(k) = norm(cross(j1,g1))-norm(cross(j2,g2));

        
        %3 Calculate Jacobian and pseudo inverse of Jacobian  (J dim is nx4)
        A1 = -2*cos(phi_1)*(sin(phi_1)*sin(theta_1)*g1(3)*g1(2)+sin(phi_1)*cos(theta_1)*g1(1)*g1(3)+sin(theta_1)*cos(theta_1)*cos(phi_1)*g1(2)*g1(1))+...
            (cos(phi_1)^2*sin(theta_1)^2)*(g1(3)^2+g1(1)^2)+sin(phi_1)^2*(g1(1)^2+g1(2)^2)+(cos(phi_1)^2*cos(theta_1)^2)*(g1(2)^2+g1(3)^2);
        A2 = -2*cos(phi_2)*(sin(phi_2)*sin(theta_2)*g2(3)*g2(2)+sin(phi_2)*cos(theta_2)*g2(1)*g2(3)+sin(theta_2)*cos(theta_2)*cos(phi_2)*g2(2)*g2(1))+...
            (cos(phi_2)^2*sin(theta_2)^2)*(g2(3)^2+g2(1)^2)+sin(phi_2)^2*(g2(1)^2+g2(2)^2)+(cos(phi_2)^2*cos(theta_2)^2)*(g2(2)^2+g2(3)^2);
        
        temp1 = (.5*(A1)^-.5)*(sin(2*phi_1)*((g1(1)^2+g1(2)^2)-sin(theta_1)^2*(g1(3)^2+g1(1)^2)-cos(theta_1)^2*(g1(2)^2+g1(3)^2))+...
            2*sin(theta_1)*g1(3)*g1(2)*(sin(phi_1)^2-cos(phi_1)^2)+2*cos(theta_1)*g1(1)*g1(3)*(sin(phi_1)^2-cos(phi_1)^2)+...
            4*sin(phi_1)*cos(phi_1)*sin(theta_1)*cos(theta_1)*g1(2)*g1(1)); %partial of e wrt phi_1
        temp3 = -(.5*(A2)^-.5)*(sin(2*phi_2)*((g2(1)^2+g2(2)^2)-sin(theta_2)^2*(g2(3)^2+g2(1)^2)-cos(theta_2)^2*(g2(2)^2+g2(3)^2))+...
            2*sin(theta_2)*g2(3)*g2(2)*(sin(phi_2)^2-cos(phi_2)^2)+2*cos(theta_2)*g2(1)*g2(3)*(sin(phi_2)^2-cos(phi_2)^2)+...
            4*sin(phi_2)*cos(phi_2)*sin(theta_2)*cos(theta_2)*g2(2)*g2(1)); %partial of e wrt phi_2
        
        temp2 = (.5*(A1)^-.5)*(sin(2*theta_1)*cos(phi_1)^2*(g1(1)^2-g1(2)^2)-2*cos(phi_1)^2*g1(2)*g1(1)*(cos(theta_1)^2-sin(theta_1)^2)-...
            2*cos(phi_1)*sin(phi_1)*g1(3)*(cos(theta_1)*g1(2)-sin(theta_1)*g1(1)));%partial of e wrt theta_1
        temp4 = -(.5*(A2)^-.5)*(sin(2*theta_2)*cos(phi_2)^2*(g2(1)^2-g2(2)^2)-2*cos(phi_2)^2*g2(2)*g2(1)*(cos(theta_2)^2-sin(theta_2)^2)-...
            2*cos(phi_2)*sin(phi_2)*g2(3)*(cos(theta_2)*g2(2)-sin(theta_2)*g2(1)));%partial of e wrt theta_1
        
        temp = [temp1, temp2, temp3, temp4];
        J = [J; temp];
        
    end
    J_plus = pinv(J);
    
    %4 Update x (j1 and j2)
    x = x - J_plus*e; %x dim 4x1, J_plus dim 4xn, e dim nx1
    x = wrapToPi(x);%wrap angles to [-pi,pi]
    
    phi_1 = x(1);
    theta_1 = x(2);
    phi_2 = x(3);
    theta_2 = x(4);
       
    j1 = [cos(phi_1)*cos(theta_1), cos(phi_1)*sin(theta_1), sin(phi_1)]';
    j2 = [cos(phi_2)*cos(theta_2), cos(phi_2)*sin(theta_2), sin(phi_2)]';
    
%     % Check alignment j1 for flexion
%     if theta_1>pi/2
%         check1 = theta_1-pi;
%         check2 = -phi_1;
%         j1 = [cos(check2)*cos(check1), cos(check2)*sin(check1), sin(check2)]';
%     elseif theta_1<-pi/2
%         check1 = theta_1+pi;
%         check2 = -phi_1;
%         j1 = [cos(check2)*cos(check1), cos(check2)*sin(check1), sin(check2)]';
%     elseif phi_1>pi/2
%         check1 = -theta_1;
%         check2 = phi_1-pi;
%         j1 = [cos(check2)*cos(check1), cos(check2)*sin(check1), sin(check2)]';
%     elseif phi_1<-pi/2
%         check1 = -theta_1;
%         check2 = phi_1+pi;
%         j1 = [cos(check2)*cos(check1), cos(check2)*sin(check1), sin(check2)]';
%     end
% % 
%       % Check alignment j1 for abduction/inversion
%     if theta_1>0
%         check1 = theta_1-pi;
%         check2 = -phi_1;
%         j1 = [cos(check2)*cos(check1), cos(check2)*sin(check1), sin(check2)]';
%     elseif theta_1<-pi
%         check1 = theta_1+pi;
%         check2 = -phi_1;
%         j1 = [cos(check2)*cos(check1), cos(check2)*sin(check1), sin(check2)]';
%     elseif phi_1>0
%         check1 = -theta_1;
%         check2 = phi_1-pi;
%         j1 = [cos(check2)*cos(check1), cos(check2)*sin(check1), sin(check2)]';
%     elseif phi_1<-pi
%         check1 = -theta_1;
%         check2 = phi_1+pi;
%         j1 = [cos(check2)*cos(check1), cos(check2)*sin(check1), sin(check2)]';
%     end
% 
        
    j1_conv = [ j1_conv; j1'];
    j2_conv = [ j2_conv; j2'];
end

% Now that we have j2, can solve for j3
j3 = [cos(phi_3)*cos(theta_3), cos(phi_3)*sin(theta_3), sin(phi_3)]';
j3_conv = j3';
conv_iter = 0;
for iter=1:iterations
    conv_iter = [conv_iter; iter];
    
    %2 Calculate error vector
    e = zeros(length(KickingData.Resamp{1}{2}(1:end,1)),1); %%%%%%%%%%%%%%%%
    J = [];
    for k=1:length(KickingData.Resamp{1}{2}(1:end,1)) %First cell index of Resamp corresponds to which sensor, second cell index indicates accel or gyro (1 or 2)
        
        g2 = KickingData.Resamp{2}{2}(k,2:4)';
        g3 = KickingData.Resamp{1}{2}(k,2:4)';
        e(k) = norm(cross(j2,g2))-norm(cross(j3,g3));
        
        %3 Calculate Jacobian and pseudo inverse of Jacobian  (J dim is nx4)
        A2 = -2*cos(phi_2)*(sin(phi_2)*sin(theta_2)*g2(3)*g2(2)+sin(phi_2)*cos(theta_2)*g2(1)*g2(3)+sin(theta_2)*cos(theta_2)*cos(phi_2)*g2(2)*g2(1))+...
            (cos(phi_2)^2*sin(theta_2)^2)*(g2(3)^2+g2(1)^2)+sin(phi_2)^2*(g2(1)^2+g2(2)^2)+(cos(phi_2)^2*cos(theta_2)^2)*(g2(2)^2+g2(3)^2);
        A3 = -2*cos(phi_3)*(sin(phi_3)*sin(theta_3)*g3(3)*g3(2)+sin(phi_3)*cos(theta_3)*g3(1)*g3(3)+sin(theta_3)*cos(theta_3)*cos(phi_3)*g3(2)*g3(1))+...
            (cos(phi_3)^2*sin(theta_3)^2)*(g3(3)^2+g3(1)^2)+sin(phi_3)^2*(g3(1)^2+g3(2)^2)+(cos(phi_3)^2*cos(theta_3)^2)*(g3(2)^2+g3(3)^2);
              

        temp3 = -(.5*(A2)^-.5)*(sin(2*phi_2)*((g2(1)^2+g2(2)^2)-sin(theta_2)^2*(g2(3)^2+g2(1)^2)-cos(theta_2)^2*(g2(2)^2+g2(3)^2))+...
            2*sin(theta_2)*g2(3)*g2(2)*(sin(phi_2)^2-cos(phi_2)^2)+2*cos(theta_2)*g2(1)*g2(3)*(sin(phi_2)^2-cos(phi_2)^2)+...
            4*sin(phi_2)*cos(phi_2)*sin(theta_2)*cos(theta_2)*g2(2)*g2(1)); %partial of e wrt phi_2
        temp5 = -(.5*(A3)^-.5)*(sin(2*phi_3)*((g3(1)^2+g3(2)^2)-sin(theta_3)^2*(g3(3)^2+g3(1)^2)-cos(theta_3)^2*(g3(2)^2+g3(3)^2))+...
            2*sin(theta_3)*g3(3)*g3(2)*(sin(phi_3)^2-cos(phi_3)^2)+2*cos(theta_3)*g3(1)*g3(3)*(sin(phi_3)^2-cos(phi_3)^2)+...
            4*sin(phi_3)*cos(phi_3)*sin(theta_3)*cos(theta_3)*g3(2)*g3(1)); %partial of e wrt phi_2
        

        temp4 = -(.5*(A2)^-.5)*(sin(2*theta_2)*cos(phi_2)^2*(g2(1)^2-g2(2)^2)-2*cos(phi_2)^2*g2(2)*g2(1)*(cos(theta_2)^2-sin(theta_2)^2)-...
            2*cos(phi_2)*sin(phi_2)*g2(3)*(cos(theta_2)*g2(2)-sin(theta_2)*g2(1)));%partial of e wrt theta_1
        temp6 = -(.5*(A3)^-.5)*(sin(2*theta_3)*cos(phi_3)^2*(g3(1)^2-g3(2)^2)-2*cos(phi_3)^2*g3(2)*g3(1)*(cos(theta_3)^2-sin(theta_3)^2)-...
            2*cos(phi_3)*sin(phi_3)*g3(3)*(cos(theta_3)*g3(2)-sin(theta_3)*g3(1)));%partial of e wrt theta_1
        
        temp = [temp3, temp4, temp5, temp6];
        J = [J; temp];
        
    end
    J_plus = pinv(J);
    
    %4 Update x (j1 and j2)
    x = x - J_plus*e; %x dim 4x1, J_plus dim 4xn, e dim nx1
    x = wrapToPi(x);%wrap angles to [-pi,pi]
    
    phi_3 = x(3);
    theta_3 = x(4);
    
    j3 = [cos(phi_3)*cos(theta_3), cos(phi_3)*sin(theta_3), sin(phi_3)]'; 
    j3_conv = [ j3_conv; j3'];
end

figure()
subplot(3,1,1)
p=plot(conv_iter,j1_conv(:,1), conv_iter,j1_conv(:,2), conv_iter,j1_conv(:,3));
axis([0 iter -1.25 1.25])
title('RThigh convergence')
line([0,iter],[thigh_conv(1),thigh_conv(1)],'Color',p(1).Color,'LineStyle',':') %X conv
line([0,iter],[thigh_conv(2),thigh_conv(2)],'Color',p(2).Color,'LineStyle',':') %y conv
line([0,iter],[thigh_conv(3),thigh_conv(3)],'Color',p(3).Color,'LineStyle',':') %Z conv
legend('x axis', 'y axis', 'z axis')

subplot(3,1,2)
p=plot(conv_iter,j2_conv(:,1), conv_iter,j2_conv(:,2), conv_iter,j2_conv(:,3));
axis([0 iter -1.25 1.25])
title('RShin convergence')
line([0,iter],[shin_conv(1),shin_conv(1)],'Color',p(1).Color,'LineStyle',':') %X conv
line([0,iter],[shin_conv(2),shin_conv(2)],'Color',p(2).Color,'LineStyle',':') %y conv
line([0,iter],[shin_conv(3),shin_conv(3)],'Color',p(3).Color,'LineStyle',':') %Z conv
legend('x axis', 'y axis', 'z axis')

subplot(3,1,3)
plot(conv_iter,j3_conv(:,1), conv_iter,j3_conv(:,2), conv_iter,j3_conv(:,3))
axis([0 iter -1.25 1.25])
title('RFoot convergence')
line([0,iter],[foot_conv(1),foot_conv(1)],'Color',p(1).Color,'LineStyle',':') %X conv
line([0,iter],[foot_conv(2),foot_conv(2)],'Color',p(2).Color,'LineStyle',':') %y conv
line([0,iter],[foot_conv(3),foot_conv(3)],'Color',p(3).Color,'LineStyle',':') %Z conv
legend('x axis', 'y axis', 'z axis')

clearvars -except j1 j2 j3 j1_conv j2_conv j3_conv init KickingData x

%% Alignment check attempt 3 (angles)
ind = 1;
thresh = 10; %%Threshold for angular rate for maxima and minima
%%%In actual implementation, will probably just look within periods of
%%%activity, will likely filter out more noisy peaks
for k=1:length(KickingData.Resamp{3}{2}(:,1))
    %ind=1:length(KickingData.Resamp{3}{2}(:,1))
g1 = KickingData.Resamp{3}{2}(ind+k-1,2:4)';
g2 = KickingData.Resamp{2}{2}(ind+k-1,2:4)';
g3 = KickingData.Resamp{1}{2}(ind+k-1,2:4)';

% a1=cross(j1,g1);
% a2=cross(j2,g2);
% a3=cross(j3,g2);

%%During activity, these should all have the same sign...
b1(k)=dot(j1,g1);
b2(k)=dot(j2,g2);
b3pca(k)=dot(j3,g3);

end

b1smooth = smoothdata(b1,2);
b2smooth = smoothdata(b2,2);
b3smooth = smoothdata(b3pca,2);

%%%Find the extrema and determine if they are aligned (same sign)
extrema1 = islocalmin(b1smooth)+islocalmax(b1smooth);
extrema2 = islocalmin(b2smooth)+islocalmax(b2smooth);
extrema3 = islocalmin(b3smooth)+islocalmax(b3smooth);

extval1 = b1smooth(abs(b1smooth.*extrema1)>thresh); %%%Filter out extrema when no movement present
extval2 = b2smooth(abs(b2smooth.*extrema2)>thresh);
extval3 = b3smooth(abs(b3smooth.*extrema3)>thresh);

%%%%%This will need to be changed
%%%Compare signs of local maxima and minima to see if aligned
if ~(sign(extval1(1))==sign(extval2(1)))
    j2=-j2;
    b2=-b2;
    b2smooth = smoothdata(b2,2);
    extrema2 = islocalmin(b2smooth)+islocalmax(b2smooth);
    extval2 = b2smooth(abs(b2smooth.*extrema2)>thresh);
    disp('j2 switch')
end

if ~(sign(extval2(1))==sign(extval3(1)))
    j3=-j3;
    b3pca=-b3pca;
    b3smooth = smoothdata(b3pca,2);
    disp('j3 switch')
end

time = KickingData.Resamp{1}{2}(:,1)-KickingData.Resamp{1}{2}(1,1);
%x=1:k;

figure()
hold on
subplot(3,1,1)
plot(time, smoothdata(b1,2))
title('Thigh (Hip) Sensor Angle')
axis([0 time(end) -200 200])
grid on

subplot(3,1,2)
plot(time, smoothdata(b2,2))
title('Shin (Knee) Sensor Angle')
axis([0 time(end) -200 200])
grid on

subplot(3,1,3)
plot(time, smoothdata(b3pca,2))
title('Foot (Ankle) Sensor Angle')
axis([0 time(end) -200 200])
grid on