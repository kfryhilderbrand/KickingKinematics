%%%1-DOF method to find axis of rotation

clear 
close all
clc
ConstantsFlags

%Create Data.mat
listdr = dir('*20*');
for i=1:length(listdr)
    cd(listdr(i).name)
    ReadCSV('Data.mat', Flags.HEADERflag)
    dinfo=dir('Data.mat');
    filename = dinfo.name;
    load(filename,'Data');
   
    KickingData.XRaw = GatherData(Data, Constants.NumMeas, Flags.HFSflag, Constants.fs);
    save('KickingData.mat','KickingData');
    cd ..
end

%% Create KickingData.mat
for dataset = 1:length(listdr)
    cd(listdr(dataset).name)
    load('KickingData.mat','KickingData')
    
%     %%%%%%%%%%%%%%%%%%Calculate offset between sensors (for now)
%     load('Data.mat')
%     V = cell(1,3);
%     k=1;
%     for j=2:2:length(Data)   %Only need gyro data
%         data=Data{j}{1};
%         data = smoothdata(data,1);
%         V{k} = zeros(length(data(:,1)),2);
%         for i=1:length(data(:,1))
%             V{k}(i,1)=data(i,1);
%             V{k}(i,2)=sqrt(data(i,2)^2+data(i,3)^2+data(i,4)^2);
%         end
%         k=k+1; 
%     end
%     
%     %Find offset between sensors
%     ind = zeros(1,3);
%     val = zeros(1,3);
%     for i=1:3
%         ind(i)=find(V{i}(:,2)>5,1);
%         val(i)=V{i}(ind(i),1)-V{i}(1,1);
%     end
%     
%     start_time = max(val);
%     offset = start_time-val;
%     for i=1:3
%         KickingData.XRaw{i}{1}(:,1)=KickingData.XRaw{i}{1}(:,1)+offset(i);
%         KickingData.XRaw{i}{2}(:,1)=KickingData.XRaw{i}{2}(:,1)+offset(i);
%     end
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Resample Data
    step = str2double(sprintf('%0.2f',1/Constants.fs));
    timestart = str2double(sprintf('%0.2f',KickingData.XRaw{1}{1}(1,1)));   %round to nearest hundreth
    timeend = str2double(sprintf('%0.2f',KickingData.XRaw{1}{1}(end,1)));   %round to nearest hundreth
    
    for i=1:length(KickingData.XRaw)
        for j=1:length(KickingData.XRaw{i})
            if KickingData.XRaw{i}{j}(1,1) > timestart
                timestart = KickingData.XRaw{i}{j}(1,1);
            elseif KickingData.XRaw{i}{j}(end,1) < timeend
                timeend = KickingData.XRaw{i}{j}(end,1);
            end
        end
    end
    time = (timestart:step:timeend)';   %create time vector
    
    KickingData.Resamp = cell(1,length(KickingData.XRaw));
    % Resamples each sensor
    for i=1:length(KickingData.XRaw)
        KickingData.Resamp{i} = cell(1,length(KickingData.XRaw{i}));
        for j=1:length(KickingData.XRaw{i})
            [~,width]=size(KickingData.XRaw{i}{j});
            temp=interp1(KickingData.XRaw{i}{j}(:,1),KickingData.XRaw{i}{j}(:,2:width),time,'pchip',0); %interpolates to time vector as defined by truth
            %note:will default to
            %0 for values outside
            %of time vector
            KickingData.Resamp{i}{j}=[time, temp];
        end
    end
    save('KickingData.mat','KickingData');
    cd ..
end
% %%
% figure()
% hold on
% for i=1:3
%     subplot(3,1,i)
%     data = KickingData.Resamp{i}{2};
%     data = smoothdata(data,1);
%     indexes = 1:length(data(:,1));%1020:1950;%1250;
%     %indexes = 900:2200;%length(data(:,1));
%     %time = Data{j}{1}(:,1)-Data{j}{1}(1,1);
%     time = data(indexes,1)-data(1,1);
%     xrot = data(indexes,2);
%     yrot = data(indexes,3);
%     zrot = data(indexes,4);
%     plot(time, xrot, time, yrot, time, zrot)
%     legend('xrot','yrot','zrot')
%     switch i  %Change the titles to match what you want
%         case 1
%             title('Foot')
%         case 2
%             title('Shin')
%         case 3
%             title('Thigh')
%     end  
%     grid on
% end
%% 3 sensor method hierarchy
%%Radians vs degrees???
%%CD to the folder you want to look at
clear all
load('KickingData.mat','KickingData')

% Expected Conv
thigh_conv = [0.99,-0.02,0];
shin_conv = [0.59,0.81,-0.05];
foot_conv = [-0.16,0.99,0.01];
% thigh_conv = [0,-0.5,-0.9];
% shin_conv = [0,0,-1];
% foot_conv = [0.99,0.09,-0.14];

iterations = 50;
Flag_flip = 0;

% Initialization
phi_1 = 0;
theta_1 = 0;
phi_2 = 0;
theta_2 = 0;
phi_3 = 0;
theta_3 = 0;
% init = [phi_1, theta_1, phi_2, theta_2, phi_3, theta_2];

% phi_1 = rand;
% theta_1 = rand;
% phi_2 = rand;
% theta_2 = rand;
% phi_3 = rand;
% theta_3 = rand;
init = [phi_1, theta_1, phi_2, theta_2, phi_3, theta_2];

%%%%Need to work on alignment issues
%define state vector x 
x = [phi_1, theta_1, phi_2, theta_2]';

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
b3(k)=dot(j3,g3);

end

b1smooth = smoothdata(b1,2);
b2smooth = smoothdata(b2,2);
b3smooth = smoothdata(b3,2);

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
    b3=-b3;
    b3smooth = smoothdata(b3,2);
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
plot(time, smoothdata(b3,2))
title('Foot (Ankle) Sensor Angle')
axis([0 time(end) -200 200])
grid on



%% Alignment check attempt 2 (projections)
%Project j_i into xy plane
% lower_lim = -pi/2;
% upper_lim = pi/2;
tolerance = 0;% pi/18;
clc

x=[1;0;0];
y=[0;1;0];
z=[0;0;1];

%Make sure you are actually looking at the flexion/extension axis (so the
%projection into the xy plane should be longer than 

%%%%%%%%%%% Essentially limits flexion to positive y direction (note, the convergence
% vector must point in pos y)
j1_proj = j1-dot(j1,z)*z; %proj into xy plane
theta1 = atan2(norm(cross(y,j1_proj)),dot(y,j1_proj));%Angle from y to j1 in xy plane
if theta1>=-pi/2-tolerance && theta1<=pi/2+tolerance %%&& theta1z>=-pi/2-tolerance && theta1z<=pi/2+tolerance
    disp('j1 aligned')
else
    %j1=-j1;
    disp('j1 misaligned')
end

j2_proj = j2-dot(j2,z)*z; %proj into xy
theta2 = atan2(norm(cross(y,j2_proj)),dot(y,j2_proj));
if theta2>=-pi/2-tolerance && theta2<=pi/2+tolerance %%&& theta2z>=-pi/2-tolerance && theta2z<=pi/2+tolerance
    disp('j2 aligned')
else
    %j2=-j2;
    disp('j2 misaligned')
end

j3_proj = j3-dot(j3,z)*z;
theta3 = atan2(norm(cross(y,j3_proj)),dot(y,j3_proj));
if theta3>=-pi/2-tolerance && theta3<=pi/2+tolerance && theta3z>=-pi/2-tolerance && theta3z<=pi/2+tolerance
    disp('j3 aligned')
else
    %j3=-j3;
    disp('j3 misaligned')
end

%%%%%%%%%%Use gravity and the now aligned flexion axes to get relative
%%%%%%%%%%rotation of sensors to each other

%%%%%%%%%%Check hip abduction: essentially limits abduction to negative z
%%%%%%%%%%direction. Once you have 

%% Alignment check attempt 3 (using acceleration to determine direction of movement.



%% Alignment check
clearvars -except j1 j2 j3 j1_conv j2_conv j3_conv init KickingData x
close all
start = 550;
width = 50;

g1_proj = zeros(length(KickingData.Resamp{1}{2}(start:start+width,1)),3);
g2_proj = zeros(length(KickingData.Resamp{1}{2}(start:start+width,1)),3);
%g2_projalt = zeros(length(KickingData.Resamp{1}{2}(start:start+width,1)),3); %testing alternative (i.e. if j2 should be -j2)
%%j1 vs j2 alignment
for k=1:length(KickingData.Resamp{1}{2}(start:start+width,1)) %First cell index of Resamp corresponds to which sensor, second cell index indicates accel or gyro (1 or 2)
    g1 = KickingData.Resamp{3}{2}(k-1+start,2:4)';
    g2 = KickingData.Resamp{2}{2}(k-1+start,2:4)';
    g1_proj(k,:) = g1-dot(g1,j1)*j1;
    g2_proj(k,:) = g2-dot(g2,j2)*j2;
    %g2_projalt(k,:) = g2-dot(g2,j2)*-j2;
end


%%
% figure()
% hold on
% plot3(g1_proj(:,1), g1_proj(:,2), g1_proj(:,3))
% plot3(g2_proj(:,1), g2_proj(:,2), g2_proj(:,3))
% xlabel('x')
% ylabel('y')
% zlabel('z')
% grid on

%  HERE we are going to rotate the g1 and g2 projections into the xy plane (so j1 and j2 align with the z axis).
%  To test whether or not the axes are aligned, reflect g2_proj. If
%  misaligned, then the reflected g2_proj will yield a smaller dist vector.
%%Rotate data to xy plane
z = [0,0,1];
Axis = cross(j1,z);
Ang = atan2(norm(cross(j1,z)),dot(j1,z));
R1 = [cos(Ang)+Axis(1)^2*(1-cos(Ang)), Axis(1)*Axis(2)*(1-cos(Ang))-Axis(3)*sin(Ang), Axis(1)*Axis(3)*(1-cos(Ang))+Axis(2)*sin(Ang);...
    Axis(1)*Axis(2)*(1-cos(Ang))+Axis(3)*sin(Ang), cos(Ang)+Axis(2)^2*(1-cos(Ang)), Axis(2)*Axis(3)*(1-cos(Ang))-Axis(1)*sin(Ang);...
    Axis(1)*Axis(3)*(1-cos(Ang))-Axis(2)*sin(Ang), Axis(2)*Axis(3)*(1-cos(Ang))+Axis(1)*sin(Ang), cos(Ang)+Axis(3)^2*(1-cos(Ang))];
g1_proj = (R1*g1_proj')';
%Center the data
g1_proj(:,1)=g1_proj(:,1)-mean(g1_proj(:,1));%%%
g1_proj(:,2)=g1_proj(:,2)-mean(g1_proj(:,2));%%%

Axis = cross(j2,z);
Ang = atan2(norm(cross(j2,z)),dot(j2,z));
R2 = [cos(Ang)+Axis(1)^2*(1-cos(Ang)), Axis(1)*Axis(2)*(1-cos(Ang))-Axis(3)*sin(Ang), Axis(1)*Axis(3)*(1-cos(Ang))+Axis(2)*sin(Ang);...
    Axis(1)*Axis(2)*(1-cos(Ang))+Axis(3)*sin(Ang), cos(Ang)+Axis(2)^2*(1-cos(Ang)), Axis(2)*Axis(3)*(1-cos(Ang))-Axis(1)*sin(Ang);...
    Axis(1)*Axis(3)*(1-cos(Ang))-Axis(2)*sin(Ang), Axis(2)*Axis(3)*(1-cos(Ang))+Axis(1)*sin(Ang), cos(Ang)+Axis(3)^2*(1-cos(Ang))];
g2_proj = (R2*g2_proj')';
%Center the data
g2_proj(:,1)=g2_proj(:,1)-mean(g2_proj(:,1));%%%
g2_proj(:,2)=g2_proj(:,2)-mean(g2_proj(:,2));%%%


%Axis = cross(-j2,z);
%Ang = atan2(norm(cross(-j2,z)),dot(-j2,z));
%Ang = Ang + pi;
%R2 = [cos(Ang)+Axis(1)^2*(1-cos(Ang)), Axis(1)*Axis(2)*(1-cos(Ang))-Axis(3)*sin(Ang), Axis(1)*Axis(3)*(1-cos(Ang))+Axis(2)*sin(Ang);...
%    Axis(1)*Axis(2)*(1-cos(Ang))+Axis(3)*sin(Ang), cos(Ang)+Axis(2)^2*(1-cos(Ang)), Axis(2)*Axis(3)*(1-cos(Ang))-Axis(1)*sin(Ang);...
%    Axis(1)*Axis(3)*(1-cos(Ang))-Axis(2)*sin(Ang), Axis(2)*Axis(3)*(1-cos(Ang))+Axis(1)*sin(Ang), cos(Ang)+Axis(3)^2*(1-cos(Ang))];
%g2_projalt = (R2*g2_projalt')';
Ang = pi;
Rx = [1,0,0;...    %Rotating about the x will reflect the data in the xy plane
      0,cos(Ang),-sin(Ang);...
      0,sin(Ang),cos(Ang)];
g2_projalt = (Rx*g2_proj')';
%g2_projalt(:,2)=g2_projalt(:,2)-2*mean(g2_projalt(:,2)); %The operation shifts the reflected data back to the centroid of the non reflected data
%Center the data
g2_projalt(:,1)=g2_projalt(:,1)-mean(g2_projalt(:,1));%%%
g2_projalt(:,2)=g2_projalt(:,2)-mean(g2_projalt(:,2));%%%

% figure()
% hold on
% subplot(3,1,1)
% plot(g1_proj(:,1), g2_proj(:,2))
% grid on
% subplot(3,1,2)
% plot(g2_proj(:,1), g2_proj(:,2))
% grid on
% subplot(3,1,3)
% plot(g2_projalt(:,1), g2_projalt(:,2))
% grid on

%%Find lines of best fit and rotate g2_proj
%line = p(1)*x+p(2)
p1 = polyfit(g1_proj(:,1),g1_proj(:,2),1); %%Line of fit for g1_proj
p2 = polyfit(g2_proj(:,1),g2_proj(:,2),1); %%Line of fit for g2_proj
p2alt = polyfit(g2_projalt(:,1),g2_projalt(:,2),1); %%Line of fit for g2_proj
                              %xvals = 0:.1:5;
p1_vect = [1,polyval(p1,1)-polyval(p1,0),0];%[xvals; polyval(p1,xvals)];
p2_vect = [1,polyval(p2,1)-polyval(p2,0),0];%[xvals; polyval(p1,xvals)];
Ang = atan2(norm(cross(p1_vect,p2_vect)),dot(p1_vect,p2_vect));
Rz = [cos(Ang),-sin(Ang),0;...
      sin(Ang),cos(Ang),0;...
      0,0,1];
g2_proj = (Rz*g2_proj')';

p2alt_vect = [1,polyval(p2alt,1)-polyval(p2alt,0),0];%[xvals; polyval(p1,xvals)];
Ang = atan2(norm(cross(p1_vect,p2alt_vect)),dot(p1_vect,p2alt_vect));
%Ang = Ang + pi;
%Ang = -Ang;
Rz2 = [cos(Ang),-sin(Ang),0;...
      sin(Ang),cos(Ang),0;...
      0,0,1];
g2_projalt = (Rz2*g2_projalt')';


figure()
hold on
subplot(3,1,1)
plot(g1_proj(:,1), g2_proj(:,2))
grid on
subplot(3,1,2)
plot(g2_proj(:,1), g2_proj(:,2))
grid on
subplot(3,1,3)
plot(g2_projalt(:,1), g2_projalt(:,2))
grid on

%%% distance between each set of projections
dist1=zeros(1,length(g1_proj(:,1)));
dist2=zeros(1,length(g1_proj(:,1)));
for k=1:length(g1_proj(:,1))
    dist1(k) = norm(g1_proj(k,:)-g2_proj(k,:));
    dist2(k) = norm(g1_proj(k,:)-g2_projalt(k,:));
end

clc
d1=sum(dist1);
d2=sum(dist2);
d3=mean(dist1);
d4=mean(dist2);
if d1>d2
    disp('misaligned')
end
if d3>d4
    disp('misaligned2')
end
%%






%% 3 sensor method
clear all
load('KickingData.mat','KickingData')

% Initialization
phi_1 = 0;
theta_1 = 0;
phi_2 = 0;
theta_2 = 0;
phi_3 = 0;
theta_3 = 0;

%define state vector x 
x = [phi_1, theta_1, phi_2, theta_2, phi_3, theta_3]';

%1 define joing axes wrt frames
j1 = [cos(phi_1)*cos(theta_1), cos(phi_1)*sin(theta_1), sin(phi_1)]';
j2 = [cos(phi_2)*cos(theta_2), cos(phi_2)*sin(theta_2), sin(phi_2)]';
j3 = [cos(phi_3)*cos(theta_3), cos(phi_3)*sin(theta_3), sin(phi_3)]';

j1_conv = j1';
j2_conv = j2';
j3_conv = j3';

conv_iter = 0;
for iter=1:100
    conv_iter = [conv_iter; iter];
    
    %End ind (i.e. when gyroscope is all 0)
    nd = 4700;
    %2 Calculate error vector
    e = zeros(length(KickingData.Resamp{1}{2}(1:end,1)),1); %%%%%%%%%%%%%%%%
    J = [];
    for k=1:length(KickingData.Resamp{1}{2}(1:end,1)) %First cell index of Resamp corresponds to which sensor, second cell index indicates accel or gyro (1 or 2)
        g1 = KickingData.Resamp{1}{2}(k,2:4)';
        g2 = KickingData.Resamp{2}{2}(k,2:4)';
        g3 = KickingData.Resamp{3}{2}(k,2:4)';
        e(k) = -norm(cross(j1,g1))+2*norm(cross(j2,g2))-norm(cross(j3,g3));
        %e(k) = norm(cross(j2,g2));
        
        %3 Calculate Jacobian and pseudo inverse of Jacobian  (J dim is nx4)
        %temp = cross(cross(g1,j1),g1)/norm(cross(g1,j1))-cross(cross(g2,j2),g2)/norm(cross(g2,j2));
        %%%%%%%%%%%%%%%%%%%%
        A1 = -2*cos(phi_1)*(sin(phi_1)*sin(theta_1)*g1(3)*g1(2)+sin(phi_1)*cos(theta_1)*g1(1)*g1(3)+sin(theta_1)*cos(theta_1)*cos(phi_1)*g1(2)*g1(1))+...
            (cos(phi_1)^2*sin(theta_1)^2)*(g1(3)^2+g1(1)^2)+sin(phi_1)^2*(g1(1)^2+g1(2)^2)+(cos(phi_1)^2*cos(theta_1)^2)*(g1(2)^2+g1(3)^2);
        A2 = -2*cos(phi_2)*(sin(phi_2)*sin(theta_2)*g2(3)*g2(2)+sin(phi_2)*cos(theta_2)*g2(1)*g2(3)+sin(theta_2)*cos(theta_2)*cos(phi_2)*g2(2)*g2(1))+...
            (cos(phi_2)^2*sin(theta_2)^2)*(g2(3)^2+g2(1)^2)+sin(phi_2)^2*(g2(1)^2+g2(2)^2)+(cos(phi_2)^2*cos(theta_2)^2)*(g2(2)^2+g2(3)^2);
        A3 = -2*cos(phi_3)*(sin(phi_3)*sin(theta_3)*g3(3)*g3(2)+sin(phi_3)*cos(theta_3)*g3(1)*g3(3)+sin(theta_3)*cos(theta_3)*cos(phi_3)*g3(2)*g3(1))+...
            (cos(phi_3)^2*sin(theta_3)^2)*(g3(3)^2+g3(1)^2)+sin(phi_3)^2*(g3(1)^2+g3(2)^2)+(cos(phi_3)^2*cos(theta_3)^2)*(g3(2)^2+g3(3)^2);
                
        temp1 = -(.5*(A1)^-.5)*(sin(2*phi_1)*((g1(1)^2+g1(2)^2)-sin(theta_1)^2*(g1(3)^2+g1(1)^2)-cos(theta_1)^2*(g1(2)^2+g1(3)^2))+...
            2*sin(theta_1)*g1(3)*g1(2)*(sin(phi_1)^2-cos(phi_1)^2)+2*cos(theta_1)*g1(1)*g1(3)*(sin(phi_1)^2-cos(phi_1)^2)+...
            4*sin(phi_1)*cos(phi_1)*sin(theta_1)*cos(theta_1)*g1(2)*g1(1)); %partial of e wrt phi_1
        temp3 = 2*(.5*(A2)^-.5)*(sin(2*phi_2)*((g2(1)^2+g2(2)^2)-sin(theta_2)^2*(g2(3)^2+g2(1)^2)-cos(theta_2)^2*(g2(2)^2+g2(3)^2))+...
            2*sin(theta_2)*g2(3)*g2(2)*(sin(phi_2)^2-cos(phi_2)^2)+2*cos(theta_2)*g2(1)*g2(3)*(sin(phi_2)^2-cos(phi_2)^2)+...
            4*sin(phi_2)*cos(phi_2)*sin(theta_2)*cos(theta_2)*g2(2)*g2(1)); %partial of e wrt phi_2
        temp5 = -(.5*(A3)^-.5)*(sin(2*phi_3)*((g3(1)^2+g3(2)^2)-sin(theta_3)^2*(g3(3)^2+g3(1)^2)-cos(theta_3)^2*(g3(2)^2+g3(3)^2))+...
            2*sin(theta_3)*g3(3)*g3(2)*(sin(phi_3)^2-cos(phi_3)^2)+2*cos(theta_3)*g3(1)*g3(3)*(sin(phi_3)^2-cos(phi_3)^2)+...
            4*sin(phi_3)*cos(phi_3)*sin(theta_3)*cos(theta_3)*g3(2)*g3(1)); %partial of e wrt phi_2
        
        temp2 = -(.5*(A1)^-.5)*(sin(2*theta_1)*cos(phi_1)^2*(g1(1)^2-g1(2)^2)-2*cos(phi_1)^2*g1(2)*g1(1)*(cos(theta_1)^2-sin(theta_1)^2)-...
            2*cos(phi_1)*sin(phi_1)*g1(3)*(cos(theta_1)*g1(2)-sin(theta_1)*g1(1)));%partial of e wrt theta_1
        temp4 = 2*(.5*(A2)^-.5)*(sin(2*theta_2)*cos(phi_2)^2*(g2(1)^2-g2(2)^2)-2*cos(phi_2)^2*g2(2)*g2(1)*(cos(theta_2)^2-sin(theta_2)^2)-...
            2*cos(phi_2)*sin(phi_2)*g2(3)*(cos(theta_2)*g2(2)-sin(theta_2)*g2(1)));%partial of e wrt theta_1
        temp6 = -(.5*(A3)^-.5)*(sin(2*theta_3)*cos(phi_3)^2*(g3(1)^2-g3(2)^2)-2*cos(phi_3)^2*g3(2)*g3(1)*(cos(theta_3)^2-sin(theta_3)^2)-...
            2*cos(phi_3)*sin(phi_3)*g3(3)*(cos(theta_3)*g3(2)-sin(theta_3)*g3(1)));%partial of e wrt theta_1
        
        temp = [temp1, temp2, temp3, temp4, temp5, temp6];
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
    phi_3 = x(5);
    theta_3 = x(6);
    
    j1 = [cos(phi_1)*cos(theta_1), cos(phi_1)*sin(theta_1), sin(phi_1)]';
    j2 = [cos(phi_2)*cos(theta_2), cos(phi_2)*sin(theta_2), sin(phi_2)]';
    j3 = [cos(phi_3)*cos(theta_3), cos(phi_3)*sin(theta_3), sin(phi_3)]';
    
    j1_conv = [ j1_conv; j1'];
    j2_conv = [ j2_conv; j2'];
    j3_conv = [ j3_conv; j3'];
end


% j1 = [cos(phi_1)*cos(theta_1), cos(phi_1)*sin(theta_1), sin(phi_1)]';
% j2 = [cos(phi_2)*cos(theta_2), cos(phi_2)*sin(theta_2), sin(phi_2)]';
figure()
plot(conv_iter,j1_conv(:,1), conv_iter,j1_conv(:,2), conv_iter,j1_conv(:,3))
axis([0 iter -1 1])
%line([0,50],[0.9978,0.9978]) %X conv
%line([0,50],[-0.04, -0.04]) %y conv
%line([0,50],[-0.052, -0.052]) % zconv
legend('x axis', 'y axis', 'z axis')
title('RF convergence')

figure()
plot(conv_iter,j2_conv(:,1), conv_iter,j2_conv(:,2), conv_iter,j2_conv(:,3))
axis([0 iter -1 1])
%line([0,50],[0.0532,0.0532]) %X conv
%line([0,50],[0.9985,0.9985]) %y conv
%line([0,50],[-0.1245,-0.1245]) % zconv
legend('x axis', 'y axis', 'z axis')
title('RS convergence')

figure()
plot(conv_iter,j3_conv(:,1), conv_iter,j3_conv(:,2), conv_iter,j3_conv(:,3))
axis([0 iter -1 1])
%line([0,50],[0.0532,0.0532]) %X conv
%line([0,50],[0.9985,0.9985]) %y conv
%line([0,50],[-0.1245,-0.1245]) % zconv
legend('x axis', 'y axis', 'z axis')
title('RT convergence')
%% 2 Sensor method
%%CD to the folder you want to look at
clear all
load('KickingData.mat','KickingData')

% Expected Conv
thigh_conv = [0,-0.5,-0.9];
shin_conv = [0,0,-1];
foot_conv = [0.99,0.09,-0.14];

iterations = 50;
Flag_flip = 0;

% Initialization
phi_1 = 0;
theta_1 = 0;
phi_2 = 0;
theta_2 = 0;
phi_3 = 0;
theta_3 = 0;
% init = [phi_1, theta_1, phi_2, theta_2, phi_3, theta_2];

% phi_1 = rand;
% theta_1 = rand;
% phi_2 = rand;
% theta_2 = rand;
% phi_3 = rand;
% theta_3 = rand;
init = [phi_1, theta_1, phi_2, theta_2, phi_3, theta_2];

%%%%Need to work on alignment issues
%define state vector x 
x = [phi_1, theta_1, phi_2, theta_2]';

%1 define joing axes wrt frames
j1 = [cos(phi_1)*cos(theta_1), cos(phi_1)*sin(theta_1), sin(phi_1)]';
j2 = [cos(phi_2)*cos(theta_2), cos(phi_2)*sin(theta_2), sin(phi_2)]';
j1_conv = j1';
j2_conv = j2';
conv_iter = 0;
for iter=1:100
    conv_iter = [conv_iter; iter];
    
    %End ind (i.e. when gyroscope is all 0)
    nd = 4700;
    %2 Calculate error vector
    e = zeros(length(KickingData.Resamp{1}{2}(1:nd,1)),1); %%%%%%%%%%%%%%%%
    J = [];
    for k=1:length(KickingData.Resamp{1}{2}(1:nd,1)) %First cell index of Resamp corresponds to which sensor, second cell index indicates accel or gyro (1 or 2)
        g1 = KickingData.Resamp{2}{2}(k,2:4)';
        g2 = KickingData.Resamp{1}{2}(k,2:4)';
        e(k) = norm(cross(j1,g1))-norm(cross(j2,g2));
        %e(k) = norm(cross(j2,g2));
        
        %3 Calculate Jacobian and pseudo inverse of Jacobian  (J dim is nx4)
        %temp = cross(cross(g1,j1),g1)/norm(cross(g1,j1))-cross(cross(g2,j2),g2)/norm(cross(g2,j2));
        %%%%%%%%%%%%%%%%%%%%
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
    j1_conv = [ j1_conv; j1'];
    j2_conv = [ j2_conv; j2'];
end


% j1 = [cos(phi_1)*cos(theta_1), cos(phi_1)*sin(theta_1), sin(phi_1)]';
% j2 = [cos(phi_2)*cos(theta_2), cos(phi_2)*sin(theta_2), sin(phi_2)]';
figure()
plot(conv_iter,j1_conv(:,1), conv_iter,j1_conv(:,2), conv_iter,j1_conv(:,3))
axis([0 iter -1 1])
%line([0,50],[0.9978,0.9978]) %X conv
%line([0,50],[-0.04, -0.04]) %y conv
%line([0,50],[-0.052, -0.052]) % zconv
legend('x axis', 'y axis', 'z axis')
title('RS convergence')

figure()
plot(conv_iter,j2_conv(:,1), conv_iter,j2_conv(:,2), conv_iter,j2_conv(:,3))
axis([0 iter -1 1])
%line([0,50],[0.0532,0.0532]) %X conv
%line([0,50],[0.9985,0.9985]) %y conv
%line([0,50],[-0.1245,-0.1245]) % zconv
legend('x axis', 'y axis', 'z axis')
title('RF convergence')

%%     %%Check Boundaries
    if theta_1>pi/2
        theta_1 = theta_1-pi;
        phi_1 = -phi_1;
    elseif theta_1<-pi/2
        theta_1 = theta_1+pi;
        phi_1 = -phi_1;
%     elseif phi_1>pi/2
%         phi_1 = phi_1-pi;
%         theta_1 = -theta_1;
%     elseif phi_1<-pi/2
%         phi_1 = phi_1+pi;
%         theta_1 = -theta_1;
    end
% %     
    if theta_2>pi/2
        theta_2 = theta_2-pi;
        phi_2 = -phi_2;
    elseif theta_2<-pi/2
        theta_2 = theta_2+pi;
        phi_2 = -phi_2;
%     elseif phi_2>pi/2
%         phi_2 = phi_2-pi;
%         theta_2 = -theta_2;
%     elseif phi_2<-pi/2
%         phi_2 = phi_2+pi;
%         theta_2 = -theta_2;
    end
    
    j1 = [cos(phi_1)*cos(theta_1), cos(phi_1)*sin(theta_1), sin(phi_1)]';
    j2 = [cos(phi_2)*cos(theta_2), cos(phi_2)*sin(theta_2), sin(phi_2)]';
    disp(j1)
    disp(j2)

%%
%     %%Check Boundaries
    if theta_1>pi
        theta_1 = theta_1-pi;
        phi_1 = -phi_1;
    elseif theta_1<0
        theta_1 = theta_1+pi;
        phi_1 = -phi_1;
% %     if phi_1>pi/2
% %         phi_1 = phi_1-pi;
% %     elseif phi_1<-pi/2
% %         phi_1 = phi_1+pi;
    end
% %     
    if theta_2>pi
        theta_2 = theta_2-pi;
        phi_2 = -phi_2;
    elseif theta_2<0
        theta_2 = theta_2+pi;
        phi_2 = -phi_2;
% %     if phi_2>pi/2  %Can specify this because of restriction of z direction
% %         phi_2 = phi_2-pi;
% %     elseif phi_2<-pi/2
% %         phi_2 = phi_2+pi;
     end
    
    j1 = [cos(phi_1)*cos(theta_1), cos(phi_1)*sin(theta_1), sin(phi_1)]';
    j2 = [cos(phi_2)*cos(theta_2), cos(phi_2)*sin(theta_2), sin(phi_2)]';
    j1_conv = [ j1_conv; j1'];
    j2_conv = [ j2_conv; j2'];
%%
%CHECK BOUNDARIES

if theta_1>pi/2
    theta_1 = theta_1-pi;
elseif theta_1<-pi/2 
    theta_1 = theta_1+pi;
elseif phi_1>pi/2
    phi_1 = phi_1-pi;
elseif phi_1<-pi/2
    phi_1 = phi_1+pi;
end

if theta_2>pi/2
    theta_2 = theta_2-pi;
elseif theta_2<-pi/2
    theta_2 = theta_2+pi;
elseif phi_2>pi/2
    phi_2 = phi_2-pi;
elseif phi_2<-pi/2
    phi_2 = phi_2+pi;
end
    j3 = [cos(phi_1)*cos(theta_1), cos(phi_1)*sin(theta_1), sin(phi_1)]';
    j4 = [cos(phi_2)*cos(theta_2), cos(phi_2)*sin(theta_2), sin(phi_2)]';

%%
%prob = optimproblem('ObjectivSense','min');
%x = optimvar('x',4,1);
%j1 = [cos(x(1))*cos(x(2)), cos(x(1))*sin(x(2)), sin(x(1))]';
%j2 = [cos(x(3))*cos(x(4)), cos(x(3))*sin(x(4)), sin(x(3))]';
x = optimvar('x',6,1);
j1 = [x(1), x(2), x(3)];
j2 = [x(4), x(5), x(6)];

%2 Calculate error vector
e = zeros(length(KickingData.Resamp{1}{2}(:,1)));
for k=1:length(KickingData.Resamp{1}{2}(:,1)) %First cell index of Resamp corresponds to which sensor, second cell index indicates accel or gyro (1 or 2)
    g1 = KickingData.Resamp{1}{2}(k,2:4)';
    g2 = KickingData.Resamp{2}{2}(k,2:4)';
    e(k) = norm(cross(j1,g1))-norm(cross(j1,g1));
end
%prob.Objective = 
