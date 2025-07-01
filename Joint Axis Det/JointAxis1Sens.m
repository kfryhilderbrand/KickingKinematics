function j3 = JointAxis1Sens(iterations, init, Gyro, Plot_Flag, foot_conv)
%%%Initialization
phi_3 = init;
theta_3 = init;

x = [phi_3, theta_3]'; %%% define state vector x

%%%1 Solve for j3
j3 = [cos(phi_3)*cos(theta_3), cos(phi_3)*sin(theta_3), sin(phi_3)]';
j3_conv = j3';
conv_iter = 0;
for iter=1:iterations
    conv_iter = [conv_iter; iter];
    
    %%%2 Calculate error vector
    e = zeros(length(Gyro(:,1)),1);
    J = [];
    for k=1:length(Gyro(:,1)) %First cell index of Resamp corresponds to which sensor, second cell index indicates accel or gyro (1 or 2)
        
        g3 = Gyro(k,2:4)';
        e(k) = norm(cross(g3,j3));
        
        %%%3 Calculate Jacobian and pseudo inverse of Jacobian  (J dim is nx2)
        A3 = -2*cos(phi_3)*(sin(phi_3)*sin(theta_3)*g3(3)*g3(2)+sin(phi_3)*cos(theta_3)*g3(1)*g3(3)+sin(theta_3)*cos(theta_3)*cos(phi_3)*g3(2)*g3(1))+...
            (cos(phi_3)^2*sin(theta_3)^2)*(g3(3)^2+g3(1)^2)+sin(phi_3)^2*(g3(1)^2+g3(2)^2)+(cos(phi_3)^2*cos(theta_3)^2)*(g3(2)^2+g3(3)^2);
        temp5 = (.5*(A3)^-.5)*(sin(2*phi_3)*((g3(1)^2+g3(2)^2)-sin(theta_3)^2*(g3(3)^2+g3(1)^2)-cos(theta_3)^2*(g3(2)^2+g3(3)^2))+...
            2*sin(theta_3)*g3(3)*g3(2)*(sin(phi_3)^2-cos(phi_3)^2)+2*cos(theta_3)*g3(1)*g3(3)*(sin(phi_3)^2-cos(phi_3)^2)+...
            4*sin(phi_3)*cos(phi_3)*sin(theta_3)*cos(theta_3)*g3(2)*g3(1)); %partial of e wrt phi_2
        temp6 = (.5*(A3)^-.5)*(sin(2*theta_3)*cos(phi_3)^2*(g3(1)^2-g3(2)^2)-2*cos(phi_3)^2*g3(2)*g3(1)*(cos(theta_3)^2-sin(theta_3)^2)-...
            2*cos(phi_3)*sin(phi_3)*g3(3)*(cos(theta_3)*g3(2)-sin(theta_3)*g3(1)));%partial of e wrt theta_1
        
        temp = [temp5, temp6];
        J = [J; temp];
    end
    J_plus = pinv(J);
    
    %4 Update x (j1 and j2)
    x = x - J_plus*e; %x dim 4x1, J_plus dim 4xn, e dim nx1
    x = wrapToPi(x);%wrap angles to [-pi,pi]
    
    phi_3 = x(1);
    theta_3 = x(2);
    
    j3 = [cos(phi_3)*cos(theta_3), cos(phi_3)*sin(theta_3), sin(phi_3)]';
    j3_conv = [ j3_conv; j3'];
end

%%% Alignment check (assumes shin is aligned)
thresh = 10;
for k = 1:length(Gyro(:,1))
    g3 = Gyro(k,2:4)';
    
    %%% Find angular rate about gi axis.
    b3(k)=dot(j3,g3);
end

b3smooth = smoothdata(b3,2);

%%% Find the extrema. This helps to find peak moments of movement
extrema3 = islocalmin(b3smooth)+islocalmax(b3smooth);

%%%Filter out extrema when no movement present
extval3 = b3smooth(abs(b3smooth.*extrema3)>thresh);

if isempty(extval3)
    j3 = zeros(3,1);
else
    
    if Plot_Flag == 1
        %%% Plot converge
        figure()
        p=plot(conv_iter,j3_conv(:,1), conv_iter,j3_conv(:,2), conv_iter,j3_conv(:,3));
        axis([0 iter -1.25 1.25])
        title('Foot convergence')
        line([0,iter],[foot_conv(1),foot_conv(1)],'Color',p(1).Color,'LineStyle',':') %X conv
        line([0,iter],[foot_conv(2),foot_conv(2)],'Color',p(2).Color,'LineStyle',':') %y conv
        line([0,iter],[foot_conv(3),foot_conv(3)],'Color',p(3).Color,'LineStyle',':') %Z conv
        legend('x axis', 'y axis', 'z axis')
        
        time = Gyro(:,1)-Gyro(1,1);
        figure()
        plot(time, b3smooth)
        title('Foot (Ankle) Sensor Angle')
        axis([0 time(end) -200 200])
        grid on
    end
end

end