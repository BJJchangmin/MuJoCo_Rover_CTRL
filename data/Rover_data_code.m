close all;

filename{1} = 'data_FL.csv';
filename{2} = 'data_FR.csv';
filename{3} = 'data_RL.csv';
filename{4} = 'data_RR.csv';
filename{5} = 'data_trunk.csv';

%Leg Data Load
for i = 1:1:4
    Arr_Leg{i} = table2array(readtable(filename{i}));
end

%Trunk Data Load
Arr_trunk = table2array(readtable(filename{5}));


t = Arr_Leg{1}(:,1);

for i = 1:1:4
    
    sus_pos_ref{i} = Arr_Leg{i}(:,2);
    sus_pos{i} = Arr_Leg{i}(:,3);
    sus_vel{i} = Arr_Leg{i}(:,4);
    sus_torque{i} = Arr_Leg{i}(:,5);
    
    steer_pos_ref{i} = Arr_Leg{i}(:,6);
    steer_pos{i} = Arr_Leg{i}(:,7);
    steer_vel{i} = Arr_Leg{i}(:,8);
    steer_torque{i} = Arr_Leg{i}(:,9);
    
    drive_vel_ref{i} = Arr_Leg{i}(:,10);
    drive_pos{i} = Arr_Leg{i}(:,11);
    drive_vel{i} = Arr_Leg{i}(:,12);
    drive_torque{i} = Arr_Leg{i}(:,13);
    
    slip_ratio{i} = Arr_Leg{i}(:,14);
    Mu{i} = Arr_Leg{i}(:,15);
    
    grf_x{i} = Arr_Leg{i}(:,16);
    grf_z{i} = Arr_Leg{i}(:,17);
    
    
end

Trunk_x_vel = Arr_trunk(:,1);
Trunk_y_vel = Arr_trunk(:,2);
Trunk_z_vel = Arr_trunk(:,3);

Trunk_x_ang_vel = Arr_trunk(:,4);
Trunk_y_ang_vel = Arr_trunk(:,5);
Trunk_z_ang_vel = Arr_trunk(:,6);

Trunk_x_pos = Arr_trunk(:,7);
Trunk_y_pos = Arr_trunk(:,8);
Trunk_z_pos = Arr_trunk(:,9);
x_offset = Trunk_x_pos(1);
y_offset = Trunk_y_pos(1);

Trunk_x_acc = Arr_trunk(:,10);
Trunk_y_acc = Arr_trunk(:,11);
Trunk_z_acc = Arr_trunk(:,12);

Trunk_roll_angle = Arr_trunk(:,13);
Trunk_pitch_angle = Arr_trunk(:,14);
Trunk_yaw_angle = Arr_trunk(:,15);

Trunk_x_vel_ref = Arr_trunk(:,16);
Trunk_y_vel_ref = Arr_trunk(:,17);
Trunk_yaw_rate_ref = Arr_trunk(:,18);
Trunk_roll_angle_ref = Arr_trunk(:,19);
Trunk_pitch_angle_ref = Arr_trunk(:,20);




%%%%%%%%%%%%%%%%%%%%% TRUNK STATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


Ts = t(2,1)-t(1,1);
for i = 1:length(sus_pos_ref)
    t(i,1) = (i-1)*Ts;
end


%%%%%%%%%%%%%%%%%%%% DATA PLOT %%%%%%%%%%%%%%%%%%%

%Plotting Parameter
lw =1;   %Line Width
FT = 7; %Title Fonte Size
sgT= 18; % subtitle plot title
Faxis = 12.5; %Axis Fonte Size
fl =10 ; % Legend Fonte Size
Ms = 3 ; %Mark Size


%%%Need to change total plot
% function Tracking_graph

figure(1)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,sus_pos_ref{i},'b-','LineWidth', lw);
    hold on
    plot(t,sus_pos{i},'r-','LineWidth',lw);
    grid on;
    legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('rad','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('Sus Joint Position Tracking','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');


figure(2)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,sus_torque{i},'b-','LineWidth', lw);
    hold on;
    plot(t,steer_torque{i},'r-','LineWidth', lw);
    hold on;
    plot(t,drive_torque{i},'g-','LineWidth', lw);
    grid on;
    ylabel('$\tau$ (Nm)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
legend('sus','steer','drive','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
sgtitle('Motor Control Input ','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(3)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,steer_pos_ref{i}*180/pi,'b-','LineWidth', lw);
    hold on
    plot(t,steer_pos{i}*180/pi,'r-','LineWidth',lw);
    grid on;
    legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('rad','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('Steer Joint Position Tracking','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(4)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,drive_vel_ref{i},'b-','LineWidth', lw);
    hold on
    plot(t,drive_vel{i},'r-','LineWidth',lw);
    grid on;
    legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('rad','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('Drive Joint Velocity Tracking','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(5)
subplot(4,1,1);
plot(t,Trunk_x_vel_ref,'k-','LineWidth', lw);
hold on;
plot(t,Trunk_x_vel,'r-','LineWidth', lw);
grid on;
ylabel('m/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블

subplot(4,1,2);
plot(t, Trunk_yaw_rate_ref,'k-','LineWidth', lw);
hold on;
plot(t,Trunk_z_ang_vel,'r-','LineWidth', lw);
grid on;
ylabel('rad/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블

subplot(4,1,3);
plot(t, Trunk_roll_angle_ref,'k-','LineWidth', lw);
hold on;
plot(t,Trunk_roll_angle,'r-','LineWidth', lw);
grid on;
ylabel('rad','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블

subplot(4,1,4);
plot(t, Trunk_pitch_angle_ref,'k-','LineWidth', lw);
hold on;
plot(t,Trunk_pitch_angle,'r-','LineWidth', lw);
grid on;
ylabel('rad','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
sgtitle('Chassis Control','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');



figure(6)
plot(-(Trunk_y_pos-y_offset),(Trunk_x_pos-x_offset),'r','LineWidth',lw*3);
xlim([-10 10]);
ylim([0 10]);
grid on;
ylabel('y (m)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('x (m)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
title('Trunk Map','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');



figure(7)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,grf_z{i},'k-','LineWidth', lw);
    grid on;
    ylabel('Fz (N)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
    xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
    
end
sgtitle('Normal Force','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(8)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,grf_x{i},'k-','LineWidth', lw);
    grid on;
    ylabel('Fx (N)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
    xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
    
end
sgtitle('Traction Force','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');



total_grf_x = grf_x{1} + grf_x{2} + grf_x{3} + grf_x{4};
figure(9)
plot(t,total_grf_x,'k-','LineWidth', lw);
hold on
plot(t,Trunk_x_acc*154,'b-','LineWidth', lw)
grid on;
ylabel('Fx (N)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
title('Total Traction Force','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

setFigurePositions(6);










