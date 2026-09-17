close all;

%이 스크립트가 있는 폴더(= data)를 기준으로 잡는다. 어디서 실행하든 동작한다.
data_dir = fileparts(mfilename('fullpath'));

%그림 관련 보조 함수는 plot_utils 폴더에 있다
addpath(fullfile(data_dir, 'plot_utils'));

filename{1} = fullfile(data_dir, 'data_FL.csv');
filename{2} = fullfile(data_dir, 'data_FR.csv');
filename{3} = fullfile(data_dir, 'data_RL.csv');
filename{4} = fullfile(data_dir, 'data_RR.csv');
filename{5} = fullfile(data_dir, 'data_trunk.csv');

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
    
    Opt_wheel_grf_x{i} = Arr_Leg{i}(:,18);
    Opt_wheel_grf_y{i} = Arr_Leg{i}(:,19);
    Opt_wheel_grf_z{i} = Arr_Leg{i}(:,20);
    
    foot_contact{i} = Arr_Leg{i}(:,21);
    
    
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


%CSV의 시간은 0.001부터 시작한다. 0부터 시작하도록 다시 만든다.
%sus_pos_ref는 1x4 cell이라 length가 4다. 샘플 수는 t에서 가져와야 한다.
Ts = t(2,1)-t(1,1);
t = (0:length(t)-1)' * Ts;


%%%%%%%%%%%%%%%%%%%% DATA PLOT %%%%%%%%%%%%%%%%%%%

%Plotting Parameter
%화면에서 보기 편한 크기 그대로 둔다. 저장할 때는 saveAllFigures가 그림 크기에 맞춰 알아서 줄인다
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
    if i == 1
        legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    end
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
    if i == 1
        legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    end
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
    if i == 1
        legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    end
    ylabel('rad','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('Drive Joint Velocity Tracking','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(5)
subplot(4,1,1);
plot(t,Trunk_x_vel_ref,'k-','LineWidth', lw);
hold on;
plot(t,Trunk_x_vel,'r-','LineWidth', lw);
grid on;
ylabel('$V_x$ (m/s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블

subplot(4,1,2);
plot(t, Trunk_yaw_rate_ref,'k-','LineWidth', lw);
hold on;
plot(t,Trunk_z_ang_vel,'r-','LineWidth', lw);
grid on;
ylabel('$\omega_z$ (rad/s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블

subplot(4,1,3);
plot(t, Trunk_roll_angle_ref,'k-','LineWidth', lw);
hold on;
plot(t,Trunk_roll_angle,'r-','LineWidth', lw);
grid on;
ylabel('$\phi$ (rad)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블

subplot(4,1,4);
plot(t, Trunk_pitch_angle_ref,'k-','LineWidth', lw);
hold on;
plot(t,Trunk_pitch_angle,'r-','LineWidth', lw);
grid on;
ylabel('$\theta$ (rad)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
sgtitle('Chassis Control','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');



figure(6)
plot(-(Trunk_y_pos-y_offset),(Trunk_x_pos-x_offset),'r','LineWidth',lw*3);
xlim([-10 10]);
ylim([0 10]);
grid on;
ylabel('$y$ (m)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('$x$ (m)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
title('Trunk Map','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');



figure(7)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,grf_z{i},'k-','LineWidth', lw);
    grid on;
    ylabel('$F_z$ (N)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
    xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
    
end
sgtitle('Normal Force','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(8)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,grf_x{i},'k-','LineWidth', lw);
    grid on;
    ylabel('$F_x$ (N)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
    xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
    
end
sgtitle('Traction Force','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');



total_grf_x = grf_x{1} + grf_x{2} + grf_x{3} + grf_x{4};
figure(9)
plot(t,total_grf_x,'k-','LineWidth', lw);
hold on
plot(t,Trunk_x_acc*154,'b-','LineWidth', lw)
grid on;
ylabel('$F_x$ (N)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
title('Total Traction Force','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

%하중 균등 분배 확인: 네 바퀴 최적화 수직력을 한 그래프에 겹쳐 본다
Opt_grf_z_color = {'r-','b-','g-','m-'};
Opt_grf_z_name = {'FL','FR','RL','RR'};
figure(10)
for i = 1:1:4
    plot(t,Opt_wheel_grf_z{i},Opt_grf_z_color{i},'LineWidth', lw);
    hold on
end
grid on;
legend(Opt_grf_z_name,'FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
ylabel('$F_z$ (N)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
title('Optimized Normal Force','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');


%접촉 여부만 확인: 0보다 큰 값은 전부 1로 처리해서 0/1로만 본다
figure(11)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,double(foot_contact{i} > 0),Opt_grf_z_color{i},'LineWidth', lw);
    grid on;
    ylim([-0.1 1.1]);
    yticks([0 1]);
    ylabel('Contact','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
    xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
    title(Opt_grf_z_name{i},'FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex');
end
sgtitle('Foot Contact','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');


%견인력 배분 확인: 네 바퀴 최적화 접선력을 한 그래프에 겹쳐 본다
figure(12)
for i = 1:1:4
    plot(t,Opt_wheel_grf_x{i},Opt_grf_z_color{i},'LineWidth', lw);
    hold on
end
grid on;
legend(Opt_grf_z_name,'FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
ylabel('$F_x$ (N)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
xlabel('Time (s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % x축 레이블
title('Optimized Traction Force','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

% 시간 축 범위. 시작은 직접 정하고 끝은 데이터 마지막 시각으로 둔다.
% Trunk Map처럼 x가 시간이 아닌 그림은 xlim을 직접 걸어 두어서 건너뛴다.
t_start = 5;
sim_end = t(end);
setXLimRange([], [t_start sim_end]);

% y축 범위를 데이터 최소/최대 + 1% 여백으로 맞춘다.
% y레이블이 같은 subplot끼리 묶어 같은 범위를 주므로 네 다리를 바로 비교할 수 있다.
% xlim 안의 데이터만 보므로 setXLimRange 다음에 불러야 한다.
setYLimMargin([], 0.01);

setFigurePositions(6);

% figure를 전부 PNG로 저장한다 (data/figure png 폴더). 저장 안 하려면 주석 처리
% 인자 순서: 폴더, 해상도, 크기[cm], 글자크기, 제목남김
% 저장본 글자 크기는 saveFigureFontSize.m 에서 항목별로 조절한다
saveAllFigures('figure_png', 300, [8.5 6]);        % 단일 컬럼 8.5 cm
% saveAllFigures('figure_png', 300, [16 9]);       % PPT 와이드
% saveAllFigures('figure_png', 300, [17.5 12]);    % 양 컬럼 폭
% saveAllFigures();                                % 화면 크기 그대로











