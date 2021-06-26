clear;
clc;

% 建立机器人模型
%       theta d a alpha offset
H=Link([0     0 0 pi/2  0]);
L1=Link([0    0 0 -pi/2 0]);
L2=Link([0    0 0 0     0]);
L3=Link([0    0 0 0     0]);
% 设定连杆长度 单位mm
len_H=1;
len_L1=37.63;
len_L2=77.22;
len_L3=81.99;
H.a=len_H;
L1.a=len_L1;
L2.a=len_L2;
L3.a=len_L3;
robot=SerialLink([H L1 L2 L3],'name','leggedRobot');

% 绘制机器人
init_q=[pi/2,-pi/2,atan(len_L3/len_L2),-pi/2];
init_T=robot.fkine(init_q);
%robot.plot(init_q);
%set(gca,'FontSize',15,'FontName','Times New Roman','LineWidth',1);

% 调节机器人
% teach(robot)

% 机器人足端轨迹规划
T=2;% 运动周期
h=6;% 步高
s=12;% 步长
sum=100;% 采样点数
time=linspace(0,T/2,sum);
spDx=[];
spDz=[];
swDx=[];
swDz=[];
swVx=[];
swVz=[];
swAx=[];
swAz=[];
syms t;
swingVx=diff(swingDx(s,t,T),t);
swingVz1=diff(swingDz1(h,t,T),t);
swingVz2=diff(swingDz2(h,t,T),t);
swingAx=diff(swingDx(s,t,T),t,2);
swingAz1=diff(swingDz1(h,t,T),t,2);
swingAz2=diff(swingDz2(h,t,T),t,2);

% 生成摆动相足端轨迹
for i=time
    swDx=[swDx swingDx(s,i,T)];
    swDz=[swDz swingDz(h,i,T)+init_T.t(3)];
    swVx=[swVx subs(swingVx,t,i)];
    swAx=[swAx subs(swingAx,t,i)];
    if i<=T/4
        swVz=[swVz subs(swingVz1,t,i)];
        swAz=[swAz subs(swingAz1,t,i)];
    else
        swVz=[swVz subs(swingVz2,t,i)];
        swAz=[swAz subs(swingAz2,t,i)];
    end
end

% 生成支撑相足端轨迹
for i=time
    spDx=[spDx supportDx(s,i,T)-2*s];
    spDz=[spDz supportDz()+init_T.t(3)];
end

%{
% 绘图及保存文件专用
swDz=swDz-init_T.t(3);
spDz=spDz-init_T.t(3);
theoData=[swDx' swDz'];
% 打开文件
f=fopen('TheoreticalPoints.txt','w');
for i=1:1:length(theoData)
    fprintf(f,'%.12f %.12f\n',theoData(i,:));
end
% 关闭文件
fclose(f);
%}

% 求解摆动相关节角度
theta=[];
for i=linspace(1,sum,sum)
    % 数据准备
    rtBottomSide=abs(swDx(i));% 底部直角边
    rtRightSide=abs(abs(swDz(i))-len_L1);% 右侧直角边
    rtSlantSide=sqrt(rtBottomSide^2+rtRightSide^2);% 直角三角形斜边
    xAngle1=atan(rtBottomSide/rtRightSide);% 直角三角形膝关节角
    cosxAngle2=(len_L2^2+rtSlantSide^2-len_L3^2)/(2*len_L2*rtSlantSide);
    xAngle2=acos(cosxAngle2);% 普通三角形膝关节角
    theta3=xAngle2-xAngle1;% 构造theta3
    coshAngleb=(len_L2^2+len_L3^2-rtSlantSide^2)/(2*len_L2*len_L3);
    hAngle=pi-acos(coshAngleb);% 踝关节角
    theta4=-hAngle;% 构造theta4
    theta1=pi/2;% 构造theta1
    theta2=-pi/2;% 构造theta2
    theta=[theta;theta1,theta2,theta3,theta4];
end

% 求解支撑相关节角度
sptheta=[];
for i=linspace(1,sum,sum)
    % 数据准备
    rtBottomSide=abs(spDx(i));% 底部直角边
    rtRightSide=abs(abs(spDz(i))-len_L1);% 右侧直角边
    rtSlantSide=sqrt(rtBottomSide^2+rtRightSide^2);% 直角三角形斜边
    xAngle1=atan(rtBottomSide/rtRightSide);% 直角三角形膝关节角
    cosxAngle2=(len_L2^2+rtSlantSide^2-len_L3^2)/(2*len_L2*rtSlantSide);
    xAngle2=acos(cosxAngle2);% 普通三角形膝关节角
    theta3=xAngle2-xAngle1;% 构造theta3
    coshAngleb=(len_L2^2+len_L3^2-rtSlantSide^2)/(2*len_L2*len_L3);
    hAngle=pi-acos(coshAngleb);% 踝关节角
    theta4=-hAngle;% 构造theta4
    theta1=pi/2;% 构造theta1
    theta2=-pi/2;% 构造theta2
    sptheta=[sptheta;theta1,theta2,theta3,theta4];
end

%{
% 绘图专用
theta=theta/pi*180;
sptheta=sptheta/pi*180;
%}

%{
% 绘制摆动相运动曲线
figure(1)
plot(time,swDx,'linewidth',2);
ylabel('x/mm')
xlabel('t/s')
set(gca,'FontSize',20,'FontName','Times New Roman','LineWidth',1);
grid on

figure(2)
plot(time,swDz,'linewidth',2);
ylabel('z/mm')
xlabel('t/s')
set(gca,'FontSize',20,'FontName','Times New Roman','LineWidth',1);
grid on

figure(3)
plot(time,swVx,'linewidth',2);
ylabel('Vx/mm·s-1')
xlabel('t/s')
set(gca,'FontSize',20,'FontName','Times New Roman','LineWidth',1);
grid on

figure(4)
plot(time,swVz,'linewidth',2);
ylabel('Vz/mm·s-1')
xlabel('t/s')
set(gca,'FontSize',20,'FontName','Times New Roman','LineWidth',1);
grid on

figure(5)
plot(time,swAx,'linewidth',2);
ylabel('Ax/mm·s-2')
xlabel('t/s')
set(gca,'FontSize',20,'FontName','Times New Roman','LineWidth',1);
grid on

figure(6)
plot(time,swAz,'linewidth',2);
ylabel('Az/mm·s-2')
xlabel('t/s')
set(gca,'FontSize',20,'FontName','Times New Roman','LineWidth',1);
grid on
%}

%{
% 绘制摆动相相角变化
figure(1)
plot(time,theta(:,2),'linewidth',2);
ylabel('\theta_1/deg')
xlabel('t/s')
set(gca,'FontSize',30,'FontName','Times New Roman','LineWidth',1);
grid on

figure(2)
plot(time,theta(:,3),'linewidth',2);
ylabel('\theta_2/deg')
xlabel('t/s')
set(gca,'FontSize',30,'FontName','Times New Roman','LineWidth',1);
grid on

figure(3)
plot(time,theta(:,4),'linewidth',2);
ylabel('\theta_3/deg')
xlabel('t/s')
set(gca,'FontSize',30,'FontName','Times New Roman','LineWidth',1);
grid on

% 绘制支撑相相角变化
figure(1)
plot(time,sptheta(:,2),'linewidth',2);
ylabel('\theta_1/deg')
xlabel('t/s')
set(gca,'FontSize',30,'FontName','Times New Roman','LineWidth',1);
grid on

figure(2)
plot(time,sptheta(:,3),'linewidth',2);
ylabel('\theta_2/deg')
xlabel('t/s')
set(gca,'FontSize',30,'FontName','Times New Roman','LineWidth',1);
grid on

figure(3)
plot(time,sptheta(:,4),'linewidth',2);
ylabel('\theta_3/deg')
xlabel('t/s')
set(gca,'FontSize',30,'FontName','Times New Roman','LineWidth',1);
grid on
%}

%{
% 正运动学求解足端轨迹
simu_T=robot.fkine(theta);
X=[];
Y=[];
Z=[];
for i=linspace(1,sum,sum)
    X=[X simu_T(i).t(1)];
    Y=[Y simu_T(i).t(2)];
    Z=[Z simu_T(i).t(3)];
end
figure(4)
plot3(X,Y,Z);%输出末端轨迹
xlim([-100 50])
ylim([-100 50])
zlim([-200 50])
grid on
hold on
robot.plot(theta,'movie','movie.gif');%动画演示
set(gca,'FontSize',15,'FontName','Times New Roman','LineWidth',1);
%}