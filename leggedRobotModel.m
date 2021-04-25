clear;
clc;

% ����������ģ��
%       theta d a alpha offset
H=Link([0     0 0 pi/2  0]);
L1=Link([0    0 0 -pi/2 0]);
L2=Link([0    0 0 0     0]);
L3=Link([0    0 0 0     0]);
% �趨���˳��� ��λmm
len_H=1;
len_L1=37.63;
len_L2=77.22;
len_L3=81.99;
H.a=len_H;
L1.a=len_L1;
L2.a=len_L2;
L3.a=len_L3;
robot=SerialLink([H L1 L2 L3],'name','leggedRobot');

% ���ƻ�����
init_q=[pi/2,-pi/2,-atan(len_L3/len_L2),pi/2];
init_T=robot.fkine(init_q);
% robot.plot(init_q);

% ���ڻ�����
% teach(robot)

% ��������˹켣�滮
T=2;% �˶�����
h=60;% ����
s=120;% ����
sum=100;% ��������
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

% ���ɰڶ�����˹켣
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

% ����֧������˹켣
for i=time
    spDx=[spDx supportDx(s,i,T)-2*s];
    spDz=[spDz supportDz()+init_T.t(3)];
end

% ���ڶ���ؽڽǶ�
theta=[];
for i=linspace(1,sum,sum)
    % ����׼��
    rtBottomSide=abs(swDx(i));% �ײ�ֱ�Ǳ�
    rtRightSide=abs(abs(swDz(i))-len_L1);% �Ҳ�ֱ�Ǳ�
    rtSlantSide=sqrt(rtBottomSide^2+rtRightSide^2);% ֱ��������б��
    xAngle1=atan(rtBottomSide/rtRightSide);% ֱ��������ϥ�ؽڽ�
    cosxAngle2=(len_L2^2+rtSlantSide^2-len_L3^2)/(2*len_L2*rtSlantSide);
    xAngle2=acos(cosxAngle2);% ��ͨ������ϥ�ؽڽ�
    theta3=-(xAngle1+xAngle2);% ����theta3
    coshAngleb=(len_L2^2+len_L3^2-rtSlantSide^2)/(2*len_L2*len_L3);
    hAngle=pi-acos(coshAngleb);% �׹ؽڽ�
    theta4=hAngle;% ����theta4
    theta1=pi/2;% ����theta1
    theta2=-pi/2;% ����theta2
    theta=[theta;theta1,theta2,theta3,theta4];
end

% ���֧����ؽڽǶ�
sptheta=[];
for i=linspace(1,sum,sum)
    % ����׼��
    rtBottomSide=abs(spDx(i));% �ײ�ֱ�Ǳ�
    rtRightSide=abs(abs(spDz(i))-len_L1);% �Ҳ�ֱ�Ǳ�
    rtSlantSide=sqrt(rtBottomSide^2+rtRightSide^2);% ֱ��������б��
    xAngle1=atan(rtBottomSide/rtRightSide);% ֱ��������ϥ�ؽڽ�
    cosxAngle2=(len_L2^2+rtSlantSide^2-len_L3^2)/(2*len_L2*rtSlantSide);
    xAngle2=acos(cosxAngle2);% ��ͨ������ϥ�ؽڽ�
    theta3=-(xAngle1+xAngle2);% ����theta3
    coshAngleb=(len_L2^2+len_L3^2-rtSlantSide^2)/(2*len_L2*len_L3);
    hAngle=pi-acos(coshAngleb);% �׹ؽڽ�
    theta4=hAngle;% ����theta4
    theta1=pi/2;% ����theta1
    theta2=-pi/2;% ����theta2
    sptheta=[sptheta;theta1,theta2,theta3,theta4];
end

% ���ưڶ����˶�����
figure(1)
subplot(3,2,1)
plot(time,swDx,'linewidth',2);
title('x-t')
subplot(3,2,2)
plot(time,swDz,'linewidth',2);
title('z-t')
subplot(3,2,3)
plot(time,swVx,'linewidth',2);
title('vx-t')
subplot(3,2,4)
plot(time,swVz,'linewidth',2);
title('vz-t')
subplot(3,2,5)
plot(time,swAx,'linewidth',2);
title('ax-t')
subplot(3,2,6)
plot(time,swAz,'linewidth',2);
title('az-t')

% ���ưڶ�����Ǳ仯
figure(2)
subplot(3,1,1)
plot(time,theta(:,2),'linewidth',2);
title('\theta_2-t')
subplot(3,1,2)
plot(time,theta(:,3),'linewidth',2);
title('\theta_3-t')
subplot(3,1,3)
plot(time,theta(:,4),'linewidth',2);
title('\theta_4-t')

% ����֧������Ǳ仯
figure(3)
subplot(3,1,1)
plot(time,sptheta(:,2),'linewidth',2);
title('\theta_2-t')
subplot(3,1,2)
plot(time,sptheta(:,3),'linewidth',2);
title('\theta_3-t')
subplot(3,1,3)
plot(time,sptheta(:,4),'linewidth',2);
title('\theta_4-t')

% ���˶�ѧ�����˹켣
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
plot3(X,Y,Z);%���ĩ�˹켣
xlim([-200 200])
ylim([-200 200])
zlim([-200 200])
grid on
hold on
robot.plot(theta,'movie','movie.gif');%������ʾ
%}