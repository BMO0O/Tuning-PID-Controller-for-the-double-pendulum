clc
clear all
close all

%% Loading 2 DOF Robot Dynamics

[m1, m2, g] = deal(1.0, 1.0, 9.806);
[L1, L2] = deal(1.0, 1.0);

syms th1 th2 dth1 dth2 tau1 tau2 
tau = [tau1; tau2];
dq = [dth1; dth2];
a1 = L2/L1*(m2/(m1+m2))*cos(th1-th2);
a2 = L1/L2*cos(th1-th2);
f1 = tau1/((m1+m2)*L1)-L2/L1*(m2/(m1+m2))*dth2^2*sin(th1-th2)-g/L1*sin(th1)-0.5/((m1+m2)*L1)*sign(dth1);
f2 = tau2/L2+L1/L2*dth1^2*sin(th1-th2)-g/L2*sin(th2)-0.5/L2*sign(dth2);

ddTH = (1/(1-a1*a2))*[f1-a1*f2; -a2*f1+f2];
dxdt3 = eval(ddTH(1));
dxdt4 = eval(ddTH(2));
EPx = L1*sin(th1)+L2*sin(th1+th2);
EPy = -L1*cos(th1)-L2*cos(th1+th2);
EP = [EPx; EPy];
[x1, y1] = deal(L1*sin(th1), -L1*cos(th1));

%Xd = 0.2*sin(2*pi*t);
%Yd = 0.2*sin(4*pi*t)-0.5;
%dXd = 0.4*cos(2*pi*t);
%dYd = 0.8*cos(4*pi*t);

%C2 = (Xd^2+Yd^2-L1^2+L2^2)/2*L1*L2;
%S2 = sqrt(1-C2^2);
%C1 = ((L1+L2*C2)*Xd+(L2*S2)*Yd)/((L1+L2*C2)^2+(L2*S2)^2);
%S1 = -(L2*S2*Xd+(L1+L2*C2)*Yd)/((L1+L2*C2)^2+(L2*S2)^2);

%dC2 = (dXd^2+dYd^2-L1^2+L2^2)/2*L1*L2;
%dS2 = sqrt(1-dC2^2);
%dC1 = ((L1+L2*dC2)*dXd+(L2*S2)*dYd)/((L1+L2*dC2)^2+(L2*dS2)^2);
%dS1 = -(L2*dS2*dXd+(L1+L2*dC2)*dYd)/((L1+L2*dC2)^2+(L2*dS2)^2);

%% 2 DOF Robot Simulation
deg = pi/180;
[dT, tf] = deal(0.002, 5.0); 
[th1, th2, dth1, dth2] = deal(15*deg, 60*deg, 0, 0);
[TH1, TH2, dTH1, dTH2, Time] = deal([],[],[],[],[]);
[EPX, EPY, YdX, YdZ] = deal([],[],[],[]);
n    = 1;

global tau1 tau2

%%%% desired q & qdot 선정
qi   = 0;  % [rad] initial q
qd   = 1;  % [rad] desired q
qf   = qd; % [rad] final q
dqd  = 0;  % [rad] desired dq

%qd1 = 1.0;
%qi1 = 0;
qd2 = 1.0;

%%%% ?????
eint1 = 0;
edot1 = 0;
e1 = 0;
eint2 = 0;
edot2 = 0;
e2 = 0;

hold on
axis([-3.0 3.0 -3.0 3.0]);
grid on

x1 = [0 L1];
y1 = [0 0];
x2 = [0 L2];
y2 = [0 0];


%p  = line(x1,y1,'EraseMode','xor','LineWidth',5,'Color','b');
%p2  = line(Ax2,Ay2,'EraseMode','xor','LineWidth',5,'Color','r');
%??? ?? ?????
p1  = line(x1,y1,'EraseMode','xor','LineWidth',5,'Color','b');
p2 = line(x2,y2,'EraseMode','xor','LineWidth',5,'Color','b');

for t = 0:dT:tf
    clc
    disp(t)
    %%%% Trajectory
    %%%% Generation

    Yd1 = 15*deg+0.45*deg*t^2+0.27*deg*t^3;
    Yd2 = 60*deg+24.75*deg*(t)+4.5*deg*(t)^2-0.93*deg*(t)^3;
    dYd1 = 0.9*deg*t+0.81*deg*(t)^2;
    
    dYd2 = 24.75*deg+9.0*deg*(t)-2.79*deg*(t)^2;

   
   % dYd = [0.4*cos(2*pi*t); 0.8*sin(4*pi*t)];
    
    %qd2 = atan2(S2, C2);
    %qd1 = atan2(S1, C1);
    %dqd2 = atan2(dS2, dC2);
    %dqd1 = atan2(dS1, dC1);
%     %%%% 1) set normal input (first method)
%     tau1 = 0.0; %step input? ??? ?
%     tau2 = 1.0;
     %%%% 2) find parameter for ZN (first method)
      %K_1 = 0.051;
      %L_1 = 0.18;
      %T_1 = 0.64;
      %K_2 = 0.102;
      %L_2 = 0.186;
      %T_2 = 0.618;
%     %%%% 3) set pid input (first method)
      %e1    = qd1 - th1;
      %edot1 = 0 - dth1;
      %eint1 = eint1 + e1*dT;
      %Kp1   = 1.2*T_1/L_1/K_1;
      %Ti1   = 2*L_1 + 0.2;      % + 0.2
      %Td1   = 0.5*L_1 + 0.05;    % + 0.05
      %tau1 = Kp1*(e1 + Td1*edot1 + 1/Ti1*eint1);
      %e2    = qd2 - th2;
      %edot2 = 0 - dth2;
      %eint2 = eint2 + e2*dT;
      %Kp2   = 1.2*T2/L_2/K2;
      %Ti2   = 2*L_2 + 0.5;      % + 0.2
      %Td2   = 0.5*L_2 + 0.08;    % + 0.05
      %tau2 = Kp2*(e2 + Td2*edot2 + 1/Ti2*eint2);
  
%     %%%% 1) set Kp (second method)
      Kp1   = 90; %??? method?? ????? ??? (??? ???)
      Kp2 = 70.5;
%      e1 = Yd1 - th1;
%      e2 = Yd2-th2;
      %tau1 = Kp*e1;
      %tau2 = 0;
%      tau1 = 0;
%      tau2 = Kp2*e2;

%     %%%% 2) find parameter for ZN (second method)
     Kcr1 = Kp1;
     Pcr1 = 0.764;
     Kcr2 = Kp2;
     Pcr2 = 0.628;
     %%%% 3) find pid input (second method)
     e1 = Yd1 - th1;
     e2 = Yd2 - th2;
%     e1    = qd1 - th1;
     edot1 = dYd1 - dth1;
     eint1 = eint1 + e1*dT;
      %e2 = qd2 - th2;
      edot2 = dYd2 - dth2;
      eint2 = eint2 + e2*dT;
%     edot = dqd - qdot;
%     eint = eint + e*dT;
     Kp1   = 0.6*Kcr1;
     Ti1   = 0.5*Pcr1 +0.8+3.0+2.0+3.0+5.0;     
     Td1   = 0.125*Pcr1 +0.3;%+3.0+2.0+3.0;   
     tau1    = (Kp1 + (1/4)^2)*(e1 + Td1*edot1 + 1/Ti1*eint1);
%     tau2 = 0;
     Kp2   = 0.6*Kcr2;
     Ti2   = 0.5*Pcr2 +1.4+3.0+2.0+3.0+5.0;     
     Td2   = 0.125*Pcr2 +0.3;%+3.0+2.0+5.0;   
     tau2    = (Kp2 + (1/4)^2)*(e2 + Td2*edot2 + 1/Ti2*eint2);
%     tau1 = 0;
%     u    = (Kp + (1/4)^2)*(e + Td*edot + 1/Ti*eint);

    %%%% Dynamics
    [st, y] = ode45('TwoLinkRobot', [0, dT], [th1;th2;dth1;dth2]); % Dynmaics 실행
    index   = size(y);
    [th1,th2,dth1,dth2] = deal(y(index(1),1),y(index(1),2),y(index(1),3),y(index(1),4));
    %[th1,th2,dth1,dth2] = deal(y(index(1),1),0,y(index(1),3),0);
    %[th1,th2,dth1,dth2] = deal(0,y(index(1),2),0,y(index(1),4));
    [TH1, TH2] = deal([TH1; th1],[TH2; th2]);
    [dTH1,dTH2] = deal([dTH1; dth1], [dTH2; dth2]);
    Time = [Time; t];
    %index   = size(x);
    %q       = x(index(1),1);    % 결과 q값 저장
    %qdot    = x(index(1),2);    % 결과 dq값 저장
 
    %%%% Kinematics? ?? ?????(??? ??)
    [x1, y1] = deal(L1*sin(th1), -L1*cos(th1));
    EPx = L1*sin(th1)+L2*sin(th1+th2);
    EPy = -L1*cos(th1)-L2*cos(th1+th2);
    %px = r*sin(q);
    %Ax1 = [0 px];
    %py = -r*cos(q);
    %Ay1 = [0 py];
    
    %px2 = 
    %py2 = 
    
    %Ax2 = [px px2]
    %Ay2 = [py py2] %?? ????? ??? ??
    
    %%%% Data save
%    save_time(n,:) = t;         % 시간 저장
%    save_q(n,:) = th1;    % 결과 저장
    
    
%    save_dq(n,:) = qdot;    % 결과 저장
    n = n + 1;
    
    %%%% 실시간 그림 그리기
    if rem(n,10) == 0
        set(p1,'X',[0,x1],'Y',[0,y1]);
        set(p2,'X',[x1,EPx],'Y',[y1,EPy]);
        drawnow
    end
    
end

%%%% 결과 그래프 도출

FG = figure(2);
axes(FG)
hold on
grid on
plot(Time, TH2);
plot(Time, TH1);
%[FX] = gradient(TH2);
%plot(save_time, save_q)

% plot(save_time, save_dq)