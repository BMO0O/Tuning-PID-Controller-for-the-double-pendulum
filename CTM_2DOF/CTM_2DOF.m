format compact
close all,clc 
FigureSize = [450,350];

%% 5. Loading 2 DOF Robot Dynamics

format compact 
load('Dyn2DOF.mat')
% Initialize 
[Cx1,Cy1,Cz1] = deal(   0,  0, -0.4); 
[Cx2,Cy2,Cz2] = deal(   0,  0, -0.4); %Cm
[Lx1,Ly1,Lz1] = deal(   0,  0, -0.4); 
[Lx2,Ly2,Lz2] = deal(   0,  0, -0.4); 
[Iyy1,Iyy2] = deal(0.1, 0.1); 
[m1,m2,g] = deal(   1,   1,   9.81); 
syms th1 th2 dth1 dth2 ddth1 ddth2 tau1 tau2 Ydx Ydz dYdx dYdz ddYdx ddYdz t
%
%Ydx =  0.1*sin(2*pi*t);
%Ydz =  0.1*sin(4*pi*t)-0.5;

tau = [tau1;tau2]; 
ddth = inv(EQ.M)*(tau -EQ.C*EQ.dq -EQ.G); 
dxdt3 = eval(ddth(1)); 
dxdt4 = eval(ddth(2)); 
EPx = eval(EQ.EP(1)); 
EPz = eval(EQ.EP(3));
tauG1 = eval(EQ.G(1));
tauG2 = eval(EQ.G(2));
Yd = [Ydx;0;Ydz];
dYd = [dYdx;0;dYdz];
ddYd = [ddYdx;0;ddYdz];
inv_Jac = (EQ.Jac.')/(EQ.Jac*(EQ.Jac.'));
Err_sum_old = 0;
Err_vel = dYd-EQ.dEP;
Err_pos = Yd-EQ.EP;
%Err_sum = Err_sum_old + Err_pos;
%Err_sum_old = Err_sum;

[Kp, Kd, Ki] = deal(500, 44.8, 7);
%tauCtrl = eval(EQ.Cdq + EQ.G + EQ.M*inv_Jac*(ddYd-EQ.dJac*EQ.dq+Kd.*(dYd-EQ.dEP)+Kp.*(Yd-EQ.EP)+Ki.*Err_sum));
%tauCtrl = eval(EQ.Cdq + EQ.G + (EQ.M)*(inv_Jac)*(ddYd-EQ.dJac*EQ.dq+Kd.*Err_vel+Kp.*Err_pos+Ki.*Err_sum));

VEx = eval(Err_vel(1));
VEz = eval(Err_vel(3));
PEx = eval(Err_pos(1));
PEz = eval(Err_pos(3));


%C2 = (Ydx^2+Ydz^2-Lz1^2-Lz2^2)/2*Lz1*Lz2;
C2 = (Ydx^2+Ydz^2-0.32)/0.32;
S2 = -sqrt(1-C2^2);
Qd2 = atan2(S2, C2);
C1 = ((Lz1+Lz2*C2)*Ydx+(Lz2*-S2)*Ydz)/((Lz1+Lz2*C2)^2+(Lz2*-S2)^2);
S1 = (-(Lz2*-S2)*Ydx+(Lz1+Lz2*C2)*Ydz)/((Lz1+Lz2*C2)^2+(Lz2*-S2)^2);
Qd1 = atan2(S1, C1);
Qd = [Qd1; Qd2];

%jacobian(EQ.EP, [th1 th2]) %tau = J_T*Ks*(Yd-Y)
%tauCtrl = eval(EQ.Jac.'*20*(Yd-EQ.EP)); %20 = Ks 


%% 6. 2DOF Robot Simulation %term project 

deg = pi/180; 
[dt,t,Tend] = deal(0.01, 0, 10.0); %dt? ?? ??? ??? ??? 0.01??? 
[th1,th2,dth1,dth2] = deal(51.32*deg,-102.37*deg,0,-0); %initial value 
%[th1,th2,dth1,dth2] = deal(0.1*sin(2*pi),0.1*sin(4*pi)-0.5,0,-0);
[TH1,TH2,dTH1,dTH2,T] = deal([],[],[],[],[]); 
[EPX,EPZ,YdX,YdZ] = deal([],[],[],[]); %
[PEX,PEZ,VEX,VEZ] = deal([],[],[],[]);
Errer_sum = 0;
Err_sum_old = 0;
[Joint1, Joint2, QD1] = deal([],[],[]);
global tau1 tau2
while(t(end)<Tend-dt) 
    
    Yd = [ 0.1*sin(2*pi*t(end)); %r=0.1, 1?? ??? = 1[rad/s] Ydx
                      0;
          0.1*sin(4*pi*t(end))-0.5];%  ???? ??, trajectory shaped like butterfly
    [Ydx, Ydz] = deal(Yd(1),Yd(3));
    
    dYd = [0.2*pi*cos(2*pi*t(end));
                      0;
           0.4*pi*cos(4*pi*t(end))];
       
   [dYdx, dYdz] = deal(dYd(1),dYd(3));
      
    ddYd = [ -0.4*pi*pi*sin(2*pi*t(end));
                      0;
             -1.6*pi*pi*sin(4*pi*t(end))];
         
   [ddYdx, ddYdz] = deal(ddYd(1),ddYd(3));
   
   Err_sum = Err_sum_old + Err_pos;
   Err_sum_old = Err_sum;
 
   tauCtrl = eval(EQ.Cdq + EQ.G + EQ.M*inv_Jac*(ddYd-EQ.dJac*EQ.dq+Kd.*(dYd-EQ.dEP)+Kp.*(Yd-EQ.EP)+Ki.*Err_sum));

    %tauCtrl = eval(EQ.Cdq + EQ.G + EQ.Jac.'*200*(Yd-EQ.EP)); % Ks = 200[Nm] spring %EQ.Cdq,,,?? ??  ??? ??
    tau1 = tauCtrl(1);%-5*dth1;
    tau2 = tauCtrl(2);%-5*dth2;
    %tau1 = eval(tauCtrl(1))-5*dth1;
    %tau2 = eval(tauCtrl(2))-5*dth2;
    %tau1 = eval(tauCtrl(1)); %5[rad/s] damper <-- CTM ????? ?? X ?? ?? ?? tauCtrl(1)
    %tau2 = eval(tauCtrl(2));

    [t,y] = ode45(@pen2dof, [t(end),t(end)+dt], [th1;th2;dth1;dth2] );
    [th1,th2,dth1,dth2]= deal(y(end,1),y(end,2),y(end,3),y(end,4)); 
    [ TH1, TH2] = deal([ TH1;  th1],[ TH2;  th2]); 
    [dTH1,dTH2] = deal([dTH1; dth1],[dTH2; dth2]);  
       
    T = [  T; t(end)]; 
    EPX = [EPX; eval(EPx)];
    EPZ = [EPZ; eval(EPz)];
    
    VEX = [VEX; eval(VEx)];
    VEZ = [VEZ; eval(VEz)];
    PEX = [PEX; eval(PEx)];
    PEZ = [PEZ; eval(PEz)];

    QD1 = [QD1; eval(Qd1)];
    Joint1 = [Joint1; eval((-Qd1+1.5725)-th1)];
    Joint2 = [Joint2; eval(Qd2-th2)];
    
    [YdX,YdZ] = deal([YdX;Yd(1)],[YdZ;Yd(3)]); %
end



% %% 7.2 DOF Robot Simulation Result Plot
close all
FG = figure(6); %draw 
set(FG, 'color', 'w', 'pos', [10 250 FigureSize]) %show on this position 10, 250 --> figure position
hold on, grid on  %grid draw line on graph
%plot(T, TH1*180/pi, 'linew', 1.5, 'displ', '\theta1') %thick of line: linew, line name: displ %theta1
%plot(T, TH2*180/pi, 'linew', 1.5, 'displ', '\theta2') %theta2

plot(T, PEX, 'linew', 1.5, 'displ', 'X') %thick of line: linew, line name: displ %theta1
plot(T, PEZ, 'linew', 1.5, 'displ', 'Z') %theta2
%useful for writing assignment 
legend show
set(gca, 'fontsize', 16)
xlabel('time[s]', 'fontsize', 16)
ylabel('Cartesian Space Error[m]', 'fontsize', 16)
title('Pendulum 2 DOF', 'fontsize', 16)



%until this, show figure 6

% %% 8. 2DOF Robot Simulation Result Plot(

FG = figure(7); %draw 
set(FG, 'color', 'w', 'pos', [450 250 FigureSize]) %show on this position
hold on, grid on  %grid draw line on graph
%plot(T, -QD1, 'linew', 1.5, 'displ', '1') %thick of line: linew, line name: displ %theta1
%plot(T, TH1, 'linew', 1.5, 'displ', '2')
plot(T, Joint1, 'linew', 1.5, 'displ', '1') %thick of line: linew, line name: displ %theta1
plot(T, Joint2, 'linew', 1.5, 'displ', '2') %theta2

%useful for writing assignment 
legend show
set(gca, 'fontsize', 16)
xlabel('time[s]', 'fontsize', 16)
ylabel('Jointspace Error[rad]', 'fontsize', 16)
title('Pendulum 2 DOF', 'fontsize', 16)

% %% 9. 2 DOF Robot Animation
FG = figure(8); clf %erase figure 
set(FG,'color','w','pos',[750 250 FigureSize]) 
hold on,grid off,axis equal 
P1 = plot(0,0,'marker','o','linew',1.5); 
P2 = plot(0,0,'marker','o','linew',1.5); 
TJ1 = plot(EPX, EPZ, '-k'); %?? ???? ?????? ?? ??
TJ2 = plot(YdX, YdZ, '--r'); %?? ???? ?????? ?? ??
TXT = text(0.2, -0.1, 'hello'); %TXT = ??? ?? ??, ?? ???? ??? ??
TXT2 = text(-0.45, -0.1, 'hello');
TXT3 = text(-0.45, -0.15, 'hello');
TXT4 = text(-0.45, -0.2, 'hello');

xlim([-0.5 0.5]),ylim([-1 0]) 
v = VideoWriter('Video.avi'); %%%%video
open(v); %%%video
tic; 

%drawing robot position 
for i1 = 1:5:numel(T) %1?? ??? ???     
    while(toc<T(i1));    end    
    th = TH1(i1);  %x-z ??? ??? ?, ???? ??? ? y??? ??? ??? x? ??? -??.  
    set(TXT, 'String', strcat('Time:', num2str(T(i1))))  % ??? string?? num2str()
    set(TXT2, 'String', strcat('Kp:', num2str(Kp)))
    set(TXT3, 'String', strcat('Kd:', num2str(Kd)))
    set(TXT4, 'String', strcat('Ki:', num2str(Ki)))
    
    [x1,y1] = deal(-0.4*sin(th),-0.4*cos(th));    
    set(P1,'Xdata',[0,x1],'Ydata',[0,y1])    
    set(P2,'Xdata',[x1,EPX(i1)],'Ydata',[y1,EPZ(i1)])
    drawnow; 
    writeVideo(v,getframe(gcf));%%%%video
end

close(v); %%video
