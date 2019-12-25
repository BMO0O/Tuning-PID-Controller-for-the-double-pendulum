function dxdt = pen2dof(~,X)
global tau1 tau2
%[ tau1, tau2] = deal( 0, 0);
[  th1,  th2] = deal(X(1),X(2));
[ dth1, dth2] = deal(X(3),X(4));
dxdt1 = X(3); % dth1
dxdt2 = X(4); % dth2
% ????? ?? ?? ????? ????? ??? ?? ??? 
%CENTRAL OF MASS IS MIDDLE OF THE LINK --> C = 0.2
% dxdt3 = - ((7*sin(th2)*dth2^2)/625 + (14*dth1*sin(th2)*dth2)/625 + (7*tau1)/50 - (6867*sin(th1 + th2))/25000 - (20601*sin(th1))/25000)/((4*cos(th2)^2)/625 - 21/500) - (((2*cos(th2))/25 + 7/50)*((981*sin(th1 + th2))/500 - tau2 + (dth1*dth2*sin(th2))/25 + (dth1*sin(th2)*(2*dth1 - dth2))/25))/((4*cos(th2)^2)/625 - 21/500); % dth1
% dxdt4 = (((4*cos(th2))/25 + 11/25)*((981*sin(th1 + th2))/500 - tau2 + (dth1*dth2*sin(th2))/25 + (dth1*sin(th2)*(2*dth1 - dth2))/25))/((4*cos(th2)^2)/625 - 21/500) + (((2*cos(th2))/25 + 7/50)*((2*sin(th2)*dth2^2)/25 + (4*dth1*sin(th2)*dth2)/25 + tau1 - (981*sin(th1 + th2))/500 - (2943*sin(th1))/500))/((4*cos(th2)^2)/625 - 21/500); % dth2
%CENTRAL OF MASS IS END OF THE LINK --> C = 0.4
dxdt3 = - ((26*sin(th2)*dth2^2)/625 + (52*dth1*sin(th2)*dth2)/625 + (13*tau1)/50 - (12753*sin(th1 + th2))/12500 - (12753*sin(th1))/6250)/((16*cos(th2)^2)/625 - 273/2500) - (((4*cos(th2))/25 + 13/50)*((981*sin(th1 + th2))/250 - tau2 + (2*dth1*dth2*sin(th2))/25 + (2*dth1*sin(th2)*(2*dth1 - dth2))/25))/((16*cos(th2)^2)/625 - 273/2500);
dxdt4 = (((8*cos(th2))/25 + 17/25)*((981*sin(th1 + th2))/250 - tau2 + (2*dth1*dth2*sin(th2))/25 + (2*dth1*sin(th2)*(2*dth1 - dth2))/25))/((16*cos(th2)^2)/625 - 273/2500) + (((4*cos(th2))/25 + 13/50)*((4*sin(th2)*dth2^2)/25 + (8*dth1*sin(th2)*dth2)/25 + tau1 - (981*sin(th1 + th2))/250 - (981*sin(th1))/125))/((16*cos(th2)^2)/625 - 273/2500);
dxdt = [dxdt1;
        dxdt2;
        dxdt3;
        dxdt4];
end

