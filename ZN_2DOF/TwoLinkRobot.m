function dxdt = TwoLinkRobot(~,X)

global tau1 tau2;

[  th1,  th2] = deal(X(1),X(2));
[ dth1, dth2] = deal(X(3),X(4));
dxdt1 = X(3); % dth1
dxdt2 = X(4); % dth2

%dxdt3 = (1000*tau1 - 100*dth1 + (4903*cos(th1 + th2))/25 + (29418*cos(th1))/25 - 100*sign(dth1) + 10*dth1*dth2*sin(th2) + 10*dth2*sin(th2)*(dth1 + dth2))/(10*cos(th2) - cos(th2)^2 + 175) + ((100*cos(th2) + 500)*((sin(th2)*dth1^2)/100 + dth2/10 - tau2 - (4903*cos(th1 + th2))/25000 + sign(dth2)/10))/(10*cos(th2) - cos(th2)^2 + 175);
%dxdt4 = - ((100*cos(th2) + 500)*(tau1 - dth1/10 + (4903*cos(th1 + th2))/25000 + (14709*cos(th1))/12500 - sign(dth1)/10 + (dth1*dth2*sin(th2))/100 + (dth2*sin(th2)*(dth1 + dth2))/100))/(10*cos(th2) - cos(th2)^2 + 175) - ((200*cos(th2) + 2000)*((sin(th2)*dth1^2)/100 + dth2/10 - tau2 - (4903*cos(th1 + th2))/25000 + sign(dth2)/10))/(10*cos(th2) - cos(th2)^2 + 175);
dxdt3 = (sign(dth1)/4 - tau1/2 + (4903*sin(th1))/500 + (dth2^2*sin(th1 - th2))/2 + (cos(th1 - th2)*(sin(th1 - th2)*dth1^2 + tau2 - sign(dth2)/2 - (4903*sin(th2))/500))/2)/(cos(th1 - th2)^2/2 - 1);
dxdt4 =  -(tau2 - sign(dth2)/2 - (4903*sin(th2))/500 + dth1^2*sin(th1 - th2) + cos(th1 - th2)*((sin(th1 - th2)*dth2^2)/2 - tau1/2 + sign(dth1)/4 + (4903*sin(th1))/500))/(cos(th1 - th2)^2/2 - 1);
dxdt = [dxdt1;
        dxdt2; 
        dxdt3; 
        dxdt4];  % dx; ddx

end