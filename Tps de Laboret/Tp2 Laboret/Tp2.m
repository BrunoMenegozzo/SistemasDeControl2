p1=0.7851+j*0.2361 % redondeado
G=zpk([0],[-1 -1],[10])
Tm=0.23
Gd=c2d(G,Tm,'zoh')
Cd=zpk([0 -0.1132],[0 1],[0.035312],0.23)
F=feedback(Gd*Cd,1)
A=Gd*Cd

sisotool(Gd)
% pole(F)
% zero(F)
% pzmap(F)
% Kp=dcgain(Gd*Cd)
%step(F) % respuesta al escalon
%
% figure(1),hold on
% plot(p1,'sq','LineWidth',3,'MarkerSize',8,'Color','red')
% plot(p1','sq','LineWidth',3,'MarkerSize',8,'Color','red'),
% rlocus(Gd*Cd)
% figure(2)
%  step(F)
% % figure(3),hold on
% % rlocus(F)


