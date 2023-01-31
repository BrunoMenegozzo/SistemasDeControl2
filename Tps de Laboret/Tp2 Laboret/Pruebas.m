% calculo de los polos complejos ***
zeta=0.5
tr=2
T=0.2
ws=2*pi/T
w0=4/zeta/tr
wd=w0*sqrt(1-zeta^2)
m=ws/wd % muestras ciclo (da 9)
r=exp(-zeta*w0*T)
Omega=wd*T
p1=r*exp(j*Omega)
% calculo LR *****
G=zpk([],[-2 0],[1])
Gd=c2d(G,0.2,'zoh')
p1=0.52+j*0.43 % redondeado
figure(1) , hold on
plot(p1,'sq','LineWidth',3,'MarkerSize',8,'Color','red')
plot(p1','sq','LineWidth',3,'MarkerSize',8,'Color','red'),
rlocus(Gd)
theta=atand(0.43/(0.52+0.876)) % angulo del cero
fi=180-atand(0.43/(1-0.52)) % polo en el origen
fiC=mod(-fi+theta,180) % controlador (59º)
d=0.43/tand(59) %distancia del cero
c=0.52-d % cero
Cd=zpk([0.6703],[0.26],[12.67],0.2)
F=feedback(Gd*Cd,1), zpk(F)
figure(2), step(F)
t=0:0.2:10 % rampa tiempo
figure(3), lsim(F,t) % simula resp rampa
figure(4), hold on 
plot(p1,'sq','LineWidth',3,'MarkerSize',8,'Color','red')
plot(p1','sq','LineWidth',3,'MarkerSize',8,'Color','red'),
rlocus(Cd*Gd)