t=0:Tm:100*Tm % genera rampa
lsim(F,t,t)
G=zpk([0],[-1 -1],[10])
Tm=0.23
Gd=c2d(G,Tm,'zoh')
Gd1=c2d(G,10*Tm,'zoh')
rlocus(Gd)
% step(Gd)