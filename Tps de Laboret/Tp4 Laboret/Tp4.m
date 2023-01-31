m= 2
b= 0.4
delta= 90 %en grados 
l=1,G=10 
[A,B,C]=linmod('pendulo_mod_tarea',delta*pi/180) 
Aa=[[A;C] zeros(3,1)] 
Ba=[B;0] 
p= -3 %dato 
K=acker(Aa,Ba,[p p p]) 
k1=K(1) 
k2=K(2) 
k3=K(3) 
eig(Aa-Ba*K) % polos lazo cerrado 
tscalc=7.5/(-p) % tiempo de respuesta calculado
sim('pendulo_pid_tarea') 
figure(1), plot(tout,yout) 
grid on, title('Salida') 
figure(2), plot(yout,velocidad) %plano de fase  
grid on, title('Plano de fases') 
figure(3), plot(tout,torque) % torque total 
grid on, title('Torque') 
figure(4), plot(tout,-accint) % acción integral 
grid on, title('Accion integral') 
mnom=m % masa nominal 
m=1.1*mnom
ymax=max(yout) % máximo valor de salida 
S=(ymax-delta)/delta*100 % sobrepaso en % 
erel=(delta-yout)/delta; %error relativo 
efinal=erel(end) % error final, debe ser cero  
ind=find(abs(erel)>.02); % índice elementos con error relativo absoluto menor a 2%  tss=tout(ind(end)) % tiempo de establecimiento (ultimo valor del vector) 
yte=yout(ind(end)) % salida al tiempo ts 
uf=torque(end) % torque final 
Intf=-accint(end) % acción integral final 
% para analizar robustez 
 % correr de nuevo el código de simulación, dibujo y analisis 
% ………………………… 
% m=1.1*mnom % ídem




