%Verificación de la EDO
% ypp+yp+y=u1+up
% Condiciones iniciales nulas, respuestas al seguimiento.
clear all;close all;clc;
T=10; Kmax=10000; At=T/Kmax;
u=zeros(1,Kmax);y=u;yp=y; t=0:At:T-At;
u=[0 sin(5*t+2)]; up=[0 diff(u)/At];
for jj=1:Kmax-1
 % ypp+yp+y=u1+u1p
 ypp=-yp(jj)-y(jj)+u(jj)+up(jj);
 %Integro
 yp(jj+1)=yp(jj)+ypp*At;
 y(jj+1) =y(jj) +yp(jj)*At;
end
figure;title('Sistema SISO');
subplot(2,2,1);plot(t,y,'.k');title('y'),hold on
subplot(2,2,3);plot(t,u(1:numel(t)),'k');title('u');xlabel('t [seg.]');
A=[0,1;-1,-1];
B=[1;0];
C=[1, 0];
D=[0];
Sis_ve=ss(A,B,C,D)
[num, den]=ss2tf(Sis_ve.a,Sis_ve.b,Sis_ve.c,Sis_ve.d,1)
x=[0;0];
U=[0];y_1(1)=0;hh=At*1;tve(1)=0;
for jj=1:Kmax-1
 U=u(jj);
 xp=A*x+B*U;
 Y =C*x+D*U;
 x=x+xp*At;
 y_1(jj+1)=Y(1);
 tve(jj+1)=tve(jj)+hh;
end
subplot(2,2,1);plot(tve,y_1,'r');
legend('EDO','Evolución en VE');
legend("boxoff");
