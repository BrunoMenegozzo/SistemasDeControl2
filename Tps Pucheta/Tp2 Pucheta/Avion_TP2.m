%--------------------------------------------------------------------------
%Simulacion de espacio de estados AVION , verificacion con Sisotool.
%-------------------------------------------------------------------------

% clear all;close all;clc;
% T=70; Kmax=10000; At=T/Kmax;
% u=zeros(Kmax);y=u;yp=y; t=0:At:T-At;
% u=[sign(t)]; %up=[0 diff(u)/At];
% 
% %Entrada
% x1=0;   %alfa
% x2=0;   %tita
% x3=0;   %titap
% x4=500;   %h
% 
% %Datos
% a=0.05;
% b=5;
% c=100;
% w=3;
% 
% %Matrices
% A= [-a a 0 0 ;0 0 1 0; w^2 -w^2 0 0; c 0 0 0] ;
% B= [0; 0; w^2*b; 0];
% C= [0 0 0 1];
% D= [0];
% x= [x1;x2;x3;x4]; 
% 
% %Verificacion
% % Sys = ss(A,B,C,D)
% % [num,den] = ss2tf(Sys.a,Sys.b,Sys.c,Sys.d) ;
% %[num, den]=ss2tf(Sis_ve.a,Sis_ve.b,Sis_ve.c,Sis_ve.d,1);
% % sys1=tf(num,den)
% % %sisotool(sys1);
% % cl=feedback(sys1*Controlador,1);
% % sisotool(cl);
% 
% %[y,x]=lsim(A,B,C,D,u,t,x0);
% 
% 
% U=[0];y_1(1)=0;hh=At*1;tve(1)=0;
% for jj=1:Kmax-1
%  U=u(jj);
%  xp=A*x+B*U;
%  Y =C*x+D*U;
%  x=x+xp*At;
% %  y_1(jj+1)=Y(1);
%  tve(jj+1)=tve(jj)+hh;
%  alfa(jj+1)=x(1,:);
%  tita(jj+1)=x(2,:);
%  tita_p(jj+1)=x(3,:);
%  h(jj+1)=x(4,:);
% end
% 
% subplot(5,1,1);plot(tve,alfa);grid on; title('Angulo alfa');xlim([0 70]); hold on%plot(t,y);
% subplot(5,1,2);plot(tve,tita);grid on; title('angulo tita');xlim([0 70]); hold on
% subplot(5,1,3);plot(tve,tita_p);grid on; title('derivada de tita');xlim([0 70]); hold on
% subplot(5,1,4);plot(tve,h);grid on; title('altura');xlim([0 70]); hold on
% subplot(5,1,5);plot(t,u);grid on; title('Entrada');xlim([0 70]); hold on
% xlabel('Tiempo en segundos')

%--------------------------------------------------------------------------
%
%-------------------------------------------------------------------------
clear all;close all;clc;
T=5; Kmax=10000; At=T/Kmax;
u=zeros(Kmax);y=u;yp=y; t=0:At:T-At;
u=[sign(t)]; %up=[0 diff(u)/At];
e=zeros(T/At,1);
hRef1=100;

%Entrada
x1=0;   %alfa
x2=0;   %tita
x3=0;   %titap
x4=500;   %h

%Datos
a=0.05;
b=5;
c=100;
w=3;

%Matrices
A= [-a a 0 0 ;0 0 1 0; w^2 -w^2 0 0; c 0 0 0] ;
B= [0; 0; w^2*b; 0];
C= [0 0 0 1];
D= [0];
x= [x1;x2;x3;x4]; 

%Datos de controlador
Kp=4.5;Ki=10;Kd=100;
Ts=At;
A1=((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
B1=(-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
C1=Kd/Ts;

U=[0];y_1(1)=0;hh=At*1;tve(1)=0;
for jj=1:Kmax-1
 k=jj+2;   
 U=u(jj);
 xp=A*x+B*U;
 Y =C*x+D*U;
 x=x+xp*At;
 e(k)=hRef1-x(4);
 u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
%  y_1(jj+1)=Y(1);
 tve(jj+1)=tve(jj)+hh;
 alfa(jj+1)=x(1,:);
 tita(jj+1)=x(2,:);
 tita_p(jj+1)=x(3,:);
 h(jj+1)=x(4,:);
end

subplot(5,1,1);plot(tve,alfa);grid on; title('Angulo alfa');xlim([0 5]); hold on%plot(t,y);
subplot(5,1,2);plot(tve,tita);grid on; title('angulo tita');xlim([0 5]); hold on
subplot(5,1,3);plot(tve,tita_p);grid on; title('derivada de tita');xlim([0 5]); hold on
subplot(5,1,4);plot(tve,h);grid on; title('altura');xlim([0 5]); hold on
subplot(5,1,5);plot(t,u);grid on; title('Entrada');xlim([0 5]); hold on
xlabel('Tiempo en segundos')

