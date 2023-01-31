% % T=1; Kmax=30000; At=T/Kmax;
% % u1=zeros(1,Kmax);y1=u1;y2=u1;y1p=y1;y2p=y2; t=0:At:T-At;y1pp=0;
% % u1=[0 1.15e-3* sign(cos(10.471*t))];u1p=[0 diff(u1)/At];
% % u2=[0 1.5* sign(cos(1.3*t+1))];u2p=[ 0 diff(u2)/At];
% % plot(t,u1(1:numel(t)));
% 
% %-------------------------------------------------------------------------
% X=-[0; 0; 0];ii=0;t_etapa=1e-7;titaRef=1;tF=0.04;
% Kp=.1;Ki=0.01;Kd=5;
% Ts=t_etapa;
% A1=((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
% B1=(-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
% C1=Kd/Ts;
% e=zeros(tF/t_etapa,1);
% u1=[0 1.15e-3* sign(cos(10.471*t))];u1p=[0 diff(u1)/At];
% u2=[0 12*sign(cos(0.001*t))];
% u=[u1;u2];
% for t=0:t_etapa:tF
%  ii=ii+1;k=ii+2;
%  X=modmotor(t_etapa, X, u);
% %  e(k)=titaRef-X(1); %ERROR
% %  u2=u2+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
%  x1(ii)=X(1);%tita
%  x2(ii)=X(2);%omega
%  x3(ii)=X(3);%ia
%  acc(ii)=u(2);
% end
% t=0:t_etapa:tF;
% % figure1
% subplot(2,2,1);hold on;
% plot(t,x1,'r');title('Tita');
% subplot(2,2,2);hold on;
% plot(t,x2,'r');title('Salida y, \omega_t');
% subplot(2,2,3);hold on;
% plot(t,x3,'r');title('Ia');
% subplot(2,2,4);hold on;
% plot(t,acc,'r');title('u, va');
% xlabel('Tiempo [Seg.]');
% 
% % Para verificar 
% % Laa=366e-6;
% % J=5e-9;
% % Ra=55.6;
% % B=0;
% % Ki=6.49e-3;
% % Km=6.53e-3;
% % num=[Ki]
% % den=[Laa*J Ra*J+Laa*B Ra*B+Ki*Km ]; %wpp*Laa*J+wp*(Ra*J+Laa*B)+w*(Ra*B+Ki*Km)=Vq*Ki
% % sys=tf(num,den)
% % step(sys)
% function [X]=modmotor(t_etapa, xant, accion)
% Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;%Tl=7.5e-2;
% Va=accion(2);
% Tl=accion(1);
% h=1e-7;
% tita = xant(1);
% w= xant(2);
% ia= xant(3);
% for ii=1:t_etapa/h
%     %Modelo con primera derivada de la velocidad angular
%  wp =(Ki/J)*ia-(B/J)*w-(1/J)*Tl
%  w=w+h*wp;
%  tita=tita+h*w;
%  iap=(-Ra/Laa)*ia-(Km/Laa)*w+(1/Laa)*Va;
%  ia = ia + h*iap;   
%    
% end
% X=[tita,w,ia];
% end

% ---------------------------------------------------------------------------
%Prueba 2 
%-------------------------------------------------------------------------------
% X=-[0; 0; 0];ii=0;t_etapa=1e-7;titaRef=1;tF=0.04;
% Kp=.1;Ki=0.01;Kd=5;
% Ts=t_etapa;
% A1=((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
% B1=(-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
% C1=Kd/Ts;
% e=zeros(tF/t_etapa,1);
% u=[0 1.15e-3* sign(cos(10.471*t))];u1p=[0 diff(u1)/At];
% for t=0:t_etapa:tF
%  ii=ii+1;k=ii+2;
%  X=modmotor(t_etapa, X, u);
%  e(k)=titaRef-X(1); %ERROR
%  u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
%  x1(ii)=X(1);%tita
%  x2(ii)=X(2);%omega
%  x3(ii)=X(3);%ia
%  acc(ii)=u(2);
% end
% t=0:t_etapa:tF;
% subplot(2,2,1);hold on;
% plot(t,x1,'r');title('Tita');
% subplot(2,2,2);hold on;
% plot(t,x2,'r');title('Salida y, \omega_t');
% subplot(2,2,3);hold on;
% plot(t,x3,'r');title('Ia');
% subplot(2,2,4);hold on;
% plot(t,acc,'r');title('u, va');
% xlabel('Tiempo [Seg.]');
% % Para verificar 
% % Laa=366e-6;
% % J=5e-9;
% % Ra=55.6;
% % B=0;
% % Ki=6.49e-3;
% % Km=6.53e-3;
% % num=[Ki]
% % den=[Laa*J Ra*J+Laa*B Ra*B+Ki*Km ]; %wpp*Laa*J+wp*(Ra*J+Laa*B)+w*(Ra*B+Ki*Km)=Vq*Ki
% % sys=tf(num,den)
% % step(sys)
% function [X]=modmotor(t_etapa, xant, accion)
% Laa=366e-6; J=5e-9;Ra=100;B=0;Ki=0.833;Km=60e-3;%Tl=7.5e-2;
% Tl=accion;
% Va=12;
% h=1e-7;
% tita = xant(1);
% w= xant(2);
% ia= xant(3);
% for ii=1:t_etapa/h
%     %Modelo con primera derivada de la velocidad angular
%  wp =(Ki/J)*ia-(B/J)*w-(1/J)*Tl
%  w=w+h*wp;
%  tita=tita+h*w;
%  iap=(-Ra/Laa)*ia-(Km/Laa)*w+(1/Laa)*Va;
%  ia = ia + h*iap;   
%    
% end
% X=[tita,w,ia];
% end

%---------------------------------------------------------------------------
%Intento 3 
%---------------------------------------------------------------------------
% clear all;close all;clc;
% T=2; Kmax=10000; At=T/Kmax;
% Tl=zeros(1,Kmax);y=Tl;yp=y; t=0:At:T-At;
% Tl=[0 ((1.15e-3/2)*sign(sin(10.472*t))+(1.15e-3/2))];
% %Tl=u0+(1.15e-3/2); up=[0 diff(u)/At];
% % for jj=1:Kmax-1
% %  % ypp+yp+y=u1+u1p
% %  ypp=-yp(jj)-y(jj)+u(jj)+up(jj);
% %  %Integro
% %  yp(jj+1)=yp(jj)+ypp*At;
% %  y(jj+1) =y(jj) +yp(jj)*At;
% % end
% % figure;title('Sistema SISO');
% % subplot(2,1,1);plot(t,y,'.k');title('y'),hold on
% % subplot(2,1,2);plot(t,u(1:numel(t)),'k');title('u');xlabel('t [seg.]');
% Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;%Tl=7.5e-2;
% X=[0;0;0];
% %Tl=u;
% Va=12;
% h=At;
% tita = X(1);
% w= X(2);
% ia= X(3);
% for ii=1:Kmax
%     %Modelo con primera derivada de la velocidad angular
%  wp =(Ki/J)*ia-(B/J)*w-(1/J)*Tl(ii)
%  w=w+h*wp;
%  tita=tita+h*w;
%  iap=(-Ra/Laa)*ia-(Km/Laa)*w+(1/Laa)*Va;
%  ia = ia + h*iap;   
%  x1(ii)=X(1);%tita
%  x2(ii)=X(2);%omega
%  x3(ii)=X(3);%ia
%  %acc(ii)=Tl;
% end
% X=[tita,w,ia];
% subplot(2,2,1);hold on;
% plot(t,x1,'r');title('Tita');
% subplot(2,2,2);hold on;
% plot(t,x2,'r');title('Salida y, \omega_t');
% subplot(2,2,3);hold on;
% plot(t,x3,'r');title('Ia');
% subplot(2,2,4);hold on;
% plot(t,acc,'r');title('u, va');
% xlabel('Tiempo [Seg.]');
% % A=[-Ra/Laa,0,-Km/Laa;0,0,1;Ki/J,0,-B/J];
% % B=[0;0;-1/J];
% % C=[1, 0, 0];
% % D=[0];
% % Sis_ve=ss(A,B,C,D);
% % [num, den]=ss2tf(Sis_ve.a,Sis_ve.b,Sis_ve.c,Sis_ve.d,1)
% % x=[0;0;0];
% % U=[0];y_1(1)=0;hh=At*1;tve(1)=0;
% % for jj=1:Kmax-1
% %  U=u(jj);
% %  xp=A*x+B*U;
% %  Y =C*x+D*U;
% %  x=x+xp*At;
% %  y_1(jj+1)=Y(1);
% %  tve(jj+1)=tve(jj)+hh;
% % end
% %subplot(2,2,1);plot(tve,y_1,'r');
% subplot(2,2,2);plot(t,u(1:numel(t)),'r');



%---------------------------------------------------------------------------
%Punto 1.1
%---------------------------------------------------------------------------
clear all;close all;
X=-[0; 0; 0];ii=0;t_etapa=1e-7;titaRef1=-1.57;titaRef2=1.57;tF=1;
Kp=4.5;Ki=200;Kd=2e-6;
Ts=t_etapa;
A1=((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
B1=(-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
C1=Kd/Ts;
e=zeros(tF/t_etapa,1);
u=12;Tl=0;
for t=0:t_etapa:tF
 ii=ii+1;k=ii+2;
 Tl_arreglo=[0 ((1.15e-3/2)*sign(sin(10.472*t))+(1.15e-3/2))];
 Tl=Tl_arreglo(2);
 U = [u Tl]; 
 X=modmotor(t_etapa, X, U);
 if(Tl<=1e-3)
     e(k)=titaRef1-X(1); %ERROR en punto1
 else
     e(k)=titaRef2-X(1); %ERROR en punto 2 
 end
 u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
 x1(ii)=X(1);%tita
 x2(ii)=X(2);%omega
 x3(ii)=X(3);%ia
 torque(ii)=Tl;%Torque Tl
 acc(ii)=u;
end
%Grafico
t=0:t_etapa:tF;
figure;
subplot(3,1,1);hold on;
plot(t,x1(1:numel(t)),'r');title('Tita');xlabel('Tiempo [Seg.]');
subplot(3,1,2);hold on;
plot(t,x2(1:numel(t)),'r');title('Salida y, \omega_t');xlabel('Tiempo [Seg.]');
subplot(3,1,3);hold on;
plot(t,x3(1:numel(t)),'r');title('Ia');xlabel('Tiempo [Seg.]');

figure;
subplot(2,1,1);hold on;
plot(t,torque(1:numel(t)),'r');title('Torque de carga');
xlabel('Tiempo [Seg.]');
subplot(2,1,2);hold on;
plot(t,acc(1:numel(t)),'r');title('Accion de control');
xlabel('Tiempo [Seg.]');

%Modelo del motor
function [X]=modmotor(t_etapa, xant, accion)
Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;
Va=accion(1);
h=1e-7;
Tl=accion(2);
tita = xant(1);
w= xant(2);
ia= xant(3);
for ii=1:t_etapa/h
    %Modelo con primera derivada de la velocidad angular
 wp=(Ki/J)*ia-(B/J)*w-(1/J)*Tl;
 w=w+h*wp;
 tita=tita+h*w;
 iap=(-Ra/Laa)*ia-(Km/Laa)*w+(1/Laa)*Va;
 ia = ia + h*iap;   
   
end
X=[tita,w,ia];
end
