% %--------------------------------------------------------------------------
% %Codigo de base
% %--------------------------------------------------------------------------
% % clear;%close all;
% X=-[0; 0];ii=0;t_etapa=1e-7;wRef=2;tF=0.04;
% %Constantes del PID
% Kp=.500;Ki=0.001;Kd=0.0001;color_='r';
% % Kp=1;Ki=0;Kd=0.0001;color_='k';
% % Kp=10;Ki=0;Kd=0;color_='b';
% Ts=t_etapa;
% A1=((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
% B1=(-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
% C1=Kd/Ts;
% e=zeros(tF/t_etapa,1);u=12;
% for t=0:t_etapa:tF
%  ii=ii+1;k=ii+2;
%  X=modmotor(t_etapa, X, u);
%  e(k)=wRef-X(1); %ERROR
%  u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
%  x1(ii)=X(1);%Omega
%  x2(ii)=X(2);%wp
%  acc(ii)=u;
% end
% t=0:t_etapa:tF;
% subplot(3,1,1);hold on;
% plot(t,x1,color_);title('Salida y, \omega_t');
% subplot(3,1,2);hold on;
% plot(t,x2,color_);title('Corriente, i_a');
% subplot(3,1,3);hold on;
% plot(t,acc,color_);title('u, va');
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
% Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;Tl=7.5e-2;
% Va=accion;
% h=1e-7;
% omega= xant(1);
% wp= xant(2);
% for ii=1:t_etapa/h
%  wpp =((-wp*(Ra*J+Laa*B)-omega*(Ra*B+Ki*Km)+Va*Ki)/(J*Laa));
%  wp=wp+h*wpp;
%  omega = omega + h*wp ;
% 
% end
% X=[omega,wp];
% end
%---------------------------------------------------------------------------
% %Prueba de ss / NO FUNCIONA
%---------------------------------------------------------------------------
% T=0.01; Kmax = 10000; At= T/Kmax; t= 0:At:T-At;
% [u0,t] = gensig("square",0.5,0.5);
% Va=12*u0;
% [Tl0,t] = gensig("square",0.5,0.5);
% Tl=12*Tl0;
%  %Va=12
%  %Tl=0
%  u=[Va ; Tl]
% %Entrada
% %ve = 12* square(2*pi*500*t);
% x1=0
% x2=0
% %Datos
% Laa= 366e-6;
% J=5e-9;
% Ra=55.6;
% Bm=0;
% Ki=6.49e-3;
% Km=6.53e-3;
% 
% 
% %Matrices
% A= [-Ra/Laa -Km/Laa ; Ki/J -Bm/J] 
% B= [1/Laa ; -1/J]
% C= [0 1]
% D= [0]
% x0= [x1 ;x2] 
% U = ve
% 
% Sys = ss(A,B,C,D)
% % [num,den] = ss2tf(Sys.a,Sys.b,Sys.c,Sys.d,1)
% TF = tf(Sys)
% [ia,wr]=lsim(A,B,C,D,U,t,x0)
% 
% subplot(3,1,1);plot(t,ia);grid on; title('Corriente ia'); hold on%plot(t,y);
% subplot(3,1,2);plot(t,wr);grid on; title('Velocidad wr'); hold on
% subplot(3,1,3);plot(t,u);grid on; title('Tensión de entrada'); hold on


% %----------------------------------------------------------------------------
% %Prueba con primera derivada usando las ecuaciones directamente
% %----------------------------------------------------------------------------
% clear;%close all;
% X=-[0; 0];ii=0;t_etapa=0.0000001;wRef=2;tF=0.002;
% %Constantes del PID
% Kp=.500;Ki=0.001;Kd=0.0001;color_='r';
% % Kp=1;Ki=0;Kd=0.0001;color_='k';
% % Kp=10;Ki=0;Kd=0;color_='b';
% Ts=t_etapa;
% A1=((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
% B1=(-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
% C1=Kd/Ts;
% e=zeros(tF/t_etapa,1);u=12;
% for t=0:t_etapa:tF
%  ii=ii+1;k=ii+2;
%  X=modmotor(t_etapa, X, u);
%  e(k)=wRef-X(1); %ERROR
%  u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
%  x1(ii)=X(1);%Ia
%  x2(ii)=X(2);%wp
%  acc(ii)=u;
% end
% t=0:t_etapa:tF;
% subplot(3,1,1);hold on;
% plot(t,x1,color_);title('Salida y, \omega_t');
% subplot(3,1,2);hold on;
% plot(t,x2,color_);title('Corriente, i_a');
% subplot(3,1,3);hold on;
% plot(t,acc,color_);title('u, va');
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
% Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;Tl=0.075;
% Va=accion;
% h=1e-7;
% w= xant(1);
% ia= xant(2);
% for ii=1:t_etapa/h
%  wp =(Ki/J)*ia-(B/J)*w-(1/J)*Tl
%  w=w+h*wp;
%  tita=tita+h*w;
%  iap=(-Ra/Laa)*ia-(Km/Laa)*w+(1/Laa)*Va;
%  ia = ia + h*iap;
% 
% end
% X=[w,ia];
% end




% 
% %--------------------------------------------------------------------------
% %Codigo de base
% %--------------------------------------------------------------------------
% clear;%close all;
% X=-[0; 0; 0];ii=0;t_etapa=1e-7;wRef=2;tF=0.001;
% %Constantes del PID
% Kp=.500;Ki=0.001;Kd=0.0001;color_='r';
% % Kp=1;Ki=0;Kd=0.0001;color_='k';
% % Kp=10;Ki=0;Kd=0;color_='b';
% Ts=t_etapa;
% A1=((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
% B1=(-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
% C1=Kd/Ts;
% e=zeros(tF/t_etapa,1);u=12;
% for t=0:t_etapa:tF
%  ii=ii+1;k=ii+2;
%  X=modmotor(t_etapa, X, u);
%  e(k)=wRef-X(2); %ERROR
%  u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
%  
%  x1(ii)=X(1);%Tita
%  x2(ii)=X(2);%Omega
%  x3(ii)=X(3);%wp
%  acc(ii)=u;
% end
% t=0:t_etapa:tF;
% subplot(3,1,1);hold on;
% plot(t,x1,color_);title('Tita');
% subplot(3,1,2);hold on;
% plot(t,x2,color_);title('Salida y, \omega_t');
% subplot(3,1,3);hold on;
% plot(t,x3,color_);title('Corriente, i_a');
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
% Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;%Tl=0;
% Va=accion;
% h=1e-7;
% tita = xant(1);
% omega= xant(2);
% wp= xant(3);
% for ii=1:t_etapa/h
%  wpp =((-wp*(Ra*J+Laa*B)-omega*(Ra*B+Ki*Km)+Va*Ki)/(J*Laa))%-Tl*Ki;
%  wp=wp+h*wpp-Tl*Ki;
%  omega = omega + h*wp;
%  tita= tita+h*omega;
% X=[tita,omega,wp];
% end
% end


%-----------------------------------------------------------------------------
%Simulink
%------------------------------------------------------------------------------
% Laa=366e-6;
% J=5e-9;
% Ra=100;
% B=0;
% Ki=0.833;
% Km=60e-3;
% sim('MotorDC')


% %--------------------------------------------------------------------------
% %Punto 4
% %--------------------------------------------------------------------------
% clear;%close all;
X=-[0; 0; 0];ii=0;t_etapa=1e-7;titaRef=1;tF=0.04;
Kp=.1;Ki=0.01;Kd=5;
Ts=t_etapa;
A1=((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
B1=(-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
C1=Kd/Ts;
e=zeros(tF/t_etapa,1);u=12;
for t=0:t_etapa:tF
 ii=ii+1;k=ii+2;
 X=modmotor(t_etapa, X, u);
 %e(k)=titaRef-X(1); %ERROR
 %u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
 x1(ii)=X(1);%tita
 x2(ii)=X(2);%omega
 x3(ii)=X(3);%ia
 acc(ii)=u;
end
t=0:t_etapa:tF;
subplot(2,2,1);hold on;
plot(t,x1,'r');title('Tita');
subplot(2,2,2);hold on;
plot(t,x2,'r');title('Salida y, \omega_t');
subplot(2,2,3);hold on;
plot(t,x3,'r');title('Ia');
subplot(2,2,4);hold on;
plot(t,acc,'r');title('u, va');
xlabel('Tiempo [Seg.]');
% Para verificar 
% Laa=366e-6;
% J=5e-9;
% Ra=55.6;
% B=0;
% Ki=6.49e-3;
% Km=6.53e-3;
% num=[Ki]
% den=[Laa*J Ra*J+Laa*B Ra*B+Ki*Km ]; %wpp*Laa*J+wp*(Ra*J+Laa*B)+w*(Ra*B+Ki*Km)=Vq*Ki
% sys=tf(num,den)
% step(sys)
function [X]=modmotor(t_etapa, xant, accion)
Laa=366e-6; J=5e-9;Ra=100;B=0;Ki=0.833;Km=60e-3;Tl=7.5e-2;
Va=accion;
h=1e-7;
tita = xant(1);
w= xant(2);
ia= xant(3);
for ii=1:t_etapa/h
    %Modelo con primera derivada de la velocidad angular
 wp=(Ki/J)*ia-(B/J)*w-(1/J)*Tl
 w=w+h*wp;
 tita=tita+h*w;
 iap=(-Ra/Laa)*ia-(Km/Laa)*w+(1/Laa)*Va;
 ia = ia + h*iap;   
   
end
X=[tita,w,ia];
end