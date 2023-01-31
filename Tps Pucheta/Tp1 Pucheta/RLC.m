% [u0,t] = gensig("square",2e-3,4e-3);
% u=24*u0-12;
% %Entrada
% 
% x1=0
% x2=0
% %Datos
% R=4000
% L=0.036
% C=26.18e-9
% 
% %Matrices
% A= [-R/L -1/L ;1/C 0] 
% B= [1/L ;0]
% C= [R 0]
% D= [0]
% x0= [x1 ;x2] 
% 
% 
% Sys = ss(A,B,C,D)
% 
% TF = tf(Sys)
% [i,vc]=lsim(A,B,C,D,u,t,x0)
% 
% subplot(3,1,1);plot(t,i);grid on; title('Corriente'); hold on%plot(t,y);
% subplot(3,1,2);plot(t,vc);grid on; title('Tensión en capacitor'); hold on
% subplot(3,1,3);plot(t,u);grid on; title('Tensión de entrada'); hold on

%-------------------------------------------------------------------------------
%Punto 3 RLC
%-----------------------------------------------------------------------------------
% [u0,t0] = gensig("square",80e-3,80e-3);
% % u=24*u0-12;
% u=12*u0;
% t=t0-40e-3
% %Entrada
% 
% x1=0
% x2=0
% %Datos
% R=4000
% L=0.0036
% C=1100e-9
% 
% %Matrices
% A= [-R/L -1/L ;1/C 0] 
% B= [1/L ;0]
% C= [R 0]
% D= [0]
% x0= [x1 ;x2] 
% 
% 
% Sys = ss(A,B,C,D)
% 
% TF = tf(Sys)
% [i,vc]=lsim(A,B,C,D,u,t,x0)
% 
% data=xlsread("Curvas_Medidas_RLC.xls");
% t_t=data(:,1)-0.01;
% Vc_t=data(:,3);
% plot(t,vc,'b');grid on; title('Tensión en capacitor');xlim([0 0.04]); hold on
% plot(t_t,Vc_t,'r');ylim([0 13]);grid on;hold on;

%-------------------------------------------------------------------------------
%Punto 4 RLC
%-----------------------------------------------------------------------------------
 [u0,t0] = gensig("square",0.04,0.1);
 u=24*u0-12;
t=t0+50e-3; %Retardo en tiempo para que comienze en 0,05
%Entrada

x1=0
x2=0
%Datos
R=4000
L=0.0036
C=1100e-9

%Matrices
A= [-R/L -1/L ;1/C 0] 
B= [1/L ;0]
C= [R 0]
D= [0]
x0= [x1 ;x2] 


Sys = ss(A,B,C,D)

TF = tf(Sys)
[i,vc]=lsim(A,B,C,D,u,t,x0)


% plot(t,vc,'b');grid on; title('Tensión en capacitor')
subplot(3,1,1);plot(t,i);grid on; title('Corriente'); hold on%plot(t,y);
subplot(3,1,2);plot(t,vc);grid on; title('Tensión en capacitor'); hold on
subplot(3,1,3);plot(t,u);grid on; title('Tensión de entrada'); hold on
