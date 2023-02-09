% Los primeros códigos son intentos fallidos, los subo también para que se
% vean los ditintos intentos para resolver el TP.
%---------------------------------------------------------------------------
% %Intento 1: TP2 1.1 / Primer intento (ESTA MAL) con un PID
% %---------------------------------------------------------------------------
% clear all;close all;
% X=-[0; 0; 0];ii=0;t_etapa=1e-7;titaRef1=-1.57;titaRef2=1.57;tF=1;
% Kp=4.5;Ki=200;Kd=2e-6; %<---- la que va
% Ts=t_etapa;
% A1=((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
% B1=(-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
% C1=Kd/Ts;
% e=zeros(tF/t_etapa,1);
% u=12;Tl=0;
% for t=0:t_etapa:tF
%  ii=ii+1;k=ii+2;
%  Tl_arreglo=[0 ((1.15e-3/2)*sign(sin(10.472*t))+(1.15e-3/2))];
%  Tl=Tl_arreglo(2);
%  U = [u Tl]; 
%  X=modmotor(t_etapa, X, U);
%  if(Tl<=1e-3)
%      e(k)=titaRef1-X(1); %ERROR en punto1
%  else
%      e(k)=titaRef2-X(1); %ERROR en punto 2 
%  end
%  u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
%  x1(ii)=X(1);%tita
%  x2(ii)=X(2);%omega
%  x3(ii)=X(3);%ia
%  torque(ii)=Tl;%Torque Tl
%  acc(ii)=u;
% end
% 
% %Grafico
% t=0:t_etapa:tF;
% figure;
% subplot(3,1,1);hold on;
% plot(t,x1(1:numel(t)),'r');title('Tita');xlabel('Tiempo [Seg.]');
% subplot(3,1,2);hold on;
% plot(t,x2(1:numel(t)),'r');title('Salida y, \omega_t');xlabel('Tiempo [Seg.]');
% subplot(3,1,3);hold on;
% plot(t,x3(1:numel(t)),'r');title('Ia');xlabel('Tiempo [Seg.]');
% 
% figure;
% subplot(2,1,1);hold on;
% plot(t,torque(1:numel(t)),'r');title('Torque de carga');
% xlabel('Tiempo [Seg.]');
% subplot(2,1,2);hold on;
% plot(t,acc(1:numel(t)),'r');title('Accion de control');
% xlabel('Tiempo [Seg.]');
% 
% %Modelo del motor
% function [X]=modmotor(t_etapa, xant, accion)
% Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;
% Va=accion(1);
% h=t_etapa;
% Tl=accion(2);
% tita = xant(1);
% w= xant(2);
% ia= xant(3);
% 
%     %Modelo con primera derivada de la velocidad angular
%  wp=(Ki/J)*ia-(B/J)*w-(1/J)*Tl;
%  w=w+h*wp;
%  tita=tita+h*w;
%  iap=(-Ra/Laa)*ia-(Km/Laa)*w+(1/Laa)*Va;
%  ia = ia + h*iap;   
%    
% 
% X=[tita,w,ia];
% end

% %---------------------------------------------------------------------------
% %Intento 2: Tp2_ 1.2 / Modelo del motor con matrices pero sin control.
% %---------------------------------------------------------------------------
% clear all;close all;
% X=-[0; 0; 0];ii=0;At=1e-5;titaRef1=-1.57;titaRef2=1.57;tF=0.9;
% Kp=4.5;Ki=200;Kd=2e-6; 
% Ts=At;
% A1=((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
% B1=(-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
% C1=Kd/Ts;
% e=zeros(tF/At,1);
% u=12;Tl=0;
% 
% %Caracteristicas del motor
% Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;
% 
% %Matrices
% A=[-Ra/Laa 0 -Km/Laa; 0 0 1;Ki/J 0 -B/J];
% B=[1/Laa;0;0];
% C=[0 1 0];
% D=[0];
% P=[0;0;-1/J]; %Matriz perturbacion
% sys=ss(A,B,C,D);
% tf=tf(sys)
% %Verifico Controlabilidad y Observabilidad
% M=[B A*B A^2*B];%Matriz Controlabilidad 
% rango_M=rank(M); 
% O=[C; C*A; C*A^2];%Matriz Observabilidad
% auto_O=eig(O);
% rango_O=rank(O);
% 
% %Cálculo del controlador por asignación de polos
% auto_val=eig(A); %<--- De |sI-A| se puede obtener ecu. caracteristic: s^3+151,9*s^2+17,48*1298K*s 
% c_ai=conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]);%<----Consultar 
% W=[c_ai(3) c_ai(2) 1;c_ai(2) 1 0;1 0 0 ]; %W=[22.69e6 151.9e3 1;151.9e3 1 0;1 0 0];
% T=M*W;             
% A_controlable=inv(T)*A*T %Verificación de que T esté bien
% 
% 
% for t=0:At:tF
%  ii=ii+1;k=ii+2;
%  estado=X;
%  Tl_arreglo=[0 ((1.15e-3/2)*sign(sin(10.472*t))+(1.15e-3/2))];
%  Tl=Tl_arreglo(2);
%  U = u; 
%  xp=A*X+B*U+P*Tl;
%  Y =C*X+D*U;
%  X=X+xp*At;
%   
%  if(Tl<=1e-3)
%      e(k)=titaRef1-X(1); %ERROR en punto1
%  else
%      e(k)=titaRef2-X(1); %ERROR en punto 2 
%  end
%  
% 
%  x1(ii)=X(1);%tita
%  x2(ii)=X(2);%omega
%  x3(ii)=X(3);%ia
%  torque(ii)=Tl;%Torque Tl
%  acc(ii)=U;
% end
% 
% %Grafico
% t=0:At:tF;
% figure;
% subplot(3,1,1);hold on;
% plot(t,x2(1:numel(t)),'r');title('Tita');xlabel('Tiempo [Seg.]');
% subplot(3,1,2);hold on;
% plot(t,x3(1:numel(t)),'r');title('Salida y, \omega_t');xlabel('Tiempo [Seg.]');
% subplot(3,1,3);hold on;
% plot(t,x1(1:numel(t)),'r');title('Ia');xlabel('Tiempo [Seg.]');
% 
% figure;
% subplot(2,1,1);hold on;
% plot(t,torque(1:numel(t)),'r');title('Torque de carga');
% xlabel('Tiempo [Seg.]');
% subplot(2,1,2);hold on;
% plot(t,acc(1:numel(t)),'r');title('Accion de control');
% xlabel('Tiempo [Seg.]');

% %---------------------------------------------------------------------------
% %Intento 3: Punto 1.1 / No funciona el controlador.
% %---------------------------------------------------------------------------
% clear all;close all;
% X=-[0; 0; 0];ii=0;t_etapa=1e-7;ref1=-1.57;ref2=1.57;tF=1;
% 
% Ts=t_etapa;
% % %Caracteristicas del motor
% Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;
% % 
% %Matrices
% A=[-Ra/Laa 0 -Km/Laa; 0 0 1;Ki/J 0 -B/J];
% B=[1/Laa;0;0];
% C=[1 0 0];
% D=[0];
% P=[0;0;-1/J]; %Matriz perturbacion
% sys=ss(A,B,C,D);
% tf=tf(sys);
% %Verifico Controlabilidad y Observabilidad
% M=[B A*B A^2*B];%Matriz Controlabilidad 
% rango_M=rank(M); 
% O=[C; C*A; C*A^2];%Matriz Observabilidad
% auto_O=eig(O);
% rango_O=rank(O);
% 
% %Cálculo del controlador por asignación de polos
% auto_val=eig(A); %<--- De |sI-A| se puede obtener ecu. caracteristic: s^3+151,9*s^2+17,48*1298K*s 
% c_ai=conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]);%<----Consultar 
% W=[c_ai(3) c_ai(2) 1;c_ai(2) 1 0;1 0 0 ]; %W=[22.69e6 151.9e3 1;151.9e3 1 0;1 0 0];
% T=M*W;             
% A_controlable=inv(T)*A*T %Verificación de que T esté bien
% 
% %CONTROLADOR Ubicación de los polos de lazo cerrado en mui :
% mui(1)=-1;mui(2)=-10; mui(3)=-1;
% alfa_i=conv(conv([1 -mui(3)],[1 -mui(2)]),[1 -mui(1)]);
% K=(alfa_i(2:4)-c_ai(2:4))*inv(T);
% Gj=-inv(C*inv(A-B*K)*B);
% 
% eig(A-B*K)
% 
% u=12;Tl=0;
% for t=0:t_etapa:tF
%  ii=ii+1;k=ii+2;
%  Tl_arreglo=[0 ((1.15e-3/2)*sign(sin(10.472*t))+(1.15e-3/2))];
%  Tl=Tl_arreglo(2);
%  U = [u Tl]; 
%  X=modmotor(t_etapa, X, U);
%  
%      
% % %   Cambio la referencia segun el angulo deseado
% %   if(Tl<=1e-3) 
% %      u=-K*X'+Gj*ref1; color='b'; %Sin Observador
% %   else  
% %      u=-K*X'+Gj*ref2; color='b'; %Sin Observador
% %   end    
% 
%  
%  x1(ii)=X(1);%tita
%  x2(ii)=X(2);%omega
%  x3(ii)=X(3);%ia
%  torque(ii)=Tl;%Torque Tl
%  acc(ii)=u;
% end
% 
% %Grafico
% t=0:t_etapa:tF;
% figure;
% subplot(3,1,1);hold on;
% plot(t,x1(1:numel(t)),'r');title('Tita');xlabel('Tiempo [Seg.]');
% subplot(3,1,2);hold on;
% plot(t,x2(1:numel(t)),'r');title('Salida y, \omega_t');xlabel('Tiempo [Seg.]');
% subplot(3,1,3);hold on;
% plot(t,x3(1:numel(t)),'r');title('Ia');xlabel('Tiempo [Seg.]');
% 
% figure;
% subplot(2,1,1);hold on;
% plot(t,torque(1:numel(t)),'r');title('Torque de carga');
% xlabel('Tiempo [Seg.]');
% subplot(2,1,2);hold on;
% plot(t,acc(1:numel(t)),'r');title('Accion de control');
% xlabel('Tiempo [Seg.]');
% 
% %Modelo del motor
% function [X]=modmotor(t_etapa, xant, accion)
% Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;
% Va=accion(1);
% h=t_etapa;
% Tl=accion(2);
% tita = xant(1);
% w= xant(2);
% ia= xant(3);
% 
%     %Modelo con primera derivada de la velocidad angular
%  wp=(Ki/J)*ia-(B/J)*w-(1/J)*Tl;
%  w=w+h*wp;
%  tita=tita+h*w;
%  iap=(-Ra/Laa)*ia-(Km/Laa)*w+(1/Laa)*Va;
%  ia = ia + h*iap;   
%    
% 
% X=[tita,w,ia];
% end

%-----------------------------------------------------------------------------
%-----------------------------------------------------------------------------
% A partir de este punto estan los codigos correctos para los puntos 1.1 y
% 1.2
% %---------------------------------------------------------------------------
% %Tp2_Punto 1.1
%Sistema con control y la referencia variable,funciona
% aunque el angulo positivo no da exactamente 90° pero el negativo si da exacto.
% %---------------------------------------------------------------------------
clear all;close all;
X=[0; 0; 0];ii=0;At=1e-5;ref1=-1.57;ref2=1.57;tF=0.9;
Ts=At;
e=zeros(tF/At,1);
u=12;Tl=0;

%Condiciones iniciales
X(1)=0;X(2)=0;X(3)=0;

%Caracteristicas del motor
Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;

%Matrices
A=[-Ra/Laa 0 -Km/Laa; 0 0 1;Ki/J 0 -B/J];
B=[1/Laa;0;0];
C=[0 1 0];
D=[0];
P=[0;0;-1/J]; %Matriz perturbacion
sys=ss(A,B,C,D);
tf=tf(sys)
%Verifico Controlabilidad y Observabilidad
M=[B A*B A^2*B];%Matriz Controlabilidad 
rango_M=rank(M); 

%Cálculo del controlador por asignación de polos
auto_val=eig(A); %<--- De |sI-A| se puede obtener ecu. caracteristic: s^3+151,9*s^2+17,48*1298K*s 
c_ai=conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]);%<----Consultar 
W=[c_ai(3) c_ai(2) 1;c_ai(2) 1 0;1 0 0 ]; %W=[22.69e6 151.9e3 1;151.9e3 1 0;1 0 0];
T=M*W;             
A_controlable=inv(T)*A*T %Verificación de que T esté bien

%CONTROLADOR Ubicación de los polos de lazo cerrado en mui :
mui(1)=-5;mui(2)=-5; mui(3)=-0.01;
alfa_i=conv(conv([1 -mui(3)],[1 -mui(2)]),[1 -mui(1)]);
K=alfa_i(2:4)-c_ai(2:4)*inv(T);
Gj=-inv(C*inv(A-B*K)*B);
eig(A-B*K)


for t=0:At:tF
 ii=ii+1;k=ii+2;
 estado=X;
 Tl_arreglo=[0 ((1.15e-3/2)*sign(sin(10.472*t))+(1.15e-3/2))];
 Tl=Tl_arreglo(2);
 U = u; 
 
 xp=A*X+B*U+P*Tl;
 X=X+xp*At;
 
 
 %Cambio la referencia segun el angulo deseado
  if(Tl<=1e-3)
     u=-K*estado+Gj*ref1;  %Sin Observador
  else     
     u=-K*estado+Gj*ref2; %Sin Observador
  end
 color='b';
    %Grafico sin observador
 x1(ii)=X(1);%tita
 x2(ii)=X(2);%omega
 x3(ii)=X(3);%ia
 
 torque(ii)=Tl;%Torque Tl
 acc(ii)=U;
end

%Grafico
t=0:At:tF;
figure;
subplot(3,1,1);hold on;
plot(t,x2(1:numel(t)),color);title('Tita');xlabel('Tiempo [Seg.]');
subplot(3,1,2);hold on;
plot(t,x3(1:numel(t)),color);title('Salida y, \omega_t');xlabel('Tiempo [Seg.]');
subplot(3,1,3);hold on;
plot(t,x1(1:numel(t)),color);title('Ia');xlabel('Tiempo [Seg.]');


% %---------------------------------------------------------------------------
% %Tp2_Punto 1.2 / Sistema con observador y sin observador.
% %---------------------------------------------------------------------------
% clear all;%close all;
% X=[0; 0; 0];ii=0;At=1e-5;ref1=-1.57;ref2=1.57;tF=0.9;
% Ts=At;
% e=zeros(tF/At,1);
% u=12;Tl=0;
% 
% %Condiciones iniciales
% X(1)=0;X(2)=0;X(3)=0;
% 
% %Caracteristicas del motor
% Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;
% 
% %Matrices
% A=[-Ra/Laa 0 -Km/Laa; 0 0 1;Ki/J 0 -B/J];
% B=[1/Laa;0;0];
% C=[0 1 0];
% D=[0];
% P=[0;0;-1/J]; %Matriz perturbacion
% sys=ss(A,B,C,D);
% tf=tf(sys)
% %Verifico Controlabilidad y Observabilidad
% M=[B A*B A^2*B];%Matriz Controlabilidad 
% rango_M=rank(M); 
% O=[C; C*A; C*A^2];%Matriz Observabilidad
% auto_O=eig(O);
% rango_O=rank(O);
% 
% %Cálculo del controlador por asignación de polos
% auto_val=eig(A); %<--- De |sI-A| se puede obtener ecu. caracteristic: s^3+151,9*s^2+17,48*1298K*s 
% c_ai=conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]);%<----Consultar 
% W=[c_ai(3) c_ai(2) 1;c_ai(2) 1 0;1 0 0 ]; %W=[22.69e6 151.9e3 1;151.9e3 1 0;1 0 0];
% T=M*W;             
% A_controlable=inv(T)*A*T %Verificación de que T esté bien
% 
% %CONTROLADOR Ubicación de los polos de lazo cerrado en mui :
%  mui(1)=-100000;mui(2)=-16000; mui(3)=-16000;
% %mui(1)=-6;mui(2)=-6; mui(3)=-0.01;
% alfa_i=conv(conv([1 -mui(3)],[1 -mui(2)]),[1 -mui(1)]);
% K=fliplr(alfa_i(2:4)-c_ai(2:4))*inv(T);
% 
% Gj=-inv(C*inv(A-B*K)*B);
% 
% eig(A-B*K)
% A_O=A';
% B_O=C';
% M_Dual=[B_O A_O*B_O A_O^2*B_O];%MatrizControlabilidad
% alfaO_i=alfa_i;% Ubicacion del Observador
% 
% mui_o=1*real(mui);
% alfaO_i=conv(conv([1 -mui_o(3)],[1 -mui_o(2)]),[1 -mui_o(1)]);
% T_O=M_Dual*W;
% Ko=(fliplr(alfaO_i(2:end)-c_ai(2:end))*inv(T_O))';
% eig(A_O'-Ko*C) %Verifico que todos los polos estén en el semiplano izquierdo
% x_hat=[0;0;0]; %Inicializo el Observador
% 
% for t=0:At:tF
%  ii=ii+1;k=ii+2;
%  estado=X;
%  Tl_arreglo=[0 ((1.15e-3/2)*sign(sin(10.472*t))+(1.15e-3/2))];
%  Tl=Tl_arreglo(2);
%  U = u; 
%  
%  xp=A*X+B*U+P*Tl;
%  X=X+xp*At;
%  
%  %________OBSERVADOR__________
%  y_sal_O(ii)=C*x_hat;
%  Y(ii)=C*estado;
%  x_hatp=A*x_hat+B*U+Ko*(Y(ii)-y_sal_O(ii));
%  x_hat=x_hat+At*x_hatp;
%  
%  %Cambio la referencia segun el angulo deseado
%   if(Tl<=1e-3)
%      u=-K*x_hat+Gj*ref1;color= 'r'; %Con Observador referencia 1 
%      %u=-K*estado+Gj*ref1; color='b'; %Sin Observador
%     else
%      u=-K*x_hat+Gj*ref2;color= 'r'; %Con Observador referencia 2
%      %u=-K*estado+Gj*ref2; color='b'; %Sin Observador
%   end
%  %color='r';
%     %Grafico sin observador
% %  x1(ii)=X(1);
% %  x2(ii)=X(2);
% %  x3(ii)=X(3);
% %  
% %     %Grafico con observador
%  x1(ii)=x_hat(1);%tita  
%  x2(ii)=x_hat(2);%omega
%  x3(ii)=x_hat(3);%ia
%  
%  torque(ii)=Tl;%Torque Tl
%  acc(ii)=U;
% end
% 
% %Grafico
% t=0:At:tF;
% %figure;
% subplot(4,1,1);hold on;
% plot(t,x2(1:numel(t)),color);title('Tita');xlabel('Tiempo [Seg.]');
% subplot(4,1,2);hold on;
% plot(t,x3(1:numel(t)),color);title('Salida y, \omega_t');xlabel('Tiempo [Seg.]');
% subplot(4,1,3);hold on;
% plot(t,x1(1:numel(t)),color);title('Ia');xlabel('Tiempo [Seg.]');
% 
% 
% subplot(4,2,7);hold on;
% plot(t,torque(1:numel(t)),'r');title('Torque de carga');
% xlabel('Tiempo [Seg.]');
% subplot(4,2,8);hold on;
% plot(t,acc(1:numel(t)),color);title('Accion de control');
% xlabel('Tiempo [Seg.]');
