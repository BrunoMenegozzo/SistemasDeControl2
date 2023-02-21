%-------------------------------------------------------------------
%Motor DC DLQR sin integrador del error. No funciona correctamente porque
%no llega a la referencia.
%-------------------------------------------------------------------

% clear all;close all;
% TamanioFuente=12;
% ii=0;At=1e-5;ref1=-1.57;ref2=1.57;tF=1; 
% Ts=At;
% u=12;Tl=0;
% 
% %Condiciones iniciales
% ia(1)=0;w(1)=0;tita(1)=0;
% X=[ia(1);w(1);tita(1)];
% 
% 
% %Caracteristicas del motor
% Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;
% 
% %Matrices
% Ac=[-Ra/Laa 0 -Km/Laa; 0 0 1;Ki/J 0 -B/J];
% Bc=[1/Laa;0;0];
% C=[1 0 0];
% Per=[0;0;-1/J]; %Matriz perturbacion
% 
% %Verifico Controlabilidad y Observabilidad
% Mc=[Bc Ac*Bc Ac^2*Bc];%Matriz Controlabilidad 
% rango_M=rank(Mc); 
% Oc=[C; C*Ac; C*Ac^2];%Matriz Observabilidad
% auto_O=eig(Oc);
% rango_O=rank(Oc);
% 
% %Matrices Discretas
% sys_c=ss(Ac,Bc,C,[0]);
% sys_d=c2d(sys_c,Ts,'zoh');
% 
% A=sys_d.a;
% B=sys_d.b;
% 
% M=[B A*B A^2*B]; %Matriz de controlabilidad
% rango=rank(M);
% 
% % %Funcional de costos
%  Q=diag([1e-3 1500000 2000]);R=100; %At=1e-5
% %Q=diag([1 10 50]);R=1; %At=1e-4
% H=[A+B*inv(R)*B'*inv(A')*Q -B*inv(R)*B'*inv(A') ; -inv(A')*Q inv(A')];
% 
% [V,D]=eig(H);MX1X2=[];
% for ii=1:6
%  if abs(D(ii,ii))<1
%  MX1X2=[MX1X2 V(:,ii)];
%  end
% end
% MX1=MX1X2(1:3,:); MX2=MX1X2(4:6,:);
% P=real(MX2*inv(MX1));
% K=inv(R+B'*P*B)*B'*P*A;
% 
% %Matriz referencia
% Gj=-inv(C*inv(A-B*K)*B);
% %Gj=inv(C(1,:)*inv(eye(3)-A+B*K)*B);
% 
% %Simulacion Lineal
% Jmin=X'*P*X;
% Mat_J=0;
% V_L=X'*P*X;Jl=0;
% for t=0:At:tF
%  ii=ii+1;k=ii+2;
%  estado=X;
%  %Torque
%  Tl_arreglo=[0 ((1.15e-8/2)*sign(sin(10.472*t))+(1.15e-8/2))];
%  Tl=Tl_arreglo(2);
%  
%  Jl=Jl+X'*Q*X+u'*R*u';
%  Mat_J=[Mat_J Jl];
%  V_L=[V_L X'*P*X];
%  
%  X=A*X+B*u+Per*Tl;
% 
%  %Cambio la referencia segun el angulo deseado
%   if(Tl<=1e-12) 
%      u=-K*estado+Gj*ref1; color='b';
%   else
%     u=-K*estado+Gj*ref2; color='b'; 
%   end
%     
%   %Alinealidad
%  if -5<u && u<5
%      u=0;
%  else
%      u=u;
%  end  
% 
%  %Grafico sin observador
%  x1(ii)=X(1); %Ia
%  x2(ii)=X(2); %w
%  x3(ii)=X(3); %tita
% 
%  torque(ii)=Tl;%Torque Tl
%  acc(ii)=u;
% end
% 
% %Grafico
% t=0:At:tF;
% figure(1);
% subplot(4,1,1);hold on;
% plot(t,x1(1:numel(t)),color);title('Ia');xlabel('Tiempo [Seg.]');hold on;
% subplot(4,1,2);hold on;
% plot(t,x2(1:numel(t)),color);title('\omega_t');xlabel('Tiempo [Seg.]');
% subplot(4,1,3);hold on;
% plot(t,x3(1:numel(t)),color);title('tita');xlabel('Tiempo [Seg.]');
% 
% subplot(4,2,7);hold on;
% plot(t,torque(1:numel(t)),'r');title('Torque de carga');
% xlabel('Tiempo [Seg.]');
% subplot(4,2,8);hold on;
% plot(t,acc(1:numel(t)),color);title('Accion de control');
% xlabel('Tiempo [Seg.]');
% 
% figure(2);
% semilogy(t,Mat_J(1:numel(t)),color);grid on;title('Modelo no lineal','FontSize',TamanioFuente);
% xlabel('Tiempo en Seg.','FontSize',TamanioFuente);ylabel('Funcionales J y V','FontSize',TamanioFuente);hold on;
% %semilogy(t,Jmin*ones(size(Mat_J)),color);
% semilogy(t,V_L(1:numel(t)),'g');

% % %-----------------------------------------------------------------------------
% Motor DC DLQR con integrador del error.
% %---------------------------------------------------------------------------
clear all;close all;
TamanioFuente=12;
ii=0;At=1e-5;ref=-1.57;tF=1;
Ts=At;
u=12;Tl=0;

%Condiciones iniciales
ia(1)=0;w(1)=0;tita(1)=0;
X=[ia(1);w(1);tita(1)];


%Caracteristicas del motor
Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;

%Matrices Tiempo Continuo
Ac=[-Ra/Laa 0 -Km/Laa; 0 0 1;Ki/J 0 -B/J];
Bc=[1/Laa;0;0];
C=[0 0 1];
Per=[0;0;-1/J]; %Matriz perturbacion

%Verifico Controlabilidad y Observabilidad
Mc=[Bc Ac*Bc Ac^2*Bc];%Matriz Controlabilidad 
rango_M=rank(Mc); 
Oc=[C; C*Ac; C*Ac^2];%Matriz Observabilidad
auto_O=eig(Oc);
rango_O=rank(Oc);

%Discretizacion de matrices
sys_c=ss(Ac,Bc,C,[0]);
sys_d=c2d(sys_c,Ts,'zoh');

A=sys_d.a;
B=sys_d.b;

M=[B A*B A^2*B]; %Matriz de controlabilidad
rango=rank(M);

Aa=[A,zeros(3,1);-C*A, eye(1)];
Ba=[B;-C*B];

Ma=[Ba Aa*Ba Aa^2*Ba Aa^3*Ba];%MatrizControlabilidad
rango_a=rank(Ma);

% %Funcional de costos
%Q=diag([10000 20000 8000]);R=1e9;
Q=diag([10 200 10 10]);R=1e2;
H=[Aa+Ba*inv(R)*Ba'*inv(Aa')*Q -Ba*inv(R)*Ba'*inv(Aa') ; -inv(Aa')*Q inv(Aa')];

[V,D]=eig(H);MX1X2=[];
for ii=1:8
 if abs(D(ii,ii))<1
 MX1X2=[MX1X2 V(:,ii)];
 end
end
MX1=MX1X2(1:4,:); MX2=MX1X2(5:8,:);
P=real(MX2*inv(MX1));
Ka=inv(R+Ba'*P*Ba)*Ba'*P*Aa;
K=Ka(1:3);KI=-Ka(4);
aut_controlador=abs(eig(Aa-Ba*Ka));
%Matriz referencia
Gj=-inv(C*inv(A-B*K)*B);

%Simulacion Lineal
% Jmin=x'*P*x;J=0;V_L=x'*P*x;
%Jmin=X'*P*X;
% Mat_J=zeros(tF/At,1);
Mat_J=0;
V_L=[X;0]'*P*[X;0];
Jl=0;Jmin_L=V_L;
ve1(1)=0;vei=0;
for t=0:At:tF
 ii=ii+1;k=ii+2;
 
 Tl_arreglo=[0 ((1.15e-8/2)*sign(sin(10.472*t))+(1.15e-8/2))];
 Tl=Tl_arreglo(2);
 
 Y_=C*X;
 e1=ref-Y_(1);
%  V_Li=[X;ve1(ii)]'*P*[X;ve1(ii)];
%  V_L=[V_L V_Li];
 vei=vei+e1;
 ve1(ii)=vei;
  u=-Ka*[X;ve1(ii)];%Estado ampliado
 
%  Jl=Jl+[X;ve1(ii)]'*Q*[X;ve1(ii)]+u'*R*u';
%  Mat_J=[Mat_J Jl];
%  V_L=[V_L X'*P*X];
 
 X=A*X+B*u+Per*Tl;

 %Cambio la referencia segun el angulo deseado
  if(Tl<=1e-12)
        ref=-1.57;
    else

        ref=1.57;
  end 
  %Alinealidad
 if -5<u && u<5
     u=0;
 else
     u=u;
 end  
 color='r';
    %Grafico sin observador
     x1(ii)=X(1); %Ia
     x2(ii)=X(2); %w
     x3(ii)=X(3); %tita
 
 torque(ii)=Tl;%Torque Tl
 acc(ii)=u;
end

%Grafico
t=0:At:tF;
figure(1);
subplot(4,1,1);hold on;
plot(t,x1(1:numel(t)),color);title('Ia');xlabel('Tiempo [Seg.]');hold on;
subplot(4,1,2);hold on;
plot(t,x2(1:numel(t)),color);title('\omega_t');xlabel('Tiempo [Seg.]');
subplot(4,1,3);hold on;
plot(t,x3(1:numel(t)),color);title('tita');xlabel('Tiempo [Seg.]');

subplot(4,2,7);hold on;
plot(t,torque(1:numel(t)),'r');title('Torque de carga');
xlabel('Tiempo [Seg.]');
subplot(4,2,8);hold on;
plot(t,acc(1:numel(t)),color);title('Accion de control');
xlabel('Tiempo [Seg.]');

% figure(2);
% semilogy(t,Mat_J(1:numel(t)),color);grid on;title('Modelo no lineal','FontSize',TamanioFuente);
% xlabel('Tiempo en Seg.','FontSize',TamanioFuente);ylabel('Funcionales J y V','FontSize',TamanioFuente);hold on;
% %semilogy(t,Jmin*ones(size(Mat_J)),color);
% semilogy(t,V_L(1:numel(t)),'g');

