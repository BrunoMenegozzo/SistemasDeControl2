% % %-----------------------------------------------------------------------------
% Motor DC DLQR con integrador del error. Funciona
% %---------------------------------------------------------------------------
clear all;%close all;
TamanioFuente=12;
ii=0;At=1e-5;ref=-1.57;tF=.15;
Ts=At;
u=12;Tl=0;

%Condiciones iniciales
ia(1)=0;tita(1)=0;w(1)=0;
X=[ia(1);tita(1);w(1)];

%Alinealidad
 Alinealidad=5;color='r';
% Alinealidad= 16;color='m';
%Alinealidad= 15;color='g';
%Alinealidad= 30;color='m';
%Alinealidad= 30;color='k';

%Caracteristicas del motor
Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;

%Matrices Tiempo Continuo
Ac=[-Ra/Laa 0 -Km/Laa; 0 0 1;Ki/J 0 -B/J];
Bc=[1/Laa;0;0];
C=[0 1 0];
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

% %Matriz A y B discretas (Otra forma de calcularlas)
% H=[0;0;0];d_tao=Ts/1;tao=0;
% for hh=1:100
%     dH=expm(Ac*tao)*Bc*d_tao;
%     H=H+dH;
%     tao=tao+d_tao;
% end
% B=H;
% A=expm(Ac*Ts);

M=[B A*B A^2*B]; %Matriz de controlabilidad discreta
rango=rank(M);

%Matrices ampliadas
Aa=[A,zeros(3,1);-C*A, eye(1)];
Ba=[B;-C*B];

Ma=[Ba Aa*Ba Aa^2*Ba Aa^3*Ba];%Matriz Controlabilidad Ampliada
rango_a=rank(Ma);

% %Funcional de costos
Q=diag([10 20 80 10]); %"Velocidad de convergencia"
R=1e1; % Magnitud de accion de control% Para TL 1.15e-8

%Construccion del Hamiltoniano de la ampliada
H=[Aa+Ba*inv(R)*Ba'*inv(Aa')*Q -Ba*inv(R)*Ba'*inv(Aa') ; -inv(Aa')*Q inv(Aa')];

[V,D]=eig(H);MX1X2=[];
for ii=1:8
 if abs(D(ii,ii))<1 %Autovalores estables dentro de circ unitario
 MX1X2=[MX1X2 V(:,ii)]; %Matriz de autovectores "estables"
 end
end
MX1=MX1X2(1:4,:); MX2=MX1X2(5:8,:); %MX1X2=[M ; PM] con M=MX1 Y PM=MX2
P=real(MX2*inv(MX1)); %P=PM * inv(M)/ Solucion de Riccati
Ka=inv(R+Ba'*P*Ba)*Ba'*P*Aa; %K optimo
K=Ka(1:3);KI=-Ka(4); 
aut_controlador=abs(eig(Aa-Ba*Ka));

% OBSERVADOR
%Matrices Observador
M_Obs=[C;(C*A);(C*A^2);(C*A^3)];
rank(M_Obs)
C_O=B';
A_O=A';
B_O=C';
M_D=M_Obs';

%Funcional de costos Observador
Qo=diag([.1 .1 .1]);
Ro=diag(1e2);

%Hamiltoniano del observador
H=[A_O+B_O*inv(Ro)*B_O'*inv(A_O')*Qo -B_O*inv(Ro)*B_O'*inv(A_O'); -inv(A_O')*Qo inv(A_O')];

[V,D]=eig(H);MX1X2=[];
for ii=1:6
    if abs(D(ii,ii))<1
        MX1X2=[MX1X2 V(:,ii)];
    end
end
MX1=MX1X2(1:3,:); MX2=MX1X2(4:6,:); 
Po=real(MX2*inv(MX1));              %Solucion ecu de Riccati
Ko=(inv(Ro+B_O'*Po*B_O)*B_O'*Po*A_O)';%Control optimo Observador
abs(eig(A-Ko*C)) %Verifico que sean estables


%Simulacion Lineal
V_L=[X;0]'*P*[X;0];
Jmin=[X;0]'*P*[X;0];
Mat_J=0;

xang=[0;0;0];
V_L=[X;0]'*P*[X;0];
Jl=0;Jmin_L=V_L;
ve1(1)=0;vei=0;
for t=0:At:tF
 ii=ii+1;k=ii+2;
 
 %Torque de carga
 Tl_arreglo=[0 ((1.15e-8/2)*sign(sin(10.472*6*t))+(1.15e-8/2))];
 Tl=Tl_arreglo(2);
 
 Y_=C*X;
 e1=ref-Y_(1);
 
 vei=vei+e1;
 ve1(ii)=vei;

 u=-Ka*[X;ve1(ii)];color='r';%Estado ampliado
 %u=-Ka*[xang;ve1(ii)];%color='b';
 
  %Liapunov
 V_Li=[X;ve1(ii)]'*P*[X;ve1(ii)]; %Candidata de Liapunov
 V_L=[V_L V_Li];
 Jl=Jl+[X;ve1(ii)]'*Q*[X;ve1(ii)]+u'*R*u'; %Funcional de Costos (Fletcher Powell)
 Mat_J=[Mat_J Jl];

 %Cambio la referencia segun el angulo deseado
  if(Tl<=1e-12)
        ref=-1.57;
    else

        ref=1.57;
  end 
%   %Alinealidad
 if -Alinealidad<u && u<Alinealidad
     u=0;
 else
     u=u;
 end
 
 X=A*X+B*u+Per*Tl;
 
    %Grafico
     x1(ii)=X(1); %Ia
     x2(ii)=X(2); %tita
     x3(ii)=X(3); %w
 
 torque(ii)=Tl;%Torque Tl
 acc(ii)=u;
 
 xang=A*xang+B*u+Ko*(Y_-C*xang);
end

%Grafico
t=0:At:tF;
figure(1);
subplot(4,1,1);hold on;
plot(t,x1(1:numel(t)),color);title('Ia');xlabel('Tiempo [Seg.]');hold on;
subplot(4,1,2);hold on;
plot(t,x2(1:numel(t)),color);title('tita');xlabel('Tiempo [Seg.]');
subplot(4,1,3);hold on;
plot(t,x3(1:numel(t)),color);title('omega');xlabel('Tiempo [Seg.]');

subplot(4,2,7);hold on;
plot(t,torque(1:numel(t)),'r');title('Torque de carga');
xlabel('Tiempo [Seg.]');
subplot(4,2,8);hold on;
plot(t,acc(1:numel(t)),color);title('Accion de control');
xlabel('Tiempo [Seg.]');
% % 
figure(2);
% subplot(2,1,1);
plot(x2,x3,color);title('Diagrama de Fases: Alinealidad = +/-50 V');xlabel('tita');ylabel('omega');grid on;hold on;
% subplot(2,1,2);semilogy(t,Mat_J(1:numel(t)),color);grid on;title('Modelo lineal','FontSize',TamanioFuente);
% xlabel('Tiempo en Seg.','FontSize',TamanioFuente);ylabel('Funcionales J y V','FontSize',TamanioFuente);hold on;
% %semilogy(t,Jmin*ones(size(Mat_J)),color);
% semilogy(t,V_L(1:numel(t)),'g');
