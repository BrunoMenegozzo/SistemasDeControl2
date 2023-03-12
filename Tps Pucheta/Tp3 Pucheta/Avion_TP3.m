%TP3_AVION:
%-------------------------------------------------------------------------
% % %-----------------------------------------------------------------------------
% Avion Con observador e integrador DLQR
% %---------------------------------------------------------------------------
clear all;%close all;
TamanioFuente=12;
clear all;
Ts=0.01;KMAX=70/Ts;TEuler=1;

%Entrada
alfa(1)=0;   %alfa
phi(1)=0;   %tita
phi_p(1)=0;   %titap
h(1)=-500;   %h

ref=100;

%Datos
a=0.07;
b=5;
c=150;
w=9;

%Alinealidad
 Alinealidad=0;color='r';
%  Alinealidad=3;color='b';  
%  Alinealidad=10;color='g';
%Matrices continuas
Ac= [-a a 0 0 ;0 0 1 0; w^2 -w^2 0 0; c 0 0 0] ;
Bc= [0; 0; w^2*b; 0];
C= [0 0 0 1; 0 1 0 0];

x= [alfa(1);phi(1);phi_p(1);h(1)];
Mat_Mc=[Bc Ac*Bc Ac^2*Bc Ac^3*Bc ];%Matriz Controlabilidad t_continuo

% Oc=[C; C*Ac; C*Ac^2; C*Ac^3];%Matriz Observabilidad t_continuo
% auto_O=eig(Oc);
% rango_O=rank(Oc);

%Discretizacion de matrices
sys_c=ss(Ac,Bc,C,[0]);
sys_d=c2d(sys_c,Ts,'zoh');

A=sys_d.a;
B=sys_d.b;

% H=[0;0;0;0];d_tao=Ts/TEuler;tao=0;
% for hh=1:100
%     dH=expm(Ac*tao)*Bc*d_tao;
%     H=H+dH;
%     tao=tao+d_tao;
% end
% 
% B=H;
% A=expm(Ac*Ts);


M=[B A*B A^2*B A^3*B ]; %Matriz de controlabilidad discreta
rango=rank(M);

Aa=[A,zeros(4,2);-C*A, eye(2)];
Ba=[B;-C*B];

Ma=[Ba Aa*Ba Aa^2*Ba Aa^3*Ba Aa^4*Ba Aa^5*Ba];%MatrizControlabilidad
rango_a=rank(Ma);

%Funcional de costos
Qc=diag([.899 .889 .978 .999 .9899 .8899]); R=4e8; 

%Contrucción del Hamiltoniano para el cálculo del controlador
H=inv([eye(6) Ba*inv(R)*Ba'; zeros(6) Aa'])*[Aa zeros(6);-Qc eye(6)];
[V,D]=eig(H);MX1X2=[];
for ii=1:12
    if abs(D(ii,ii))<1 %Autovalores estables dentro de circ unitario
        MX1X2=[MX1X2 V(:,ii)]; %Matriz de autovectores "estables"
    end
end
MX1=MX1X2(1:6,:); MX2=MX1X2(7:12,:); %MX1X2=[M ; PM] con M=MX1 Y PM=MX2
Pc=real(MX2*inv(MX1));  %P=PM * inv(M)/ Solucion de Riccati
Ka=inv(R+Ba'*Pc*Ba)*Ba'*Pc*Aa; %K optimo
K=Ka(1:4);KI=-Ka(5);
aut_controlador=abs(eig(Aa-Ba*Ka));
K_=Ka(5:6);

% % % %Cálculo del controlador por asignación de polos
% auto_val=eig(A);
% c_ai=poly(auto_val);
% Mat_W=[c_ai(4) c_ai(3) c_ai(2) 1;c_ai(3) c_ai(2) 1 0; c_ai(2) 1 0 0; 1 0 0 0];
% Mat_T=Mat_Mc*Mat_W;
% A_controlable=inv(Mat_T)*A*Mat_T %Verificación de T
% %Ubicación de los polos de lazo cerrado en mui:
% mui(1)=-15+15i; mui(2)=conj(mui(1)); mui(3)= -.5+.5i; mui(4)=conj(mui(2));
% alfa_i=poly(mui);
% Ka=fliplr(alfa_i(2:5)-c_ai(2:5))*inv(Mat_T);
% Ka=[Ka K_];

%------------------------------------------------------------------
% Observador
M_Obs=[C;(C*A);(C*A^2);(C*A^3)];
rank(M_Obs)
C_O=B';
A_O=A';
B_O=C';
M_D=M_Obs';

%Funcional de costos Observador
 Qo=diag([.1 .1 .9 .9]);Ro=diag([1e10 1e4]); %Valores sin alinealidad
 %Qo=diag([100 100 100 100]);Ro=diag([1e15 1e15]); %Valores con alinealidad

H=[A_O+B_O*inv(Ro)*B_O'*inv(A_O')*Qo -B_O*inv(Ro)*B_O'*inv(A_O'); -inv(A_O')*Qo inv(A_O')];
[V,D]=eig(H);MX1X2=[];
for ii=1:8
    if abs(D(ii,ii))<1
        MX1X2=[MX1X2 V(:,ii)];
    end
end
MX1=MX1X2(1:4,:); MX2=MX1X2(5:8,:);
P=real(MX2*inv(MX1));
Ko=(inv(Ro+B_O'*P*B_O)*B_O'*P*A_O)';
abs(eig(A-Ko*C))
%----------------------------------------------------------------
%Simulacion lineal
t=0; x=[0;0;0;h(1)];
alfa(1)=x(1); phi(1)=x(2); phi_p(1)=x(3); h(1)=x(4);
ve(1)=0;
u_k(1)=0;
xang=[0 ;0;0;0];
u0(1)=0;
% Jmin=x'*P*x;J=0;V_L=x'*P*x;
Jmin=x'*P*x;
J=0;
V_L=x'*P*x;
ve1(1)=0;ve1_i=0;ve2_i=0;ve2(1)=0;
ref2=0;
for ki=1:KMAX
    t=[t ki*Ts];

    Y_=C*x;
    e1=ref-Y_(1);
    e2=ref2-Y_(2);

    ve1_i=ve1_i+e1;
    ve1(ki)=ve1_i;

    ve2_i=ve2_i+e2;
    ve2(ki)=ve2_i;
    
  
   %u=-Ka*[x;ve1(ki);0];color='r'; %Sin observador
    u=-Ka*[xang;ve1(ki);0];color='b';%Con observador
    %--------------------------------------------
    %Alinealidad
    if -Alinealidad<u && u<Alinealidad
        u=0;
    else
        u=u;
    end

    ys=C*x; %Medicion de y
    J=[J J(ki)+x'*Qo*x+u'*R*u]; %Funcional de costos

    x=A*x+B*u;

    %  
    V_L=[V_L x'*P*x]; %Liapunov
    alfa(ki+1)=x(1);
    phi(ki+1)=x(2);
    phi_p(ki+1)=x(3);
    h(ki+1)=x(4);
    u_k(ki+1)=u;
    xang=A*xang+B*u+Ko*(ys-C*xang);%Acá se usa y.
end
%------------------------------------------------------------------------
%Grafico

TamanioFuente=12;
u=u_k;figure(1);hold on;
subplot(3,2,1);plot(t,alfa,color);grid on;title('\alpha_t','FontSize',TamanioFuente);hold on;
subplot(3,2,2);plot(t,phi,color);grid on;
title('\phi_t','FontSize',TamanioFuente);hold on;
subplot(3,2,3); plot(t,phi_p,color);grid on;title('$\dot{\phi_t}$','Interpreter','latex','FontSize',TamanioFuente);hold on;
subplot(3,2,4);plot(t,h,color);grid on;title('Altura','FontSize',TamanioFuente);hold on;
subplot(3,1,3);plot(t,u_k,color);grid on;title('Acción de control','FontSize',TamanioFuente);
xlabel('Tiempo en Seg.','FontSize',TamanioFuente);hold on;


figure(2);hold on;
%subplot(2,1,1);
plot(alfa,h,color);grid on;xlabel('Ángulo','FontSize',TamanioFuente);ylabel('altura','FontSize',TamanioFuente);hold on;
% subplot(2,1,2);plot(t,J,color);grid on;title('Modelo lineal','FontSize',TamanioFuente);
% xlabel('Tiempo en Seg.','FontSize',TamanioFuente);ylabel('Funcionales J y V','FontSize',TamanioFuente);hold on;
% plot(t,Jmin*ones(size(J)),color); plot(t,V_L,color);
