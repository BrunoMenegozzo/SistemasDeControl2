%------------------------------------------------------------------------------
%Carro Pendulo codigo final
%------------------------------------------------------------------------------

clc;clear all;%close all;
m=.1;Fricc=0.1; long=2.6;g=9.8;M=.5;
%TIEMPOS: de muestreo, de simulación, de Euler y su integración
Ts=0.01;KMAX=5000-1;Veces_Euler=100;h=Ts/Veces_Euler;t_d=(0:KMAX)*Ts;
TamanioFuente=12;

%Referencias
ref=10;

%Condiciones iniciales
alfa(1)=pi*0; color_='r';

%Alinealidad
Alinealidad=0;%color='r';
% Alinealidad=50;color='b';
% Alinealidad=75;color='g';
% Alinealidad=150;color='m';

%Versión linealizada en el equilibrio estable
Mat_Ac=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 -Fricc/(long*M) -(g*(m+M)/(long*M)) 0];
Mat_Bc=[0; 1/M; 0; 1/(long*M)];

%Versión linealizada en el equilibrio inestable
% Mat_Ac=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 Fricc/(long*M) g*(m+M)/(long*M) 0];
% Mat_Bc=[0; 1/M; 0; -1/(long*M)];

Mat_C=[1 0 0 0; 0 0 1 0];% Dos variables observables angulo y posicion

% Oc=[Mat_C; Mat_C*Mat_Ac; Mat_C*Mat_Ac^2; Mat_C*Mat_Ac^3];%Matriz Observabilidad
% auto_O=eig(Oc);
% rango_O=rank(Oc);

%Punto de linealizacion
x_op=[0; 0; pi; 0];

I=eye(4);
%Matriz A y B discretas
H=[0;0;0;0];d_tao=Ts/100;tao=0;
for hh=1:100
    dH=expm(Mat_Ac*tao)*Mat_Bc*d_tao;
    H=H+dH;
    tao=tao+d_tao;
end
Mat_B=H;
Mat_A=expm(Mat_Ac*Ts);

% sys_c=ss(Mat_Ac,Mat_Bc,Mat_C,0);
% sys_d=c2d(sys_c,Ts,'zoh');
% A = sys_d.A;
% B = sys_d.B;

%Matrices Ampliadas
Mat_Aa=[Mat_A,zeros(4,1);-Mat_C(1,:)*Mat_A, 1];
Mat_Ba=[Mat_B;-Mat_C(1,:)*Mat_B];
Mat_Ma=[Mat_Ba Mat_Aa*Mat_Ba Mat_Aa^2*Mat_Ba Mat_Aa^3*Mat_Ba Mat_Aa^4*Mat_Ba];%Matriz Controlabilidad
rango=rank(Mat_Ma);% verifico controlabilidad

%Funcional de costos( Q = velocidad de x->ref ; R = magnitud de acc)
Qc=diag([1e0 1e0 1e1 1e0 1e-3 ]); R=1e-3; %Ts=0.01;

%Contrucción del Hamiltoniano para el cálculo del controlador
H= inv([eye(5) Mat_Ba*inv(R)*Mat_Ba'; zeros(5) Mat_Aa'])*[Mat_Aa zeros(5);-Qc eye(5)];

[V,D]=eig(H);MX1X2=[]; 
for ii=1:10
    if abs(D(ii,ii))<1 %Discrimino los polos que esten en circulo unitario
        MX1X2=[MX1X2 V(:,ii)];
    end
end
MX1=MX1X2(1:5,:); MX2=MX1X2(6:10,:);
Pc=real(MX2*inv(MX1)); %Solucion de Riccati
Ka= inv(R)*Mat_Ba'*Pc; %Controloador optimo
% K=Ka(1:4);KI=-Ka(5);

% Solucion con la funcion dlqr
[Ka,Pc] = dlqr(Mat_Aa,Mat_Ba,Qc,R);

aut_controlador=abs(eig(Mat_Aa-Mat_Ba*Ka))%Verifico que sean estables

%Observador
Mat_Adual=Mat_A';
Mat_Bdual=Mat_C';
Mat_Cdual=Mat_B';

%Funcional de costos observador
Qobs=diag([1 1 1e0 1]);Ro=diag([1e2 1e1]);

%Contrucción del Hamiltoniano para el cálculo del Observador
Ho=inv([eye(4) Mat_Bdual*inv(Ro)*Mat_Bdual'; zeros(4) Mat_Adual'])*[Mat_Adual zeros(4);-Qobs eye(4)];
[Vo,Do]=eig(Ho);MX1X2=[];
for ii=1:8
    if abs(Do(ii,ii))<1 %Discrimino los polos que esten en circulo unitario
        MX1X2=[MX1X2 Vo(:,ii)];
    end
end
MX1o=MX1X2(1:4,:); MX2o=MX1X2(5:8,:);
Po=real(MX2o*inv(MX1o)); %Solucion de Riccati
Kobs=(inv(Ro+Mat_Bdual'*Po*Mat_Bdual)*Mat_Bdual'*Po*Mat_Adual)'; %K optimo
p_observador=abs(eig(Mat_A-Kobs*Mat_C)); %Verifico polos de observabilidad

%Condiciones iniciales para simulacion
t=0; 
x=[0;0;alfa(1);0];x_hat=[0;0;0;0];

p(1)=x(1); p_p(1)=x(2); alfa(1)=x(3); omega(1)=x(4);ve1(1)=0;

V_L=[x;0]'*Pc*[x;0];x_hat=[0;0;0;0];Jl=0;Jmin_L=V_L;
flag=0; ref=10;

for ki=2:KMAX
    t=[t (ki-1)*Ts];
    Y_=Mat_C*(x-x_op);  %Mido y inicialmente
    e1=ref-Y_(1);       %Error de referencia 
    V_L=[V_L [x;ve1(ki-1)]'*Pc*[x;ve1(ki-1)]]; %Liapunov
    ve1(ki)=ve1(ki-1)+e1;   
    
    if ki>KMAX/2 %El carro vuelve
        if (flag==0)
            ref=0; %Cambio de referencia
            m=m*10;
            flag=1;
        end
    end 
    
   u= -Ka* [x_hat-x_op; ve1(ki)];color='b';%Estado ampliado observador
     %u=-Ka*[(x-x_op);ve1(ki)];color='r';%Estado ampliado sin observador    
    
    %Alinealidad
    if -Alinealidad<u && u<Alinealidad
        u=0;
    else
        u=u;
    end 
    
    x = (Mat_A*(x-x_op) + Mat_B*u)+x_op; %x de OPERACION
    Jl=[Jl Jl(ki-1)+[x;ve1(ki)]'*Qc*[x;ve1(ki)]+u'*R*u];%Funcional de costos
    
    %Grafico
    p(ki)=x(1);
    p_p(ki)=x(2);
    alfa(ki)=x(3);
    omega(ki)=x(4);
    u_k(ki)=u;
    
    y_hat=Mat_C*(x_hat-x_op);
    x_hat=Mat_A*(x_hat-x_op) + Mat_B*u + Kobs*(Y_-y_hat) + x_op;%Se actualiza acá

end
Jl=[Jl Jl(ki-1)+[x;ve1(ki)]'*Qc*[x;ve1(ki)]+u'*R*u];
V_L=[V_L [x;ve1(ki-1)]'*Pc*[x;ve1(ki-1)]];u=u_k;

figure(1);hold on;
subplot(3,2,1);plot(t,omega,color);title('Velocidad angular');grid on;
subplot(3,2,2);plot(t,alfa,color);title('Angulo pendulo');grid on;
subplot(3,2,3); plot(t,p,color);title('Desplazamiento Carro');grid on;
subplot(3,2,4);plot(t,p_p,color);title('Velocidad Carro');grid on;
subplot(3,1,3);plot(t,u,color);grid on;

figure(2);hold on;
subplot(2,2,1);plot(alfa,omega,color);xlabel('Velocidad angular');ylabel('Angulo pendulo');grid on; 
subplot(2,2,2);plot(p,p_p,color);xlabel('Desplazamiento Carro');ylabel('Velocidad Carro');grid on;
subplot(2,2,3); semilogy(t_d,Jl,color);hold on; semilogy(t_d,Jmin_L*ones(size(Jl)),color);
semilogy(t_d,V_L,color);grid on;
