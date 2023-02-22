%------------------------------------------------------------------------------------------------------
clc;clear all;close all;
m=.1;Fricc=0.1; long=2.6;g=9.8;M=.5;
%TIEMPOS: de muestreo, de simulación, de Euler y su integración
Ts=0.01;KMAX=5000-1;Veces_Euler=100;h=Ts/Veces_Euler;t_d=(0:KMAX)*Ts;
TamanioFuente=12;

%Referencias
ref1=10;ref2=pi;

%Condiciones iniciales
alfa(1)=pi*0; color_='r';

%Versión linealizada en el equilibrio estable
Mat_Ac=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 -Fricc/(long*M) -(g*(m+M)/(long*M)) 0];
Mat_Bc=[0; 1/M; 0; 1/(long*M)];

%Versión linealizada en el equilibrio inestable
% Mat_Ac=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 Fricc/(long*M) g*(m+M)/(long*M) 0];
% Mat_Bc=[0; 1/M; 0; -1/(long*M)];

Mat_C=[1 0 0 0;0 0 1 0];% Dos variables observables angulo y posicion

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

[V,D]=eig(H);MX1X2=[]; %Puede que MX1X2 no sea cuadrada por los valores de Q !!!
for ii=1:10
    if abs(D(ii,ii))<1
        MX1X2=[MX1X2 V(:,ii)];
    end
end
MX1=MX1X2(1:5,:); MX2=MX1X2(6:10,:);
Pc=real(MX2*inv(MX1));
Ka= inv(R)*Mat_Ba'*Pc;
% K=Ka(1:4);KI=-Ka(5);

% 
[Ka,Pc] = dlqr(Mat_Aa,Mat_Ba,Qc,R);



aut_controlador=abs(eig(Mat_Aa-Mat_Ba*Ka))
%  break
%Cálculo del Observador de estados
Mat_Adual=Mat_A';
Mat_Bdual=Mat_C';
Mat_Cdual=Mat_B';
Mat_Qobs=[Mat_C;Mat_C*Mat_A;Mat_C*Mat_A^2;Mat_C*Mat_A^3];
rango_matriz_obs=rank(Mat_Qobs);
Qobs=diag([1 1 1e0 1]);Ro=diag([1e2 1e1]);

%Contrucción del Hamiltoniano para el cálculo del Observador
Ho=inv([eye(4) Mat_Bdual*inv(Ro)*Mat_Bdual'; zeros(4) Mat_Adual'])*[Mat_Adual zeros(4);-Qobs eye(4)];
[Vo,Do]=eig(Ho);MX1X2=[];
for ii=1:8
    if abs(Do(ii,ii))<1
        MX1X2=[MX1X2 Vo(:,ii)];
    end
end
MX1o=MX1X2(1:4,:); MX2o=MX1X2(5:8,:);
Po=real(MX2o*inv(MX1o));
Kobs=(inv(Ro+Mat_Bdual'*Po*Mat_Bdual)*Mat_Bdual'*Po*Mat_Adual)';
p_observador=abs(eig(Mat_A-Kobs*Mat_C)); %Verifica polos de observabilidad

%Condiciones iniciales para simulacion
t=0; x=[0;0;alfa(1);0];x_hat=[0;0;0;0];
p(1)=x(1); p_p(1)=x(2); alfa(1)=x(3); omega(1)=x(4);ve1(1)=0;ve2(1)=0;
p_(1)=0;p_p_(1)=0;alfa_(1)=0;omega_(1)=0; u_k(1)=0;
V_L=[x;0]'*Pc*[x;0];x_hat=[0;0;0;0];Jl=0;Jmin_L=V_L;%e1=0;e2=0;
flag=0; ref1=10;
for ki=2:KMAX
    t=[t (ki-1)*Ts];
    Y_=Mat_C*(x-x_op); %Se mide ACÁ
    e1=ref1-Y_(1); e2=ref2-Y_(2);
    V_L=[V_L [x;ve1(ki-1)]'*Pc*[x;ve1(ki-1)]];
    ve1(ki)=ve1(ki-1)+e1;    
%     if ki>KMAX/2 %El carro vuelve
%         if (flag==0)
%             ref1=0;
%             m=m*10;
%             flag=1;
%         end
%     end    
    ve2(ki)=ve2(ki-1)+e2;
    u= -Ka* [x_hat-x_op; ve1(ki)];color='b';%Estado ampliado observador
%     u=-Ka*[(x-x_op);ve1(ki)];cotlor='r';%Estado ampliado sin observador    
    %Alinealidad
%     if -.25<u && u<.25
%         u=0;
%     else
%         u=u;
%     end    
    x = (Mat_A*(x-x_op) + Mat_B*u)+x_op; %Faltaba sumarle a x(k+1) el x de OPERACION
    Jl=[Jl Jl(ki-1)+[x;ve1(ki)]'*Qc*[x;ve1(ki)]+u'*R*u];
    y_hat=Mat_C*(x_hat-x_op);
    x_hat=Mat_A*(x_hat-x_op) + Mat_B*u + Kobs*(Y_-y_hat) + x_op;%Se actualiza acá
    
    p(ki)=x(1);
    p_p(ki)=x(2);
    alfa(ki)=x(3);
    omega(ki)=x(4);
    %Valores Observados
    p_(ki)=x(1);
    p_p_(ki)=x(2);
    alfa_(ki)=x(3);
    omega_(ki)=x(4);
    u_k(ki)=u;
end
Jl=[Jl Jl(ki-1)+[x;ve1(ki)]'*Qc*[x;ve1(ki)]+u'*R*u];
V_L=[V_L [x;ve1(ki-1)]'*Pc*[x;ve1(ki-1)]];u=u_k;
figure(1);hold on;
subplot(3,2,1);plot(t,omega,color); plot(t,omega_,color);
subplot(3,2,2);plot(t,alfa,color);plot(t,alfa_,color);title('Angulo');
subplot(3,2,3); plot(t,p,color); plot(t,p_,color);
subplot(3,2,4);plot(t,p_p,color); plot(t,p_p_,color);
subplot(3,1,3);plot(t,u,color);
figure(2);hold on;
subplot(2,2,1);plot(alfa,omega,color); subplot(2,2,2);plot(p,p_p,color);
subplot(2,2,3); semilogy(t_d,Jl,color);hold on; semilogy(t_d,Jmin_L*ones(size(Jl)),color);
semilogy(t_d,V_L,color);

% %Verificación de la solución con el modelo no lineal en tiempo continuo.
% T=t(ki);x=[0;0;alfa(1);0];
% p=x(1); p_p=x(2); alfa=x(3); omega=x(4); tita_pp(1)=0;p_pp(1)=0;
% u=[];
% x_hat=[0;0;0;0];y=Mat_C*x;y_hat=0;ve1(1)=0;ve2(1)=0;alfa_(1)=alfa(1);
% Jn=0;V_NL=[x;0;0]'*Pc*[x;0;0];Jmin_NL=V_NL;i=1;
% for ki=2:KMAX+1
%  V_NL=[V_NL [x;ve1(ki-1);ve2(ki-1)]'*Pc*[x;ve1(ki-1);ve2(ki-1)]];
%  Y_=Mat_C*x;
%  ve1(ki)=ve1(ki-1)+ref1-Y_(1);
%  ve2(ki)=ve2(ki-1)+ref2-Y_(2);%No es necesario. Ya que no es Observable.
%  u1(ki)=-Ka*[x_hat;ve1(ki);ve2(ki)];
%  for kii=1:Veces_Euler % Tiene relacion h con Ts
%  u(i)=u1(ki);
% %   if ki>KMAX/2
% %      ref1=0;
% %      m=m*10;
% %  end
%  p_pp=(1/(M+m))*(u(i)-m*long*tita_pp*cos(alfa(i))+m*long*omega(i)^2*sin(alfa(i))- Fricc*p_p(i));
%  tita_pp=(1/long)*(g*sin(alfa(i))-p_pp*cos(alfa(i)));
%  p_p(i+1)=p_p(i)+h*p_pp;
%  p(i+1)=p(i)+h*p_p(i+1);
%  omega(i+1)=omega(i)+h*tita_pp;
%  alfa(i+1)=alfa(i)+h*omega(i+1);
%  alfa_(i)=x_hat(3);
%  i=i+1;
%  end
%  x=[p(i-1); p_p(i-1); alfa(i-1); omega(i-1)]; %Acá está x(k+1)
%  x_hat=Mat_A*x_hat+Mat_B*u1(ki)+Kobs*(Y_-y_hat);
%  y_hat=Mat_C*x_hat;
%  Jn=[Jn Jn(ki-1)+[x;ve1(ki);ve2(ki)]'*Qc*[x;ve1(ki);ve2(ki)]+u1(ki)'*R*u1(ki)];
% end

% u(i)=u1(ki);t=(1:i)*h;
% figure(1);hold on;
% subplot(3,2,1);plot(t,alfa,color);grid on;title('\phi_t','FontSize',TamanioFuente);hold on;
% subplot(3,2,2);plot(t,omega,color);grid on;
% title('$\dot{\phi_t}$','Interpreter','latex','FontSize',TamanioFuente);hold on;
% subplot(3,2,3); plot(t,p,color);grid on;title('\delta_t','FontSize',TamanioFuente);hold on;
% subplot(3,2,4);plot(t,p_p,color);grid on;title('$\dot{\delta_t}$','Interpreter','latex','FontSize',TamanioFuente);hold on;
% subplot(3,1,3);plot(t,u,color);grid on;title('Acción de control','FontSize',TamanioFuente);xlabel('Tiempo en Seg.','FontSize',TamanioFuente);hold on;

% figure(2);hold on;
% subplot(2,2,1);plot(alfa,omega,color);grid on;
% xlabel('\phi_t','FontSize',TamanioFuente);
% ylabel('$\dot{\phi_t}$','Interpreter','latex','Rotation',0,'FontSize',TamanioFuente);hold on;
% subplot(2,2,2);plot(p,p_p,color);grid on;xlabel('Posición carro','FontSize',TamanioFuente);hold on;
% ylabel('$\dot{\delta_t}$','Interpreter','latex','Rotation',0,'FontSize',TamanioFuente);hold on;
% xlabel('\delta_t','FontSize',TamanioFuente);
% subplot(2,2,3);
% semilogy(t_d,Jl,color);hold on;%grid on;
% semilogy(t_d,Jmin_L*ones(size(Jl)),color); semilogy(t_d,V_L,color);
% title('Modelo lineal','FontSize',TamanioFuente);xlabel('Tiempo en Seg.','FontSize',TamanioFuente);
% ylabel('Funcionales J y V','FontSize',TamanioFuente);
% set(gca,'fontsize',TamanioFuente);
% subplot(2,2,4);
% semilogy(t_d,Jn,colorc);grid on;title('Modelo no lineal','FontSize',TamanioFuente);
% xlabel('Tiempo en Seg.','FontSize',TamanioFuente);ylabel('Funcionales J y V','FontSize',TamanioFuente);hold on;
% semilogy(t_d,Jmin_NL*ones(size(Jn)),colorc);
% semilogy(t_d,V_NL,colorc);
% set(gca,'fontsize',TamanioFuente);
