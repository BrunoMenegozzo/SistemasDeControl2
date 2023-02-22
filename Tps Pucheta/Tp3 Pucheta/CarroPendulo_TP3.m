%Los primeros codigos comentados son de prueba, el codigo realmente
%comienza en la linea 265.
%-------------------------------------------------------------------------
%LQR sin ref sin observador
%-------------------------------------------------------------------------
% clc;clear all;
% m=.1;Fricc=0.1; long=0.6;g=9.8;M=.5;
% h=0.0001;tiempo=(50/h);p_pp=0;tita_pp=0;
% %Condiciones iniciales
% fi(1)=.1; color='r';
% % fi(1)=.5; color='g';
% % fi(1)=.8; color='b';
% % fi(1)=-pi; color='+-b';
% % fi(1)=pi; color='.-r';
% omega(1)=0; p_p(1)=0; u(1)=0; p(1)=0; i=1;
% %Versión linealizada en el equilibrio inestable. Sontag Pp 104.
% %estado=[p(i); p_p(i); fi(i); omega(i)]
% Mat_A=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 Fricc/(long*M) g*(m+M)/(long*M) 0]
% Mat_B=[0; 1/M; 0; -1/(long*M)]
% Mat_M=[Mat_B Mat_A*Mat_B Mat_A^2*Mat_B Mat_A^3*Mat_B ];%Matriz Controlabilidad
% rank(Mat_M)
% %Cálculo del LQR
% % Q=diag([.2 10 .10 .1]);R=100; %Nótese con R=1 no cumple restricciones de u
% Q=diag([.002 1.0 .10 .01]);R=50; %Nótese con R=1 no cumple restricciones de u
% %Contrucción del Hamiltoniano para el cálculo del controlador
% H=[Mat_A -Mat_B*inv(R)*Mat_B'; -Q -Mat_A'];
% [n_,va]=size(H);
% [V,D]=eig(H);MX1X2=[];
% for(ii=1:n_)
%  if real(D(ii,ii))<0
%  MX1X2=[MX1X2 V(:,ii)];
%  end
% end
% MX1=MX1X2(1:n_/2,:); MX2=MX1X2(n_/2+1:end,:);
% P=real(MX2*inv(MX1));
% K=inv(R)*Mat_B'*P;
% estado=[p(1); p_p(1); fi(1); omega(1)];
% J_(1)=0; V_(1)=0;
% while(i<(tiempo+1))
%  estado=[p(i); p_p(i); fi(i); omega(i)];
%  V_(i)=estado'*P*estado;
%  if(i<=tiempo/2)
%      ref=10; %posicion de referencia 10m
%      m=0.1;
%  else
%      ref=0; %posicion de referencia 0m
%      m=1;
%  end
%   %-----------------------------------------
% %  fi1=estado(3);
% %     if abs(fi1)>pi
% %         while abs(fi1)>2*pi
% %             fi1=sign(fi1)*(abs(fi1)-2*pi);
% %         end
% %         if fi1<-pi
% %             fi1=2*pi+(fi1);%Lo hace positivo
% %         end
% %  
% %         if fi1>pi
% %             fi1=-2*pi+(fi1);%Lo hace negativo
% %         end
% %     end
% %  estado(3)=fi1;
% %-------------------------------------------------------------
%  u(i)=-K*estado;
%  J_(i+1)=J_(i)+(estado'*Q*estado+u(i)'*R*u(i))*h;
% % u(i)=min( 100,u(i));
% % u(i)=max(-100,u(i));
%  p_pp=(1/(M+m))*(u(i)-m*long*tita_pp*cos(fi(i))+m*long*omega(i)^2*sin(fi(i))-Fricc*p_p(i));
%  tita_pp=(1/long)*(g*sin(fi(i))-p_pp*cos(fi(i)));
%  p_p(i+1)=p_p(i)+h*p_pp;
%  p(i+1)=p(i)+h*p_p(i);
%  omega(i+1)=omega(i)+h*tita_pp;
%  fi(i+1)=fi(i)+h*omega(i);
%  % estado_p=Mat_A*estado+Mat_B*u(i);
%  % p_p(i+1)=p_p(i)+h*estado_p(2);
%  % p(i+1)=p(i)+h*estado_p(1);
%  % omega(i+1)=omega(i)+h*estado_p(4);
%  % fi(i+1)=fi(i)+h*estado_p(3);
%  i=i+1;
% end
% 
% V_(i)=estado'*P*estado;t=0:tiempo; t=t*h;
% figure(1);hold on;
% subplot(3,2,1); plot(t,p,color);grid on;title('Posición carro');hold on;
% subplot(3,2,3);plot(t,p_p,color);grid on;title('Velocidad carro');hold on;
% subplot(3,2,2);plot(t,fi,color);grid on;title('Ángulo');hold on;
% subplot(3,2,4);plot(t,omega,color);grid on; title('Velocidad angulo');hold on;
% subplot(3,1,3);plot(t(1:end-1),u,color);grid on;title('Acción de control');xlabel('Tiempo en Seg.');hold on;
% figure(2);hold on;
% subplot(2,2,1);plot(fi,omega,color);grid on;xlabel('Ángulo');ylabel('Velocidad angular');hold on;
% subplot(2,2,2);plot(p,p_p,color);grid on;xlabel('Posición carro');ylabel('Velocidad carro');hold on;
% subplot(2,2,3);semilogy(t,J_,color);title('Funcional de costos J(x,u)');hold on;xlabel('Tiempo Seg.');hold on;v=axis;
% subplot(2,2,4);semilogy(t,V_,color);title('Funcion Liapunov V(x)');xlabel('Tiempo Seg.');hold on;axis(v);

%-------------------------------------------------------------------------
%Discreto; con ref; y observador 
%-------------------------------------------------------------------------
% clc;clear all;
% Ts=0.01;m=.1;Fricc=0.1; long=0.6;g=9.8;M=.5;KMAX=5000;TEuler=0.001;
% %pkg load control; pkg load signal %Una sola vez
% TamanioFuente=12;
% %Condiciones iniciales
% alfa(1)=pi; color='.r';colorc='r';
% % alfa(1)=.7; color='.b';colorc='b';
% %alfa(1)=.8; color='.b';colorc='k';
% ref=10;
% % %Punto de linealizacion
% x_op=[0;0;pi;0];
% % 
% % %Versión linealizada en el equilibrio estable
% % estado=[p(i); p_p(i); alfa(i); omega(i)]
% Mat_Ac=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 -Fricc/(long*M) -(g*(m+M)/(long*M)) 0];
% Mat_Bc=[0; 1/M; 0; 1/(long*M)];
% Mat_C= [1 0 0 1];
% Mat_D= [0];
% I=eye(4); %<-- Matriz identidad de 4x4
% 
% sys_c=ss(Mat_Ac,Mat_Bc,Mat_C,[0]);
% sys_d=c2d(sys_c,Ts,'zoh');
% 
% Mat_A=sys_d.a;
% Mat_B=sys_d.b;
% Mat_Aa=[Mat_A,zeros(4,1);-Mat_C*Mat_A, 1];
% Mat_Ba=[Mat_B;-Mat_C*Mat_B];
% Mat_Ma=[Mat_Ba Mat_Aa*Mat_Ba Mat_Aa^2*Mat_Ba Mat_Aa^3*Mat_Ba Mat_Aa^4*Mat_Ba];%Matriz Controlabilidad
% rango=rank(Mat_Ma);
% 
% %Cálculo del controlador por asignación de polos
% auto_val=eig(Mat_Aa);
% c_ai=poly(auto_val);
% Mat_Wa=[c_ai(5) c_ai(4) c_ai(3) c_ai(2) 1;c_ai(4) c_ai(3) c_ai(2) 1 0;c_ai(3) c_ai(2) 1 0 0;c_ai(2) 1 0 0 0;1 0 0 0 0];
% Mat_T=Mat_Ma*Mat_Wa;
% A_controlable=inv(Mat_T)*Mat_Aa*Mat_T %Verificación de que T esté bien
% %-----------------------------------------------------------------------
% %Cálculo del LQR en Tiempo discreto
% Q=1e2*eye(5);R=1;
% t=0; x=[0;0;alfa(1);0];
% p(1)=x(1); p_p(1)=x(2); alfa(1)=x(3); omega(1)=x(4);
% %----------------------------------------------------------------
% %Contrucción del Hamiltoniano para el cálculo del controlador
% H=[Mat_Aa+Mat_Ba*inv(R)*Mat_Ba'*inv(Mat_Aa')*Q -Mat_Ba*inv(R)*Mat_Ba'*inv(Mat_Aa') ; -inv(Mat_Aa')*Q inv(Mat_Aa')];
% %O también
% %H=inv([eye(4) Mat_B*inv(R)*Mat_B'; zeros(4) Mat_A'])*[Mat_A zeros(4);-Q eye(4)]
% [V,D]=eig(H);MX1X2=[];
% for ii=1:8
%  if abs(D(ii,ii))<1
%  MX1X2=[MX1X2 V(:,ii)];
%  end
% end
% MX1=MX1X2(1:4,:); MX2=MX1X2(5:8,:);
% P=real(MX2*inv(MX1)); % [K1,P,E]=dlqr(Mat_A,Mat_B,Q,R);
% Ka=inv(R+Mat_B'*P*Mat_B)*Mat_B'*P*Mat_A;
% Jmin=x'*P*x;J=0;V_L=x'*P*x;
% %-----------------------------------------------------------------
% % %Ubicación de los polos de lazo cerrado en mui:
% % mui(1)=0.5; mui(2)=0.9; mui(3)= conj(mui(2)); mui(4)=0.99; mui(5)=0.99;
% % alfa_i=poly(mui);
% % Ka=fliplr(alfa_i(2:6)-c_ai(2:6))*inv(Mat_T);
% K=Ka(1:4);
% KI=-Ka(5);
% % abs(eig(Mat_Aa-Mat_Ba*Ka))
% %____________________________OBSERVADOR______________
% Mat_M_Obs=[Mat_C;(Mat_C*Mat_A);(Mat_C*Mat_A^2);(Mat_C*Mat_A^3)];
% rank(Mat_M_Obs)
% %Como el Rango es 4, se genera el sistema Dual y se vuelve a repetir la
% %secuencia de cálculo vista
% Mat_C_O=Mat_B';
% Mat_A_O=Mat_A';
% Mat_B_O=Mat_C';
% Mat_M_D=Mat_M_Obs';%[Mat_Ba Mat_Aa*Mat_Ba Mat_Aa^2*Mat_Ba Mat_Aa^3*Mat_Ba];%Matriz Controlabilidad
% c_aid=poly(Mat_A);
% Mat_Wa=[c_aid(4) c_aid(3) c_aid(2) 1;c_aid(3) c_aid(2) 1 0;c_aid(2) 1 0 0;1 0 0 0];
% Mat_T_D=Mat_M_D*Mat_Wa;
% %alfa_i_O=poly([.0999 .095 .094 .093]); %Polos deseados del Observador muy bueno para el caso lineal
% alfa_i_O=poly([.7 .7 .71 .7]); %Polos deseados del Observador
% Ko=(fliplr(alfa_i_O(2:5)-c_aid(2:5))*inv(Mat_T_D))';
% abs(eig(Mat_A-Ko*Mat_C))
% t=0; x=[0;0;alfa(1);0];
% p(1)=x(1); p_p(1)=x(2); alfa(1)=x(3); omega(1)=x(4);ve(1)=0;
% u_k(1)=0;xang=[0;0;0;0];
% for ki=2:KMAX
%  t=[t ki*Ts];
%  ve(ki)=ve(ki-1)+ref-Mat_C*x;
%   u=-K*(x-x_op)+KI*ve(ki);
%  %u=-K*(xang-x_op)+KI*ve(ki);
%  ys=Mat_C*x; %Acá DEBE medirse y.
%  
%    if(ki<=KMAX/2)
%      ref=10; %posicion de referencia 10m
%      m=0.1;
%  else
%      ref=0; %posicion de referencia 0m
%      m=1;
%   end
%  
%  
%  x=Mat_A*x+Mat_B*u;
% 
%  p(ki)=x(1);
%  p_p(ki)=x(2);
%  alfa(ki)=x(3);
%  omega(ki)=x(4);
%  u_k(ki)=u;
%  xang=Mat_A*xang+Mat_B*u+Ko*(ys-Mat_C*(xang-x_op));%Acá se usa y.
% end
% % u=u_k;figure(1);hold on;
% % subplot(3,2,1);plot(t,omega,color);grid on; title('Velocidad ángulo','FontSize',TamanioFuente);hold on;
% % subplot(3,2,2);plot(t,alfa,color);grid on;title('Ángulo','FontSize',TamanioFuente);hold on;
% % subplot(3,2,3); plot(t,p,color);grid on;title('Posición carro','FontSize',TamanioFuente);hold on;
% % subplot(3,2,4);plot(t,p_p,color);grid on;title('Velocidad carro','FontSize',TamanioFuente);hold on;
% % subplot(3,1,3);plot(t,u,color);grid on;title('Acción de control','FontSize',TamanioFuente);
% % xlabel('Tiempo en Seg.','FontSize',TamanioFuente);hold on;
% % figure(2);hold on;
% % subplot(2,2,1);plot(alfa,omega,color);grid on;xlabel('Ángulo','FontSize',TamanioFuente);ylabel('Velocidad angular','FontSize',TamanioFuente);hold on;
% % subplot(2,2,2);plot(p,p_p,color);grid on;xlabel('Posición carro','FontSize',TamanioFuente);
% % ylabel('Velocidad carro','FontSize',TamanioFuente);hold on;
% 
% %Verificación de la solución con el modelo no lineal en tiempo continuo.
% T=t(ki);x=[0;0;alfa(1);0];
% p=x(1); p_p=x(2); alfa=x(3); omega=x(4); tita_pp(1)=0;Vh=Ts/TEuler;h=TEuler; u=[];i=1;
% u_k(1)=0;xang=[0;0;0;0];
% for ki=1:KMAX
%  ve(ki+1)=ve(ki)+ref-Mat_C*x;
%  u1(ki)=-K*(xang-x_op)+KI*ve(ki); %Control con observación de estados
%  % u1(ki)=-K*(x-x_op)+KI*ve(ki); %Sin Observador
%  ys=Mat_C*x;%Acá se mide la salida.
%  
%    if(ki<=KMAX/2)
%      ref=10; %posicion de referencia 10m
%      m=0.1;
%  else
%      ref=0; %posicion de referencia 0m
%      m=1;
%   end
%  
%  
%  for kii=1:Vh
%  u(i)=u1(ki);
%  p_pp=(1/(M+m))*(u(i)-m*long*tita_pp*cos(alfa(i))+m*long*omega(i)^2*sin(alfa(i))- Fricc*p_p(i));
%  tita_pp=(1/long)*(g*sin(alfa(i))-p_pp*cos(alfa(i)));
%  p_p(i+1)=p_p(i)+h*p_pp;
%  p(i+1)=p(i)+h*p_p(i);
%  omega(i+1)=omega(i)+h*tita_pp;
%  alfa(i+1)=alfa(i)+h*omega(i);
%  i=i+1;
%  end
%  x=[p(i-1); p_p(i-1); alfa(i-1); omega(i-1)];
%  xang=Mat_A*xang+Mat_B*u1(ki)+Ko*(ys-Mat_C*(xang-x_op));%Acá se usa y.
% end
% u(i)=u1(ki);t=0:h:KMAX*(Vh)*h;
% figure(1);hold on;
% subplot(3,2,1);plot(t,omega,colorc);grid on; title('Velocidad ángulo','FontSize',TamanioFuente);hold on;
% subplot(3,2,2);plot(t,alfa,colorc);grid on;title('Ángulo','FontSize',TamanioFuente);hold on;
% subplot(3,2,3);plot(t,p,colorc);grid on;title('Posición carro','FontSize',TamanioFuente);hold on;
% subplot(3,2,4);plot(t,p_p,colorc);grid on;title('Velocidad carro','FontSize',TamanioFuente);hold on;
% subplot(3,1,3);plot(t,u,colorc);grid on;title('Acción de control','FontSize',TamanioFuente);
% xlabel('Tiempo en Seg.','FontSize',TamanioFuente);hold on;
% figure(2);hold on;
% % % % subplot(2,2,1);
% plot(alfa,omega,colorc);grid on;xlabel('Ángulo','FontSize',TamanioFuente);ylabel('Velocidad angular','FontSize',TamanioFuente);hold on;
% % % subplot(2,2,2);plot(p,p_p,colorc);grid on;xlabel('Posición carro','FontSize',TamanioFuente);ylabel('Velocidad carro','FontSize',TamanioFuente);hold on;

%------------------------------------------------------------------------------------------------------
clc;clear all;
m=.1;Fricc=0.1; long=2.6;g=9.8;M=.5;
%TIEMPOS: de muestreo, de simulación, de Euler y su integración
Ts=0.01;KMAX=5000-1;Veces_Euler=100;h=Ts/Veces_Euler;t_d=(0:KMAX)*Ts;
TamanioFuente=12;

%Referencias
ref1=10;ref2=pi;

%Condiciones iniciales
alfa(1)=pi; color_='r';

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

%Matrices Ampliadas
Mat_Aa=[Mat_A,zeros(4,2);-Mat_C*Mat_A, eye(2)];
Mat_Ba=[Mat_B;-Mat_C*Mat_B];
Mat_Ma=[Mat_Ba Mat_Aa*Mat_Ba Mat_Aa^2*Mat_Ba Mat_Aa^3*Mat_Ba Mat_Aa^4*Mat_Ba];%Matriz Controlabilidad
rango=rank(Mat_Ma);% verifico controlabilidad

%Funcional de costos( Q = velocidad de x->ref ; R = magnitud de acc)
Qc=diag([0.95 0.9 0.9 0.9 1e-4 1e-4]); R=1; %Ts=0.01; 

%Contrucción del Hamiltoniano para el cálculo del controlador
H= inv([eye(6) Mat_Ba*inv(R)*Mat_Ba'; zeros(6) Mat_Aa'])*[Mat_Aa zeros(6);-Qc eye(6)];

[V,D]=eig(H);MX1X2=[]; %Puede que MX1X2 no sea cuadrada por los valores de Q !!!
for ii=1:12
 if abs(D(ii,ii))<1
 MX1X2=[MX1X2 V(:,ii)];
 end
end
MX1=MX1X2(1:6,:); MX2=MX1X2(7:12,:);
Pc=real(MX2*inv(MX1));
Ka= inv(R)*Mat_Ba'*Pc;
K=Ka(1:4);KI=-Ka(5);
aut_controlador=abs(eig(Mat_Aa-Mat_Ba*Ka));

%Cálculo del Observador de estados
Mat_Adual=Mat_A';
Mat_Bdual=Mat_C';
Mat_Cdual=Mat_B';
Mat_Qobs=[Mat_C;Mat_C*Mat_A;Mat_C*Mat_A^2;Mat_C*Mat_A^3];
rango_matriz_obs=rank(Mat_Qobs);
Qobs=diag([1 1 1e1 1]);Ro=diag([1e-2 1e-1]);

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
V_L=[x;0;0]'*Pc*[x;0;0];x_hat=[0;0;0;0];Jl=0;Jmin_L=V_L;%e1=0;e2=0;

for ki=2:KMAX
 t=[t (ki-1)*Ts];
 Y_=Mat_C*(x-x_op); %Se mide ACÁ
 e1=ref1-Y_(1);e2=ref2-Y_(2);
 V_L=[V_L [x;ve1(ki-1);ve2(ki-1)]'*Pc*[x;ve1(ki-1);ve2(ki-1)]];
 ve1(ki)=ve1(ki-1)+e1;
 
 if ki>KMAX/2 %El carro vuelve 
     ref1=0;
     m=m*10;
 end
 
 ve2(ki)=ve2(ki-1)+e2;
 u=-Ka*[x_hat;ve1(ki);ve2(ki)];color='b';%Estado ampliado observador
 %u=-Ka*[x;ve1(ki);ve2(ki)];color='r';%Estado ampliado sin observador
 
 %Alinealidad
 if -.25<u && u<.25
     u=0;
 else
     u=u;
 end
 
 x=Mat_A*x+Mat_B*u;
 Jl=[Jl Jl(ki-1)+[x;ve1(ki);ve2(ki)]'*Qc*[x;ve1(ki);ve2(ki)]+u'*R*u];
 y_hat=Mat_C*(x_hat-x_op);
 x_hat=Mat_A*x_hat+Mat_B*u+Kobs*(Y_-y_hat);%Se actualiza acá
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
Jl=[Jl Jl(ki-1)+[x;ve1(ki);ve2(ki)]'*Qc*[x;ve1(ki);ve2(ki)]+u'*R*u];
V_L=[V_L [x;ve1(ki-1);ve2(ki-1)]'*Pc*[x;ve1(ki-1);ve2(ki-1)]];u=u_k;
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