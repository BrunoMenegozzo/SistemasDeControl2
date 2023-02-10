% Los primeros códigos son intentos fallidos, los subo también para que se
% vean los ditintos intentos para resolver el TP.
%--------------------------------------------------------------------------
%Intento 1 : Punto 3/ Controlador sin Referencia
%-------------------------------------------------------------------------
% clear all;
% T=70; Kmax=100000; At=T/Kmax;
% %u=zeros(Kmax);y=u;yp=y; 
% t=0:At:T-At;
% u=[sign(t)]; 
% e=zeros(T/At,1);
% ref=-100;
% 
% %Datos
% a=0.05;
% b=5;
% c=100;
% w=3;
% 
% %Condiciones iniciales
% 
% alfa(1)=0; tita_p(1)=0; tita(1)=0; h(1)=-500;
% 
% %Matrices
% A= [-a a 0 0 ;0 0 1 0; w^2 -w^2 0 0; c 0 0 0] ;
% B= [0; 0; w^2*b; 0];
% C= [0 0 0 1];
% D= [0];
% x= [alfa(1);tita_p(1);tita(1);h(1)]; 
% M=[B A*B A^2*B A^3*B ];%Matriz Controlabilidad
% 
% %Cálculo del controlador por asignación de polos
% auto_val=eig(A);
% c_ai=conv(conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]),[1 -auto_val(4)]);
% W=[c_ai(4) c_ai(3) c_ai(2) 1;c_ai(3) c_ai(2) 1 0;c_ai(2) 1 0 0;1 0 0 0];
% T=M*W;
% A_controlable=inv(T)*A*T %Verificación de que T esté bien
% 
% %CONTROLADOR Ubicación de los polos de lazo cerrado en mui :
% mui(1)=-15 + 15i;mui(2)=-15 - 15i; mui(3)=-.5 + .5i;mui(4)=-.5 - .5i;
% alfa_i=conv(conv(conv([1 -mui(3)],[1 -mui(4)]),[1 -mui(2)]),[1 -mui(1)]);
% K=fliplr(alfa_i(2:5)-c_ai(2:5))*inv(T);
% eig(A-B*K)
% 
% tve(1)=0;u(1)=0;
% for jj=1:Kmax-1
%  
%  estado=[alfa(jj); tita_p(jj); tita(jj); h(jj)];
%  u(jj)=-K*estado; color='b'; %Sin Observador
%  
%  alfa_p=a*(tita(jj)-alfa(jj));
%  alfa(jj+1)=alfa(jj)+At*alfa_p;
%  tita_pp=-w^2*(tita(jj)-alfa(jj)-b*u(jj));
%  tita_p(jj+1)=tita_p(jj)+At*tita_pp;
%  tita(jj+1)=tita(jj)+At*tita_p(jj);
%  h_p(jj+1)=c*alfa(jj);
%  h(jj+1)=h(jj)+At*h_p(jj);
%  
%  y_sal(jj+1)=C*estado;
% 
%  tve(jj+1)=tve(jj)+At;
% end
% figure;
% subplot(5,1,1);plot(tve,alfa);grid on; title('Angulo alfa');xlim([0 70]); hold on%plot(t,y);
% subplot(5,1,2);plot(tve,tita);grid on; title('angulo tita');xlim([0 70]); hold on
% subplot(5,1,3);plot(tve,y_sal);grid on; title('y_s_a_l');xlim([0 70]); hold on
% subplot(5,1,4);plot(tve,h);grid on; title('altura');xlim([0 70]); hold on
% subplot(5,1,5);plot(t,u);grid on; title('u');xlim([0 70]); hold on
% xlabel('Tiempo en segundos')

% %--------------------------------------------------------------------------
% %Intento 2 : Punto 3/ Controlador con Referencia.
% %-------------------------------------------------------------------------
% clear all;
% T=70; Kmax=100000; At=T/Kmax;
% %u=zeros(Kmax);y=u;yp=y; 
% t=0:At:T-At;
% u=[sign(t)]; 
% e=zeros(T/At,1);
% ref=-100;
% 
% %Datos
% a=0.05;
% b=5;
% c=100;
% w=3;
% 
% %Condiciones iniciales
% 
% alfa(1)=0; tita_p(1)=0; tita(1)=0; h(1)=-500;
% 
% %Matrices
% A= [-a a 0 0 ;0 0 1 0; w^2 -w^2 0 0; c 0 0 0] ;
% B= [0; 0; w^2*b; 0];
% C= [0 0 0 1];
% D= [0];
% x= [alfa(1);tita_p(1);tita(1);h(1)]; 
% M=[B A*B A^2*B A^3*B ];%Matriz Controlabilidad
% 
% %Cálculo del controlador por asignación de polos
% auto_val=eig(A);
% c_ai=conv(conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]),[1 -auto_val(4)]);
% W=[c_ai(4) c_ai(3) c_ai(2) 1;c_ai(3) c_ai(2) 1 0;c_ai(2) 1 0 0;1 0 0 0];
% T=M*W;
% A_controlable=inv(T)*A*T %Verificación de que T esté bien
% 
% %CONTROLADOR Ubicación de los polos de lazo cerrado en mui :
% mui(1)=-15+15i;mui(2)=-15-15i; mui(3)=-.5 + .5i;mui(4)=-.5 - .5i;
% alfa_i=conv(conv(conv([1 -mui(4)],[1 -mui(3)]),[1 -mui(2)]),[1 -mui(1)]);
% K=fliplr(alfa_i(2:5)-c_ai(2:5))*inv(T);
% Gj=-inv(C*inv(A-B*K)*B);
% eig(A-B*K)
% 
% tve(1)=0;u(1)=0;
% for jj=1:Kmax-1
%  
%  estado=[alfa(jj); tita_p(jj); tita(jj); h(jj)];
%  u(jj)=-K*estado+Gj*ref; color='b'; %Sin Observador
%  
%  alfa_p=a*(tita(jj)-alfa(jj));
%  alfa(jj+1)=alfa(jj)+At*alfa_p;
%  tita_pp=-w^2*(tita(jj)-alfa(jj)-b*u(jj));
%  tita_p(jj+1)=tita_p(jj)+At*tita_pp;
%  tita(jj+1)=tita(jj)+At*tita_p(jj);
%  h_p(jj+1)=c*alfa(jj);
%  h(jj+1)=h(jj)+At*h_p(jj);
%  
%  y_sal(jj+1)=C*estado; % lo mismo que h
% 
%  tve(jj+1)=tve(jj)+At;
% end
% figure;
% subplot(5,1,1);plot(tve,alfa);grid on; title('Angulo alfa');xlim([0 70]); hold on%plot(t,y);
% subplot(5,1,2);plot(tve,tita);grid on; title('angulo tita');xlim([0 70]); hold on
% subplot(5,1,3);plot(tve,tita_p);grid on; title('Derivada de tita');xlim([0 70]); hold on
% subplot(5,1,4);plot(tve,y_sal);grid on; title('Altura ; y_s_a_l');xlim([0 70]); hold on
% subplot(5,1,5);plot(t,u);grid on; title('u');xlim([0 70]); hold on
% xlabel('Tiempo en segundos')

% % %--------------------------------------------------------------------------
% % %Intento 1 : Punto 3/ Avion con controlador,integrador y referencia.
% % %-------------------------------------------------------------------------
% clear all;
% T=70; Kmax=100000; At=T/Kmax;
% t=0:At:T-At;
% u=[sign(t)]; 
% e=zeros(T/At,1);
% ref=100;
% 
% %Datos
% a=0.05;
% b=5;
% c=100;
% w=3;
% 
% %Condiciones iniciales
% alfa(1)=0; tita_p(1)=0; tita(1)=0; h(1)=-500;
% 
% %Matrices
% A= [-a a 0 0 ;0 0 1 0; w^2 -w^2 0 0; c 0 0 0] ;
% B= [0; 0; w^2*b; 0];
% C= [0 0 0 1];
% D= [0];
% x= [alfa(1);tita_p(1);tita(1);h(1)]; 
% M=[B A*B A^2*B A^3*B ];%Matriz Controlabilidad
% 
% % Construcción del sistema ampliado
% Aa=[A zeros(4,1);-C 0];
% Ba=[B;0];
% Ma=[Ba Aa*Ba Aa^2*Ba Aa^3*Ba Aa^4*Ba];%Matriz Controlabilidad

% %Cálculo del controlador por asignación de polos
% auto_val=eig(Aa);
% c_ai=conv(conv(conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]),[1 -auto_val(4)]),[1 -auto_val(5)]);
% Wa=[c_ai(5) c_ai(4) c_ai(3) c_ai(2) 1;c_ai(4) c_ai(3) c_ai(2) 1 0;c_ai(3) c_ai(2) 1 0 0;c_ai(2) 1 0 0 0;1 0 0 0 0];
% Ta=Ma*Wa;
% A_controlable=inv(Ta)*Aa*Ta; %Verificación de que T esté bien
% %Ubicación de los polos de lazo cerrado en mui:
% mui(1)=-15+15i;mui(2)=-15-15i; mui(3)=-.5 + .5i;mui(4)=-.5 - .5i;mui(5)=-1;
% alfa_ia=conv(conv(conv(conv([1 -mui(3)],[1 -mui(4)]),[1 -mui(2)]),[1 -mui(1)]),[1 -mui(5)]);
% Ka=(alfa_ia(2:6)-c_ai(2:6))*inv(Ta);
% eig(Aa-Ba*Ka)
% K=Ka(1:4);  
% Gj=-inv(C*inv(A-B*K)*B); %<---- Revisar
% KI=-Ka(5)+Gj*ref;
% 
% %----------------------------------------------------------------------------
% tve(1)=0;u(1)=0;
% psi(1)=0;
% for jj=1:Kmax-1
%  
%  estado=[alfa(jj); tita_p(jj); tita(jj); h(jj)];
%  %u(jj)=-K*estado+Gj*ref; color='b'; %Sin Observador
%  psi_p=ref-C* estado;
%  psi(jj+1)=psi(jj)+psi_p*At;
%  u(jj)=-K*estado+KI*psi(jj+1);
%  
%  alfa_p=a*(tita(jj)-alfa(jj));
%  alfa(jj+1)=alfa(jj)+At*alfa_p;
%  tita_pp=-w^2*(tita(jj)-alfa(jj)-b*u(jj));
%  tita_p(jj+1)=tita_p(jj)+At*tita_pp;
%  tita(jj+1)=tita(jj)+At*tita_p(jj);
%  h_p(jj+1)=c*alfa(jj);
%  h(jj+1)=h(jj)+At*h_p(jj);
%  
%  y_sal(jj+1)=C*estado;
%  
% 
%  tve(jj+1)=tve(jj)+At;
% 
% end
% figure;
% subplot(5,1,1);plot(tve,alfa);grid on; title('Angulo alfa');xlim([0 70]); hold on%plot(t,y);
% subplot(5,1,2);plot(tve,tita);grid on; title('angulo tita');xlim([0 70]); hold on
% subplot(5,1,3);plot(tve,y_sal);grid on; title('y_s_a_l');xlim([0 70]); hold on
% subplot(5,1,4);plot(tve,h);grid on; title('altura');xlim([0 70]); hold on
% subplot(5,1,5);plot(t,u);grid on; title('u');xlim([0 70]); hold on
% xlabel('Tiempo en segundos')

%-----------------------------------------------------------------------------
%-----------------------------------------------------------------------------
% A partir de este punto estan los codigos correctos para los puntos 3 y
% 4
%--------------------------------------------------------------------------
%TP2_Punto 3: 
%Avion con observador,FUNCIONA CON OBSERVADOR, pero no funciona SIN OBSERVADOR. 
%Con polos a lazo cerrado en -15+/-15i ; -.5+/-.5i
%-------------------------------------------------------------------------
clear all;
T=70; Kmax=10000; At=T/Kmax;
u=zeros(Kmax);y=u;yp=y; t=0:At:T-At;
u=[sign(t)]; 
e=zeros(T/At,1);
ref=100;

%Condiciones iniciales

alfa(1)=0; tita_p(1)=0; tita(1)=0; h(1)=500;

%Entrada
x1=0;   %alfa
x2=0;   %tita
x3=0;   %titap
x4=500;   %h

%Datos
a=0.05;
b=5;
c=100;
w=3;

%Matrices
A= [-a a 0 0 ;0 0 1 0; w^2 -w^2 0 0; c 0 0 0] ;
B= [0; 0; w^2*b; 0];
C= [0 0 0 1];
D= [0];
x= [x1;x2;x3;x4]; 
M=[B A*B A^2*B A^3*B ];%Matriz Controlabilidad

%Verifico Observabilidad
O=[C; C*A; C*A^2; C*A^3];%Matriz Observabilidad
auto_O=eig(O);
rango_O=rank(O);

%Cálculo del controlador por asignación de polos
auto_val=eig(A);
c_ai=conv(conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]),[1 -auto_val(4)]);
W=[c_ai(4) c_ai(3) c_ai(2) 1;c_ai(3) c_ai(2) 1 0;c_ai(2) 1 0 0;1 0 0 0];
T=M*W;
A_controlable=inv(T)*A*T %Verificación de que T esté bien

%CONTROLADOR Ubicación de los polos de lazo cerrado en mui :
mui(1)=-15-15i;mui(2)=-15+15i; mui(3)=-.5-.5i;mui(4)=-.5 + .5i;
alfa_i=conv(conv(conv([1 -mui(3)],[1 -mui(4)]),[1 -mui(2)]),[1 -mui(1)]);
K=fliplr(alfa_i(2:5)-c_ai(2:5))*inv(T);
%K=(alfa_i(2:5)-c_ai(2:5))*inv(T);
eig(A-B*K)
Gj=-inv(C*inv(A-B*K)*B);
A_O=A';
B_O=C';
M_Dual=[B_O A_O*B_O A_O^2*B_O A_O^3*B_O];%MatrizControlabilidad
alfaO_i=alfa_i;% Ubicacion del Observador

% Controlador Observador Ko
mui_o= mui; %<----Mismos polos a lazo cerrado que el sistema principal
alfaO_i=conv(conv(conv([1 -mui_o(3)],[1 -mui_o(4)]),[1 -mui_o(2)]),[1 -mui_o(1)]);
T_O=M_Dual*W;
Ko=(fliplr(alfaO_i(2:end)-c_ai(2:end))*inv(T_O))';
eig(A_O'-Ko*C) %Verifico que todos los polos estén en el semiplano izquierdo
x_hat=[0;0;0;0]; %Inicializo el Observador


tve(1)=0;u(1)=0;
for jj=1:Kmax-1
%  estado=[alfa(jj); tita_p(jj); tita(jj); h(jj)]; Está incorrecto. 
  estado=[alfa(jj); tita(jj); tita_p(jj);  h(jj)];

  
  %u(jj)=-K*estado+Gj*ref; color='b'; %Sin Observador
  u(jj)=-K*x_hat+Gj*ref; color='r'; %Con Observador
  
 %Integracion de Euler
  alfa_p=a*(tita(jj)-alfa(jj));
  alfa(jj+1)=alfa(jj)+At*alfa_p;
  tita_pp=-w^2*(tita(jj)-alfa(jj)-b*u(jj));
  tita_p(jj+1)=tita_p(jj)+At*tita_pp;
  tita(jj+1)=tita(jj)+At*tita_p(jj);
  h_p(jj+1)=c*alfa(jj);
  h(jj+1)=h(jj)+At*h_p(jj);

  y_sal(jj)=C*estado;
%________OBSERVADOR__________
 y_sal_O(jj)=C*x_hat;
 x_hatp=A*x_hat+B*u(jj)+Ko*(y_sal(jj)-y_sal_O(jj));
 x_hat=x_hat+At*x_hatp;
 %--------------------
end
% t=1:jj;t=t*At;
figure;
subplot(5,1,1);plot(t,alfa,color);grid on; title('Angulo alfa');xlim([0 70]); hold on
subplot(5,1,2);plot(t,tita,color);grid on; title('angulo tita');xlim([0 70]); hold on
subplot(5,1,3);plot(t,tita_p,color);grid on; title('derivada de tita');xlim([0 70]); hold on
subplot(5,1,4);plot(t(1:end-1),y_sal,color);grid on; title('altura');xlim([0 70]); hold on
subplot(5,1,5);plot(t,u,color);grid on; title('Entrada');xlim([0 70]); hold on
xlabel('Tiempo en segundos')

%--------------------------------------------------------------------------
%TP2_Punto 4: 
%Avion con observador,FUNCIONA CON OBSERVADOR, pero no funciona SIN OBSERVADOR. 
%Con polos a lazo cerrado mui(1)=-50;mui(2)=-30; mui(3)=-1 + 1i;mui(4)=-1 - 1i;
%-------------------------------------------------------------------------
% clear all;
% T=70; Kmax=10000; At=T/Kmax;
% u=zeros(Kmax);y=u;yp=y; t=0:At:T-At;
% u=[sign(t)]; 
% e=zeros(T/At,1);
% ref=100;
% 
% %Condiciones iniciales
% 
% alfa(1)=0; tita_p(1)=0; tita(1)=0; h(1)=500;
% 
% %Entrada
% x1=0;   %alfa
% x2=0;   %tita
% x3=0;   %titap
% x4=500;   %h
% 
% %Datos
% a=0.05;
% b=5;
% c=100;
% w=3;
% 
% %Matrices
% A= [-a a 0 0 ;0 0 1 0; w^2 -w^2 0 0; c 0 0 0] ;
% B= [0; 0; w^2*b; 0];
% C= [0 0 0 1];
% D= [0];
% x= [x1;x2;x3;x4]; 
% M=[B A*B A^2*B A^3*B ];%Matriz Controlabilidad
% 
% %Verifico Observabilidad
% O=[C; C*A; C*A^2; C*A^3];%Matriz Observabilidad
% auto_O=eig(O);
% rango_O=rank(O);
% 
% %Cálculo del controlador por asignación de polos
% auto_val=eig(A);
% c_ai=conv(conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]),[1 -auto_val(4)]);
% W=[c_ai(4) c_ai(3) c_ai(2) 1;c_ai(3) c_ai(2) 1 0;c_ai(2) 1 0 0;1 0 0 0];
% T=M*W;
% A_controlable=inv(T)*A*T %Verificación de que T esté bien
% 
% %CONTROLADOR Ubicación de los polos de lazo cerrado en mui :
% mui(1)=-50;mui(2)=-30; mui(3)=-1 + 1i;mui(4)=-1 - 1i;
% alfa_i=conv(conv(conv([1 -mui(3)],[1 -mui(4)]),[1 -mui(2)]),[1 -mui(1)]);
% K=fliplr(alfa_i(2:5)-c_ai(2:5))*inv(T);
% %K=(alfa_i(2:5)-c_ai(2:5))*inv(T);
% eig(A-B*K)
% Gj=-inv(C*inv(A-B*K)*B);
% A_O=A';
% B_O=C';
% M_Dual=[B_O A_O*B_O A_O^2*B_O A_O^3*B_O];%MatrizControlabilidad
% alfaO_i=alfa_i;% Ubicacion del Observador
% 
% % Controlador Observador Ko
% mui_o= mui; %<----Mismos polos a lazo cerrado que el sistema principal
% alfaO_i=conv(conv(conv([1 -mui_o(3)],[1 -mui_o(4)]),[1 -mui_o(2)]),[1 -mui_o(1)]);
% T_O=M_Dual*W;
% Ko=(fliplr(alfaO_i(2:end)-c_ai(2:end))*inv(T_O))';
% eig(A_O'-Ko*C) %Verifico que todos los polos estén en el semiplano izquierdo
% x_hat=[0;0;0;0]; %Inicializo el Observador
% 
% 
% tve(1)=0;u(1)=0;
% for jj=1:Kmax-1
%   estado=[alfa(jj); tita_p(jj); tita(jj); h(jj)];
%   
%   %u(jj)=-K*estado+Gj*ref; color='b'; %Sin Observador
%   u(jj)=-K*x_hat+Gj*ref; color='r'; %Con Observador
%   
%  %Integracion de Euler
%   alfa_p=a*(tita(jj)-alfa(jj));
%   alfa(jj+1)=alfa(jj)+At*alfa_p;
%   tita_pp=-w^2*(tita(jj)-alfa(jj)-b*u(jj));
%   tita_p(jj+1)=tita_p(jj)+At*tita_pp;
%   tita(jj+1)=tita(jj)+At*tita_p(jj);
%   h_p(jj+1)=c*alfa(jj);
%   h(jj+1)=h(jj)+At*h_p(jj);
% 
%   y_sal(jj)=C*estado;
% %________OBSERVADOR__________
%  y_sal_O(jj)=C*x_hat;
%  x_hatp=A*x_hat+B*u(jj)+Ko*(y_sal(jj)-y_sal_O(jj));
%  x_hat=x_hat+At*x_hatp;
%  %--------------------
% end
% % t=1:jj;t=t*At;
% figure;
% subplot(5,1,1);plot(t,alfa,color);grid on; title('Angulo alfa');xlim([0 70]); hold on
% subplot(5,1,2);plot(t,tita,color);grid on; title('angulo tita');xlim([0 70]); hold on
% subplot(5,1,3);plot(t,tita_p,color);grid on; title('derivada de tita');xlim([0 70]); hold on
% subplot(5,1,4);plot(t(1:end-1),y_sal,color);grid on; title('altura');xlim([0 70]); hold on
% subplot(5,1,5);plot(t,u,color);grid on; title('Entrada');xlim([0 70]); hold on
% xlabel('Tiempo en segundos')

