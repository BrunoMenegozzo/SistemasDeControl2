% %-----------------------------------------------------------------------------
% %TP2_ Punto 5 / Controlador con distintos angulos de inicio para probar el limite
% %del sistema
% %----------------------------------------------------------------------------
% clc;clear all;
% m=.1;Fricc=0.1; long=0.6;g=9.8;Ma=.5;
% h=0.0001;tiempo=(30/h);p_pp=0;tita_pp=0; t=0:h:tiempo*h;
% omega(1)=0;p_p=0:h:tiempo*h; u=linspace(0,0,tiempo+1);
% 
% 
% %Condiciones iniciales
%  alfa(1)=.1; color='r';
%  %alfa(1)=.5; color='g';
%  %alfa(1)=.8; color='b';
%  %alfa(1)=1; color='m';
%  %alfa(1)=1.055; color='r';
% 
% ref_p=10; %Referencia
% 
% p(1)=0; p_p(1)=0; u(1)=0; p(1)=0; i=1;
% 
% %estado=[p(i); p_p(i); alfa(i); omega(i)]
% A=[0 1 0 0;0 -Fricc/Ma -m*g/Ma 0; 0 0 0 1; 0 Fricc/(long*Ma) g*(m+Ma)/(long*Ma) 0];
% B=[0; 1/Ma; 0; -1/(long*Ma)];
% C= [1 0 0 0];
% D= [0];
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
% mui(1)=-2;mui(2)=-2; mui(3)=-2+1i;mui(4)=conj(mui(3));
% alfa_i=conv(conv(conv([1 -mui(3)],[1 -mui(4)]),[1 -mui(2)]),[1 -mui(1)]);
% K=(alfa_i(2:5)-c_ai(2:5))*inv(T);
% Gj=-inv(C*inv(A-B*K)*B);
% eig(A-B*K)
% 
% X0=[0 0 pi 0]';x=[0 0 alfa(1) 0]';
% while(i<(tiempo+1))
%  %Variables del sistema no lineal
%  estado=[p(i); p_p(i); alfa(i); omega(i)];
%  u(i)=-K*estado+Gj*ref_p;
%  %Sistema no lineal
%  p_pp=(1/(Ma+m))*(u(i)-m*long*tita_pp*cos(alfa(i))+m*long*omega(i)^2*sin(alfa(i))- Fricc*p_p(i));
%  tita_pp=(1/long)*(g*sin(alfa(i))-p_pp*cos(alfa(i)));
%  p_p(i+1)=p_p(i)+h*p_pp;
%  p(i+1)=p(i)+h*p_p(i);
%  omega(i+1)=omega(i)+h*tita_pp;
%  alfa(i+1)=alfa(i)+h*omega(i);
% %  %Variables del sistema lineal
% %  pl(i)=x(1); p_pl(i)=x(2);alfal(i)=x(3);omegal(i)=x(4);
% %  %Sistema lineal
% %  xp=A*(x-X0)+B*u(i);
% %  x=x+h*xp;
%  i=i+1;
% end
% % pl(i)=x(1); p_pl(i)=x(2);alfal(i)=x(3);omegal(i)=x(4);
% figure(1);hold on;
% subplot(3,2,1);plot(t,omega,color);grid on; title('Velocidad Ángulo');hold on;%plot(t,omegal,'r');
% subplot(3,2,2);plot(t,alfa,color);hold on;%plot(t,pi*ones(size(t)),'r');%plot(t,alfal,'r');
% grid on;title('Ángulo');hold on;
% subplot(3,2,3); plot(t,p,color);grid on;title('Posición carro');hold on;%plot(t,pl,'r');
% subplot(3,2,4);plot(t,p_p,color);grid on;title('Velocidad carro');hold on;%plot(t,p_pl,'r');
% subplot(3,1,3);plot(t,u,color);grid on;title('Acción de control');xlabel('Tiempo enSeg.');hold on;
% figure(2);hold on;
% subplot(2,2,1);plot(alfa,omega,color);grid on;xlabel('Ángulo');ylabel('Velocidadangular');hold on;
% % subplot(2,2,1);plot(alfal,omegal,'r');
% subplot(2,2,2);plot(p,p_p,color);grid on;xlabel('Posición carro');ylabel('Velocidadcarro');hold on;
% % subplot(2,2,2);plot(pl,p_pl,'r');

% %-----------------------------------------------------------------------------
% %TP2_ Punto 6/ Ahora con un observador que solo puede ver el desplazamiento
% %----------------------------------------------------------------------------
% clc;clear all;
% m=.1;Fricc=0.1; long=0.6;g=9.8;Ma=.5;
% h=0.0001;tiempo=(30/h);p_pp=0;tita_pp=0; t=0:h:tiempo*h;
% omega(1)=0;p_p=0:h:tiempo*h; u=linspace(0,0,tiempo+1);
% 
% 
% %Condiciones iniciales
%  alfa(1)=.1; %color='r';
%  %alfa(1)=.5; color='g';
%  %alfa(1)=.8; color='b';
%  %alfa(1)=1; color='m';
%  %alfa(1)=1.055; color='r';
% 
% ref_p=10; %Referencia
% 
% p(1)=0; p_p(1)=0; u(1)=0; p(1)=0; i=1;
% 
% %estado=[p(i); p_p(i); alfa(i); omega(i)]
% A=[0 1 0 0;0 -Fricc/Ma -m*g/Ma 0; 0 0 0 1; 0 Fricc/(long*Ma) g*(m+Ma)/(long*Ma) 0];
% B=[0; 1/Ma; 0; -1/(long*Ma)];
% C= [1 0 0 0];
% D= [0];
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
% mui(1)=-2;mui(2)=-2; mui(3)=-2+1i;mui(4)=conj(mui(3));
% alfa_i=conv(conv(conv([1 -mui(3)],[1 -mui(4)]),[1 -mui(2)]),[1 -mui(1)]);
% K=fliplr(alfa_i(2:5)-c_ai(2:5))*inv(T);
% Gj=-inv(C*inv(A-B*K)*B);
% eig(A-B*K)
% 
% %Matrices Observador
% A_O=A';
% B_O=C';
% M_Dual=[B_O A_O*B_O A_O^2*B_O A_O^3*B_O];%MatrizControlabilidad
% alfaO_i=alfa_i;% Ubicacion del Observador
% 
% % Controlador Observador Ko
% mui_o= real(mui)*10; %<----Mismos polos a lazo cerrado que el sistema principal
% alfaO_i=conv(conv(conv([1 -mui_o(3)],[1 -mui_o(4)]),[1 -mui_o(2)]),[1 -mui_o(1)]);
% T_O=M_Dual*W;
% Ko=(fliplr(alfaO_i(2:end)-c_ai(2:end))*inv(T_O))';
% eig(A_O'-Ko*C) %Verifico que todos los polos estén en el semiplano izquierdo
% x_hat=[0;0;0;0]; %Inicializo el Observador
% 
% 
% X0=[0 0 pi 0]';x=[0 0 alfa(1) 0]';
% while(i<(tiempo+1))
%  %Variables del sistema no lineal
%  estado=[p(i); p_p(i); alfa(i); omega(i)];
%  u(i)=-K*x_hat+Gj*ref_p;color='b'; %Con Observador
%  %u(i)=-K*estado+Gj*ref_p;color='r';
%  %Sistema no lineal
%  p_pp=(1/(Ma+m))*(u(i)-m*long*tita_pp*cos(alfa(i))+m*long*omega(i)^2*sin(alfa(i))- Fricc*p_p(i));
%  tita_pp=(1/long)*(g*sin(alfa(i))-p_pp*cos(alfa(i)));
%  p_p(i+1)=p_p(i)+h*p_pp;
%  p(i+1)=p(i)+h*p_p(i);
%  omega(i+1)=omega(i)+h*tita_pp;
%  alfa(i+1)=alfa(i)+h*omega(i);
%  
% %________OBSERVADOR__________
%  y_sal_O(i)=C*x_hat;
%  y_sal(i)=C*estado;
%  x_hatp=A*x_hat+B*u(i)+Ko*(y_sal(i)-y_sal_O(i));
%  x_hat=x_hat+h*x_hatp;
%  %--------------------
%  
%  i=i+1;
% end
% 
% figure(1);hold on;
% subplot(3,2,1);plot(t,omega,color);grid on; title('Velocidad Ángulo');hold on;
% subplot(3,2,2);plot(t,alfa,color);hold on;
% grid on;title('Ángulo');hold on;
% subplot(3,2,3); plot(t,p,color);grid on;title('Posición carro');hold on;
% subplot(3,2,4);plot(t,p_p,color);grid on;title('Velocidad carro');hold on;
% subplot(3,1,3);plot(t,u,color);grid on;title('Acción de control');xlabel('Tiempo enSeg.');hold on;
% figure(2);hold on;
% subplot(2,2,1);plot(alfa,omega,color);grid on;xlabel('Ángulo');ylabel('Velocidadangular');hold on;
% subplot(2,2,2);plot(p,p_p,color);grid on;xlabel('Posición carro');ylabel('Velocidadcarro');hold on;

%-----------------------------------------------------------------------------
%TP2_5 / Controlador con distintos angulos de inicio para probar el limite
%del sistema
%----------------------------------------------------------------------------
% clc;clear all;
% m=.1;Fricc=0.1; long=0.6;g=9.8;Ma=.5;
% h=0.0001;tiempo=(50/h);p_pp=0;tita_pp=0; t=0:h:tiempo*h;
% omega(1)=0;p_p=0:h:tiempo*h; u=linspace(0,0,tiempo+1);
% 
% 
% %Condiciones iniciales
%  alfa(1)=pi; color='r';
%  %alfa(1)=.5; color='g';
%  %alfa(1)=.8; color='b';
%  %alfa(1)=1; color='m';
%  %alfa(1)=1.055; color='r';
% 
% ref1_p=-10; %Referencia 1 
% ref2_p=0; %Referencia 2
% 
% p(1)=0; p_p(1)=0; u(1)=0; p(1)=0; i=1;
% 
% %estado=[p(i); p_p(i); alfa(i); omega(i)]
% %Matrices para esquilibrio estable
% A=[0 1 0 0;0 -Fricc/Ma -m*g/Ma 0; 0 0 0 1; 0 -Fricc/(long*Ma) -(g*(m+Ma)/(long*Ma)) 0];
% B=[0; 1/Ma; 0; 1/(long*Ma)];
% C= [1 0 0 0];
% D= [0];
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
% %  mui(1)=-8;mui(2)=-8; mui(3)=-.5+.1i;mui(4)=conj(mui(3));
% mui(1)=-1.5;mui(2)=-5; mui(3)=-1.5;mui(4)=-5;%conj(mui(3));
% alfa_i=conv(conv(conv([1 -mui(3)],[1 -mui(4)]),[1 -mui(2)]),[1 -mui(1)]);
% % K=fliplr(alfa_i(2:5)-c_ai(2:5))*inv(T);
% K=(alfa_i(2:5)-c_ai(2:5))*inv(T);
% Gj=-inv(C*inv(A-B*K)*B);
% eig(A-B*K)
% 
% 
% while(i<(tiempo+1))
%  %Variables del sistema no lineal
%  estado=[p(i); p_p(i); alfa(i); omega(i)];
%  u(i)=-K*estado+Gj*ref1_p;
%  
% %  if(i<=tiempo/2)
% %      u(i)=-K*estado+Gj*ref1_p; %posicion de referencia 10m
% %  else
% %      u(i)=-K*estado+Gj*ref2_p; %posicion de referencia 0m
% %   end
%  
%  %Sistema no lineal
%  p_pp=(1/(Ma+m))*(u(i)-m*long*tita_pp*cos(alfa(i))+m*long*omega(i)^2*sin(alfa(i))- Fricc*p_p(i));
%  tita_pp=(1/long)*(g*sin(alfa(i))-p_pp*cos(alfa(i)));
%  p_p(i+1)=p_p(i)+h*p_pp;
%  p(i+1)=p(i)+h*p_p(i);
%  omega(i+1)=omega(i)+h*tita_pp;
%  alfa(i+1)=alfa(i)+h*omega(i);
% 
%    
%  
%  i=i+1;
% end
% 
% figure(1);hold on;
% subplot(3,2,1);plot(t,omega,color);grid on; title('Velocidad Ángulo');hold on;
% subplot(3,2,2);plot(t,alfa,color);hold on;
% grid on;title('Ángulo');hold on;
% subplot(3,2,3); plot(t,p,color);grid on;title('Posición carro');hold on;
% subplot(3,2,4);plot(t,p_p,color);grid on;title('Velocidad carro');hold on;
% subplot(3,1,3);plot(t,u,color);grid on;title('Acción de control');xlabel('Tiempo enSeg.');hold on;
% figure(2);hold on;
% subplot(2,2,1);plot(alfa,omega,color);grid on;xlabel('Ángulo');ylabel('Velocidadangular');hold on;
% subplot(2,2,2);plot(p,p_p,color);grid on;xlabel('Posición carro');ylabel('Velocidadcarro');hold on;

% %-----------------------------------------------------------------------------
% %TP2_ Punto 7 / Controlador con integrador funciona mucho mejor pero el angulo no
% %se mantiene en pi, siempre da una "vuelta" y se estabiliza en -pi.
% %CONSULTAR
% %----------------------------------------------------------------------------
% 
% clc;clear all;
% m=.1;Fricc=0.1; long=0.6;g=9.8;M=.5;
% h=0.0001;tiempo=(50/h);p_pp=0;tita_pp=0; t=0:h:tiempo*h;
% omega=0:h:tiempo*h; alfa=0:h:tiempo*h; p=0:h:tiempo*h;
% p_p=0:h:tiempo*h; u=linspace(0,0,tiempo+1);
% %Condiciones iniciales
% alfa(1)=pi; color='r';
% % alfa(1)=.2; color='g';
% % alfa(1)=.7; color='b';
% ref=10;
% omega(1)=0; p_p(1)=0; u(1)=0; p(1)=0; i=1;indice=0;
% %Versión linealizada en el equilibrio estable
% % estado=[p(i); p_p(i); alfa(i); omega(i)]
% Mat_A=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 -Fricc/(long*M) -(g*(m+M)/(long*M)) 0];
% Mat_B=[0; 1/M; 0; 1/(long*M)];
% Mat_C= [1 0 0 0];
% Mat_D= [0];
% %Mat_M=[B A*B A^2*B A^3*B ];%Matriz Controlabilidad
% 
% % Construcción del sistema ampliado
% Mat_Aa=[Mat_A zeros(4,1);-Mat_C 0];
% Mat_Ba=[Mat_B;0];
% Mat_Ma=[Mat_Ba Mat_Aa*Mat_Ba Mat_Aa^2*Mat_Ba Mat_Aa^3*Mat_Ba Mat_Aa^4*Mat_Ba];%Matri Controlabilidad
% %Cálculo del controlador por asignación de polos
% auto_val=eig(Mat_Aa);
% c_ai=conv(conv(conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]),[1 -auto_val(4)]),[1 -auto_val(5)]);
% Mat_Wa=[c_ai(5) c_ai(4) c_ai(3) c_ai(2) 1;c_ai(4) c_ai(3) c_ai(2) 1 0;c_ai(3) c_ai(2) 1 0 0;c_ai(2) 1 0 0 0;1 0 0 0 0];
% Mat_Ta=Mat_Ma*Mat_Wa;
% A_controlable=inv(Mat_Ta)*Mat_Aa*Mat_Ta; %Verificación de que T esté bien
% %Ubicación de los polos de lazo cerrado en mui:
% mui(1)=-0.2;mui(2)=-0.2; mui(3)=-10 + 0.01i;mui(4)=conj(mui(3));mui(5)=-1;
% alfa_ia=conv(conv(conv(conv([1 -mui(3)],[1 -mui(4)]),[1 -mui(2)]),[1 -mui(1)]),[1 -mui(5)]);
% Ka=(alfa_ia(2:6)-c_ai(2:6))*inv(Mat_Ta);
% eig(Mat_Aa-Mat_Ba*Ka)
% K=Ka(1:4); KI=-Ka(5); %Los valores del controlador de obtienen del K ampliado
% psi(1)=0;
% while(i<(tiempo+1))
%  estado=[p(i); p_p(i); alfa(i); omega(i)];
%   if(i<=tiempo/2)
%      ref=10; %posicion de referencia 10m
%      m=0.1;
%  else
%      ref=0; %posicion de referencia 0m
%      m=1;
%   end
%  
%  psi_p=ref-Mat_C* estado;
%  psi(i+1)=psi(i)+psi_p*h;
%  u(i)=-K*estado+KI*psi(i+1);
%  
%  
%  %Sistema no lineal
%  p_pp=(1/(M+m))*(u(i)-m*long*tita_pp*cos(alfa(i))+m*long*omega(i)^2*sin(alfa(i))- Fricc*p_p(i));
%  tita_pp=(1/long)*(g*sin(alfa(i))-p_pp*cos(alfa(i)));
%  p_p(i+1)=p_p(i)+h*p_pp;
%  p(i+1)=p(i)+h*p_p(i);
%  omega(i+1)=omega(i)+h*tita_pp;
%  alfa(i+1)=alfa(i)+h*omega(i);
%  y_sal(i)=Mat_C*estado; i=i+1;
% end
% figure(1);hold on;
% subplot(3,2,1);plot(t,omega,color);grid on; title('Velocidad ángulo');hold on;
% subplot(3,2,2);plot(t,alfa,color);grid on;title('Ángulo');hold on;
% subplot(3,2,3); plot(t,p,color);grid on;title('Posición carro');hold on;
% subplot(3,2,4);plot(t,p_p,color);grid on;title('Velocidad carro');hold on;
% subplot(3,1,3);plot(t,u,color);grid on;title('Acción de control');xlabel('Tiempo en Seg.');hold on;
% figure(2);hold on;subplot(2,2,1);plot(alfa,omega,color);grid on;xlabel('Ángulo');ylabel('Velocidad angular');hold on;
% subplot(2,2,2);plot(p,p_p,color);grid on;xlabel('Posicion carro');ylabel('Velocidad carro');hold on;

%-----------------------------------------------------------------------------
%TP2_Punto 8 / Controlador con integrador y observador.
%
% SIN EL OBSERVADOR, cuando el carro va de 0 a 10, el angulo se mueve de pi a -pi y se
% estabiliza ahi, "da una vuelta" y no se como corregirlo. Pero cuando va
% de 10 a 0 no lo hace.
%CON EL OBSERVADOR,cuando el carro va de 0 a 10, el angulo no se mueve, queda en pi
%Pero cuando va de 10 a 0 comienza a oscilar y el angulo aumenta mucho.
% 
%----------------------------------------------------------------------------

clc;clear all;%close all;
m=.1;Fricc=0.1; long=0.6;g=9.8;M=.5;
h=0.0001;tiempo=(20/h);p_pp=0;tita_pp=0; t=0:h:tiempo*h;
omega=0:h:tiempo*h; alfa=0:h:tiempo*h; p=0:h:tiempo*h;
p_p=0:h:tiempo*h; u=linspace(0,0,tiempo+1);
%Condiciones iniciales
alfa(1)=pi; color='r';
% alfa(1)=.2; color='g';
% alfa(1)=.7; color='b';
ref=10;
omega(1)=0; p_p(1)=0; u(1)=0; p(1)=0; i=1;indice=0;
%Versión linealizada en el equilibrio estable
% estado=[p(i); p_p(i); alfa(i); omega(i)]
Mat_A=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 -Fricc/(long*M) -(g*(m+M)/(long*M)) 0];
Mat_B=[0; 1/M; 0; 1/(long*M)];
Mat_C= [1 0 0 0];
Mat_D= [0];
Mat_M=[Mat_B Mat_A*Mat_B Mat_A^2*Mat_B Mat_A^3*Mat_B ];%Matriz Controlabilidad
%%Verifico Observabilidad
Mat_O=[Mat_C; Mat_C*Mat_A; Mat_C*Mat_A^2; Mat_C*Mat_A^3];%Matriz Observabilidad
auto_O=eig(Mat_O);
rango_O=rank(Mat_O);
xOP=[0;0;pi;0];

% %Cálculo del controlador por asignación de polos
auto_val=eig(Mat_A);
c_ai=conv(conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]),[1 -auto_val(4)]);
Mat_W=[c_ai(4) c_ai(3) c_ai(2) 1;c_ai(3) c_ai(2) 1 0;c_ai(2) 1 0 0;1 0 0 0];
Mat_T=Mat_M*Mat_W;
A_controlable=inv(Mat_T)*Mat_A*Mat_T %Verificación de que T esté bien

% Construcción del sistema ampliado
Mat_Aa=[Mat_A zeros(4,1);-Mat_C 0];
Mat_Ba=[Mat_B;0];
Mat_Ma=[Mat_Ba Mat_Aa*Mat_Ba Mat_Aa^2*Mat_Ba Mat_Aa^3*Mat_Ba Mat_Aa^4*Mat_Ba];%Matriz Controlabilidad

%Cálculo del controlador por asignación de polos
auto_val=eig(Mat_Aa);
c_ai=conv(conv(conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]),[1 -auto_val(4)]),[1 -auto_val(5)]);
Mat_Wa=[c_ai(5) c_ai(4) c_ai(3) c_ai(2) 1;c_ai(4) c_ai(3) c_ai(2) 1 0;c_ai(3) c_ai(2) 1 0 0;c_ai(2) 1 0 0 0;1 0 0 0 0];
Mat_Ta=Mat_Ma*Mat_Wa;
A_controlable=inv(Mat_Ta)*Mat_Aa*Mat_Ta; %Verificación de que T esté bien

%Ubicación de los polos de lazo cerrado en mui:
mui(1)=-1.7;mui(2)=-4; mui(3)=-1.0 + 0.4i;mui(4)=conj(mui(3));mui(5)=-10;
%  mui(1)=-20;mui(2)=-.2; mui(3)=-.1;mui(4)=conj(mui(3));mui(5)=-10;
% mui(1)=-50;mui(2)=-500; mui(3)=-10 + 0.05i;mui(4)=conj(mui(3));mui(5)=-1;
alfa_ia=conv(conv(conv(conv([1 -mui(3)],[1 -mui(4)]),[1 -mui(2)]),[1 -mui(1)]),[1 -mui(5)]);
Ka=flip(alfa_ia(2:6)-c_ai(2:6))*inv(Mat_Ta);
eig(Mat_Aa-Mat_Ba*Ka)

%Matrices Observador
Mat_A_O=Mat_A';
Mat_B_O=Mat_C';
M_Dual=[Mat_B_O Mat_A_O*Mat_B_O Mat_A_O^2*Mat_B_O Mat_A_O^3*Mat_B_O];%MatrizControlabilidad
%alfaO_i=alfa_ia;% Ubicacion del Observador

% Controlador Observador Ko
%mui_o= 1*mui; %<----Mismos polos a lazo cerrado que el sistema principal
mui_o=real(mui)*8.7;
alfaO_i=conv(conv(conv([1 -mui_o(3)],[1 -mui_o(4)]),[1 -mui_o(2)]),[1 -mui_o(1)]);
Mat_T_O=M_Dual*Mat_W;
Ko=(fliplr(alfaO_i(2:end)-c_ai(2:end-1))*inv(Mat_T_O))';
eig(Mat_A-Ko*Mat_C) %Verifico que todos los polos estén en el semiplano izquierdo
x_hat=[0;0;0;0]; %Inicializo el Observador


K=Ka(1:4); KI=-Ka(5); %Los valores del controlador de obtienen del K ampliado
psi(1)=0;
while(i<(tiempo+1))
 estado=[p(i); p_p(i); alfa(i); omega(i)];
  if(i<=tiempo/2)
     ref=10; %posicion de referencia 10m
     m=0.1;
%   elseif(i>=tiempo/2 && i<tiempo*3/4)
%       ref=5;
%       m=1;
 else
     ref=0; %posicion de referencia 0m
     m=1;
  end
 
 psi_p= ref-Mat_C* estado;
 psi(i+1)=psi(i)+psi_p*h;
 
 u(i)=-K*(estado-xOP)+KI*psi(i+1);%Sin Observador
% u(i)=-K*(x_hat-xOP)+KI*psi(i+1); %Con Observador

 %Sistema no lineal
 p_pp=(1/(M+m))*(u(i)-m*long*tita_pp*cos(alfa(i))+m*long*omega(i)^2*sin(alfa(i))- Fricc*p_p(i));
 tita_pp=(1/long)*(g*sin(alfa(i))-p_pp*cos(alfa(i)));
 p_p(i+1)=p_p(i)+h*p_pp;
 p(i+1)=p(i)+h*p_p(i);
 omega(i+1)=omega(i)+h*tita_pp;
 alfa(i+1)=alfa(i)+h*omega(i);
 
 
 %________OBSERVADOR__________
 y_sal_O(i)=Mat_C*x_hat;
 y_sal(i)=Mat_C*(estado-xOP);
 x_hatp=Mat_A*(x_hat-xOP)+Mat_B*u(i)+Ko*(y_sal(i)-y_sal_O(i));
 x_hat=x_hat+h*x_hatp;
 i=i+1;
% xOP=[0;0;pi;0];
end
figure(1);hold on; t=1:i;t=t*h;
subplot(3,2,1);plot(t,omega,color);grid on; title('Velocidad ángulo');hold on;
subplot(3,2,2);plot(t,alfa,color);grid on;title('Ángulo');hold on;
subplot(3,2,3); plot(t,p,color);grid on;title('Posición carro');hold on;
subplot(3,2,4);plot(t,p_p,color);grid on;title('Velocidad carro');hold on;
subplot(3,1,3);plot(t,u,color);grid on;title('Acción de control');xlabel('Tiempo en Seg.');hold on;
figure(2);hold on;subplot(2,2,1);plot(alfa,omega,color);grid on;xlabel('Ángulo');ylabel('Velocidad angular');hold on;
subplot(2,2,2);plot(p,p_p,color);grid on;xlabel('Posicion carro');ylabel('Velocidad carro');hold on;
