%---------------------------------------------------------------------------
%Tp2_Punto 1.2
%---------------------------------------------------------------------------
clear all;close all;
X=-[0; 0; 0];ii=0;At=1e-6;titaRef1=-1.57;titaRef2=1.57;tF=0.6;
Kp=4.5;Ki=200;Kd=2e-6; 
Ts=At;
A1=((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
B1=(-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
C1=Kd/Ts;
e=zeros(tF/At,1);
u=12;Tl=0;

%Caracteristicas del motor
Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;

%Matrices
A=[-Ra/Laa 0 -Km/Laa; 0 0 1;Ki/J 0 -B/J];
B=[1/Laa 0; 0 0;0 -1/J];
C=[0 1 0];
D=[0];

%Verifico Controlabilidad y Observabilidad
M=[B A*B A^2*B];%Matriz Controlabilidad <==== LA MATRIZ ES DE 3X6, NO ES INVERTIBLE
rango_M=rank(M); 
O=[C; C*A; C*A^2];%Matriz Observabilidad
auto_O=eig(O);
rango_O=rank(O);

%Cálculo del controlador por asignación de polos
auto_val=eig(A); %<--- De |sI-A| se puede obtener ecu. caracteristic: s^3+151,9*s^2+17,48*1298K*s 
%c_ai=conv(conv([1 -auto_val(1)],[1 -auto_val(2)]));%<----Consultar 
W=[22.69e6 151.9e3 1;151.9e3 1 0;1 0 0];
T=M*W;             %<==== NO PUEDEN MULTIPLICARSE POR SUS DIMENSIONES
A_controlable=inv(T)*A*T %Verificación de que T esté bien

for t=0:At:tF
 ii=ii+1;k=ii+2;
 Tl_arreglo=[0 ((1.15e-3/2)*sign(sin(10.472*t))+(1.15e-3/2))];
 Tl=Tl_arreglo(2);
 U = [u ; Tl]; 
 
 xp=A*X+B*U;
 Y =C*X+D*U;
 X=X+xp*At;
 
 
 if(Tl<=1e-3)
     e(k)=titaRef1-X(1); %ERROR en punto1
 else
     e(k)=titaRef2-X(1); %ERROR en punto 2 
 end
 u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
 x1(ii)=X(1);%tita
 x2(ii)=X(2);%omega
 x3(ii)=X(3);%ia
 torque(ii)=Tl;%Torque Tl
 acc(ii)=u;
end

%Grafico
t=0:At:tF;
figure;
subplot(3,1,1);hold on;
plot(t,x1(1:numel(t)),'r');title('Tita');xlabel('Tiempo [Seg.]');
subplot(3,1,2);hold on;
plot(t,x2(1:numel(t)),'r');title('Salida y, \omega_t');xlabel('Tiempo [Seg.]');
subplot(3,1,3);hold on;
plot(t,x3(1:numel(t)),'r');title('Ia');xlabel('Tiempo [Seg.]');

figure;
subplot(2,1,1);hold on;
plot(t,torque(1:numel(t)),'r');title('Torque de carga');
xlabel('Tiempo [Seg.]');
subplot(2,1,2);hold on;
plot(t,acc(1:numel(t)),'r');title('Accion de control');
xlabel('Tiempo [Seg.]');
