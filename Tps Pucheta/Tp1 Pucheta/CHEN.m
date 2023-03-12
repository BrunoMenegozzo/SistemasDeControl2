% % Extraido de "Identification for the second-order systems based on the step response"
% % Lei Chena, Junhong Li b,c, Ruifeng Ding

% % CIRCUITO RLC-CHEN

clear all; clc

StepAmplitude = 1;

ii=0; % for t_inic=10:15
ii=ii+1;

%selecciono 3 puntos de la tabla
t_t1=0.003;		
y_t1=5.20835075;

t_2t1=0.006; 	
y_2t1=8.602;

t_3t1=0.009; 
y_3t1=10.301;


y_end=12;
K=y_end/StepAmplitude;
k1=(1/StepAmplitude)*y_t1/K-1; %Afecto el valor del Escalon
k2=(1/StepAmplitude)*y_2t1/K-1;
k3=(1/StepAmplitude)*y_3t1/K-1;

be=4*k1^3*k3-3*k1^2*k2^2-4*k2^3+k3^2+6*k1*k2*k3;
alfa1=(k1*k2+k3-sqrt(be))/(2*(k1^2+k2));
alfa2=(k1*k2+k3+sqrt(be))/(2*(k1^2+k2));
beta=(k1+alfa2)/(alfa1-alfa2);

T1_ang=-t_t1/log(alfa1);
T2_ang=-t_t1/log(alfa2);
T3_ang=beta*(T1_ang-T2_ang)+T1_ang;
T1(ii)=T1_ang;
T2(ii)=T2_ang;
T3(ii)=T3_ang;
T3_ang=sum(T3/length(T3));
T2_ang=sum(T2/length(T2));
T1_ang=sum(T1/length(T1));
sys_G_ang=tf(K*[T3_ang 1],conv([T1_ang 1],[T2_ang 1]))%Funcion de transferencia obtenida

data=xlsread("Curvas_Medidas_RLC.xls");
t_t=data(:,1)-0.01;
Vc_t=data(:,3);

figure(1);hold on;
step(StepAmplitude*sys_G_ang);ylim([0 13]);grid on;hold on; %Respuesta al escalon de ft obtenida
plot(t_t,Vc_t,'r'); %Grafico de datos dados por tabla
xlim([0 0.04]);ylim([0 13]);grid on;hold on;legend('Identificada','Real')

