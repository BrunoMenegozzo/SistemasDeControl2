p1= 3 
p2= -1
K= 5
G=zpk([],[p1 p2],K)
% sisotool(G)
% Exportar el controlador (C) 
% 
Kc= 0.8583 % ganancia
a= 1% cero del controlador con signo invertido
%  M=1 % ganancia rele
%  T=0.1 % histeresis
% lineal=1 % simula control lineal
M=Kc % ganancia rele = +-ganancia Kc
T=K*Kc
%lineal=0% simula no lineal
sim('bang_bang_hist_DI_PD')
subplot(1,3,1)
plot(tout,yout(:,1));title('Error');xlabel('Tiempo [Seg.]'); % error
grid on
subplot(1,3,2)
plot(yout(:,1),yout(:,3));title('Plano de Fase');xlabel('Error');ylabel('Derivada de Error'); % plano de fases: eje x error, eje y derivada del error
grid on
subplot(1,3,3)
plot(tout,yout(:,2));title('Señal de Control');xlabel('Tiempo [Seg.]'); % señal de control
grid on