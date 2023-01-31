clc;clear all;
m=.1;Fricc=0.1; long=1.2;g=9.8;M=.5;
h=0.0001;tiempo=(10/h);p_pp=0;tita_pp=0; t=0:h:tiempo*h;
omega(1)=0;p_p=0:h:tiempo*h; u=linspace(0,0,tiempo+1);
%Condiciones iniciales
alfa(1)=-0.01; color='b';
p(1)=0; p_p(1)=0; u(1)=0; p(1)=0; i=1;
%Versión linealizada en el equilibrio estable. Sontag Pp 104.
%estado=[p(i); p_p(i); alfa(i); omega(i)]
Mat_A=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 Fricc/(long*M) g*(m+M)/(long*M) 0]
Mat_B=[0; 1/M; 0; -1/(long*M)]
X0=[0 0 pi 0]';x=[0 0 alfa(1) 0]';
while(i<(tiempo+1))
 %Variables del sistema no lineal
 estado=[p(i); p_p(i); alfa(i); omega(i)];
 u(i)=0;
 %Sistema no lineal
 p_pp=(1/(M+m))*(u(i)-m*long*tita_pp*cos(alfa(i))+m*long*omega(i)^2*sin(alfa(i))- Fricc*p_p(i));
 tita_pp=(1/long)*(g*sin(alfa(i))-p_pp*cos(alfa(i)));
 p_p(i+1)=p_p(i)+h*p_pp;
 p(i+1)=p(i)+h*p_p(i);
 omega(i+1)=omega(i)+h*tita_pp;
 alfa(i+1)=alfa(i)+h*omega(i);
 %Variables del sistema lineal
 pl(i)=x(1); p_pl(i)=x(2);alfal(i)=x(3);omegal(i)=x(4);
 %Sistema lineal
 xp=Mat_A*(x-X0)+Mat_B*u(i);
 x=x+h*xp;
 i=i+1;
end
pl(i)=x(1); p_pl(i)=x(2);alfal(i)=x(3);omegal(i)=x(4);
figure(1);hold on;
subplot(3,2,1);plot(t,omega,color);grid on; title('Velocidad Ángulo');hold on;plot(t,omegal,'r');
subplot(3,2,2);plot(t,alfa,color);hold on;plot(t,pi*ones(size(t)),'r');plot(t,alfal,'r');
grid on;title('Ángulo');hold on;
subplot(3,2,3); plot(t,p,color);grid on;title('Posición carro');hold on;plot(t,pl,'r');
subplot(3,2,4);plot(t,p_p,color);grid on;title('Velocidad carro');hold on;plot(t,p_pl,'r');
subplot(3,1,3);plot(t,u,color);grid on;title('Acción de control');xlabel('Tiempo enSeg.');hold on;
figure(2);hold on;
subplot(2,2,1);plot(alfa,omega,color);grid on;xlabel('Ángulo');ylabel('Velocidadangular');hold on;
subplot(2,2,1);plot(alfal,omegal,'r');
subplot(2,2,2);plot(p,p_p,color);grid on;xlabel('Posición carro');ylabel('Velocidadcarro');hold on;
subplot(2,2,2);plot(pl,p_pl,'r');