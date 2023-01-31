T=5; Kmax = 5000; At= T/Kmax; t= 0:At:T-At;
[u0,t0] = gensig("square",40,20,1e-3);
u=u0+1;
t=t0
%Entrada
x1=0
x2=0
x3=0
x4=0

%Datos
a=0.05
b=5
c=50
w=2

%Matrices
A= [-a a 0 0 ;0 0 1 0; w^2 -w^2 0 0; c 0 0 0] 
B= [0; 0; w^2*b; 0]
C= [1 0 0 0]
D= [0]
x0= [x1;x2;x3;x4] 

Sys = ss(A,B,C,D)
[y,x]=lsim(A,B,C,D,u,t,x0)
alfa=x(:,1)
tita=x(:,2)
tita_p=x(:,3)
h=x(:,4)
subplot(5,1,1);plot(t,alfa);grid on; title('Angulo alfa');xlim([0 20]); hold on%plot(t,y);
subplot(5,1,2);plot(t,tita);grid on; title('angulo tita');xlim([0 20]); hold on
subplot(5,1,3);plot(t,tita_p);grid on; title('derivada de tita');xlim([0 20]); hold on
subplot(5,1,4);plot(t,h);grid on; title('altura');xlim([0 20]); hold on
subplot(5,1,5);plot(t,u);grid on; title('Entrada');xlim([0 20]); hold on
xlabel('Tiempo en segundos')
