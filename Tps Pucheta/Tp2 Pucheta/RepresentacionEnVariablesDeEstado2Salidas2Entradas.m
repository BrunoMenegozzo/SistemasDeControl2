% Verificación de la EDO
% Condiciones iniciales nulas.
% clear all;clc;close all;
% pkg load control;
% pkg load signal;
T=15; Kmax=30000; At=T/Kmax;TamanioFuente=12;
u1=zeros(1,Kmax);y1=u1;y2=u1;y1p=y1;y2p=y2; t=0:At:T-At;y1pp=0;
u1=[0 12* sign(cos(1.2*t))];u1p=[0 diff(u1)/At];
u2=[0 1.5* sign(cos(1.3*t+1))];u2p=[ 0 diff(u2)/At];
for jj=1:Kmax-1
 y1pp=(1/4)*(-u1p(jj)+2*u1(jj)+2*u2p(jj)+u2(jj)-4*y1p(jj)-y1(jj));
 y2pp=(1/4)*(-3*u1(jj)+u2(jj)-u2p(jj)-4*y2p(jj)-y2(jj));
 %Integro
 y2p(jj+1) =y2p(jj)+y2pp*At;
 y2(jj+1) =y2(jj) +y2p(jj)*At;
 y1p(jj+1) =y1p(jj)+y1pp*At;
 y1(jj+1) =y1(jj) +y1p(jj)*At;
end
figure;
subplot(2,2,1);plot(t,y1,'.k'); title('y_1'),hold on
subplot(2,2,2);plot(t,y2,'.k');title('y_2'); hold on
subplot(2,2,3);plot(t,u1(1:numel(t)),'k'); title('u_1');xlabel('t[seg.]')
subplot(2,2,4);plot(t,u2(1:numel(t)),'k');title('u_2');xlabel('t[seg.]')
A=[0,1;-.25,-1];
A=[A zeros(size(A));zeros(size(A)) A];
B1=[ -.25 .5; 3/4 -.25];
B2=[0 -.25;-3/4 3/4];
B=[B1;B2];
C=[1 0 0 0;0 0 1 0];
D=[0 0;0 0];
% Para verificar usando Octave
% num11=[-1 2];num12=[2 1];
% num21=[-3]; num22=[-1 2];
% den=[4 4 1];
% H11=tf(num11,den);H12=tf(num12,den);
% H21=tf(num21,den);H22=tf(num22,den);
% H=[H11 H12;H21 H22];
% sys=ss(H,'minimal');
% A=sys.a;
% B=sys.b;
% C=sys.c;
% D=sys.d;
x=[0;0;0;0];U=[0;0];y_2(1)=0;
for jj=1:Kmax-1
 U(1)=u1(jj);
 U(2)=u2(jj);
 xp=A*x+B*U;
 y =C*x+D*U;
 x=x+xp*At;
 x11(jj)=x(1);
 x22(jj)=x(2);
 y_1(jj+1)=y(1);
 y_2(jj+1)=y(2);
end
figure
subplot(2,2,1);plot(t,y_1,'r');
subplot(2,2,2);plot(t,y_2,'r');legend('EDO','VE');legend('boxoff');