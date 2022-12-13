clear all;

close all;

global M A B C eq k

ts=0.001;

T=2;

TimeSet=[0:ts:T];

c=15;

C=[c,1];

para=[c];

[t,x]=ode45('chap2_1eq',TimeSet,[0.50 0.50],[],para);

x1=x(:,1);

x2=x(:,2);

s=c*x(:,1)+x(:,2);

if M==3

for kk=1:1:T/ts+1

xk=[x1(kk);x2(kk)];

sk(kk)=c*x1(kk)+x2(kk);

slaw(kk)=-eq*sign(sk(kk))-k*sk(kk); %Exponential trending law

u(kk)=inv(C*B)*(-C*A*xk+slaw(kk));

end

end

figure(1);

plot(x(:,1),x(:,2),'r',x(:,1),-c*x(:,1),'b');

xlabel('x1');ylabel('x2');

%figure(2);

%plot(t,x(:,1),'r');

%xlabel('time(s)');ylabel('x1');

%figure(3);

%plot(t,x(:,2),'r');

%xlabel('time(s)');ylabel('x2');

%figure(4);

%plot(t,s,'r');

%xlabel('time(s)');ylabel('s');

if M==3

figure(5);

plot(t,u,'r');

xlabel('time(s)');ylabel('u');

end