function dx=DynamicModel(t,x,flag,para)

global M A B C eq k

a=25;b=133;

c=para(1);

s=c*x(1)+x(2);

A=[0 1;0 -a];

B=[0;b];

M=3;

eq=5.0;

if M==1 % M=1为等速趋近律，M=2为指数趋近律，M=3为幂次趋近律，M=4为一般趋近律

slaw=-eq*sign(s); %Equal velocity trending law

elseif M==2

k=10;

slaw=-eq*sign(s)-k*s; %Exponential velocity trending law

elseif M==3

k=10;

alfa=0.50;

slaw=-k*abs(s)^alfa*sign(s); %Power trending law

elseif M==4

k=1;

slaw=-eq*sign(s)-k*s^3; %General trending law

end

u=inv(C*B)*(-C*A*x+slaw);

dx=zeros(2,1);

dx(1)=x(2);

dx(2)=-a*x(2)+b*u;