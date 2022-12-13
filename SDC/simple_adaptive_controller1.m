
function [sys,x0,str,ts] = simple_adaptive_controller1(t, x, u, flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % 调用初始化子函数
  case 1,
    sys=[];
  case 2,
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u);    %计算输出子函数
  case 4,
    sys=[];   %计算下一仿真时刻子函数
  case 9,
    sys=[];    %终止仿真子函数
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
 
end
 function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes   %初始化子函数
 
sizes = simsizes;
 
sizes.NumContStates  = 0;  %连续状态变量个数
sizes.NumDiscStates  = 0;  %离散状态变量个数
sizes.NumOutputs     = 1;  %输出变量个数
sizes.NumInputs      = 4;   %输入变量个数
sizes.DirFeedthrough = 1;   %输入信号是否在输出端出现
sizes.NumSampleTimes = 0;   % at least one sample time is needed
 
sys = simsizes(sizes);
x0  = [];   %初始值
str = [];   
ts  = [];   %[0 0]用于连续系统，[-1 0]表示继承其前的采样时间设置
simStateCompliance = 'UnknownSimState';
 function sys=mdlOutputs(t,x,u)   %计算输出子函数
 
J = 2;
thd = u(1);
th = u(2);
dth = u(3);
 
e = th - thd;
de = dth;
c = 10;
s = c*e + de;
xite = 1.1;
 
k = 0;
% ut = J*(-c*dth-1/J*(k*s+xite*sign(s)));
% ut = -c*J*dth - sign(s)/J;
ut = -J*sign(s) - J*c*(de - u(4)) - thd;
sys(1) = ut;
