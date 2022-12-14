function [A,B1,B2,K,x0]=Parameters()
   clear,clc
    M=1; %小车质量
    m=0.1; %倒立摆质量 
    L=0.5; %摆的长度
    g=9.8 ;%重力加速度
    J=m*L^2/3; %转动惯量
    
    k=J*(M+m)+m*M*L^2 ;
    k1=(M+m)*m*g*L/k ;
    k2=-m^2*g*L^2/k ; 
    k3=-m*L/k ;
    k4=( J+m*L^2 ) /k ; %中间变量
    x0=[1,2,2,1]/2

    %------PlantA------------
    
    A=[0,0,1,0 ;0,0,0,1 ;k1,0,0,0 ;k2,0,0,0];
%     B1=[0; 0; 0.09; 0.11 ];
    B1=[0;0;0.2;0.2] ;
    B2=[0; 0; k3; k4];
    C1=[1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1; 0,0,0,0] ;
%     C=[1,0,0,0;0,0,0,0] ;
    N=length(A);
    D11=[0;0;0];
    D12=[0; 0; 0; 0; 1]; %状态方程各个矩阵
%     D12=0;
%--------直接求出K-------------
    [X,W,gamma]=LMID(A,B1,B2,C1,D11,D12,N)
    K=W*inv(X)
%--------------------------
%     gamma=1.5;
%-----根据所得的次优gamma重新求解K--------
    K=LMIC(A,B1,B2,C1,D11,D12,gamma,N)
    C1 = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]


end