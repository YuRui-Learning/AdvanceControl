function [X,W,gamma]=LMID(A,B1,B2,C1,D11,D12,N)
%{
程序功能：
1、利用LMI求解倒立摆
2、状态反馈控制，求控制器
3、\gamam-次优控制

参考文献：[1]吕 申,武俊峰.基于 LMI 优化的鲁棒控制器设计[J].工业仪表与自动化装置,2017,3:123-128.
%}
%     clear,clc
%     A=[0,1,0,0;  0,-0.0618,-0.7167,0;  0,0,0,1;  0,0.2684,31.6926,0];
%     B1=[0; -2.6838; 0; 118.6765];
%     B2=[0 ;0.8906;0;-2.6838];
%     C1=[1e-5,0,0,0;  0,-1e-4,0,0;  0,0,-0.01,0;  0,0,0,-0.01];
%     D11=[0;0;0];
%     D12=[0;0;0];% 参数初始化
%     gamma=1;
    %构建矩阵变量
    setlmis([]);
    X=lmivar(1,[N,1]); %4阶的对称满块
    W=lmivar(2,[1,N]); %的矩阵
    g2=lmivar(1,[1,1]); 
    
    %描述LMI
    
    lmiterm([1,1,1,X],A,1,'s');  %AX+(AX)'
    lmiterm([1,1,1,W],B2,1,'s'); %B2*W+(B2*W)'
    lmiterm([1,1,2,0],B1); %B1
    lmiterm([1,1,3,-W],1,D12');%W*D2'
    lmiterm([1,1,3,X],1,C1');%X*C1'
    lmiterm([1,2,2,0],-1); %-I
    lmiterm([1,2,3,0],D11' );%D11'
    lmiterm([1,3,3,g2], -1,1);%-gamma^2*I
    
    lmiterm([-2,1,1,X],1,1); %X正定
    lmisys=getlmis ;%获取lmi信息
    %-------------------------------
%     lmiterm([1,1,1,X],A,1,'s');  %AX+(AX)'
%     lmiterm([1,1,1,W],B2,1,'s');  %B2*W+(B2*W)'
%     lmiterm([1,1,1,0],gamma^(-2)*(B1*B1') );
%     lmiterm([1,2,1,X],C1,1 );  %C1*X
%     lmiterm([1,2,1,W],D12,1 ); %D12*W
%     lmiterm([1,2,2,0], -1 ); %-I
%     lmisys=getlmis ;%获取lmi信息
    
    %求解可行解X,W
%     [tmin, xfeas]=feasp(lmisys);
%     P=ltisys( A,[B1,B2],[C1;C2],[D11,D12;D21,D22] );
%     gopt=ltiflmi(P,[1,1]);
%     c=mat2dec(lmisys, eye(3) ); %目标值：最优X的trace
    n=decnbr(lmisys);
    c=zeros(n,1);
    for j=1:n
        bj=defcx(lmisys, j, g2);
        c(j)=bj;
    end
%     options=[1e-5, 0, 0, 0, 0]; %计算精度
    [copt,xopt]=mincx(lmisys,c ); %求解过程
%     if(tmin<0)
%         sys=[];
%         disp('Feasible');
%     else
%         return
%         
%     end
    X=dec2mat(lmisys, xopt, X);
    W=dec2mat(lmisys, xopt, W);
    gamma=sqrt(copt);
%     W=dec2mat(lmisys, xfeas, W);
%     
%     %获取反馈控制器
%     K=W*inv(X);
%     K=W/X;
    
    
%     sys=P;
    
end