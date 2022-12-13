clear ;

close all;

clc;

A=[1 0.1;0 2];
n = size(A,1);

B=[0.2 1;0.5 2];
p=size(B,2);

Q=[100 0;0 1];
F=[100 0;0 1];

R=[1 0;0 1];

k_steps =100;

X_K = zeros(n,k_steps);

X_K(:,1)=[20;-20];

U_K = zeros(p,k_steps);

%定义预测区间K
N=5;
[E,H]=MPC_Matrices(A,B,Q,R,F,N);%求解E和H矩阵

for k =1:k_steps
    
U_K(:,k) = Prediction(X_K(:,k),E,H,N,p);
X_K(:,k+1)=(A*X_K(:,k)+B*U_K(:,k));
end

subplot(2, 1, 1);
hold;
for i =1 :size (X_K,1)
plot (X_K(i,:));
end

legend("x1","x2")
hold off;
subplot (2, 1, 2);
hold;

for i =1 : size (U_K,1)
plot (U_K(i,:));
end

legend("u1","u2")

