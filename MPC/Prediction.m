function u_k= Prediction(x_k,E,H,N,p)

U_k = zeros(N*p,1); % NP x 1

U_k = quadprog(H,E*x_k);%求解最优函数

u_k = U_k(1:p,1); % 取第一个结果

end 