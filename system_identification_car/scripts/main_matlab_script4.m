P = omega_minimal_ss();
% P=tf([1 -10],[ 1 11 10]); 
[Ap,Bp,Cp,Dp]=ssdata(P);
np=max(size(Ap));
% filters for performance index
% We=tf(1,[1 0.01]); [Ae,Be,Ce,De]=ssdata(We);
We=tf([3, 1],[1 0.01]); [Ae,Be,Ce,De]=ssdata(We);
ne=max(size(Ae));
% Wu=tf([1 2],[1 10]); [Au,Bu,Cu,Du]=ssdata(Wu);
Wu=tf([1 2],[1 4]); [Au,Bu,Cu,Du]=ssdata(Wu);
nu=max(size(Au));
% augmented plant
A = [Ap, zeros(np,ne), zeros(np,nu);...
-Be*Cp, Ae, zeros(ne,nu);...
zeros(nu,np), zeros(nu,ne), Au];
B2 =[Bp; zeros(ne,1); Bu];
B1 =[Bp, zeros(np,1); zeros(ne,1), Be; zeros(nu,1), zeros(nu,1)];
C1 =[zeros(1,np), Ce, zeros(1,nu);...
zeros(1,np), zeros(1,ne), Cu];
D12=[ zeros(1,nu); Du];
C2 =[-Cp, zeros(1,ne), zeros(1,nu)];
D21=[0, 1];
% Find F
QQ = C1'*C1-C1'*D12/(D12'*D12)*D12'*C1;
AA = A-B2/(D12'*D12)*D12'*C1;
RR = B2/(D12'*D12)*B2';
Pu = are(AA,RR,QQ);
F = -(D12'*D12)\D12'*C1 - (D12'*D12)\B2'*Pu;
% Find L,
% C1->B1’, D12->D21’, A->A’, B2->C2’, F->L’
QQ = B1*B1'-B1*D21'/(D21*D21')*D21*B1';
AA = A'-C2'/(D21*D21')*D21*B1';
RR = C2'/(D21*D21')*C2;
Py = are(AA,RR,QQ);
L = ( -(D21*D21')\D21*B1' - (D21*D21')\C2*Py )';
% Compute the optimal controller
% Ac=A+B2*F+L*C2; Bc=-L; Cc=F; Dc=0;
Ac=A+B2*F + L*C2; Bc= -L; Cc=F; Dc=0;
K = minreal( ss(Ac,Bc,Cc,Dc) );
%%
Ts = 0.04;
B = [B1, B2];
C = [C1; C2];
D = [zeros(2), D12; D21, 0];
ss2 = ss(A, B, C, D);
K_2 = h2syn(ss2, 1, 1);
[K_inf, ~, gamma] = hinfsyn(ss2, 1, 1);
%%
G_miu = tf([252.2, 497.8, 2519], [1, 187.5, 3348, 872.6]);
discrete_G_miu = c2d(G_miu, 0.4);
K_inf = c2d(K_inf, Ts);
K_2 = c2d(K_2, Ts);
%%
save ./matlab_omega_control/H_2_statefeedback&estimation  F L
[KA, KB, KC, KD] = ssdata(K_2);
save ./matlab_omega_control/H_2_control_matrix.mat  KA KB KC KD
[KA, KB, KC, KD] = ssdata(K_inf);
save ./matlab_omega_control/H_inf_control_matrix.mat  KA KB KC KD