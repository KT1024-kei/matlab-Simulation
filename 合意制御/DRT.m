%% 
B = [1 0 -1;
     -1 1 0;
     0 -1 1;];
I2 = eye(2);

B_ = kron(B, I2)';

S1 = [1 0 0;
      0 1 0;
      0 0 1];
S2 = S1 - B;

T = [S1 S2];
T_ = [kron(S1, I2) kron(S2, I2)];

BT = B' * T;
BT_ = B_' * T_;

syms n1 n2 n3 m1 m2 m3
n = [n1 n2 n3]';
m = [m1 m2 m3]';

A = [

BTnm = BT * [n;m];



