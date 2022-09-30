clc; clear; close all;  %������
n = 20;                 %�f�[�^��
a1 = -1;                %�X��
a0 = 3;                 %�ؕ�
e = 5.0;                %�m�C�Y�̑傫��

x = [-n/2:n/2]';  %[-3,3]�͈̔͂ɓK���ɌP���f�[�^�𐶐�
lam = a1*x + a0 + e*(0.5 - rand(n+1,1));  %�P���f�[�^�ɂ̓m�C�Y���܂܂��


%% �P���f�[�^�Ɛ^�̊֐��i�m�C�Y�Ȃ��j�̕`��
x1 = min(x);
x2 = max(x);
lam_true_min = a1*x1 + a0;
lam_true_max = a1*x2 + a0;

figure(1)
plot(x,lam,'k+', 'MarkerSize', 20, 'LineWidth', 3);
hold on; grid on;
line([x1 x2],[lam_true_min lam_true_max],'Color', 'blue', 'LineStyle', '-', 'LineWidth', 3);



%% �w�K�i�P��A�j�A���S���Y��

X = [ones(n+1,1) x];      %�s��X�̐���
c = inv(X'*X)*X'*lam;   %�W���̌���i���ꂪ�œK���j


%% �w�K���ʂ̕`��

lam_learned_min = c(2)*x1 + c(1);
lam_learned_max = c(2)*x2 + c(1);

line([x1 x2],[lam_learned_min lam_learned_max],'Color', 'red', 'LineStyle', '-', 'LineWidth', 3);