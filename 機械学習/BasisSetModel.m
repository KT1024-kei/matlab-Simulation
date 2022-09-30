%% �����ݒ�i�D���ɐݒ肵�ėǂ��j
clc; clear; close all;  %������
n = 30; %�f�[�^��
a2 = 3;
a1 = -1;                %�X��
a0 = 1;                 %�ؕ�
e = 1.5;                %�m�C�Y�̑傫��

x_rand = -3 + 6*rand(n,1); %[-3,3]�͈̔͂ɓK���ɌP���f�[�^�𐶐�
x = sort(x_rand);
lam = a2 + a1*cos(x) + a0*x + e*(0.5 - rand(n,1));  %�P���f�[�^�ɂ̓m�C�Y���܂܂��


%% �P���f�[�^�Ɛ^�̊֐��i�m�C�Y�Ȃ��j�̕`��

lam_true = a2 + a1*cos(x) + a0 * x;     %line�֐��Ńv���b�g���邽�߂̐ݒ�:y���W

figure(1)
plot(x, lam, 'k+', 'MarkerSize', 20, 'LineWidth', 3);
hold on; grid on;
plot(x, lam_true, 'Color', 'blue', 'LineStyle', '-', 'LineWidth', 3);
xlabel('x', 'Fontsize', 20); ylabel('lambda', 'Fontsize', 20);
legend({'Training Data', 'True Function (without Noise)'}, 'Location', 'best');
set(gca, 'FontSize', 20);
axis equal;


%% �w�K�i�P��A�j�A���S���Y��
X = [ones(n,1) cos(x) x];      %�s��X�̐���
c = inv(X'*X)*X'*lam;   %�W���̌���i���ꂪ�œK���j


%% �w�K���ʂ̕`��

lam_led = c(3)*x+ c(2)*cos(x) + c(1); %line�֐��Ńv���b�g���邽�߂̐ݒ�:y���W

 
plot(x, lam_led, 'Color', 'red', 'LineStyle', '-', 'LineWidth', 3);
legend({'Training Data', 'True Function (without Noise)', 'Solution'}, 'Location', 'best'); 