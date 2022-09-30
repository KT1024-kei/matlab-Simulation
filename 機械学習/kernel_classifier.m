%% �����ݒ�i��S��̖��Q�̂悤�ɂ���͈͓��Ń����_���Ƀf�[�^�_������Ă��烉�x���t�����邱�Ƃ𐄏��j
clc; clear; close all;  %������
N = 20;                 %D_+��D_-���ꂼ��̃f�[�^���i����͓��������p�Ӂj
load data1;             %D_-�̃f�[�^�i����͔z�z�f�[�^���g�p���邪�C�����ō쐬���Ă��ǂ��j          
load data2;             %D_+�̃f�[�^


%% �P���f�[�^�Ƌ��E���̕`��
figure;
plot(data2(:, 1), data2(:, 2), 'kx', 'MarkerSize', 14, 'Linewidth', 2);
hold on;
plot(data1(:, 1), data1(:, 2), 'ko', 'MarkerSize', 8, 'Linewidth', 2);
theta = 0:0.01:2*pi;                                                                             %�v���b�g�p�̊p�x�̓��Ԋu�f�[�^
plot(1.5*cos(theta) - 2, sin(theta) - 1, 'Linestyle', '--', 'Color', 'blue', 'Linewidth', 3);    %�^�̑ȉ~��̋��E���̃v���b�g
set(gca, 'FontSize',20, 'FontName', 'Times')
legend({'$\lambda=+1$', '$\lambda=-1$', 'Boundary'}, 'Location', 'NorthEast', 'NumColumns', 3, 'Interpreter', 'latex');
xlabel('$x$', 'Interpreter', 'latex', 'Fontsize', 20); ylabel('$y$', 'Interpreter', 'latex', 'Fontsize', 20);
xlim([-5 1]); ylim([-3 1]);


%% �w�K�A���S���Y��
data = [data1; data2];                      %D_-��D_+���܂Ƃ߂�
Lambda = diag([-ones(1, N) ones(1, N)]);    %���킹�ă��x���t��
K = [];                                     %�J�[�l���֐��s��̔�

%�J�[�l���֐��s��̍Č�
for i = 1:2*N
    for j = 1:2*N
       K(i,j) = [1; data(i, 1); data(i, 2); data(i, 1)^2; data(i, 2)^2]'*[1; data(j, 1); data(j, 2); data(j, 1)^2; data(j, 2)^2];   
    end
end
       
H = [K zeros(2*N, 1); zeros(1, 2*N + 1)];
f = zeros(2*N + 1, 1);
A = -Lambda * [K ones(2*N, 1)];
b = -ones(2*N, 1);

c = quadprog(H, f, A, b);

%% �w�K���ʂ̕`��i3�ڂ�f(x,y)=0�̃v���b�g�͍D���ȕ��@�ŗǂ��j
figure;
plot(data2(:, 1), data2(:, 2), 'kx', 'MarkerSize', 14, 'Linewidth', 2);
hold on;
plot(data1(:, 1), data1(:, 2), 'ko', 'MarkerSize', 8, 'Linewidth', 2);
fimplicit(@(x,y) c(1:2*N)'*[ones(2*N,1) data data.^2]*[1; x; y; x.^2; y.^2] + c(2*N + 1), 'Linestyle', '-', 'Color', 'red', 'LineWidth', 3);
set(gca, 'FontSize',20, 'FontName', 'Times')
legend({'$\lambda=+1$', '$\lambda=-1$', '$f(x,y)=0$'}, 'Location', 'NorthEast', 'NumColumns', 3, 'Interpreter', 'latex');
xlabel('$x$', 'Interpreter', 'latex', 'Fontsize', 20); ylabel('$y$', 'Interpreter', 'latex', 'Fontsize', 20);
xlim([-5 1]); ylim([-3 1]);