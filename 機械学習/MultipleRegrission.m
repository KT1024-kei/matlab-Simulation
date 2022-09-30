clc; clear; close all;  %������
n = 20;                 %�f�[�^��
a2 = 2;
a1 = -1;                %�X��
a0 = 3;                 %�ؕ�
e = 5.0;                %�m�C�Y�̑傫��

x = -3 + 6*rand(n,1);  %[-3,3]�͈̔͂ɓK���ɌP���f�[�^�𐶐�
y = 3 + 4*rand(n,1);
lam = a2*y + a1*x + a0 + e*(0.5 - rand(n,1));  %�P���f�[�^�ɂ̓m�C�Y���܂܂��


%% �P���f�[�^�Ɛ^�̊֐��i�m�C�Y�Ȃ��j�̕`��
x1 = linspace(min(x),max(x),50);
x2 = linspace(min(y),max(y),50);
[X1,X2] = meshgrid(x1,x2);

Lam_true = a2*X2 + a1*X1+ a0;

figure(1)
plot3(x,y,lam, 'k+', 'MarkerSize', 20, 'LineWidth', 3);
hold on; grid on;
surf(X1,X2,Lam_true,...
    'FaceLighting','none',...
    'EdgeLighting','flat',...
    'LineStyle','none',...
    'FaceColor','r',...
    'EdgeColor','r',...
    'FaceAlpha',0.3);
legend({'Training Data', 'True Function (without Noise)'})
xlabel('x')
ylabel('y')
zlabel('lam')


%% �w�K�A���S���Y��

X = [ones(n,1) x y];      %�s��X�̐���
c = inv(X'*X)*X'*lam;   %�W���̌���i���ꂪ�œK���j


%% �w�K���ʂ̕`��
lam_learned =c(3,:)*X2 +c(2,:)*X1 +c(1,:);
surf(X1,X2,lam_learned,...
    'FaceLighting','none',...
    'EdgeLighting','flat',...
    'LineStyle','none',...
    'FaceAlpha',0.3,...
    'FaceColor','b',...
    'EdgeColor','b');
legend({'Training Data', 'True Function (without Noise)', 'Solution'}, 'Location', 'best');