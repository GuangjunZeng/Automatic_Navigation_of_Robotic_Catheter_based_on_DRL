clear all;
P = -1:0.04:1;
T = sin(2*pi*P)+0.1*randn(size(P));
net = newff(P, T, 18, {}, 'trainbr'); %创建了一个新的前馈神经网络，网络有18个隐藏层神经元，使用的训练函数是Levenberg-Marquardt优化（'trainbr'）
net.trainParam.show = 10;  %设置了神经网络训练的显示频率，每10个迭代周期显示一次
net.trainParam.epochs=100; %设置了神经网络训练的最大迭代次数为100。
net=train(net,P,T);
Y=sim(net,P);
figure;
plot(P, T, '-', P, Y, '+');
legend('原始信号','网络输出信号');
set(gcf,'Position',[20,20,500,400]);


