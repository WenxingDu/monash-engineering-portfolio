clear
close all

m=load('assignment_data.mat');
t=m.C1(6001:6601,1);
left=m.C1(6001:6601,2);
right=m.C1(6001:6601,3);
fs=120;

figure(1)
plot(t,left);
hold on;grid on;
plot(t,right);
xlabel('Time (s)');
ylabel('VGRF (Newton)')
legend('left foot','right foot')
title('VGRF vs Time')

% Task3 Plot the VGRF signal in frequency domain
figure(2)
subplot(2,1,1);
freqplot(left,fs);
xlim([0 60])
grid on;hold on

title('magnitude of left foot in frequency domain');
subplot(2,1,2);
freqplot(right,fs);
xlim([0 60])
grid on;hold on

title('magnitude of right foot in frequency domain')



