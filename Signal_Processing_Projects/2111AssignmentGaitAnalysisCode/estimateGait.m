clear 
close all;
load('assignment_data.mat')
t=C1(:,1);
left=C1(:,2);
right=C1(:,3);   
fs=120;


% Method I: Low Pass Filter
Order = 40;
f_LPF = [0 1/6 1/3 1]; % Real Frequency = f*fs/2 
a_LPF = [1 1 0 0];
[b_LPF,err_LPF,res_LPF] = firpm(Order,f_LPF,a_LPF);
left_LPFiltered = conv(left,b_LPF);
right_LPFiltered = conv(right,b_LPF);

% % Method II: Band Stop Filter
% f_PSF = [0 40 45 50 55 60]/60
% a_PSF = [1 1 0 0 0 1];
% 
% [b_PSF,err_PSF,res_PSF] = firpm(Order,f_PSF,a_PSF);
% left_PSFiltered = conv(left,b_PSF);
% 
% figure(2)
% plot(t,left)
% hold on
% plot(t,left_PSFiltered(1:length(t)))
% xlim([5 10])
% legend('Original Left','Filtered Left')
% xlabel('Time (s)')
% ylabel('VGRF (Newton)')

% Task 1, 1. Plot the filtered VGRF for both feet (overlaid) vs. time 
% (in seconds) for some choice of 5 second interval.
threshold = 25
figure(1)
set(gcf,'Position',[100 100 550 320])
plot(t,left_LPFiltered(1:length(t)),'LineWidth',1.5)
hold on
plot(t,right_LPFiltered(1:length(t)),'LineWidth',1.5)
plot(t,threshold*ones(1,length(t)),'--k')
xlim([15 20])
ylim([-100 900])
xlabel('Time (s)')
ylabel('VGRF (Newton)')

% Task2 Mark the onset of swing and stance phases on the filtered VGRF time-domain plot.


for n = 1:length(t)
    % Find the start points of Stance-Left
    if (left_LPFiltered(n)<=threshold)&(left_LPFiltered(n+1)>=threshold)
        Stance_Start_Left(n)=1;
    else
        Stance_Start_Left(n)=0;
    end
    % Find the End points of Swing-Left
    if (left_LPFiltered(n)>=threshold)&(left_LPFiltered(n+1)<=threshold)
        Swing_Start_Left(n)=1;
    else
        Swing_Start_Left(n)=0;
    end
       % Find the start points of Stance-Right
    if (right_LPFiltered(n)<=threshold)&(right_LPFiltered(n+1)>=threshold)
        Stance_Start_Right(n)=1;
    else
        Stance_Start_Right(n)=0;
    end
    % Find the End points of Swing-Right
    if (right_LPFiltered(n)>=threshold)&(right_LPFiltered(n+1)<=threshold)
        Swing_Start_Right(n)=1;
    else
        Swing_Start_Right(n)=0;
    end 
    
end

Stance_Start_Index_Left = find(Stance_Start_Left==1);
Swing_Start_Index_Left = find(Swing_Start_Left==1);

Stance_Start_Index_Right = find(Stance_Start_Right==1);
Swing_Start_Index_Right = find(Swing_Start_Right==1);

plot(t(Stance_Start_Index_Left),zeros(1,length(Stance_Start_Index_Left)),'sqb')
plot(t(Swing_Start_Index_Left),zeros(1,length(Swing_Start_Index_Left)),'ob')

plot(t(Stance_Start_Index_Right),zeros(1,length(Stance_Start_Index_Right)),'sqr')
plot(t(Swing_Start_Index_Right),zeros(1,length(Swing_Start_Index_Right)),'or')

legend('Filtered Left','Filtered Right','Zero Threshold Line', ...
    'Start of Stance(L)','Start of Swing(L)','Start of Stance(R)','Start of Swing(R)')
grid on


% Task3 Plot the filtered VGRF signal in frequency domain
figure(3)
subplot(2,1,1);
freqplot(left,fs);
grid on;hold on
freqplot(left_LPFiltered,fs);
xlim([0 60])
title('Filtered magnitude of left foot in frequency domain');
legend('Original','Filtered') 

subplot(2,1,2);
freqplot(right,fs);
grid on;hold on
freqplot(right_LPFiltered,fs);
xlim([0 60])
title('Filtered magnitude of right foot in frequency domain')
legend('Original','Filtered') 

% Taks4 Plot the 
% stride time vs. cycle, 
% swing time vs. 
% cycle and stance time vs. 
% cycle for each foot. 
% Note that these are discrete-time signals since the gait cycles are indexed by integers.
STl = (Swing_Start_Index_Left-Stance_Start_Index_Left(1:length(Swing_Start_Index_Left)))/fs;
STl = STl(2:end); % Eliminate the abnormal value

STr = (Swing_Start_Index_Right-Stance_Start_Index_Right(1:length(Swing_Start_Index_Right)))/fs;
STr = STr(2:end); % Eliminate the abnormal value

SWl = (Stance_Start_Index_Left(2:end)-Swing_Start_Index_Left)/fs;
SWr = (Stance_Start_Index_Right(2:end)-Swing_Start_Index_Right(1:end-1))/fs;

Sl = diff(Swing_Start_Index_Left)/fs;
Sr = diff(Swing_Start_Index_Right)/fs;


figure(4)
MarkerSize = 3
subplot(3,1,1)
plot(1:1:length(STl),STl,'-o',"MarkerSize",MarkerSize)
hold on
plot(1:1:length(STr),STr,'-sq',"MarkerSize",MarkerSize)
legend('STl','STr')
xlabel('Cycle')
ylabel('Stance Time (s)')

subplot(3,1,2)
plot(1:1:length(SWl),SWl,'-o',"MarkerSize",MarkerSize)
hold on
plot(1:1:length(SWr),SWr,'-sq',"MarkerSize",MarkerSize)
legend('SWl','SWr')
xlabel('Cycle')
ylabel('Swing Time (s)')

subplot(3,1,3)
plot(1:1:length(Sl),Sl,'-o',"MarkerSize",MarkerSize)
hold on
plot(1:1:length(Sr),Sr,'-sq',"MarkerSize",MarkerSize)
legend('Sl','Sr')
xlabel('Cycle')
ylabel('Stride Time (s)')




