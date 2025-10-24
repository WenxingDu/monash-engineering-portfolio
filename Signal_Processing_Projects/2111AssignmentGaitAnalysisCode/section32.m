clear
close all;
load('assignment_data.mat');

t=C1(4801:8401,1);
left=C1(4801:8401,2);
right=C1(4801:8401,3);    %select 40-70s data
fs=120; % Sampling Rate
Nw = [10 20 50 128 256]; %length of window

for n = 1:length(Nw)
    n
    noverlap=floor(99.9/100*Nw(n))
    nfft = Nw(n)
    figure(n)
    s = spectrogram(left,Nw(n),noverlap,nfft,fs);
    spectrogram(left,Nw(n),noverlap,nfft,fs,'yaxis');
    set(gca,'xticklabel',{'45','50','55','60','65','70'}) %To match the abscissa with the actual acquisition 40-70s
    %view(-24,38) % for 3D view
    colormap jet
    yticks([0:10:60]);
    h=colorbar;
    h.Label.String = 'Power/Frequency(db/HZ)';
    str=['spectrogram of left foot VGRF at window length of ',num2str(Nw(n))];
    title(str);

end


