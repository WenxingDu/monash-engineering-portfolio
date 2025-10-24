function [f,FFT]=freqplot(x,fs)
n=length(x);             
X=abs(fftshift(fft(x))); 
FFT=X/n;
f=linspace(-fs/2,(fs/2)-1,n); 
plot(f,FFT),xlabel('Hz'),ylabel('Maltitude')

