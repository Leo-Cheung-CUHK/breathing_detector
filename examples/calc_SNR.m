


function SNR = calc_SNR(noise,pkt)

N=sum(abs(noise).^2)/length(noise);
P=sum(abs(pkt).^2)/length(pkt);
SNR = 10*log10((P-N)/N);

end