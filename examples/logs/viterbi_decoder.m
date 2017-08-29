% Viterbi decoder for convolutional code
% 2011-1-17

% by lulu 23/11/2012 to add the soft decoding module

function output_symbol = viterbi_decoder(input_symbol)
global DECODER_TYPE
% CodeGen = [133 171]; %generator polynomials used for FTW OFDM
CodeGen = [117 155];   %used for Raw OFDM
K = 7;%constraint length 
Trellis = poly2trellis(K,CodeGen); 

tblen = 36;%traceback depth   
signal_encoded = [input_symbol; zeros(tblen*3,1);]; %pading zeros at the end 

if strcmp(DECODER_TYPE,'hard') == 1
    [signal_decode m p in] = vitdec(signal_encoded, Trellis, tblen, 'cont', 'hard');% viterbi hard decoding 
end

if strcmp(DECODER_TYPE, 'soft') == 1
    [signal_decode m p in] = vitdec(signal_encoded, Trellis, tblen, 'cont', 'soft', 8);% viterbi soft decoding to decode the symbols to 0-255 level, 255 = 2^8 - 1 
end

if strcmp(DECODER_TYPE, 'exact') == 1
    [signal_decode m p in] = vitdec(signal_encoded, Trellis, tblen, 'cont', 'soft', 8);% viterbi soft decoding to decode the symbols to 0-255 level, 255 = 2^8 - 1 
end

output_symbol = signal_decode (tblen+1:tblen+length(input_symbol)/2);%removing zeros form the begining


