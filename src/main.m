clear;
close all;
clc;


% Plot the amplitude of the combined signal
cf = 0;

signal_gen( 0 );
[ SNR11, SNR12, SNR21, SNR22, rx_signal ] = decode();
SNR_decoded = [ SNR_decoded ; [ SNR11, SNR12, SNR21, SNR22 ] ];
