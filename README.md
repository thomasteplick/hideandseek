# hideandseek
Hide linear frequency-modulated (LFM) pulses of given pulse duration and bandwidth in Gaussian noise.

This is a web application written in Go and uses the html/template package to generate the HTML.  The program can be accessed
from a web browser by entering http://127.0.0.1:8080/hideandseek in the URL bar.  The user enters the number of samples to 
generate, the sample frequency in Hz, the locations in milliseconds of the two LFM pulses, their pulsewidth (ms), bandwidth (Hz),
and the signal-to-noise ratio (SNR) in dB.  The Hide button will display the unfiltered pulses when the form is submitted, and the
Seek button will display the match filtered waveform when the form is submitted.  The Location text fields will display the 
locations of the two pulse-compressed LFMs when Seek is selected.  The LFM waveform allows pulse-compression to be used which 
gives greater time/range resolution.  Longer pulses can be used to increase the SNR while the bandwidth can be increased to 
still have fine time/range resolution.
