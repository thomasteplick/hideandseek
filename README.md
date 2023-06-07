# hideandseek
Hide linear frequency-modulated (LFM) pulses of given pulse duration and bandwidth in Gaussian noise.

This is a web application written in Go and uses the html/template package to generate the HTML.  The program can be accessed
from a web browser by entering http://127.0.0.1:8080/hideandseek in the URL bar.  The user enters the number of samples to 
generate, the sample frequency in Hz, the locations in milliseconds of the two LFM pulses, their pulsewidth (ms), bandwidth (Hz),
and the signal-to-noise ratio (SNR) in dB.  The <i>Hide</i> button will display the unfiltered pulses when the form is submitted, and the
<i>Seek</i> button will display the match filtered waveform when the form is submitted.  The Location text fields will display the 
locations of the two pulse-compressed LFMs when <i>Seek</i> is selected.  The LFM waveform allows pulse-compression to be used which 
gives greater time/range resolution.  Longer pulses can be used to increase the SNR while the bandwidth can be increased to 
still have fine time/range resolution.

The period of the complete waveform is samples/sample frequency.  The pulse width must be less than this period.  The LFM pulses must be
located inside the complete waveform.  The pulse bandwidth must be less than the Nyquist frequency, which is one-half the sample frequency.
If this is violated, frequency aliasing will result and the resultant correlation will not be optimum.

A replica of the LFM is used for cross-correlation with the waveform.  That is, a match filter is used to extract the location of the LFM
pulses in the waveform.

<h4>Widely separated pulses without match filtering</h4>

![image](https://github.com/thomasteplick/hideandseek/assets/117768679/a5c96eaa-9b34-4045-a64f-a06d2c73e350)

<h4>Widely separated pulses with match filtering</h4>

![image](https://github.com/thomasteplick/hideandseek/assets/117768679/23ad8fd3-fd45-4d7b-978b-0e5a412d127a)

