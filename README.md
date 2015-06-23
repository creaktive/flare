# nRF905 demodulator/FLARM decoder

```bash
make                                            # compile everything
rtl_sdr -f 868.05m -s 1.6m -g 49.6 -p 49 - |    # tune to 868.05 MHz, set sample rate to 1.6 MHz, gain to 49.6 dB, and tuner error to 49 ppm
    ./nrf905_demod 29 |                         # demodulate nRF905 packets with 29 bytes per message
    ./flarm_decode 43.21 5.43 12                # decode FLARM packets for ground station located at latitude 43.21, longitude 5.43 and geoid height 12
```

# References
 - [Controlling the Wattcher Display](https://pushstack.wordpress.com/2014/07/12/controlling-the-wattcher-display/)
 - [FLARM receiver for GNU Radio](https://github.com/argilo/gr-flarm)
 - [G.9959(ZWave) demodulator](https://github.com/andersesbensen/rtl-zwave)
 - [Sniffing and decoding NRF24L01+ and Bluetooth LE packets for under $30](http://blog.cyberexplorer.me/2014/01/sniffing-and-decoding-nrf24l01-and.html)
 - [Using a RTLSDR dongle to validate NRF905 configuration](http://www.embeddedrelated.com/showarticle/548.php)
 - [Implementation of Digital Signal Processing: Some Background on GFSK Modulation](http://wwwhome.ewi.utwente.nl/~gerezsh/sendfile/sendfile.php/gfsk-intro.pdf?sendfile=gfsk-intro.pdf)
 - [An Arctangent Type Wideband PM/FM Demodulator with Improved Performances](http://ketabkhanemelli.com/Scientific/IEEE/iel2/565/3783/00140755.pdf)
 - [nRF905 Single chip 433/868/915MHz Transceiver Product Specification](http://www.nordicsemi.com/eng/content/download/2452/29528/file/Product_Specification_nRF905_v1.5.pdf)
 - [The Sliding DFT](http://www.comm.toronto.edu/~dimitris/ece431/slidingdft.pdf)
