gnss-test: Rust based L1 scanner.


to activate bias-t and power gps/lna antenna:

$ rtl_biast -d 0 -b 1

to sample L1 at 2046KHz for 10sec:

$ rtl_sdr -f 1575420000 -s 2046000 -n 20460000 output.bin


references:
- https://www.ocf.berkeley.edu/~marsy/resources/gnss/A%20Software-Defined%20GPS%20and%20Galileo%20Receiver.pdf
- https://jeremyclark.ca/wp/telecom/rtl-sdr-for-satellite-gps/
- https://s-taka.org/en/gnss-sdr-with-rtl-tcp/
- https://destevez.net/2022/03/timing-sdr-recordings-with-gps/
