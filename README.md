# gnss-acq: GPS L1 C/A acquisition

This tool takes as input an SDR IQ recording at 2046KHz and identifies the GPS satellites that are "in sight". Various files format are supported: 2x `float32` or `int16` or `int8`. It then identifies the signals sent by GPS satellites. Output is via log only for now.

## Requirements.
A raspberry pi will do, I use an RTL-SDR dongle with a cheap GPS antenna w/ an SMA connector.

## Perform an acquisition
- you need to activate bias-t and power the gps/lna antenna:
```
$ rtl_biast -d 0 -b 1
```
- command to sample L1 at 2046KHz for 10 sec:
```
$ rtl_sdr -f 1575420000 -s 2046000 -n 20460000 output.bin
```

## Simulate a GPS SDR recording
Cf [GPS-SDR-SIM](https://github.com/osqzss/gps-sdr-sim)
```
 ./gps-sdr-sim -b 16 -d 60 -t 2022/01/01,01:02:03 -l 35.681298,139.766247,10.0 -e brdc0010.22n -s 2046000
```
This generates an IQ recording w/ 2 int16 per I and Q sample.

## run gnss-acq
Once you have a trace file available, you can run *gnss-acq*:
```
# cargo run --release -- -f path/to/recording.bin
```

Resources:
- [RTL-SDR](https://www.rtl-sdr.com/buy-rtl-sdr-dvb-t-dongles/)
- [Software Defined GPS](https://www.ocf.berkeley.edu/~marsy/resources/gnss/A%20Software-Defined%20GPS%20and%20Galileo%20Receiver.pdf)
- [GPS-SDR-SIM](https://github.com/osqzss/gps-sdr-sim)
- [Python GPS software: Gypsum](https://github.com/codyd51/gypsum)

A few online SDR recordings at 1575,42 MHz are available online:
- https://jeremyclark.ca/wp/telecom/rtl-sdr-for-satellite-gps/
- https://s-taka.org/en/gnss-sdr-with-rtl-tcp/
- https://destevez.net/2022/03/timing-sdr-recordings-with-gps/
