```
-rw-r--r-- 1 pi 12699331696 Nov  5 23:15 nov_3_time_18_48_st_ives
-rw-r--r-- 1 pi   490221600 Apr 25 22:33 gpssim.bin
-rw-r--r-- 1 pi         716 Apr 25 22:33 gpssim.txt
-rw-r--r-- 1 pi   240000000 Apr 25 15:55 GPS-L1-2022-03-27.sigmf-data
-rw-r--r-- 1 pi    16368000 Mar  3  2014 gioveAandB_short.bin
```

## nov_3_time_18_48_st_ives
https://github.com/codyd51/gypsum/releases
file-type: 2xf32
Captured in the UK in Nov 2023.
You can use this script to download/unzip the file [get_iq_samples.sh](./resources/get_iq_samples.sh).

## gpssim.bin
cf https://github.com/osqzss/gps-sdr-sim
result of:
 ./gps-sdr-sim -b 16 -d 60 -t 2022/01/01,01:02:03 -l 35.681298,139.766247,10.0 -e brdc0010.22n -s 2046000
file-type: 2xi16

```
./gps-sdr-sim -b 16 -d 60 -t 2022/01/01,01:02:03 -l 35.681298,139.766247,10.0 -e brdc0010.22n -s 2046000
Using static location mode.
xyz =  -3959617.5,   3350136.6,   3699531.5
llh =   35.681298,  139.766247,        10.0
Start time = 2022/01/01,01:02:03 (2190:522123)
Duration = 60.0 [sec]
05  146.8  12.9  24517023.8   9.6
10  315.8  31.4  22789584.0   4.9
12  157.5  30.8  22679311.2   6.1
13   79.9  19.7  23736998.7   7.6
15   77.0  50.6  21203005.0   4.1
18  230.4  24.7  23191867.9   6.2
23  298.9  65.8  20585659.2   3.3
24  356.1  79.4  19958939.4   3.2
25  186.1   9.4  24769194.9  10.1
28   42.8  14.7  24631379.7   7.9
32  285.5   2.0  25712738.2   6.4
Time into run = 60.0
Done!
Process time = 7.5 [sec]
```

## GPS-L1-2022-03-27.sigmf-data
source: https://zenodo.org/records/6394603
complex i16 @4KHz -- not usable just yet

## gioveAandB_short.bin
http://gfix.dk/matlab-gnss-sdr-book/gnss-signal-records/
sampling at 16367600Hz -- not usable yet
one signal sample is stored as one signed byte (int8)
