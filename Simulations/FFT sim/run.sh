#!/bin/sh
echo "" > last_data_point.dat
python3 sim.py &
python3 plot.py &
python3 plot_fft.py &
