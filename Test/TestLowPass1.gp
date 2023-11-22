set terminal wxt size 800,600 

# Set the grid and title
set grid
set title "Bode Plot"

# Magnitude plot settings (left y-axis)
set logscale y
set ylabel "Magnitude (dB)"
set ytics nomirror

# Phase plot settings (right y-axis)
set y2label "Phase (Degrees)"
set y2tics
set autoscale y2

# X-axis (Frequency)
set xlabel "Frequency (Hz)"

# Plot the data
plot "TestLowPass1.dat" using 2:3 with lines title 'Magnitude in dB',  \
	 "TestLowPass1.dat" using 2:4 axes x1y2 with lines title 'Phase in Degrees'

pause -1 "Press any key to continue"