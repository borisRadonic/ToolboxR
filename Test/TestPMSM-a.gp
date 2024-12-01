set terminal wxt size 1600,1200 

# Set the grid
set grid

# Set the title and axis labels
set title "WaveForm Tracer Data"
set xlabel "Time"
set ylabel "Value"

# Set the legend (key) position
set key outside top left

# Plot the data
plot "TestPMSM.dat" using 1:9 with lines title "EMF_alpha" lc "blue", \
     "TestPMSM.dat" using 1:10 with lines title "EMF_beta" lc "red", \
	 "TestPMSM.dat" using 1:13 with lines title "Ialpha+Ibeta", \
     "TestPMSM.dat" using 1:6 with lines title "Vel" lc "green"

# Pause (for terminals that close immediately after displaying the plot)
pause -1 "Press any key to continue"