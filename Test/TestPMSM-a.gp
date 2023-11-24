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
plot "TestPMSM.dat" using 1:7 with lines title "V_alpha" lc "blue", \
     "TestPMSM.dat" using 1:8 with lines title "V_beta" lc "red", \
     "TestPMSM.dat" using 1:6 with lines title "Vel" lc "green"
        

# Pause (for terminals that close immediately after displaying the plot)
pause -1 "Press any key to continue"