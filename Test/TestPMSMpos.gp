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
plot "TestPMSMPos.dat" using 1:2 with lines title "Id" lc "blue", \
     "TestPMSMPos.dat" using 1:5 with lines title "refVel" lc "red", \
     "TestPMSMPos.dat" using 1:6 with lines title "Vel" lc "green", \
     "TestPMSMPos.dat" using 1:3 with lines title "refIq" lc "purple", \
     "TestPMSMPos.dat" using 1:4 with lines title "Iq" lc "orange" lw 2
    

# Pause (for terminals that close immediately after displaying the plot)
pause -1 "Press any key to continue"