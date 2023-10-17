set terminal wxt size 800,600 

# Set the grid
set grid

# Set the title and axis labels
set title "WaveForm Tracer Data"
set xlabel "Time"
set ylabel "Value"

# Set the legend (key) position
set key outside top left

# Plot the data
plot "TestTrajectory1.dat" using 1:2 with lines title "accel", \
     "TestTrajectory1.dat" using 1:3 with lines title "vel", \
     "TestTrajectory1.dat" using 1:4 with lines title "pos"


# Pause (for terminals that close immediately after displaying the plot)
pause -1 "Press any key to continue"