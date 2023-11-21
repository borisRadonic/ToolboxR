set terminal wxt size 800,600 

# Set the grid
set grid

# Set the title and axis labels
set title "WaveForm Tracer Data"
set xlabel "Freq"
set ylabel "Value"

# Set the legend (key) position
set key outside top left

# Plot the data
plot "TestLowPass1.dat" using 3:2 with lines
	 


# Pause (for terminals that close immediately after displaying the plot)
pause -1 "Press any key to continue"