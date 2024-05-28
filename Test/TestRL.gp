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
plot "TestLR.dat" using 1:2 with lines title "I" lc "blue", \
	 "TestLR.dat" using 1:3 with lines title "refI" lc "red", \
     "TestLR.dat" using 1:4 with lines title "Iadc" lc "green"
    
set label 1 'Red: refI' at graph 0.85, 0.95 tc rgb "red"
set label 2 'Green: adcI' at graph 0.85, 0.90 tc rgb "green"
set label 3 'Blue: I' at graph 0.85, 0.85 tc rgb "blue"


# Pause (for terminals that close immediately after displaying the plot)
pause -1 "Press any key to continue"