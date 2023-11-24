import pandas as pd

data = pd.read_csv('TestPMSMPos.dat', sep='\s+', header=0)  # sep='\s+' handles any number of spaces or tabs as delimiters
import matplotlib.pyplot as plt

plt.figure(figsize=(10, 6))
# plt.plot(data['Time'], 10.00 * data['refIq'], label='refIq x 10', color='blue')
# plt.plot(data['Time'], 10.00 * data['Iq'], label='Iq x 10', color='green')
plt.plot(data['Time'], data['refVel'], label='ref_vel', color='yellow')
plt.plot(data['Time'], data['Vel'], label='Velocity', color='red')
plt.plot(data['Time'], data['refPos'], label='ref_pos')
plt.plot(data['Time'], data['isPos'], label='Position')

plt.plot(data['Time'], 1000 * (data['refPos'] - data['isPos']), label='pos_error [mrad]', color='green')

plt.plot(data['Time'], data['Jerk'], label='Jerk', color='blue')
 
plt.xlabel('Time')

plt.ylabel('Iq')

plt.title('PMS Position Controller Test')
plt.legend()
plt.show()
