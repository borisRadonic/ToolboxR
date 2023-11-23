import pandas as pd

data = pd.read_csv('TestPMSMPos.dat', sep='\s+', header=0)  # sep='\s+' handles any number of spaces or tabs as delimiters
import matplotlib.pyplot as plt

plt.figure(figsize=(10, 6))
plt.plot(data['Time'], data['refIq'], label='refIq')
plt.plot(data['Time'], data['Iq'], label='Iq')
plt.plot(data['Time'], data['refVel'], label='refVel')
plt.plot(data['Time'], data['Vel'], label='Vel')
plt.plot(data['Time'], data['refPos'], label='refPos')
plt.plot(data['Time'], data['isPos'], label='isPos')

plt.xlabel('Time')

plt.ylabel('Iq')

plt.title('PMS Position Controller Test')
plt.legend()
plt.show()
