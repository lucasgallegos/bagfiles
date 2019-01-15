import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal


# Do the following in Terminal with a Roscore Running
# $rostopic echo -b <name>.bag -p /wrench > <name>.csv

data = pd.read_csv("hebi_bag.csv")

time = data["%time"]-(data["%time"][0])

fx = data["field.wrench.force.x"]
fy = data["field.wrench.force.y"]
fz = data["field.wrench.force.z"]

tx = data["field.wrench.torque.x"]
ty = data["field.wrench.torque.y"]
tz = data["field.wrench.torque.z"]

#Digital Signal Processing 
#Butterworth Filter

fs = 124.956672444 #sampling frequency len(fx)/57.7
#print(time)
#print(fs)

fc = 1  # Cut-off frequency of the filter
w = fc / (fs / 2) # Normalize the frequency
b, a = signal.butter(1, w, 'low')

#Forces
plt.subplot(3,2,1)
plt.plot(time,fx, label='No Filter')
#output_fx = signal.filtfilt(b, a, fx) #Forward and Backward Filter
output_fx = signal.lfilter(b, a, fx) #Forward filter
#plt.plot(time, output_fx, label='Filtered FiltFilt',linewidth=2)
plt.plot(time, output_fx, label='Filtered',linewidth=2)
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('Fx [N]')
plt.title('Force')

plt.suptitle('Low Pass Filter',fontsize=16)

plt.subplot(3,2,3)
plt.plot(time,fy)
output_fy = signal.lfilter(b, a, fy)
plt.plot(time, output_fy, label='Filtered',linewidth=2)
plt.xlabel('time [s]')
plt.ylabel('Fy [N]')

plt.subplot(3,2,5)
plt.plot(time,fz)
output_fz = signal.lfilter(b, a, fz)
plt.plot(time, output_fz, label='Filtered',linewidth=2)
plt.xlabel('time [s]')
plt.ylabel('Fz [N]')

#Torques
plt.subplot(3,2,2)
plt.plot(time,tx, label='No Filter')
output_tx = signal.lfilter(b, a, tx)
plt.plot(time, output_tx, label='Filtered',linewidth=2)
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('Tx [N-m]')
plt.title('Torque')

plt.subplot(3,2,4)
plt.plot(time,ty)
output_ty = signal.lfilter(b, a, ty)
plt.plot(time, output_ty, label='Filtered',linewidth=2)
plt.xlabel('time [s]')
plt.ylabel('Ty [N-m]')

plt.subplot(3,2,6)
plt.plot(time,tz)
output_tz = signal.lfilter(b, a, tz)
plt.plot(time, output_tz, label='Filtered',linewidth=2)
plt.xlabel('time [s]')
plt.ylabel('Tz [N-m]')


plt.show()


