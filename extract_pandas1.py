import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
import numpy as np
import scipy
from kalman import SingleStateKalmanFilter


# Do the following in Terminal with a Roscore Running
# $rostopic echo -b <name>.bag -p /wrench > <name>.csv

data = pd.read_csv("ur3_wrench.csv")

time = data["%time"]-(data["%time"][0])

time = time.values

#print(len(time_array))

fx = data["field.wrench.force.x"] 
fy = data["field.wrench.force.y"]
fz = data["field.wrench.force.z"]

tx = data["field.wrench.torque.x"]
ty = data["field.wrench.torque.y"]
tz = data["field.wrench.torque.z"]

fx = fx.values
fy = fy.values
fz = fz.values

tx = tx.values
ty = ty.values
tz = tz.values

#############################################################################

#Digital Signal Processing 
#Butterworth Filter

FX = []
FY = []
FZ = []

TX = []
TY = []
TZ = []

fx_low = np.zeros(fx.size)
fy_low = np.zeros(fy.size)
fz_low = np.zeros(fz.size)

tx_low = np.zeros(tx.size)
ty_low = np.zeros(ty.size)
tz_low = np.zeros(tz.size)

fs = 124.956672444 #sampling frequency len(fx)/57.7 subject to change

fc = 55 # Cut-off frequency of the filter
w = fc / (fs / 2) # Normalize the frequency
b, a = signal.butter(3, w, 'low')

zi = signal.lfilter_zi(b,a) #This sets the initial state so that initial values match

for data in fx:
	z_x , y_x = signal.lfilter(b, a, [data],zi=zi*fx[0]) #Forward filters
	FX.append(z_x)

for data in fy:
	z_y , y_y = signal.lfilter(b, a, [data], zi=zi*fy[0])
	FY.append(z_y)

for data in fz:
	z_z , y_z = signal.lfilter(b, a, [data], zi=zi*fz[0])
	FZ.append(z_z)

for data in tx:
	z_tx , y_tx = signal.lfilter(b, a, [data], zi=zi*tx[0])
	TX.append(z_tx)

for data in ty:
	z_ty , y_ty = signal.lfilter(b, a, [data], zi=zi*ty[0])
	TY.append(z_ty)

for data in tz:
	z_tz , y_tz = signal.lfilter(b, a, [data], zi=zi*tz[0])
	TZ.append(z_tz)

##############################################################################

#Kalman Flilter

A = 1
C = 1
B = 0
Q = 0.005
R = 2
x = 0
P = 1

kalman_filter_fx = SingleStateKalmanFilter(A, B, C, x, P, Q, R)
kalman_filter_fy = SingleStateKalmanFilter(A, B, C, x, P, Q, R)
kalman_filter_fz = SingleStateKalmanFilter(A, B, C, x, P, Q, R)

kalman_filter_tx = SingleStateKalmanFilter(A, B, C, x, P, Q, R)
kalman_filter_ty = SingleStateKalmanFilter(A, B, C, x, P, Q, R)
kalman_filter_tz = SingleStateKalmanFilter(A, B, C, x, P, Q, R)


kalman_filter_est_fx = []
kalman_filter_est_fy = []
kalman_filter_est_fz = []

kalman_filter_est_tx = []
kalman_filter_est_ty = []
kalman_filter_est_tz = []

#Force

for data in fx:
	kalman_filter_fx.step(0, data)
	kalman_filter_est_fx.append(kalman_filter_fx.current_state())

for data in fy:
	kalman_filter_fy.step(0, data)
	kalman_filter_est_fy.append(kalman_filter_fy.current_state())

for data in fz:
	kalman_filter_fz.step(0, data)
	kalman_filter_est_fz.append(kalman_filter_fz.current_state())

#Torque

for data in tx:
	kalman_filter_tx.step(0, data)
	kalman_filter_est_tx.append(kalman_filter_tx.current_state())

for data in ty:	
	kalman_filter_ty.step(0, data)
	kalman_filter_est_ty.append(kalman_filter_ty.current_state())

for data in tz:
	kalman_filter_tz.step(0, data)
	kalman_filter_est_tz.append(kalman_filter_tz.current_state())


#Forces
plt.subplot(3,2,1)
plt.plot(time,fx, label='No Filter')
plt.plot(time, FX, label='Low Pass Butterworth',linewidth=1)
plt.plot(time,kalman_filter_est_fx, label='Single State Kalman')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('Fx [N]')
plt.title('Force')

plt.subplot(3,2,3)
plt.plot(time,fy, label='No Filter')
plt.plot(time, FY, label='LowPassButterworth',linewidth=1)
plt.plot(time,kalman_filter_est_fy, label='Single State Kalman')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('Fy [N]')


plt.subplot(3,2,5)
plt.plot(time,fz, label='No Filter')
plt.plot(time, FZ, label='LowPassButterworth',linewidth=1)
plt.plot(time,kalman_filter_est_fz, label='Single State Kalman')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('Fz [N]')


#Torques

plt.subplot(3,2,2)
plt.plot(time,tx, label='No Filter')
plt.plot(time, TX, label='LowPassButterworth',linewidth=1)
plt.plot(time,kalman_filter_est_tx, label='Single State Kalman')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('Tx [N]')
plt.title('Torque')

plt.subplot(3,2,4)
plt.plot(time,ty, label='No Filter')
plt.plot(time, TY, label='LowPassButterworth',linewidth=1)
plt.plot(time,kalman_filter_est_ty, label='Single State Kalman')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('Ty [N]')


plt.subplot(3,2,6)
plt.plot(time,tz, label='No Filter')
plt.plot(time, TZ, label='LowPassButterworth',linewidth=1)
plt.plot(time,kalman_filter_est_tz, label='Single State Kalman')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('Tz [N]')

plt.suptitle('Low Pass Filter',fontsize=16)
plt.show()


