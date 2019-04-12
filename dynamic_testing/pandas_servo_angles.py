import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
import numpy as np



# Do the following in Terminal with a Roscore Running
# $rostopic echo -b <name>.bag -p /wrench > <name>.csv

data1 = pd.read_csv("hebi_drop1.csv") #reads the data from the csv file

time = data1["%time"]-(data1["%time"][0]) #Pulls and normalizes the data to start from t=0

angle1 = data1["field.Angle1"] #See pandas documentation on how to pull data 
angle2 = data1["field.Angle2"]
angle3 = data1["field.Angle3"]

angle4 = data1["field.Angle4"]
angle5 = data1["field.Angle5"]
angle6 = data1["field.Angle6"]

time = time.values #converts to np array

angle1 = angle1.values
angle2 = angle2.values
angle3 = angle3.values

angle4 = angle4.values
angle5 = angle5.values
angle6 = angle6.values


#Forces
plt.subplot(6,2,1)
plt.plot(time,angle1, label='Series Elastic Robot')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('Angle [deg]')
#plt.title('Force')

plt.subplot(6,2,3)
plt.plot(time,angle2, label='Series Elastic Robot')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('Angle [deg]')


plt.subplot(6,2,5)
plt.plot(time,angle3, label='Series Elastic Robot')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('Angle [deg]')


#Torques

plt.subplot(6,2,7)
plt.plot(time,angle4, label='Series Elastic Robot')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('Angle [deg]')
#plt.title('Torque')

plt.subplot(6,2,9)
plt.plot(time,angle5, label='Series Elastic Robot')
plt.legend()
#plt.xlabel('time [s]')
plt.ylabel('Angle [deg]')


plt.subplot(6,2,11)
plt.plot(time,angle6, label='Series Elastic Robot')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('Angle [deg]')

plt.suptitle('Servo Angles',fontsize=16)
plt.show()


