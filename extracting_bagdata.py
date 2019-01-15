#<<<<<<< HEAD
f = open('ur3_wrench_noforces.csv', 'w')
#=======
import string
f = open('ur3_wrench_noforces.csv', 'r')
print(f)
fx = []
fy = []
fz = []
tx = []
ty = []
tz = []
n=0
for line in f:
    line = line.strip()
    if n == 3:
        fx.append(float(line[3:]))
        n = n-1
    if n == 2:
        fy.append(float(line[3:]))
        n = n-1
    if n == 1:
        fz.append(float(line[3:]))
        n = n-1
    if n == 15:
        tx.append(float(line[3:]))
        n = n-5
    if n == 10:
        ty.append(float(line[3:]))
        n = n-5
    if n == 5:
        tz.append(float(line[3:]))
        n = n-5
    if line == 'force:':
        n = 3
    if line == 'torque:':
        n = 15
    print(fx)
    print(fy)
    print(fz)
    print(tx)
    print(ty)
    print(tz)
#>>>>>>> 563b0d365cab0245bca03d1961583aa21c16c80e
