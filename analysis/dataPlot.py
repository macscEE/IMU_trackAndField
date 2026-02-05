import numpy as np
import IMUprocess as imu
import matplotlib.pyplot as plt

# Import the file with data
# file structure: first 4 data are q.w, q.x, q.y, q.z; then ax, ay, az in m/s^2
data = np.loadtxt("putty20260205COM7.txt", delimiter=",")

# Splitting data in different variables
qW = data[:, 0]
qX = data[:, 1]
qY = data[:, 2]
qZ = data[:, 3]

aX = data[:, 4]
aY = data[:, 5]
aZ = data[:, 6]

# np.set_printoptions(threshold=np.inf) to print all data without truncation
imuData = imu.IMUprocess(qW, qX, qY, qZ, aX, aY, aZ)
absAccData = imuData.absAcc()

plt.plot(absAccData)
plt.title("Absolute Acceleration from IMU data")
plt.xlabel("Samples")
plt.ylabel("Acceleration (m/s^2)")
plt.grid()
plt.legend(["|a|"])


plt.show()

plt.figure("Absolute Velocity")
absVelData = imuData.absVelocity()*3.6  # Convert m/s to km/h
plt.plot(absVelData)
plt.title("Absolute Velocity from IMU data")
plt.xlabel("Samples")
plt.ylabel("Velocity (km/h)")
plt.grid()
plt.legend(["|v|"])
plt.show()