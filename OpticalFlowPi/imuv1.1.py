from mpu6050 import mpu6050
sen = mpu6050(0x68)
import numpy as np
import time 



pitch, roll = 0.0,0.0

ac = sen.get_accel_data()
gy = sen.get_gyro_data()
p = np.pi

angleX = np.arctan2(ac["y"],ac["z"])*180/p
angleY = np.arctan2(ac["x"],ac["z"])*180/p
time_taken = 0.0


while(True):
    start = time.time()
    ac = sen.get_accel_data()
    gy = sen.get_gyro_data()
    
    accDataX = np.arctan2(ac["y"],ac["z"])
    accDataY = np.arctan2(ac["x"],ac["z"])
    
    angleX = 0.98*(angleX + gy["x"]*time_taken) + 0.02*accDataX*180/p
    
    angleY = 0.98*(angleY + gy["y"]*time_taken) - 0.02*accDataY*180/p
    
    s= str(angleX) + '\t'+ str(angleY) + '\n' 
    file = open('imu_angleXY.txt','w')
    file.write(s)
    # writing the data to a file imu_angleXY
    # run logic here
    file.close()
    
    print("X",angleX)
    print("Y",angleY)
    time_taken = time.time() - start

