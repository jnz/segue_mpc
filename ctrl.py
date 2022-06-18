import numpy as np
import ctypes

lib = ctypes.cdll.LoadLibrary('./libpynav.so')
libMPC = ctypes.cdll.LoadLibrary('./libctrl.so')

import time
import random
from datetime import datetime

import time
import serial

ser = serial.Serial(
        port='/dev/ttyS0',
        baudrate = 250000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.001
)

if lib.pynavInit() == False:
    print("pynav library failed to initialize")
    quit()

tempC_c = ctypes.c_float(30.0)
roll_c = ctypes.c_float()
pitch_c = ctypes.c_float()
yaw_c = ctypes.c_float()
dt_sec = ctypes.c_float()
timestamp = ctypes.c_uint()

epoch = 0
last_fps_calc = 0
current_fps = 0
imuId = 0
timestamp0 = time.time()
timestamp_last = timestamp0
sample_time = 1/100.0
frame_time = 0
start_time = time.time()

isMpcInit = False
u = 0
theta_correction = 0.5
theta_c = ctypes.c_double(pitch_c.value*np.pi/180.0)
thetadot_c = ctypes.c_double(0)
thetadot_correction_c = ctypes.c_float(0)
pos_x = 0;
vel_x = 0
meters_per_tick = 0.00027195

def motorStop():
    ser.write(b'<0,0>')

def shutdown():
    motorStop()
    quit()

flog = open("logfile.txt", "w")

buffer_string = ''
try:
    while True:
        timestamp_current = time.time()

        if timestamp_current - last_fps_calc >= 1.0:
            last_fps_calc = timestamp_current
            current_fps = epoch
            epoch = 0

        bytesReady = ser.inWaiting()
        if bytesReady < 1:
            time.sleep(0)
            continue;
        buffer_string += ser.read(bytesReady).decode()

        if '\n' in buffer_string: 
            lines = buffer_string.split('\n')
            cc = lines[-2]
            buffer_string = lines[-1]
        else:
            continue

        cl = cc.split(' ')
        if len(cl) != 12 or cl[0] != "L":
            print("Incomplete low-level machine message")
            continue

        dt_ms_ticks = float(cl[1]) # time since last wheel tick measurement
        ticks_left = float(cl[8])
        ticks_right = float(cl[9])
        voltage = float(cl[10])/10.0
        inputfreqHz = int(cl[11])
        dx = 0.5*(ticks_left+ticks_right)*meters_per_tick
        pos_x = pos_x + dx
        vel_x = dx/(dt_ms_ticks/1000.0)

        accX =-float(cl[2 + 1])*((1/8192.0)*9.81)
        accY =-float(cl[2 + 0])*((1/8192.0)*9.81)
        accZ =-float(cl[2 + 2])*((1/8192.0)*9.81)
        gyrX =-float(cl[2 + 4])*((1/32.8)*np.pi/180)
        gyrY =-float(cl[2 + 3])*((1/32.8)*np.pi/180)
        gyrZ =-float(cl[2 + 5])*((1/32.8)*np.pi/180)

        epoch += 1
        time_sec = timestamp_current - timestamp0
        time_sec_c = ctypes.c_uint(int(1000*time_sec)) # timestamp in milliseconds

        # if isMpcInit == True:
        #     if timestamp_current - start_time > 10.0:
        #         shutdown()

        dt_sec = timestamp_current - timestamp_last
        dt_sec_c = ctypes.c_float(dt_sec)
        timestamp_last = timestamp_current

        accX_c = ctypes.c_float(accX)
        accY_c = ctypes.c_float(accY)
        accZ_c = ctypes.c_float(accZ)
        gyrX_c = ctypes.c_float(gyrX)
        gyrY_c = ctypes.c_float(gyrY)
        gyrZ_c = ctypes.c_float(gyrZ)

        statusAhrsValid = lib.pynavImuUpdate(imuId, time_sec_c, dt_sec_c, accX_c, accY_c, accZ_c, gyrX_c, gyrY_c, gyrZ_c, tempC_c, ctypes.byref(roll_c), ctypes.byref(pitch_c), ctypes.byref(yaw_c))

        if statusAhrsValid > 0:
            lib.pynavImuGetGyroBias(imuId, 0, ctypes.byref(thetadot_correction_c), 0)
            theta_c = ctypes.c_double(-pitch_c.value*np.pi/180.0 + theta_correction*np.pi/180.0)
            thetadot_c = ctypes.c_double(-(gyrY - thetadot_correction_c.value))

            if np.abs(theta_c.value) < 22.0*np.pi/180.0 and isMpcInit:
                u_c = ctypes.c_double(u)
                pos_x_c = ctypes.c_double(pos_x)
                vel_x_c = ctypes.c_double(vel_x)
                statusMPC = libMPC.MPC_Run(pos_x_c, vel_x_c, theta_c, thetadot_c, ctypes.byref(u_c))
                if statusMPC == True:
                    u = u_c.value
                    # u = -(-0.001)*pos_x_c.value -(0.0096)*vel_x_c.value -(-2.2)*theta_c.value -(-0.5)*thetadot_c.value;
                else:
                    print("No MPC solution found")
                    # shutdown()
            else:
                u = 0

            if isMpcInit == False and np.abs(theta_c.value) < 7.0*np.pi/180.0:
                pos_x_c = ctypes.c_double(pos_x)
                vel_x_c = ctypes.c_double(vel_x)
                f1_c = ctypes.c_double(-7.54)
                f2_c = ctypes.c_double(0.03)
                f3_c = ctypes.c_double(0.0)
                f4_c = ctypes.c_double(30.0)
                b1_c = ctypes.c_double(5.73)
                b2_c = ctypes.c_double(-200)
                rbar = ctypes.c_double(0.6)
                if libMPC.MPC_Init(pos_x_c, vel_x_c, theta_c, thetadot_c, f1_c, f2_c, f3_c, f4_c, b1_c, b2_c, rbar) == False:
                    print("Failed to initialize MPC library")
                    shutdown()
                else:
                    isMpcInit = True
                    start_time = time.time()
        else:
            u = 0

        if u > 1.0:
            u = 1.0
        if u < -1.0:
            u = -1.0

        print("Time %.3f %2i Hz (%2.0f/%2.0fms) AHRS=%i CTRL=%i u=%5.2f x=%8.3f v=%6.3f theta=%6.1f dot=%6.0f V=%.1f %i Hz" % (time_sec, current_fps, frame_time*1000, sample_time*1000, statusAhrsValid, isMpcInit, u, pos_x, vel_x, theta_c.value*180.0/np.pi, thetadot_c.value*180.0/np.pi, voltage, inputfreqHz)) # , flush=True) # end='\r'
        flog.write("%8.3f %i %5.2f %8.3f %6.3f %6.1f %6.1f %6.1f %6.1f %6.1f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f\n" % (time_sec, statusAhrsValid, u, pos_x, vel_x, theta_c.value*180.0/np.pi, thetadot_c.value*180.0/np.pi, roll_c.value, pitch_c.value, yaw_c.value, accX, accY, accZ, gyrX, gyrY, gyrZ))

        ser.write(b'<%i,%i>' % (u*255.0, u*255.0))
        # ser.write(b'<%i,%i>' % (0, 0))

        # flush_start = time.time()
        # ser.flush()
        # flush_end = time.time() - flush_start
        # print("Time flush: %.3f ms" % (flush_end*1000))

        frame_time = time.time()-timestamp_current
        time.sleep(max(sample_time - frame_time, 0))

except KeyboardInterrupt:
    shutdown()
    pass

