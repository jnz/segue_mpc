# Glue code for linear MPC of a two-wheel robot
#
# Tasks:
#   - Read in sensor data from UART/serial port from Arduino
#   - Feed sensor data (accelerometer/gyroscope) to external attitude
#     estimation library (libpynav.so)
#   - Run libctrl.so for MPC command
#   - Send back motor commands via UART to Arduino


import numpy as np
import ctypes

lib = ctypes.cdll.LoadLibrary('./libpynav.so')    # State estimation library
libMPC = ctypes.cdll.LoadLibrary('./libctrl.so')  # MPC control library

import time
import random
from datetime import datetime

import time
import serial

# Connected to Arduino
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

roll_c = ctypes.c_float()     # attitude (roll) output by libpynav
pitch_c = ctypes.c_float()    # attitude (pitch) output by libpynav
yaw_c = ctypes.c_float()      # attitude (yaw) output by libpynav
dt_sec = ctypes.c_float()     # time interval (in sec) for libpynav integration interval
timestamp = ctypes.c_uint()   # calculate timestamp for libpynav
tempC_c = ctypes.c_float(30.0) # feed some temperature to libpynav

epoch = 0
last_fps_calc = 0
current_fps = 0
imuId = 0
timestamp0 = time.time()
timestamp_last = timestamp0
sample_time = 1/50.0
frame_time = 0
start_time = time.time()

# induce kicks for system identification
timestamp_last_kick = time.time()
u_kick = 0

isMpcInit = False              # libctrl.so initialized and ready?
u = 0                          # motor command between -1.0 and 1.0 (0.0 = off)
theta_correction = 0.8         # robot angle correction in degree
theta_c = ctypes.c_double(pitch_c.value*np.pi/180.0) # robot angle (0 = upright)
thetadot_c = ctypes.c_double(0) # rotation rate of robot
thetadot_correction_c = ctypes.c_float(0) # gyroscope/rotation rate bias correction
pos_x = 0                      # start robot position at 0
vel_x = 0                      # initial velocity
meters_per_tick = 0.00027195   # convert wheel odometry ticks to meter
VEL_CLIP = 0.5                 # to be safe: don't supply large velocities to controller

# Angle ctrl
thetaCtrlMax = 5*np.pi/180
thetaCtrl = 0.0

pid_heading_p = 0.01
pid_setpoint_heading = 0
pid_max_heading_err = 8.0*np.pi/180.0
pid_heading_pwm = 0.0
heading_err = 0

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def motorStop():
    ser.write(b'<0,0>')        # set PWM to 0 (= motors off)

def shutdown():
    motorStop()
    quit()

def normalize_angle(angle):
        if angle < -np.pi:
            angle = 2.0 * np.pi + angle
        if angle > np.pi:
            angle = -2.0 * pi + angle
        return angle

flog = open("logfile.txt", "w") # log data (can be viewed with MATLAB/analyze_log.m)

buffer_string = ''  # UART input buffer for parser
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

        # split up complete lines and only consume the latest message
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

        # input data from Arduino is a space separated list of ASCII text values
        dt_ms_ticks = float(cl[1]) # time since last wheel tick measurement
        ticks_left = float(cl[8])  # ticks from left wheel since last message
        ticks_right = float(cl[9]) # ticks from right wheel since last message
        voltage = float(cl[10])/10.0 # voltage of Arduino/motors (2x3.7 V nominal level)
        inputfreqHz = int(cl[11])  # number of messages the Arduino receives from the Raspberry per second

        dx = 0.5*(ticks_left+ticks_right)*meters_per_tick # wheel odometry increment
        pos_x = pos_x + dx
        vel_x = dx/(dt_ms_ticks/1000.0) # horizontal velocity from wheel ticks

        # convert 16-bit raw MPU6050 IMU data to rad/s and m/s² (max. 4G and 1000 °/s)
        accX =-float(cl[2 + 1])*((1/8192.0)*9.81)
        accY =-float(cl[2 + 0])*((1/8192.0)*9.81)
        accZ =-float(cl[2 + 2])*((1/8192.0)*9.81)
        gyrX =-float(cl[2 + 4])*((1/32.8)*np.pi/180)
        gyrY =-float(cl[2 + 3])*((1/32.8)*np.pi/180)
        gyrZ =-float(cl[2 + 5])*((1/32.8)*np.pi/180)

        epoch += 1
        time_sec = timestamp_current - timestamp0
        time_sec_c = ctypes.c_uint(int(1000*time_sec)) # timestamp in milliseconds

        dt_sec = timestamp_current - timestamp_last
        dt_sec_c = ctypes.c_float(dt_sec)
        timestamp_last = timestamp_current

        # convert data for C-library (libpynav.so)
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

            if np.abs(theta_c.value) < 35.0*np.pi/180.0 and isMpcInit:
                thetaCtrl = constrain(thetaCtrl, -thetaCtrlMax, thetaCtrlMax)
                thetaCtrl_c = ctypes.c_double(thetaCtrl)
                libMPC.MPC_SetThetaRef(thetaCtrl_c)

                heading_err = normalize_angle(pid_setpoint_heading - yaw_c.value*np.pi/180)
                heading_err_limit = constrain(heading_err, -pid_max_heading_err, pid_max_heading_err)
                pid_setpoint_heading = heading_err_limit + yaw_c.value*np.pi/180
                pid_heading_pwm = 0.025*heading_err_limit/pid_max_heading_err

                u_c = ctypes.c_double(u)
                pos_x_c = ctypes.c_double(pos_x)
                vel_x_c = ctypes.c_double(constrain(vel_x, -VEL_CLIP, VEL_CLIP))
                statusMPC = libMPC.MPC_Run(pos_x_c, vel_x_c, theta_c, thetadot_c, ctypes.byref(u_c))
                if statusMPC == True:
                    u = u_c.value
                    # if time.time() - timestamp_last_kick > 3.0:
                    #     timestamp_last_kick = time.time()
                    #     u_kick = 3
                    #     l = [0.7, 0.8, 0.9]
                    #     u_kick_value = random.choice(l)

                    # if u_kick > 0:
                    #     u_kick = u_kick - 1
                    #     u = u_kick_value
                    #     if u_kick == 0 and u_kick_value > 0:
                    #         u_kick = 3
                    #         u_kick_value = -u_kick_value

                    # Experimental LQR:
                    # u = -(-0.001)*pos_x_c.value -(0.0096)*vel_x_c.value -(-2.2)*theta_c.value -(-0.5)*thetadot_c.value;
                else:
                    print("No MPC solution found")
            else:
                u = 0

            if isMpcInit == False and np.abs(theta_c.value) < 7.0*np.pi/180.0:
                # Robot model system parameters (Elegoo Tumbller robot kit)
                pos_x_c = ctypes.c_double(pos_x)
                vel_x_c = ctypes.c_double(constrain(vel_x, -VEL_CLIP, VEL_CLIP))
                f1_c = ctypes.c_double(-8.0)
                f2_c = ctypes.c_double(0.1)
                f3_c = ctypes.c_double(0.0)
                f4_c = ctypes.c_double(40.0)
                b1_c = ctypes.c_double(5.83)
                b2_c = ctypes.c_double(-150) # ext. state
                rbar = ctypes.c_double(0.7) # 0.7-0.8 for ext. state, way less for non-ext state
                if libMPC.MPC_Init(pos_x_c, vel_x_c, theta_c, thetadot_c, f1_c, f2_c, f3_c, f4_c, b1_c, b2_c, rbar) == False:
                    print("Failed to initialize MPC library")
                    shutdown()
                else:
                    isMpcInit = True
                    start_time = time.time()
        else:
            u = 0

        u = constrain(u, -1.0, 1.0)

        print("Time %.3f %2i Hz (%2.0f/%2.0fms) AHRS=%i CTRL=%i u=%5.2f x=%8.3f v=%6.3f theta=%6.1f dot=%6.0f V=%.1f %i Hz | Theta Offset %.1f deg | Heading corr: %.2f Err: %.1f" % (time_sec, current_fps, frame_time*1000, sample_time*1000, statusAhrsValid, isMpcInit, u, pos_x, vel_x, theta_c.value*180.0/np.pi, thetadot_c.value*180.0/np.pi, voltage, inputfreqHz, thetaCtrl*180.0/np.pi, pid_heading_pwm, heading_err*180.0/np.pi)) # , flush=True) # end='\r'
        flog.write("%8.3f %i %5.2f %8.3f %6.3f %6.1f %6.1f %6.1f %6.1f %6.1f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.1f\n" % (time_sec, statusAhrsValid, u, pos_x, vel_x, theta_c.value*180.0/np.pi, thetadot_c.value*180.0/np.pi, roll_c.value, pitch_c.value, yaw_c.value, accX, accY, accZ, gyrX, gyrY, gyrZ, thetaCtrl*180.0/np.pi))

        uL = constrain(u+pid_heading_pwm, -1.0, 1.0)
        uR = constrain(u-pid_heading_pwm, -1.0, 1.0)
        ser.write(b'<%i,%i>' % (uL*255.0, uR*255.0))
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

