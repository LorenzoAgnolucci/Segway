import numpy as np
from ev3dev2.motor import Motor, OUTPUT_A, OUTPUT_B, OUTPUT_D
from ev3dev2.sensor import INPUT_4
from ev3dev2.sensor.lego import GyroSensor

from model_parameters import cond_i

gyro = GyroSensor(INPUT_4)
motor_dx = Motor(OUTPUT_A)
motor_sx = Motor(OUTPUT_D)
kickstand_servo = Motor(OUTPUT_B)


# psi
def gyro_angle():
    return np.deg2rad(gyro.angle) - cond_i


# psi_dot
def gyro_angular_velocity():
    return np.deg2rad(gyro.rate)


# theta
def get_avg_position():
    return np.average([np.deg2rad(motor_dx.position), np.deg2rad(motor_sx.position)])


# theta_dot
def get_speed():
    degrees_per_s = np.average([motor_dx.speed, motor_sx.speed]) / np.average([motor_dx.count_per_rot, motor_sx.count_per_rot])
    return np.deg2rad(degrees_per_s)
