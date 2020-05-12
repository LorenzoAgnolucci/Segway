import numpy as np
from ev3dev2.motor import Motor, OUTPUT_A, OUTPUT_D
from ev3dev2.sensor import INPUT_4
from ev3dev2.sensor.lego import GyroSensor

gyro = GyroSensor(INPUT_4)
motor_a = Motor(OUTPUT_A)
motor_b = Motor(OUTPUT_D)


# psi
def gyro_angle():
    return np.deg2rad(gyro.angle)


# psi_dot
def gyro_angular_velocity():
    return np.deg2rad(gyro.rate)


# theta
def get_avg_position():
    return np.average(motor_a.position, motor_b.position)


# theta_dot
def get_speed():
    degrees_per_s = np.average(motor_a.speed, motor_b.speed) / np.average(motor_a.count_per_rot, motor_b.count_per_rot)
    return np.deg2rad(degrees_per_s)
