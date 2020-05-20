import numpy as np
from ev3dev2.motor import Motor, OUTPUT_A, OUTPUT_B, OUTPUT_D
from ev3dev2.sensor import INPUT_4
from ev3dev2.sensor.lego import GyroSensor

from model_parameters import cond_i

gyro = GyroSensor(INPUT_4)
motor_dx = Motor(OUTPUT_A)
motor_sx = Motor(OUTPUT_D)
kickstand_servo = Motor(OUTPUT_B)


def fast_read(file):
    file.seek(0)
    return int(file.read().decode().strip())


def fast_write(file, value):
    file.truncate(0)
    file.write(str(int(value)))
    file.flush()


gyro_angle = open(gyro._path + "/value0", "rb")
gyro_rate = open(gyro._path + "/value1", "rb")
motor_dx_position = open(motor_dx._path + "/position", "rb")
motor_sx_position = open(motor_sx._path + "/position", "rb")
motor_dx_speed_read = open(motor_dx._path + "/speed_sp", "rb")
motor_sx_speed_read = open(motor_sx._path + "/speed_sp", "rb")
motor_dx_speed_write = open(motor_dx._path + "/duty_cycle_sp", "w")
motor_sx_speed_write = open(motor_sx._path + "/duty_cycle_sp", "w")
kickstand_servo_speed = open(kickstand_servo._path + "/duty_cycle_sp", "w")


# psi
def get_gyro_angle():
    return np.deg2rad(fast_read(gyro_angle)) - cond_i


# psi_dot
def get_gyro_angular_velocity():
    return np.deg2rad(fast_read(gyro_rate))


# theta
def get_avg_position():
    return np.average([np.deg2rad(fast_read(motor_dx_position)), np.deg2rad(fast_read(motor_sx_position))])


# theta_dot
def get_speed():
    degrees_per_s = np.average([fast_read(motor_dx_speed_read), motor_sx_speed_read])
    return np.deg2rad(degrees_per_s)
