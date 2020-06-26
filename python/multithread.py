import logging
import os
import threading
from datetime import datetime, timedelta
import time

from ev3dev2.motor import SpeedPercent

print("imported: logging, datetime, threading")

import numpy as np

print("import numpy as np")

from sensors import get_avg_position, get_gyro_angle, get_speed, get_gyro_angular_velocity, motor_dx, motor_sx, gyro, \
    kickstand_servo, kickstand_servo_speed, fast_write, motor_dx_speed_write, motor_sx_speed_write

print("sensors")

from model_parameters import Kx, Ki
from schedule import every, SampleData
from io import StringIO

print("end imports")

logging.getLogger('transitions').setLevel(logging.INFO)
log_stream = StringIO()
logging.basicConfig(
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    level=logging.DEBUG,
    datefmt='%Y-%m-%d %H:%M:%S',
    stream=log_stream)

output_data = np.empty((0, 6))

stop_control_thread: bool = False
disable_control: bool = False
engine_power: int = 0
SAMPLING_INTERVAL = 0.070


class Control(threading.Thread):
    def __init__(self):
        super().__init__()

    def run(self):
        theta_int = 0
        previous_start_time = time.time()
        accumulated_time = 0
        while not stop_control_thread:
            current_start_time = time.time()
            theta_int += (current_start_time - previous_start_time) * get_avg_position()
            accumulated_time += current_start_time - previous_start_time

            params = np.hstack((np.ravel(Kx[0, :]), Ki))
            x = np.array([-get_avg_position(), get_gyro_angle(), -get_speed(), get_gyro_angular_velocity(), -theta_int])
            #logging.info(f"[pos, angle, speed, ang_vel, theta_int, time] =  {x} {accumulated_time}")
            global output_data
            output_data = np.vstack([output_data, np.hstack([x, accumulated_time])])
            engine_speed = np.dot(params, x)
            engine_percent_gain = 100 / 9
            engine_speed *= engine_percent_gain

            if disable_control:
                engine_speed = engine_power

            motor_dx.on(np.clip(engine_speed, -100, 100))
            motor_sx.on(np.clip(engine_speed, -100, 100))

            while time.time() - current_start_time < SAMPLING_INTERVAL:
                time.sleep(0.001)

            previous_start_time = current_start_time


def main():
    os.setpriority(os.PRIO_PROCESS, 0, -20)
    calibrate()

    control_thread = Control()
    control_thread.start()

    kickstand_up()
    time.sleep(1)

    segway()
    time.sleep(10)

    kickstand_down()
    time.sleep(1)

    post_kickstand_down()

    global engine_power
    global disable_control
    global stop_control_thread

    engine_power = -50
    disable_control = False

    time.sleep(0.30)
    stop_control_thread = True

    engine_stop()

    with open("output", "w") as f:
        np.save("output_data", output_data)


def engine_stop():
    global engine_power
    engine_power = 0
    motor_dx.off()
    motor_sx.off()


def post_kickstand_down():
    kickstand_servo.off()


def kickstand_down():
    kickstand_servo_angular_velocity = -20
    kickstand_servo.on(SpeedPercent(kickstand_servo_angular_velocity))


def segway():
    kickstand_servo_angular_velocity = 0
    kickstand_servo.on(SpeedPercent(kickstand_servo_angular_velocity))


def kickstand_up():
    kickstand_servo_angular_velocity = 20
    kickstand_servo.on(SpeedPercent(kickstand_servo_angular_velocity))

def calibrate():
    gyro.calibrate()
    gyro.mode = gyro.MODE_GYRO_G_A
    motor_sx.reset()
    motor_dx.reset()


if __name__ == '__main__':
    main()
