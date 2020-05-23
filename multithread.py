import logging
import threading
from datetime import datetime, timedelta
import time

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


stop_control_thread: bool = False
disable_control: bool = False
engine_power: int = 0
SAMPLING_INTERVAL = timedelta(milliseconds=1)


class Control(threading.Thread):
    def __init__(self):
        super().__init__()

    def run(self):
        theta_int = 0
        previous_start_time = datetime.now()
        while not stop_control_thread:
            current_start_time = datetime.now()
            theta_int += (current_start_time - previous_start_time) / timedelta(seconds=1) * get_avg_position()

            params = np.hstack((np.ravel(Kx[0, :]), Ki))
            x = np.array([get_avg_position(), get_gyro_angle(), get_speed(), get_gyro_angular_velocity(), theta_int])
            # print(f"[pos, angle, speed, ang_vel, theta_int] = {x}")
            logging.info("")
            engine_speed = np.dot(params, x)
            engine_percent_gain = 100 / 9
            engine_speed *= engine_percent_gain

            if disable_control:
                engine_speed = engine_power

            fast_write(motor_dx_speed_write, np.clip(engine_speed, -100, 100))
            fast_write(motor_sx_speed_write, np.clip(engine_speed, -100, 100))

            while datetime.now() - current_start_time < SAMPLING_INTERVAL:
                time.sleep(0.001)

            previous_start_time = current_start_time


def main():
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

    engine_power = -50
    disable_control = False

    time.sleep(0.30)
    stop_control_thread = True

    engine_stop()

    with open("output", "w") as f:
        f.write(log_stream.getvalue())


def engine_stop():
    global engine_power
    engine_power = 0
    fast_write(motor_dx_speed_write, 0)
    fast_write(motor_sx_speed_write, 0)


def post_kickstand_down():
    fast_write(kickstand_servo_speed, 0)


def kickstand_down():
    kickstand_servo_angular_velocity = -20
    fast_write(kickstand_servo_speed, kickstand_servo_angular_velocity)


def segway():
    kickstand_servo_angular_velocity = 0
    fast_write(kickstand_servo_speed, kickstand_servo_angular_velocity)


def kickstand_up():
    kickstand_servo_angular_velocity = 20
    fast_write(kickstand_servo_speed, kickstand_servo_angular_velocity)


def calibrate():
    gyro.calibrate()
    gyro.mode = gyro.MODE_GYRO_G_A
    motor_sx.reset()
    motor_dx.reset()
    motor_sx.run_direct()
    motor_dx.run_direct()
    kickstand_servo.run_direct()


if __name__ == '__main__':
    main()
