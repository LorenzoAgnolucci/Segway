from datetime import datetime, timedelta

import numpy as np
from ev3dev2.motor import SpeedPercent

from model_parameters import Kx, Ki
from schedule import every, SampleData
from sensors import get_avg_position, gyro_angle, get_speed, gyro_angular_velocity, motor_a, motor_b

SAMPLING_INTERVAL = timedelta(milliseconds=1)
theta_int = 0


@every(SAMPLING_INTERVAL)
def control_loop(sampling_data: SampleData):
    global theta_int
    current_time = datetime.now()
    theta_int += (current_time - sampling_data.last_cycle_end) / timedelta(seconds=1) * theta_int
    sampling_data.last_cycle_end = current_time

    params = np.hstack((Kx[0, :].A1, Ki))
    x = -np.array([get_avg_position(), gyro_angle(), get_speed(), gyro_angular_velocity(), theta_int])
    engine_speed = np.dot(params, x)
    engine_percent_gain = 100 / 9
    engine_speed *= engine_percent_gain

    motor_a.on(SpeedPercent(engine_speed))
    motor_b.on(SpeedPercent(engine_speed))
