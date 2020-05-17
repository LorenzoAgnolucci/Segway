import logging
from datetime import datetime, timedelta

import numpy as np
from ev3dev2.motor import SpeedPercent
from transitions import Machine
from transitions.extensions.states import Timeout, add_state_features

from model_parameters import Kx, Ki
from schedule import every, SampleData
from sensors import get_avg_position, gyro_angle, get_speed, gyro_angular_velocity, motor_dx, motor_sx, gyro, \
    kickstand_servo

logging.getLogger('transitions').setLevel(logging.INFO)

logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.DEBUG,
    datefmt='%Y-%m-%d %H:%M:%S')


class Robot:
    kickstand_servo_angular_velocity: int = 0
    engine_power: int = 0
    enable_control = True

    def on_enter_calibration(self):
        gyro.calibrate()
        motor_sx.reset()
        motor_dx.reset()
        print(f"pos: [{motor_dx.position} {motor_sx.position}]")

    def on_enter_control(self):
        control_loop(self)

    def on_enter_kickstand_up(self):
        self.kickstand_servo_angular_velocity = 40
        kickstand_servo.on(SpeedPercent(self.kickstand_servo_angular_velocity))

    def on_enter_segway(self):
        self.kickstand_servo_angular_velocity = 0
        kickstand_servo.off()

    def on_enter_kickstand_down(self):
        self.kickstand_servo_angular_velocity = -20
        kickstand_servo.on(SpeedPercent(self.kickstand_servo_angular_velocity))

    def on_exit_kickstand_down(self):
        kickstand_servo.off()

    def on_enter_trust_test(self):
        self.engine_power = -50
        self.enable_control = False

    def on_engine_stop(self):
        self.engine_power = 0
        motor_dx.off()
        motor_sx.off()


@add_state_features(Timeout)
class RobotAutomata(Machine):
    states = [
        {"name": "idle"},
        {"name": "calibration", "timeout": 5, "on_timeout": "t1"},
        {"name": "waiting", "timeout": 1, "on_timeout": "t2"},
        {"name": "kickstand_up", "timeout": 1, "on_timeout": "t3"},
        {"name": "segway", "timeout": 10, "on_timeout": "t4"},
        {"name": "kickstand_down", "timeout": 1, "on_timeout": "t5"},
        {"name": "trust_test", "timeout": 0.1, "on_timeout": "t6"},
        {"name": "engine_stop", "timeout": 0.5, "on_timeout": "t7"},
        {"name": "end"}
    ]

    transitions = [["t0", "idle", "calibration"],
                   ["t1", "calibration", "waiting"],
                   ["t2", "waiting", "kickstand_up"],
                   ["t3", "kickstand_up", "segway"],
                   ["t4", "segway", "kickstand_down"],
                   ["t5", "kickstand_down", "trust_test"],
                   ["t6", "trust_test", "engine_stop"],
                   ["t7", "engine_stop", "end"]]


SAMPLING_INTERVAL = timedelta(milliseconds=1)
theta_int = 0


@every(SAMPLING_INTERVAL)
def control_loop(robot: Robot, sampling_data: SampleData):
    global theta_int
    current_time = datetime.now()
    theta_int += (current_time - sampling_data.last_cycle_end) / timedelta(seconds=1) * theta_int
    sampling_data.last_cycle_end = current_time

    params = np.hstack((Kx[0, :].A1, Ki))
    x = -np.array([get_avg_position(), gyro_angle(), get_speed(), gyro_angular_velocity(), theta_int])
    engine_speed = np.dot(params, x)
    engine_percent_gain = 100 / 9
    engine_speed *= engine_percent_gain
    engine_speed += robot.engine_power

    motor_dx.on(SpeedPercent(engine_speed))
    motor_sx.on(SpeedPercent(engine_speed))


def main():
    r = Robot()
    m = RobotAutomata(model=r, states=RobotAutomata.states, initial="idle", transitions=RobotAutomata.transitions)

    r.to_calibration()

    while r.state != "end":
        pass


if __name__ == '__main__':
    main()
