#pragma once
#include "ev3dev.h"

double get_gyro_angle(ev3dev::gyro_sensor);

double get_gyro_rate(ev3dev::gyro_sensor);

double get_avg_position(const ev3dev::motor&, const ev3dev::motor&);

double get_speed(const ev3dev::motor&, const ev3dev::motor&);