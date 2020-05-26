#include "sensors.h"
#include "utils.h"

using ev3dev::gyro_sensor;
using ev3dev::motor;

double get_gyro_angle(gyro_sensor gyro) {
    return deg2rad(double(gyro.angle()));
}

double get_gyro_rate(gyro_sensor gyro) {
    return deg2rad(double(gyro.rate()));
}

double get_avg_position(const motor& motor_1, const motor& motor_2) {
    return deg2rad((double(motor_1.position()) + double(motor_2.position())) / 2);
}

double get_speed(const motor& motor_1, const motor& motor_2) {
    return deg2rad((double(motor_1.speed()) + double(motor_2.speed())) / 2);
}
