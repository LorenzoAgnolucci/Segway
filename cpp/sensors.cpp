#include "sensors.h"
#include "utils.h"
#include "model_parameters.h"

using ev3dev::gyro_sensor;
using ev3dev::motor;

double get_gyro_angle(gyro_sensor gyro) {
    int angle;
    std::tie(angle, std::ignore) = gyro.rate_and_angle();
    return deg2rad(double(angle)) - cond_i;
}

double get_gyro_rate(gyro_sensor gyro) {
    int rate;
    std::tie(std::ignore, rate) = gyro.rate_and_angle();
    return deg2rad(static_cast<double>(rate));
}

double get_avg_position(const motor& motor_1, const motor& motor_2) {
    return deg2rad((double(motor_1.position()) + double(motor_2.position())) / 2);
}

double get_speed(const motor& motor_1, const motor& motor_2) {
    return deg2rad((double(motor_1.speed()) + double(motor_2.speed())) / 2);
}
