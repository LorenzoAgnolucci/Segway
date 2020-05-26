#include <iostream>
#include <chrono>
#include <execinfo.h>
#include "ev3dev.h"
#include "sensors.h"
#include "utils.h"
#include "resource_guard.h"

using namespace std::chrono;
using ev3dev::gyro_sensor;
using ev3dev::motor;

void handler() {
    void *trace_elems[20];
    int trace_elem_count(backtrace(trace_elems, 20));
    char **stack_syms(backtrace_symbols(trace_elems, trace_elem_count));
    for (int i = 0; i < trace_elem_count; ++i) {
        std::cout << stack_syms[i] << "\n";
    }
    free(stack_syms);

    exit(1);
}

int main() {
    std::set_terminate(handler);

    system_clock::time_point start = system_clock::now();
    gyro_sensor gyro(ev3dev::INPUT_4);
    motor motor_dx(ev3dev::OUTPUT_A);
    motor motor_sx(ev3dev::OUTPUT_D);

    resource_guard<motor> motor_dx_guard(motor_dx);
    resource_guard<motor> motor_sx_guard(motor_sx);

    motor_dx.reset();
    motor_sx.reset();

    motor_dx.run_direct();
    motor_sx.run_direct();

    int last_speed_percent = 0;
    system_clock::time_point t;
    while (t = system_clock::now(), t - start < seconds(10)) {
        auto duration = t - start;
        last_speed_percent += 10;

        motor_dx.set_duty_cycle_sp(clamp(last_speed_percent, -100, 100));
        motor_sx.set_duty_cycle_sp(clamp(last_speed_percent, -100, 100));


        std::cout << "[position, angle, speed, rate] = ["
                  << get_avg_position(motor_dx, motor_sx) << " "
                  << get_gyro_angle(gyro) << " "
                  << get_speed(motor_dx, motor_sx) << " "
                  << get_gyro_rate(gyro)
                  << "] "
                  << "t: " << duration_cast<milliseconds>(duration).count()
                  << std::endl;
    }
}