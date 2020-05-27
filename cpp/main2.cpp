#include <thread>
#include <iostream>
#include <chrono>
#include <sys/resource.h>
#include <deque>
#include <numeric>
#include <sstream>
#include "ev3dev.h"
#include "sensors.h"
#include "utils.h"
#include "resource_guard.h"
#include "model_parameters.h"

using namespace std::chrono;
using ev3dev::gyro_sensor;
using ev3dev::motor;

const auto SAMPLING_TIME = milliseconds(10);
volatile bool stop_control_thread = false;
volatile bool disable_control = false;
gyro_sensor gyro(ev3dev::INPUT_4);
motor motor_dx(ev3dev::OUTPUT_A);
motor motor_sx(ev3dev::OUTPUT_D);
motor kickstand_servo(ev3dev::OUTPUT_B);
volatile double engine_power = 0.0;


void calibrate();
void segway();
void kickstand_up();
void kickstand_down();
void post_kickstand_down();
void engine_stop();

/*void handler() {
    void *trace_elems[20];
    int trace_elem_count(backtrace(trace_elems, 20));
    char **stack_syms(backtrace_symbols(trace_elems, trace_elem_count));
    for (int i = 0; i < trace_elem_count; ++i) {
        std::cout << stack_syms[i] << "\n";
    }
    free(stack_syms);

    exit(1);
}*/

void control_loop() {
    pthread_t this_thread = pthread_self();
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    int ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
    if (ret != 0) {
        std::cout << "Unsuccessful in setting thread realtime prio" << std::endl;
        return;
    }

    double theta_int = 0.;
    double gyro_angle = -cond_i;
    std::deque<double> motor_angle_history(5);
    auto previous_start_time = high_resolution_clock::now();
    std::vector<int> timestamps(5, 10);
    std::vector<std::string> data;

    motor_angle_history.push_back(0);

    while (!stop_control_thread) {
        auto current_start_time = high_resolution_clock::now();
        auto last_cycle_duration = current_start_time - previous_start_time;
        auto last_cycle_ms = duration_cast<milliseconds>(last_cycle_duration).count();

        auto engine_position = get_avg_position(motor_dx, motor_sx);
        auto gyro_rate = get_gyro_rate(gyro);

        auto motor_angular_speed = 1000 * (engine_position - motor_angle_history[0]) / std::accumulate(timestamps.end() - 5, timestamps.end(), 0);
        motor_angle_history.pop_front();
        motor_angle_history.push_back(engine_position);

        auto pos_component = -k_pos * motor_angle_history.back();
        auto ang_component = k_angle * gyro_angle;
        auto speed_component = -k_speed * motor_angular_speed;
        auto ang_vel_component = k_ang_vel * gyro_rate;
        auto ang_int_component = -k_angle_int * theta_int;

        auto engine_speed = pos_component + ang_component + speed_component + ang_vel_component + ang_int_component;
        double engine_percent_gain = 100. / 9;
        engine_speed *= engine_percent_gain;

        if (disable_control) {
            engine_speed = engine_power;
        }

        motor_dx.set_duty_cycle_sp(clamp(engine_speed, -100., 100.));
        motor_sx.set_duty_cycle_sp(clamp(engine_speed, -100., 100.));

        theta_int += static_cast<double>(last_cycle_ms) * engine_position / 1000.;
        gyro_angle += static_cast<double>(last_cycle_ms) * gyro_rate / 1000.;

        std::stringstream cur_data;
        cur_data << "[position, angle, speed, rate] = ["
                 << get_avg_position(motor_dx, motor_sx) << " "
                 << get_gyro_angle(gyro) << " "
                 << get_speed(motor_dx, motor_sx) << " "
                 << get_gyro_rate(gyro)
                 << "] "
                 << "t: [ms] " << last_cycle_ms;
        data.push_back(cur_data.str());
        timestamps.push_back(last_cycle_ms);

        /*
        while (high_resolution_clock::now() - current_start_time < SAMPLING_TIME) {
            std::this_thread::sleep_for(microseconds (100));
        }
        */

        previous_start_time = current_start_time;
    }

    for (const auto& s : data) {
        std::cout << s << "\n";
    }
    std::cout << std::endl;
}

int main() {
    setpriority(PRIO_PROCESS, 0, -20);
    resource_guard<motor> motor_dx_guard(motor_dx);
    resource_guard<motor> motor_sx_guard(motor_sx);

    std::cout << "Starting in 3 seconds..." << std::flush;
    std::this_thread::sleep_for(seconds(3));
    std::cout << " started" << std::endl;

    calibrate();

    std::thread control_thread(control_loop);

    kickstand_up();
    std::this_thread::sleep_for(seconds(1));

    segway();
    std::this_thread::sleep_for(seconds(10));

    kickstand_down();
    std::this_thread::sleep_for(seconds(1));

    post_kickstand_down();

    engine_power = -50;
    disable_control = false;

    std::this_thread::sleep_for(milliseconds(30));
    stop_control_thread = true;

    engine_stop();
    motor_dx.reset();
    motor_sx.reset();

    std::cout << "Ended" << std::endl;
    std::this_thread::sleep_for(seconds(5));
}

void calibrate() {
    motor_dx.reset();
    motor_sx.reset();

    auto current_mode = gyro.mode();
    gyro.set_mode(gyro_sensor::mode_gyro_cal);
    std::this_thread::sleep_for(seconds(2));
    gyro.set_mode(ev3dev::gyro_sensor::mode_gyro_rate);

    motor_dx.run_direct();
    motor_sx.run_direct();
    kickstand_servo.run_direct();
}


void kickstand_up() {
    kickstand_servo.set_duty_cycle_sp(20);
}

void segway() {
    kickstand_servo.set_duty_cycle_sp(0);
}

void kickstand_down() {
    kickstand_servo.set_duty_cycle_sp(-20);
}

void post_kickstand_down() {
    kickstand_servo.set_duty_cycle_sp(0);
}

void engine_stop() {
    engine_power = 0;
    motor_dx.set_duty_cycle_sp(0);
    motor_sx.set_duty_cycle_sp(0);
}
