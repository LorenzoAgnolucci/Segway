#include <thread>
#include <iostream>
#include <chrono>
#include <sys/resource.h>
#include <sstream>
#include <deque>
#include <numeric>
#include <execinfo.h>
#include <csignal>
#include "ev3dev.h"
#include "sensors.h"
#include "utils.h"
#include "resource_guard.h"
#include "model_parameters.h"

using namespace std::chrono;
using ev3dev::gyro_sensor;
using ev3dev::motor;

const auto SAMPLING_TIME = milliseconds(8);
double DUTY_RADIANS_CONVERSION_RATE = deg2rad(9.6); // Actual speed in radians/sec per unit of motor speed
volatile bool stop_control_thread = false;
gyro_sensor gyro(ev3dev::INPUT_4);
motor motor_dx(ev3dev::OUTPUT_A);
motor motor_sx(ev3dev::OUTPUT_D);
motor kickstand_servo(ev3dev::OUTPUT_B);
const int MAX_REFERENCE_TARGET_SPEED = 100;
const int MAX_STEERING = 50;
volatile int duty_cycle_target_speed = 0;
volatile int initial_duty_cycle_target_speed = 0;
volatile int steering = 0;

void calibrate();

void segway();

void kickstand_up();

void kickstand_down();

void post_kickstand_down();

void engine_stop();

std::thread start_control();

void stop_control(std::thread &control_thread);

void handler(int sig) {
    void *trace_elems[20];
    int trace_elem_count(backtrace(trace_elems, 20));
    char **stack_syms(backtrace_symbols(trace_elems, trace_elem_count));
    for (int i = 0; i < trace_elem_count; ++i) {
        std::cout << stack_syms[i] << "\n";
    }
    free(stack_syms);

    exit(1);
}

void control_loop() {
    std::cout << "Starting control loop...\nGive commands (speed_ref=, steer_ref=) to control the robot.\n" << std::endl;
    pthread_t this_thread = pthread_self();
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    int ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
    if (ret != 0) {
        std::cout << "Unsuccessful in setting thread realtime prio" << std::endl;
        return;
    }

    double theta_int = 0.;
    auto previous_start_time = high_resolution_clock::now();
    std::vector<int> timestamps;
    std::vector<std::string> data;
    int accumulated_time = 0;

    double motor_angle_reference = 0;
    int stop_stabilize_counter = 0;
    int actual_duty_cycle_target_speed = duty_cycle_target_speed;
    int last_duty_cycle_target_speed = duty_cycle_target_speed;
    int smoothing_target_speed_counter = 0;

    while (stop_stabilize_counter < 5 * initial_duty_cycle_target_speed + 20) {
        if (stop_control_thread) {
            stop_stabilize_counter++;
            duty_cycle_target_speed -= sign(duty_cycle_target_speed) * (stop_stabilize_counter % 5 == 0);
            steering -= sign(steering) * (stop_stabilize_counter % 5 == 0);
        } else if (duty_cycle_target_speed != actual_duty_cycle_target_speed) {
            if (last_duty_cycle_target_speed == duty_cycle_target_speed) {
                smoothing_target_speed_counter++;

                if (smoothing_target_speed_counter % 5 == 0) {
                    actual_duty_cycle_target_speed += sign(duty_cycle_target_speed - actual_duty_cycle_target_speed);
                }
            } else {
                last_duty_cycle_target_speed = duty_cycle_target_speed;
                smoothing_target_speed_counter = 0;
            }
        }

        auto current_start_time = high_resolution_clock::now();

        double engine_position = get_avg_position(motor_dx, motor_sx);
        double gyro_angle = get_gyro_angle(gyro);
        double motor_speed = get_speed(motor_dx, motor_sx);
        double gyro_rate = get_gyro_rate(gyro);

        int cycle_duration = duration_cast<milliseconds>(current_start_time - previous_start_time).count();
        timestamps.push_back(cycle_duration);
        accumulated_time += cycle_duration;

        auto angular_speed_reference = actual_duty_cycle_target_speed * DUTY_RADIANS_CONVERSION_RATE;
        motor_angle_reference += angular_speed_reference * cycle_duration / 1000;
        auto motor_angle_error = engine_position - motor_angle_reference;
        auto motor_angular_speed_error = motor_speed - angular_speed_reference;

        auto pos_component = -k_pos * motor_angle_error;
        auto ang_component = k_angle * gyro_angle;
        auto speed_component = -k_speed * motor_angular_speed_error;
        auto ang_vel_component = k_ang_vel * gyro_rate;
        auto ang_int_component = -k_angle_int * theta_int;

        auto engine_speed = pos_component + ang_component + speed_component + ang_vel_component + ang_int_component;
        double engine_percent_gain = 100. / 9;
        engine_speed *= engine_percent_gain;

        std::stringstream cur_data;
        cur_data << "[position, angle, speed, rate, theta_int, power, error, ang_ref, duty] = ["
                 << engine_position << ", "
                 << gyro_angle << ", "
                 << motor_speed << ", "
                 << gyro_rate << ", "
                 << theta_int << ", "
                 << engine_speed << ", "
                 << motor_angle_error << ", "
                 << angular_speed_reference << ", "
                 << actual_duty_cycle_target_speed << ", "
                 //<< "t: [ms] " << duration_cast<milliseconds>(current_start_time - previous_start_time).count();
                 << accumulated_time << "]";
        data.push_back(cur_data.str());

        motor_dx.set_duty_cycle_sp(clamp(engine_speed - steering, -100., 100.));
        motor_sx.set_duty_cycle_sp(clamp(engine_speed + steering, -100., 100.));

        while (high_resolution_clock::now() - current_start_time < SAMPLING_TIME) {
            std::this_thread::sleep_for(microseconds(100));
        }

        theta_int += static_cast<double>(cycle_duration) * motor_angle_error / 1000.;
        previous_start_time = current_start_time;
    }

    int engine_speed = 50;
    for (int i = 0; i < 10*2; i++) {
        auto current_start_time = high_resolution_clock::now();

        int cycle_duration = duration_cast<milliseconds>(current_start_time - previous_start_time).count();
        timestamps.push_back(cycle_duration);
        accumulated_time += cycle_duration;

        double engine_position = get_avg_position(motor_dx, motor_sx);
        double gyro_angle = get_gyro_angle(gyro);
        double motor_speed = get_speed(motor_dx, motor_sx);
        double gyro_rate = get_gyro_rate(gyro);

        auto angular_speed_reference = duty_cycle_target_speed * DUTY_RADIANS_CONVERSION_RATE;
        motor_angle_reference += angular_speed_reference * cycle_duration / 1000;
        auto motor_angle_error = engine_position - motor_angle_reference;

        std::stringstream cur_data;
        cur_data << "[position, angle, speed, rate, theta_int, power, error, ang_ref, duty] = ["
                 << engine_position << ", "
                 << gyro_angle << ", "
                 << motor_speed << ", "
                 << gyro_rate << ", "
                 << theta_int << ", "
                 << engine_speed << ", "
                 << motor_angle_error << ", "
                 << angular_speed_reference << ", "
                 << duty_cycle_target_speed << ", "
                 //<< "t: [ms] " << duration_cast<milliseconds>(current_start_time - previous_start_time).count();
                 << accumulated_time << "]";
        data.push_back(cur_data.str());

        motor_dx.set_duty_cycle_sp(engine_speed);
        motor_sx.set_duty_cycle_sp(engine_speed);
        previous_start_time = current_start_time;
    }

    steering = 0;
    duty_cycle_target_speed = 0;
    motor_dx.set_duty_cycle_sp(0);
    motor_sx.set_duty_cycle_sp(0);

    for (int i = 0; i < 15*40; i++) {
        auto current_start_time = high_resolution_clock::now();

        int cycle_duration = duration_cast<milliseconds>(current_start_time - previous_start_time).count();
        timestamps.push_back(cycle_duration);
        accumulated_time += cycle_duration;

        double engine_position = get_avg_position(motor_dx, motor_sx);
        double gyro_angle = get_gyro_angle(gyro);
        double motor_speed = get_speed(motor_dx, motor_sx);
        double gyro_rate = get_gyro_rate(gyro);

        auto angular_speed_reference = duty_cycle_target_speed * DUTY_RADIANS_CONVERSION_RATE;
        motor_angle_reference += angular_speed_reference * cycle_duration / 1000;
        auto motor_angle_error = engine_position - motor_angle_reference;

        std::stringstream cur_data;
        cur_data << "[position, angle, speed, rate, theta_int, power, error, ang_ref, duty] = ["
                 << engine_position << ", "
                 << gyro_angle << ", "
                 << motor_speed << ", "
                 << gyro_rate << ", "
                 << theta_int << ", "
                 << "-1" << ", "
                 << motor_angle_error << ", "
                 << angular_speed_reference << ", "
                 << duty_cycle_target_speed << ", "
                 //<< "t: [ms] " << duration_cast<milliseconds>(current_start_time - previous_start_time).count();
                 << accumulated_time << "]";
        data.push_back(cur_data.str());

        previous_start_time = current_start_time;
    }

    for (const auto &s : data) {
        std::cout << s << std::endl;
    }
}

int main() {
    signal(SIGSEGV, handler);
    setpriority(PRIO_PROCESS, 0, -20);
    resource_guard motor_dx_guard([]() {
        std::cout << "stop";
        motor_dx.set_duty_cycle_sp(0);
    });
    resource_guard motor_sx_guard([]() { motor_sx.set_duty_cycle_sp(0); });
    resource_guard kickstand_guard([]() { kickstand_down(); });

    std::cout << "Connected\n" << std::endl;

    std::string line;
    std::getline(std::cin, line);
    while (line != "exit") {
        while (line != "start") {
            std::getline(std::cin, line);
        }

        auto control_thread = start_control();

        const int speed_command_prefix_length = std::string("speed_ref=").length();
        const int steer_command_prefix_length = std::string("steer_ref=").length();

        while (line != "stop") {
            std::getline(std::cin, line);
            if (line.find("speed_ref=") != std::string::npos) {
                int ref = std::stoi(line.substr(speed_command_prefix_length));

                if (abs(ref) < MAX_REFERENCE_TARGET_SPEED) {
                    duty_cycle_target_speed = ref;
                }
            }
            if (line.find("steer_ref=") != std::string::npos) {
                int ref = std::stoi(line.substr(steer_command_prefix_length));

                if (abs(ref) < MAX_STEERING) {
                    steering = ref;
                }
            }
        }

        stop_control(control_thread);
        std::getline(std::cin, line);
    }

    std::cout << "\nEnded" << std::endl;
    std::cout << "---" << std::endl;
}

void stop_control(std::thread &control_thread) {
    kickstand_down();
    std::this_thread::sleep_for(seconds(2));

    post_kickstand_down();

    std::this_thread::sleep_for(milliseconds(100));
    initial_duty_cycle_target_speed = duty_cycle_target_speed;
    stop_control_thread = true;

    control_thread.join();

    engine_stop();
}

std::thread start_control() {
    std::cout << "Calibrating...";
    calibrate();
    std::cout << " calibration finished" << std::endl;

    stop_control_thread = false;
    std::thread control_thread(control_loop);

    kickstand_up();
    std::this_thread::sleep_for(seconds(2));

    segway();

    return control_thread;
}

void calibrate() {
    motor_dx.reset();
    motor_sx.reset();
    kickstand_servo.reset();

    auto current_mode = gyro.mode();
    gyro.set_mode(gyro_sensor::mode_gyro_cal);
    std::this_thread::sleep_for(seconds(2));
    gyro.set_mode(ev3dev::gyro_sensor::mode_gyro_g_a);

    motor_dx.run_direct();
    motor_sx.run_direct();
    kickstand_servo.run_direct();
}


void kickstand_up() {
    kickstand_servo.set_speed_sp(500);
    kickstand_servo.run_forever();
}

void segway() {
    kickstand_servo.run_direct();
    kickstand_servo.set_duty_cycle_sp(0);
}

void kickstand_down() {
    kickstand_servo.set_speed_sp(-500);
    kickstand_servo.run_forever();
}

void post_kickstand_down() {
    kickstand_servo.set_speed_sp(0);
    kickstand_servo.run_forever();
}

void engine_stop() {
    motor_dx.set_duty_cycle_sp(0);
    motor_sx.set_duty_cycle_sp(0);
}

