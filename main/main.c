// Main program

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "motor.h"
#include "gy511.h"


void callbacks(uint gpio, uint32_t events)
{
    switch (gpio)
    {
    // Left wheel encoder callback
    case L_ENCODER_OUT:
        read_encoder_pulse(L_ENCODER_OUT, events);
        break;
    // Right wheel encoder callback
    case R_ENCODER_OUT:
        read_encoder_pulse(R_ENCODER_OUT, events);
        break;
    // Ultrasonic callback
    case ECHOPIN:
        read_echo_pulse(ECHOPIN, events);
        break;
    default:
        break;
    }
}

// Function to init all sensors and motors
void init_all()
{
    // Initialise standard I/O
    stdio_init_all();
    sleep_ms(1000);

    // Initialise motor pins and PWM
    motor_init();
    motor_pwm_init();
    printf("Motor pins and PWM initialised\n");
    sleep_ms(500);

    // Initialise ultrasonic sensor
    ultrasonic_init();
    printf("Ultrasonic pins initialised\n");
    sleep_ms(500);

    // Initialise ultrasonic sensor
    gy511_init();
    printf("Magnetometer pins initialised\n");
    sleep_ms(500);

    // Initialise encoder sensor
    encoder_init();
    printf("Encoder pins initialised\n");
    sleep_ms(500);
}

// Function to init all interrupts
void init_interrupts()
{
    printf("Interrupts initialised\n");
    // Initialise interrupts for needed sensors
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &callbacks);
    gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &callbacks);
    gpio_set_irq_enabled_with_callback(R_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &callbacks);
}

// Function to gradually start the motors
void gradual_start(float target_pwmL, float target_pwmR, float increment) {
    float start_pwmL = PWM_MIN;
    float start_pwmR = PWM_MIN;

    while (start_pwmL < target_pwmL || start_pwmR < target_pwmR) {
        if (start_pwmL < target_pwmL) {
            start_pwmL += increment;
        }
        if (start_pwmR < target_pwmR) {
            start_pwmR += increment;
        }

        move_motor(start_pwmL, start_pwmR);
        sleep_ms(50);
    }
}

int balance_counter = 0;

void initial_balance_adjustment() {
    float correction = 1; // Adjust step size for subtle balance correction

    if (balance_counter % 5 == 0) {
        if (actual_speed_left < actual_speed_right) {
            pwmL += correction;
        } else if (actual_speed_right < actual_speed_left) {
            pwmR += correction;
        }
    }

    balance_counter++;
}



// Function to run a straight movement test with gradual start and balancing
void run_straight_test() {
    init_all();
    setpoint_speed = 15.0;

    struct repeating_timer timer;
    int interval = 100;
    add_repeating_timer_ms(interval, encoder_set_distance_speed_callback, &interval, &timer);

    printf("Starting gradual ramp-up...\n");
    gradual_start(pwmL, pwmR, 10.0);

    printf("Starting straight movement test...\n");
    while (1) {
        initial_balance_adjustment();
        update_motor_speed();
        move_motor(pwmL, pwmR);

        printf("Target Speed: %.2f | Left Speed: %.2f | Right Speed: %.2f | PWM Left: %.2f | PWM Right: %.2f\n",
               setpoint_speed, actual_speed_left, actual_speed_right, pwmL, pwmR);

        sleep_ms(100);
    }
}


void station_1_test()
{
    printf("Starting test\n");

    struct repeating_timer timer;
    int interval = 1000;
    add_repeating_timer_ms(interval, encoder_set_distance_speed_callback, &interval, &timer);

    kalman_state *state = kalman_init(5.0, 0.5, 0.1, 100.0);

    bool obstacle_detected = false;
    double cm, prev_cm;

    // GO STRAIGHT UNTIL OBSTACLE
    // while (1)
    // {
    //     // Read ultrasonic sensor
    //     for (int i = 0; i < 20; i++)
    //     {
    //         cm = get_cm(state);
    //     }
    //     obstacle_detected = cm < MIN_CM;

    //     printf("----\n");
    //     // Control motor based on obstacle detection
    //     if (obstacle_detected)
    //     {
    //         printf("Obstacle detected\n");
    //         break;
    //     }
    //     else
    //     {
    //         update_motor_speed();   // Adjust motor speed based on encoder feedback

    //         // printf("Target Speed: %.2f | Left Speed: %.2f, Right Speed: %.2f | PWM Left: %.2f, PWM Right: %.2f\n",
    //         //        setpoint_speed, actual_speed_left, actual_speed_right, pwmL, pwmR);
    //     }

    //     if (cm != prev_cm)
    //     {
    //         printf("Obstacle distance: %.2lf cm\n", cm);
    //         printf("----\n");
    //         prev_cm = cm;
    //     }
    // }

    // // TURN RIGHT
    // turn_motor(RIGHT_WHEEL);
    // total_average_distance = 0;

    // // STOP AFTER 90CM
    // while (1)
    // {
    //     sleep_ms(250); // Reduced sleep for more responsive readings

    //     if (total_average_distance >= 90)
    //     {
    //         stop_motor();
    //         printf("Station 1 complete!\n");
    //         break;
    //     }
    //     else
    //     {
    //         update_motor_speed(); // Adjust motor speed based on encoder feedback
    //     }
    // }

    while (1)
    {
        sleep_ms(1000); // Reduced sleep for more responsive readings
        turn_motor(RIGHT_WHEEL);
    }
}

int main()
{
    // Init all required
    init_all();
    init_interrupts();

    run_straight_test();

    return 0;
}

// double normalise(double value, double min, double max) {
//    // Ensure value is within bounds
//     if (value < min) value = min;
//     if (value > max) value = max;

//     return (value - min) / (max - min);
// }
// double normalised = normalise(cm, MIN_CM, MAX_CM);
// int normalised_duty_cycle = (int)(PWM_MIN + ((PWM_MAX - PWM_MIN) * normalised));