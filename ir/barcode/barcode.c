#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "barcode.h"
#include "tcp_server.h"

#define RESET_BUTTON_PIN 20  

// Global variables
volatile bool is_scanning_allowed = false;
volatile bool is_scanning_active = false;
bool is_scan_reversed = false;
bool has_scan_started = false;
uint64_t last_transition_time = 0;
uint64_t timing_for_bars[CODE_LENGTH] = {0};
uint16_t num_bars_scanned = 0;
uint16_t num_chars_scanned = 0;
char scanned_binary_code[CODE_LENGTH + 1] = "";
char decoded_barcode_char = INVALID_CHAR;

// Flag to check the button press
volatile bool reset_button_pressed  = false;

// Semaphore handle
SemaphoreHandle_t barcode_semaphore;

typedef struct {
    uint16_t index;
    uint64_t timing;
} BarTiming;

// External declaration for TCP server state
extern TCP_SERVER_T state;
extern void send_decoded_data_to_server(const char *data);

// Initialize barcode pin
void initialize_barcode_pin() {
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
    gpio_set_pulls(IR_SENSOR_PIN, true, false);
}

// Reset barcode data
void reset_barcode_data() {
    num_bars_scanned = 0;
    strcpy(scanned_binary_code, "");
    for (uint16_t i = 0; i < CODE_LENGTH; i++) {
        timing_for_bars[i] = 0;
    }
    has_scan_started = false;
}

// // Add this function to handle the reset button interrupt
// void reset_button_handler(uint gpio, uint32_t events) {
//     if (gpio == RESET_BUTTON_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
//         reset_button_pressed = true; // Set the flag
//     }
// }

// // Initialize the reset button
// void initialize_reset_button() {
//     gpio_init(RESET_BUTTON_PIN);
//     gpio_set_dir(RESET_BUTTON_PIN, GPIO_IN);
//     gpio_pull_up(RESET_BUTTON_PIN);  // Use internal pull-up
// }



// Function to compare two BarTiming structures for sorting
int compare_bar_timings(const void *a, const void *b) {
    BarTiming *bar_a = (BarTiming *)a;
    BarTiming *bar_b = (BarTiming *)b;

    // Compare based on the 'timing' field (ascending order)
    if (bar_a->timing < bar_b->timing) return -1;
    if (bar_a->timing > bar_b->timing) return 1;
    return 0;
}

// Decode scanned bars
char decode_scanned_bars() {
    BarTiming bar_timings[CODE_LENGTH];
    for (uint16_t i = 0; i < CODE_LENGTH; ++i) {
        bar_timings[i].index = i;
        bar_timings[i].timing = timing_for_bars[i];
    }
    qsort(bar_timings, CODE_LENGTH, sizeof(BarTiming), compare_bar_timings);

    for (uint16_t i = 0; i < CODE_LENGTH; ++i) {
        scanned_binary_code[i] = '0';
    }
    scanned_binary_code[CODE_LENGTH] = '\0';

    for (uint16_t i = 0; i < 3; ++i) {
        scanned_binary_code[bar_timings[i].index] = '1';
    }

    char decoded_char = INVALID_CHAR;
    bool is_match_found = false;

    char char_array[TOTAL_CHAR] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', 'G',
                                   'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
                                   'Y', 'Z', '_', '.', '$', '/', '+', '%', ' '};
    char *code_array[TOTAL_CHAR] = {"000110100", "100100001", "001100001", "101100000", "000110001", "100110000", "001110000",
                                    "000100101", "100100100", "001100100", "100001001", "001001001", "101001000", "000011001",
                                    "100011000", "001011000", "000001101", "100001100", "001001100", "000011100", "100000011",
                                    "001000011", "101000010", "000010011", "100010010", "001010010", "000000111", "100000110",
                                    "001000110", "000010110", "110000001", "011000001", "111000000", "010010001", "110010000",
                                    "011010000", "010000101", "110000100", "010101000", "010100010", "010001010", "000101010",
                                    "011000100"};
    char *reverse_code_array[TOTAL_CHAR] = {"001011000", "100001001", "100001100", "000001101", "100011000", "000011001",
                                            "000011100", "101001000", "001001001", "001001100", "100100001", "100100100",
                                            "000100101", "100110000", "000110001", "000110100", "101100000", "001100001",
                                            "001100100", "001110000", "110000001", "110000100", "010000101", "110010000",
                                            "010010001", "010010100", "111000000", "011000001", "011000100", "011010000",
                                            "100000011", "100000110", "000000111", "100010010", "000010011", "000010110",
                                            "101000010", "001000011", "000101010", "010001010", "010100010", "010101000",
                                            "001000110"};

    if (num_chars_scanned == 1 || num_chars_scanned == 3) {
        if (strcmp(scanned_binary_code, DELIMITER_CODE) == 0) {
            decoded_char = DELIMITER_CHAR;
            is_match_found = true;
        } else if (strcmp(scanned_binary_code, DELIMITER_REVERSED_CODE) == 0) {
            decoded_char = DELIMITER_CHAR;
            is_match_found = true;
            is_scan_reversed = true;
        }
    } else {
        if (!is_scan_reversed) {
            for (int i = 0; i < TOTAL_CHAR; i++) {
                if (strcmp(scanned_binary_code, code_array[i]) == 0) {
                    decoded_char = char_array[i];
                    is_match_found = true;
                    break;
                }
            }
        } else {
            for (int i = 0; i < TOTAL_CHAR; i++) {
                if (strcmp(scanned_binary_code, reverse_code_array[i]) == 0) {
                    decoded_char = char_array[i];
                    is_match_found = true;
                    break;
                }
            }
        }
    }
    return decoded_char;
}

void capture_barcode_signal() {
    static bool is_white_bar = true; // Start assuming the first bar is white.

    if (!has_scan_started) {
        has_scan_started = true;
        printf("Initial detection: White bar detected.\n");
    } else {
        timing_for_bars[num_bars_scanned] = time_us_64() - last_transition_time;
        ++num_bars_scanned;

        // Print the type of bar detected based on the current state
        if (is_white_bar) {
            printf("Detected: White bar [%d]\n", num_bars_scanned);
        } else {
            printf("Detected: Black bar [%d]\n", num_bars_scanned);
        }

        // Toggle the state for the next detection
        is_white_bar = !is_white_bar;

        printf("Number of scanned [%d]\n", num_bars_scanned);

        if (num_bars_scanned == CODE_LENGTH) {
            ++num_chars_scanned;

            char scanned_char = decode_scanned_bars();
            bool is_valid_char = (scanned_char != INVALID_CHAR);

            if (is_valid_char) {
                switch (num_chars_scanned) {
                    case 1:
                        if (scanned_char != DELIMITER_CHAR) {
                            printf("\nNo starting delimiter character found! Backup car and reset all characters scanned so far..\n");
                            is_scan_reversed = false;
                            num_chars_scanned = 0;
                        }
                        break;
                    case 2:
                        decoded_barcode_char = scanned_char;
                        break;
                    case 3:
                        if (scanned_char != DELIMITER_CHAR) {
                            printf("\nNo ending delimiter character found! Backup car and reset all characters scanned so far..\n");
                            is_scan_reversed = false;
                            num_chars_scanned = 0;
                        } else {
                            char result[50];
                            sprintf(result, "Decoded character is: %c\n", decoded_barcode_char);
                            send_decoded_data_to_server(result);
                            is_scan_reversed = false;
                            num_chars_scanned = 0;
                        }
                        break;
                    default:
                        break;
                }
            } else {
                printf("\nInvalid barcode character scanned! Backup car and reset all characters scanned so far..\n");
                reset_barcode_data();
                printf("Resetting...\n");
                is_scan_reversed = false;
                num_chars_scanned = 0;
                // Restart
                printf("Start in 3\n");
                sleep_ms(1000);
                printf("Start in 2\n");
                sleep_ms(1000);
                printf("Start in 1\n");
                sleep_ms(1000);
                printf("GO\n");
            }
            reset_barcode_data();
        }
    }
    last_transition_time = time_us_64();
}


void barcode_interrupt_handler(uint gpio, uint32_t events) {
    // printf("ISR triggered on GPIO %d, events: %u\n", gpio, events);
    // printf("Current time: %llu, Last transition time: %llu\n", time_us_64(), last_transition_time);
    // printf("is_scanning_allowed: %d\n", is_scanning_allowed);

    if ((time_us_64() - last_transition_time) > DEBOUNCE_DELAY_US && gpio == IR_SENSOR_PIN && is_scanning_allowed) {
        //printf("Conditions met for giving semaphore\n");
        is_scanning_active = true; // Ensure this is set when conditions are met
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(barcode_semaphore, &xHigherPriorityTaskWoken);
        last_transition_time = time_us_64(); // Update last transition time
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// void reset_button_task(void *params) {
//     while (1) {
//         // Check if the button is pressed (active low)
//         if (gpio_get(RESET_BUTTON_PIN) == 0) {  // Assuming button press pulls the pin low
//             printf("\nReset button pressed. Resetting barcode data...\n");

//             // Reset the barcode data and relevant variables
//             reset_barcode_data();
//             is_scan_reversed = false;
//             num_chars_scanned = 0;
//             last_transition_time = 0;

//             printf("Barcode data reset completed.\n");

//             // Debounce delay to avoid multiple triggers
//             vTaskDelay(pdMS_TO_TICKS(500));
//         }
//         // Add a small delay to avoid continuous polling
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }


void barcode_scanning_task(void *params) {
    printf("Barcode scanning task started\n");

    while (1) {
        //printf("Waiting for semaphore...\n");
        if (xSemaphoreTake(barcode_semaphore, portMAX_DELAY)) {
            //printf("Semaphore taken, calling capture_barcode_signal\n");
            if (is_scanning_active) {
                capture_barcode_signal();
                is_scanning_active = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


// Task for sending barcode data to server
void tcp_sending_task(void *params) {
    while (1) {
        if (decoded_barcode_char != INVALID_CHAR) {
            char result[50];
            sprintf(result, "Decoded character is: %c\n", decoded_barcode_char);
            send_decoded_data_to_server(result);
            decoded_barcode_char = INVALID_CHAR; // Reset after sending
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1-second interval for sending data
    }
}



int start_barcode() {

    printf("\nbarcode started\n");
    stdio_init_all();
    initialize_barcode_pin();
    //initialize_reset_button();

    // Create a binary semaphore
    barcode_semaphore = xSemaphoreCreateBinary();

    if (barcode_semaphore == NULL) {
        printf("Failed to create barcode semaphore!\n");
    }
    
    is_scanning_allowed = true;

    // Create FreeRTOS tasks
    xTaskCreate(barcode_scanning_task, "Barcode Scanning Task", 1024, NULL, 1, NULL);
    xTaskCreate(tcp_sending_task, "TCP Sending Task", 1024, NULL, 1, NULL);
    //xTaskCreate(reset_button_task, "Reset Barcode Task", 1024, NULL, 1, NULL);

    gpio_set_irq_enabled_with_callback(IR_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true, &barcode_interrupt_handler);
    //gpio_set_irq_enabled_with_callback(RESET_BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, &reset_button_handler);

    // Start FreeRTOS scheduler
    //vTaskStartScheduler();



    return 0;
}