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
volatile bool reset_button_pressed = false;

// Semaphore handle
SemaphoreHandle_t barcode_semaphore;

typedef struct
{
    uint16_t index;
    uint64_t timing;
} BarTiming;

// External declaration for TCP server state
extern TCP_SERVER_T state;
extern void send_decoded_data_to_server(const char *data);

// Initialize barcode pin
void initialize_barcode_pin()
{
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
    gpio_set_pulls(IR_SENSOR_PIN, true, false);
}

// Function to initialize the reset button
void init_reset_button()
{
    gpio_init(RESET_BUTTON_PIN);
    gpio_set_dir(RESET_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(RESET_BUTTON_PIN);
}

// Reset barcode data
void reset_barcode_data()
{
    num_bars_scanned = 0;
    strcpy(scanned_binary_code, "");
    for (uint16_t i = 0; i < CODE_LENGTH; i++)
    {
        timing_for_bars[i] = 0;
    }
    has_scan_started = false;
}

// Function to compare two BarTiming structures for sorting
int compare_bar_timings(const void *a, const void *b)
{
    BarTiming *barA = (BarTiming *)a;
    BarTiming *barB = (BarTiming *)b;
    return (barB->timing > barA->timing) - (barB->timing < barA->timing); // Sort in descending order
}
// Function to parse scanned bars
char decode_scanned_bars()
{
    // Initialize an array of structs to hold the indexes and their corresponding timings
    BarTiming bar_timings[CODE_LENGTH];
    for (uint16_t i = 0; i < CODE_LENGTH; ++i)
    {
        bar_timings[i].index = i;
        bar_timings[i].timing = timing_for_bars[i];
    }

    // Use qsort to sort the bars based on timing in descending order
    qsort(bar_timings, CODE_LENGTH, sizeof(BarTiming), compare_bar_timings);

    // Generate the final binary representation string (initialize all characters to '0', narrow bars)
    for (uint16_t i = 0; i < CODE_LENGTH; ++i)
    {
        scanned_binary_code[i] = '0';
    }
    // Null-terminate the string
    scanned_binary_code[CODE_LENGTH] = '\0';

    // Set the top 3 indexes (the 3 bars with the largest timings) to '1', wide bars
    for (uint16_t i = 0; i < 3; ++i)
    {
        scanned_binary_code[bar_timings[i].index] = '1';
    }

    // Initialize the decoded character
    char decoded_char = INVALID_CHAR;

    /*
        NOTE: Each character in Barcode 39 is encoded using 5 black bars, 4 white bars, and 3 wide bars. To represent each of the
        44 unique characters, a binary representation is used, whereby 1 indicates a wide bar, and 0 indicates a narrow bar.
        The binary representation does not capture any information on the colour of the bar (whether it is black or white).
    */
    // Initialise array used to store each barcode character
    char char_array[TOTAL_CHAR] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', 'G',
                                   'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
                                   'Y', 'Z', '_', '.', '$', '/', '+', '%', ' '};

    // Initialise array used to store binary representation of each character
    char *code_array[TOTAL_CHAR] = {"000110100", "100100001", "001100001", "101100000", "000110001", "100110000", "001110000",
                                    "000100101", "100100100", "001100100", "100001001", "001001001", "101001000", "000011001",
                                    "100011000", "001011000", "000001101", "100001100", "001001100", "000011100", "100000011",
                                    "001000011", "101000010", "000010011", "100010010", "001010010", "000000111", "100000110",
                                    "001000110", "000010110", "110000001", "011000001", "111000000", "010010001", "110010000",
                                    "011010000", "010000101", "110000100", "010101000", "010100010", "010001010", "000101010",
                                    "011000100"};

    // Initialise array used to store the reversed binary representation of each character
    char *reverse_code_array[TOTAL_CHAR] = {"001011000", "100001001", "100001100", "000001101", "100011000", "000011001",
                                            "000011100", "101001000", "001001001", "001001100", "100100001", "100100100",
                                            "000100101", "100110000", "000110001", "000110100", "101100000", "001100001",
                                            "001100100", "001110000", "110000001", "110000100", "010000101", "110010000",
                                            "010010001", "010010100", "111000000", "011000001", "011000100", "011010000",
                                            "100000011", "100000110", "000000111", "100010010", "000010011", "000010110",
                                            "101000010", "001000011", "000101010", "010001010", "010100010", "010101000",
                                            "001000110"};

    // Rest of the code to match the decoded character...
    bool is_match_found = false;
    (void) is_match_found;

    // Check if parsing for delimit character
    if (num_chars_scanned == 1 || num_chars_scanned == 3)
    {
        // Check for a matching delimit character
        if (strcmp(scanned_binary_code, DELIMITER_CODE) == 0)
        {
            // Update decoded character
            decoded_char = DELIMITER_CHAR;
            is_match_found = true;
        }
        else if (strcmp(scanned_binary_code, DELIMITER_REVERSED_CODE) == 0)
        {
            // Update decoded character
            decoded_char = DELIMITER_CHAR;
            is_match_found = true;
            // Update scan direction
            is_scan_reversed = true;
        }
    }
    else
    { // Parsing for character
        // Check scan direction
        if (!is_scan_reversed)
        {
            // Loop through all possible binary representations for a matching decoded character
            for (int i = 0; i < TOTAL_CHAR; i++)
            {
                if (strcmp(scanned_binary_code, code_array[i]) == 0)
                {
                    // Update decoded character and immediately break out of loop
                    decoded_char = char_array[i];
                    is_match_found = true;
                    break;
                }
            }
        }
        // Reversed scan direction
        else
        {
            // Loop through all possible reverse binary representations for a matching decoded character
            for (int i = 0; i < TOTAL_CHAR; i++)
            {
                if (strcmp(scanned_binary_code, reverse_code_array[i]) == 0)
                {
                    // Update decoded character and immediately break out of loop
                    decoded_char = char_array[i];
                    is_match_found = true;
                    break;
                }
            }
        }
    }
    // Return decoded character to caller
    return decoded_char;
}
// Function to read from ADC
void capture_barcode_signal()
{
    // Check whether to start scan
    if (!has_scan_started)
    { // Ignore initial change of state
        // Change scan status to start scanning in the next round
        has_scan_started = true;
    }
    // Start scanning
    else
    {
        // Store time difference between state change in array
        timing_for_bars[num_bars_scanned] = time_us_64() - last_transition_time;
        // Update number of bars scanned
        ++num_bars_scanned;
        // Print for debugging
        // printf("\n\nTime difference [%d]: %lld", num_bars_scanned, timing_for_bars[num_bars_scanned - 1]);
        // Start decoding when number of bars scanned reaches required code length
        if (num_bars_scanned == CODE_LENGTH)
        {
            // Update number of characters scanned
            ++num_chars_scanned;
            // for debugging
            // printf("number of chars %d",num_chars_scanned);
            //  Parse scanned bars
            char scanned_char = decode_scanned_bars();
            // Check validity of scanned character
            bool is_valid_char = (scanned_char != INVALID_CHAR) ? true : false;
            // Check if scanned character is valid
            if (is_valid_char)
            {
                // Check number of characters scanned
                switch (num_chars_scanned)
                {
                // Check for a delimiter character
                case 1:
                    // Check if the scanned character matches the delimiter character
                    if (scanned_char != DELIMITER_CHAR)
                    {
                        printf("\nNo starting delimiter character found! Backup car and reset all characters scanned so far..\n");
                        /* Prepare for next scan */
                        // Reset scan direction
                        is_scan_reversed = false;
                        // Reset number of characters scanned
                        num_chars_scanned = 0;
                        // TODO: Backup car..
                    }
                    break;
                // Check for a valid character
                case 2:
                    // Update barcode character scanned
                    decoded_barcode_char = scanned_char;
                    break;
                case 3:
                    // Check if the scanned character matches the delimiter character
                    if (scanned_char != DELIMITER_CHAR)
                    {
                        printf("\nNo ending delimiter character found! Backup car and reset all characters scanned so far..\n");
                        /* Prepare for next scan */
                        // Reset scan direction
                        is_scan_reversed = false;
                        // Reset number of characters scanned
                        num_chars_scanned = 0;
                        // TODO: Backup car..
                    }
                    else
                    {
                        char result[50]; // Allocate a buffer for the full message (adjust size as needed)
                        // Print for debugging
                        printf("\n\nBarcode Output: %c\n", decoded_barcode_char);
                        sprintf(result, "Decoded character is: %c\n", decoded_barcode_char);
                        send_decoded_data_to_server(result);

                        // TODO: Transmit scanned code..
                        /* Prepare for next scan */
                        // Reset scan direction
                        is_scan_reversed = false;
                        // Reset number of characters scanned
                        num_chars_scanned = 0;
                    }
                    break;
                default:
                    break;
                }
            }
            else
            {
                // Invalid character scanned
                printf("\nInvalid barcode character scanned! Backup car and reset all characters scanned so far..\n");
                /* Prepare for next scan */
                // Reset scan direction
                is_scan_reversed = false;
                // Reset number of characters scanned
                num_chars_scanned = 0;
                // TODO: Backup car..
            }
            // Reset barcode after reading a character
            reset_barcode_data();
        }
    }
    // Update last state change time after all computations are completed
    last_transition_time = time_us_64();
}

void gpio_isr_handler(uint gpio, uint32_t events)
{
    // Ensure that we only process valid transitions
    if ((time_us_64() - last_transition_time) > DEBOUNCE_DELAY_US && gpio == IR_SENSOR_PIN && is_scanning_allowed)
    {
        // Only trigger scanning if it's a fresh transition
        is_scanning_active = true;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(barcode_semaphore, &xHigherPriorityTaskWoken); // Signal the scanning task
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                        // Yield the task to let it process the barcode
    }
}

void barcode_scanning_task(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(barcode_semaphore, portMAX_DELAY))
        { // Wait for semaphore
            if (is_scanning_active)
            {
                capture_barcode_signal();   // Process the captured signal
                is_scanning_active = false; // Reset after processing the signal
            }
        }
    }
}
// Task to monitor the reset button
void reset_button_task(void *params)
{
    while (1)
    {
        // Check if the button is pressed (active low)
        if (gpio_get(RESET_BUTTON_PIN) == 0)
        {
            printf("Reset button pressed! Resetting barcode data...\n");
            reset_barcode_data();           // Your existing function to reset the data
            vTaskDelay(pdMS_TO_TICKS(500)); // Debounce delay
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Polling interval
    }
}

int barcode_init()
{
    initialize_barcode_pin();
    // init_reset_button();

    // Create a binary semaphore
    barcode_semaphore = xSemaphoreCreateBinary();

    if (barcode_semaphore == NULL)
    {
        printf("Failed to create barcode semaphore!\n");
    }

    is_scanning_allowed = true;

    // // Create FreeRTOS tasks
    xTaskCreate(barcode_scanning_task, "Barcode Scanning Task", 1024, NULL, 1, NULL);
    // xTaskCreate(reset_button_task, "ResetButtonTask", 1024, NULL, 1, NULL);

    return 0;
}