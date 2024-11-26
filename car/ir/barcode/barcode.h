

#ifndef BARCODE_H
#define BARCODE_H

/* Macros */
// Barcode 39
#define TOTAL_CHAR 44                     // Total number of characters encoded by Barcode 39 representation
#define CODE_LENGTH 9                     // Length of each barcode's binary representation
#define DELIMITER_CHAR '*'                // Delimiter used as a start/stop character before actual data reading
#define DELIMITER_CODE "010010100"        // Binary representation of delimiter character
#define DELIMITER_REVERSED_CODE "001010010" // Reversed binary representation of delimiter character
#define INVALID_CHAR '#'                  // Error character

// Sensors
#define RESET_BUTTON_PIN 22                     // Maker kit button pin
#define IR_SENSOR_PIN 4                  // IR sensor pin
#define DEBOUNCE_DELAY_US 5000            // Debounce delay in microseconds (us)

/* Function Prototypes */
void initialize_barcode_pin();                          // Function to setup barcode pin to digital
void initialize_reset_button();
void reset_barcode_data();                               // Function to reset barcode data
char decode_scanned_bars();                              // Function to decode scanned bars
void capture_barcode_signal();                           // Function to read from ADC
void barcode_irq_handler(uint gpio, uint32_t events); // Interrupt callback function
void gpio_isr_handler(uint gpio, uint32_t events);
int barcode_init();                  

extern char decoded_barcode_char;             // Character variable to store scanned and parsed barcode character
extern volatile bool is_scanning_allowed;     // Boolean to indicate when scanning is allowed
extern bool has_scan_started;                 // Boolean to indicate barcode is scanning

#endif
