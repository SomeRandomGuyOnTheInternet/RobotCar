#include <stdio.h>
#include <stdlib.h>
#include "tcp_client.h" // Include your TCP client header
#include "pico/cyw43_arch.h" // Include necessary headers
#include "pico/stdlib.h"

#include "gy511.h"
#include "tcp_client.h"

#define WIFI_SSID "some phone"
#define WIFI_PASSWORD "password"

// Define BUF_SIZE if not defined
#define BUF_SIZE 2048 

// Define authentication type
#define CYW43_AUTH_WPA2_PSK 2 // Use the correct value for WPA2 PSK

// Function to initialize Wi-Fi
static void init_wifi() {
    if (cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_PSK)) {
        printf("Failed to connect to Wi-Fi\n");
    } else {
        printf("Connected to Wi-Fi\n");
    }
}

int main() {
    stdio_init_all(); // Initialize standard I/O
    gy511_init(); // Initialize the GY-511 sensor

    // Connect to Wi-Fi
    start_wifi(); // Use the new function to connect
    printf("Hello stupid PICO W\n");

    return 0;
}