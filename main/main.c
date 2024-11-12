#include <stdio.h>  // For stdio initialization functions
#include <stdlib.h>
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"  // For Wi-Fi handling
#include "lwip/tcp.h"    // For TCP server and client
#include "lwip/ip_addr.h"
#include "lwip/netif.h"


// Include the header for the server functions
#include "tcp_server.h"  // Assuming the server functions are in tcp_server.c
#include "barcode.h"

int main() {
    stdio_init_all();  // Initialize stdio (for printing)

    // Call the start_server function to initialize Wi-Fi and start the server tasks
    if (start_server() != 0) {
        printf("Server startup failed\n");
        return -1;
    }
    //printf("attempting to start barcode");
    if (start_barcode() != 0) {
        printf("Barcode startup failed\n");
        return -1;
    }

    vTaskStartScheduler();

        // If the scheduler stops, this will run indefinitely.
    while (1) {
        tight_loop_contents();
    }


    // Main loop is now handled by FreeRTOS, so no need to do anything else here.
    return 0;
}
