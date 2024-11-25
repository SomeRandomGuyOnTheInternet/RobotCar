#include <string.h>
#include <time.h>
#include <math.h>
#include <unistd.h> // For close()
#include "lwip/sockets.h"
#include "lwip/netdb.h"

 // Ensure you include the correct header file
#include <stdint.h> // Include this header for int16_t

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/netif.h" // Include lwIP network interface header


#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "gy511.h" // Include your GY-511 header file

#define WIFI_SSID "some phone"
#define WIFI_PASSWORD "password"

#define SERVER_IP "172.20.10.8"
#define SERVER_PORT 5000 // Replace with your server's port

#define TCP_PORT 5000
#define DEBUG_printf printf
#define BUF_SIZE 2048

#define TEST_ITERATIONS 10
#define POLL_TIME_S 5

#if 0
static void dump_bytes(const uint8_t *bptr, uint32_t len) {
    unsigned int i = 0;

    printf("dump_bytes %d", len);
    for (i = 0; i < len;) {
        if ((i & 0x0f) == 0) {
            printf("\n");
        } else if ((i & 0x07) == 0) {
            printf(" ");
        }
        printf("%02x ", bptr[i++]);
    }
    printf("\n");
}
#define DUMP_BYTES dump_bytes
#else
#define DUMP_BYTES(A,B)
#endif

typedef struct TCP_CLIENT_T_ {
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;
    uint8_t buffer[BUF_SIZE];
    int buffer_len;
    bool complete;
} TCP_CLIENT_T;

static err_t tcp_client_close(void *arg) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    if (state->tcp_pcb != NULL) {
        // Print or log the state before closing
        DEBUG_printf("Closing TCP connection...\n");

        tcp_arg(state->tcp_pcb, NULL);
        tcp_close(state->tcp_pcb);
        state->tcp_pcb = NULL;
    }
    return ERR_OK;
}

// Called with results of operation
static err_t tcp_result(void *arg, int status) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    if (status == 0) {
        DEBUG_printf("test success\n");
    } else {
        DEBUG_printf("test failed %d\n", status);
    }
    state->complete = true;
    return tcp_client_close(arg);
}

static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    state->complete = true; // Mark as complete
    tcp_output(tpcb); // Send immediately

    return ERR_OK;
}

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg; // Correctly cast arg to pointer type

    if (err != ERR_OK) {
        return tcp_client_close(arg);
    }

    // Get sensor data
    char* sensor_data = gy511_read_data(); // Read data from the GY-511
    state->buffer_len = strlen(sensor_data);
    DEBUG_printf("Connected! Sending sensor data: %s\n", sensor_data);

    // Prepare and send the message
    tcp_write(tpcb, sensor_data, state->buffer_len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb); // Send immediately
    return ERR_OK;

}

static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb) {
    DEBUG_printf("tcp_client_poll\n");
    return tcp_result(arg, -1); // no response is an error?
}

static void tcp_client_err(void *arg, err_t err) {
    if (err != ERR_ABRT) {
        DEBUG_printf("tcp_client_err %d\n", err);
        tcp_result(arg, err);
    }
}

static bool tcp_client_open(void *arg) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    DEBUG_printf("Connecting to %s port %u\n", ip4addr_ntoa(&state->remote_addr), TCP_PORT);
    state->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&state->remote_addr));
    if (!state->tcp_pcb) {
        DEBUG_printf("failed to create pcb\n");
        return false;
    }

    tcp_arg(state->tcp_pcb, state);
    tcp_poll(state->tcp_pcb, tcp_client_poll, POLL_TIME_S * 2);
    tcp_sent(state->tcp_pcb, tcp_client_sent);
    tcp_err(state->tcp_pcb, tcp_client_err);

    state->buffer_len = 0;

    // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
    // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
    // these calls are a no-op and can be omitted, but it is a good practice to use them in
    // case you switch the cyw43_arch type later.
    cyw43_arch_lwip_begin();
    err_t err = tcp_connect(state->tcp_pcb, &state->remote_addr, TCP_PORT, tcp_client_connected);
    cyw43_arch_lwip_end();

    return err == ERR_OK;
}

char* gy511_read_data() { 
    // Declare variables to hold the X, Y, Z values as int16_t 
    int16_t x, y, z; 
    int16_t fixed_x, fixed_y; // Additional variables for fixed values 

    // Call the existing function to retrieve the values 
    gy511_get_data(&x, &y, &z, &fixed_x, &fixed_y); 

    // Calculate percentages 
    float percentage_x = (fixed_x / 1.0f) * 100.0f; // Adjust denominator as needed 
    float percentage_y = (fixed_y / 3.0f) * 100.0f; // Adjust denominator as needed 

    // Determine directions 
    const char* direction_x = (x < 0) ? "left" : "right"; 
    const char* direction_y = (y < 0) ? "backward" : "forward"; 

    // Allocate space for the formatted string (ensure it's big enough) 
    static char data_buffer[512]; // Increased size to accommodate more text 

    // Format the data including fixed values, movement direction, and percentage into the buffer 
    snprintf(data_buffer, sizeof(data_buffer), 
             "X: %d, Y: %d, Z: %d, Fixed_X: %d, Fixed_Y: %d - "
             "The car is moving %s by %.2f%%, moving %s by %.2f%%\n", 
             x, y, z, fixed_x, fixed_y, direction_x, fabs(percentage_x), direction_y, fabs(percentage_y)); 

    return data_buffer; // Return the formatted string 
}




// Perform initialisation
static TCP_CLIENT_T* tcp_client_init(void) {
    TCP_CLIENT_T *state = calloc(1, sizeof(TCP_CLIENT_T));
    if (!state) {
        return NULL;
    }
    ip4addr_aton(SERVER_IP, &state->remote_addr); // Set the server IP
    return state;
}

void run_tcp_client_test(void) {
    TCP_CLIENT_T *state = tcp_client_init();
    if (!state) {
        return;
    }
    state->tcp_pcb = tcp_new();
    if (!state->tcp_pcb) {
        tcp_client_close(state);
        return;
    }

    tcp_arg(state->tcp_pcb, state);
    tcp_sent(state->tcp_pcb, tcp_client_sent);

    err_t err = tcp_connect(state->tcp_pcb, &state->remote_addr, SERVER_PORT, tcp_client_connected);
    if (err != ERR_OK) {
        tcp_client_close(state);
        return;
    }

    // Main loop to keep the program running and handle TCP events
    while (1) {
        cyw43_arch_poll(); // Poll lwIP to handle events

        // Check if enough buffer is available for new data
        if (state->tcp_pcb && tcp_sndbuf(state->tcp_pcb) > 64) {
            char* sensor_data = gy511_read_data();
            state->buffer_len = strlen(sensor_data);
            DEBUG_printf("Sending sensor data:\n%s", sensor_data);

            // Try sending the data with a retry if ERR_MEM (-11) occurs
            err_t write_err = tcp_write(state->tcp_pcb, sensor_data, state->buffer_len, TCP_WRITE_FLAG_COPY);
            if (write_err == ERR_OK) {
                tcp_output(state->tcp_pcb); // Send data immediately
            } else if (write_err == ERR_MEM) {
                DEBUG_printf("Failed to write data, error: %d. Retrying...\n", write_err);
                sleep_ms(200);  // Short delay to let buffer clear up before retry
            } else {
                DEBUG_printf("Failed to write data, error: %d\n", write_err);
            }
        }

        sleep_ms(1000); // Send data every 2 seconds
    }

    free(state);
}


int start_wifi() {
    // Initialize the WiFi module
    if (cyw43_arch_init()) {
        printf("WiFi module initialization failed\n");
        return 1;
    }

    printf("Attempting to connect to WiFi SSID: %s\n", WIFI_SSID);

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect.\n");
        return 1;
    } else {
        printf("Connected to Wi-Fi successfully.\n");
    }

    // Get the local IP address after connecting to Wi-Fi
    struct netif *netif = netif_default; // Get the default network interface
    if (netif != NULL) {
        ip4_addr_t ip = netif->ip_addr;
        printf("Client IP address: %s\n", ip4addr_ntoa(&ip));
    } else {
        printf("Network interface not found.\n");
    }

    // Run the TCP client
    run_tcp_client_test(); 

    return 0;
}

