#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "barcode.h"

#define WIFI_SSID "Nelson"
#define WIFI_PASSWORD "nelsonczh"

#define TCP_PORT 5000
#define CLIENT_SERVER_IP "172.20.10.3" // Replace with actual IP
#define CLIENT_SERVER_PORT 5001
#define DEBUG_printf printf
#define BUF_SIZE 2048
#define POLL_TIME_S 5

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    bool complete;
    uint8_t buffer_recv[BUF_SIZE];
    int recv_len;
    // New members for barcode handling
    char barcode_data[1024]; // Buffer to hold barcode data
    size_t buffer_len; // Length of the buffer for sending
} TCP_SERVER_T;

// Global state variable for TCP connection
static struct tcp_pcb *client_pcb = NULL;

// FreeRTOS Queue for handling IRQ events
QueueHandle_t tcp_event_queue;

static void tcp_server_task(void *pvParameters);
static void tcp_client_task(void *pvParameters);

static TCP_SERVER_T* tcp_server_init(void) {
    TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
    if (!state) {
        DEBUG_printf("Failed to allocate state\n");
        return NULL;
    }
    return state;
}

// Client Connection
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err != ERR_OK) {
        DEBUG_printf("Failed to connect to the remote server\n");
        return err;
    }
    DEBUG_printf("Connected to the remote server\n");

    const char *message = "Hello from TCP client";
    tcp_write(tpcb, message, strlen(message), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);
    return ERR_OK;
}

int tcp_client_connect(void) {
    client_pcb = tcp_new(); // Initialize pcb_client here
    if (client_pcb != NULL) {
        printf("client_pcb is not null");
        ip_addr_t server_ip;
        ip4addr_aton(CLIENT_SERVER_IP, &server_ip); // Convert IP string to IP address struct
        // Set up connection (assuming 'server_ip' and 'port' are defined)
        err_t err = tcp_connect(client_pcb, &server_ip, CLIENT_SERVER_PORT, tcp_client_connected);
        if (err != ERR_OK) {
            printf("Failed to initiate TCP client connection, error code: %d\n", err);
            return false;
        }
        return true;
    } else {
        printf("client_pcb null");
        return false;
    }
}

// Server Code
static err_t tcp_server_close(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    err_t err = ERR_OK;
    if (state->client_pcb != NULL) {
        tcp_arg(state->client_pcb, NULL);
        tcp_poll(state->client_pcb, NULL, 0);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        err = tcp_close(state->client_pcb);
        if (err != ERR_OK) {
            DEBUG_printf("Close failed %d, calling abort\n", err);
            tcp_abort(state->client_pcb);
            err = ERR_ABRT;
        }
        state->client_pcb = NULL;
    }
    return err;
}

static err_t tcp_server_result(void *arg, int status) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (status == 0) {
        DEBUG_printf("Test success\n");
    } else {
        DEBUG_printf("Test failed %d\n", status);
    }
    state->complete = true;
    return ERR_OK;
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;

    if (!p) {
        DEBUG_printf("Client disconnected or error occurred.\n");
        tcp_server_close(arg);
        return ERR_OK;
    }

    cyw43_arch_lwip_check();
    if (p->tot_len > 0) {
        const uint16_t buffer_left = BUF_SIZE - state->recv_len;
        uint16_t bytes_received = pbuf_copy_partial(p, state->buffer_recv + state->recv_len,
                                                     p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
        state->recv_len += bytes_received;
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);

    if (state->recv_len > 0) {
        if (state->recv_len < BUF_SIZE) {
            state->buffer_recv[state->recv_len] = '\0';
        } else {
            state->buffer_recv[BUF_SIZE - 1] = '\0';
        }

        DEBUG_printf("Message received:\n%s", state->buffer_recv);

        // Send data to server
        send_gy511_data_to_server(state->buffer_recv);
        state->recv_len = 0;
        memset(state->buffer_recv, 0, BUF_SIZE);
    }
    return ERR_OK;
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) {
    return ERR_OK;
}

static void tcp_server_err(void *arg, err_t err) {
    if (err == ERR_CLSD) {
        DEBUG_printf("Client disconnected gracefully.\n");
    }
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        DEBUG_printf("Failure in accept\n");
        tcp_server_result(arg, err);
        return ERR_VAL;
    }
    DEBUG_printf("Client connected\n");

    state->client_pcb = client_pcb;
    tcp_arg(client_pcb, state);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);

    return ERR_OK;
}

static bool tcp_server_open(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    struct netif *netif = netif_list;

    printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif)), TCP_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        DEBUG_printf("Failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err) {
        DEBUG_printf("Failed to bind to port %u\n", TCP_PORT);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        DEBUG_printf("Failed to listen\n");
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    printf("Server is listening on IP: %s and Port: %u\n", ip4addr_ntoa(netif_ip4_addr(netif)), TCP_PORT);
    return true;
}

void send_gy511_data_to_server(const char *data) {
    printf("SENDING GY5111:\n%s", data);
    if (client_pcb == NULL) {
        // Handle error, as pcb_client is not initialized
        return;
    }
    // Send data using pcb_client
    tcp_write(client_pcb, data, strlen(data), TCP_WRITE_FLAG_COPY);
    tcp_output(client_pcb);
}

void send_decoded_data_to_server(const char *data) {
    printf("Sending decoded data: %s\n", data);
    if (client_pcb == NULL) {
        // Handle error, as pcb_client is not initialized
        return;
    }
    // Send data using pcb_client
    tcp_write(client_pcb, data, strlen(data), TCP_WRITE_FLAG_COPY);
    tcp_output(client_pcb);
}

// FreeRTOS task to run the TCP server
static void tcp_server_task(void *pvParameters) {
    TCP_SERVER_T *state = tcp_server_init();
    if (!state) {
        vTaskDelete(NULL);
        return;
    }
    if (!tcp_server_open(state)) {
        tcp_server_result(state, -1);
        vTaskDelete(NULL);
    }
    while (1) {
        cyw43_arch_poll(); // Poll lwIP to handle events
        vTaskDelay(pdMS_TO_TICKS(2000)); // Poll every 2 seconds
    }
}

// FreeRTOS task to handle the TCP client connection
static void tcp_client_task(void *pvParameters) {
    if (!tcp_client_connect()) {
        DEBUG_printf("Failed to initiate TCP client connection\n");
    }
    while (1) {
        cyw43_arch_poll(); // Poll lwIP to handle events
        vTaskDelay(pdMS_TO_TICKS(500)); // Poll every 0.5 seconds
    }
}


int start_server() {

    stdio_init_all();
    
    if (cyw43_arch_init()) {
        printf("Failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect.\n");
        return 1;
    } else {
        printf("Connected to Wi-Fi successfully.\n");
    }

    // Create FreeRTOS tasks
    xTaskCreate(tcp_server_task, "TCP Server", 2048, NULL, 1, NULL);
    xTaskCreate(tcp_client_task, "TCP Client", 2048, NULL, 1, NULL);

    // Start the scheduler
    //vTaskStartScheduler();

    return 0;
}

