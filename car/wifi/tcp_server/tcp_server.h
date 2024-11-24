#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "lwip/ip_addr.h"   // Include necessary lwIP headers
#include "lwip/tcp.h"       // Include lwIP TCP header
#include "lwip/netif.h"     // Include network interface header
#include "FreeRTOS.h"       // Include FreeRTOS headers
#include "task.h"           // For FreeRTOS task-related functions
#include "queue.h"      // For FreeRTOS queues

// Define buffer size
#define BUF_SIZE 2048

// Define a structure for your TCP server state
typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;    // Pointer to TCP PCB (Protocol Control Block)
    struct tcp_pcb *client_pcb;    // Pointer for the currently connected client
    bool complete;                 // Indicates if the transaction is complete
    uint8_t buffer_recv[BUF_SIZE]; // Buffer for receiving data
    char barcode_data[BUF_SIZE];  // Buffer for barcode data
    int recv_len;                  // Length of data received
} TCP_SERVER_T;

extern TCP_SERVER_T state; // Declare state as an external variable

// Function prototypes
static TCP_SERVER_T* tcp_server_init(void);      // Initialize TCP server state
static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err); // Accept a new client connection
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err); // Handle data reception
static void tcp_server_err(void *arg, err_t err); // Handle errors
static bool tcp_server_open(void *arg);           // Open the server
void run_tcp_server_test(void);                   // Run the TCP server test
void send_decoded_data_to_server(const char *data); // Send decoded barcode data to server
void send_gy511_data_to_server(const char *data);  // Send sensor data to server

int init_server(void);  // Start the server (the main function entry)
const char *get_tcp_last_rcvd();

#endif // TCP_SERVER_H
