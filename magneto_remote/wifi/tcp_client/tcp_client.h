#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include "lwip/ip_addr.h" // Include necessary lwIP headers
#include "lwip/tcp.h" // Include lwIP TCP header

// Define a structure for your TCP client state
typedef struct TCP_CLIENT_T_ {
    struct tcp_pcb *tcp_pcb; // Pointer to TCP PCB (Protocol Control Block)
    ip_addr_t remote_addr; // Remote IP address
    uint8_t buffer[2048]; // Buffer for data
    int buffer_len; // Length of data in buffer
    bool complete; // Indicates if the transaction is complete
} TCP_CLIENT_T;

// Function prototypes
void run_tcp_client_test(void); // Function to run the TCP client test
char* gy511_read_data(); // Function to read GY-511 data
int start_wifi();

#endif // TCP_CLIENT_H
