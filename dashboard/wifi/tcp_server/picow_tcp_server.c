/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"

#define WIFI_SSID "some phone"
#define WIFI_PASSWORD "password"

#define TCP_PORT 5001
#define DEBUG_// printf // printf
#define BUF_SIZE 2048 // Constraint for fixed buffer size
#define POLL_TIME_S 5

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    bool complete;
    uint8_t buffer_recv[BUF_SIZE];
    int recv_len;
} TCP_SERVER_T;

static TCP_SERVER_T* tcp_server_init(void) {
    TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
    if (!state) {
        DEBUG_// printf("Failed to allocate state\n");
        return NULL;
    }
    return state;
}

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
            DEBUG_// printf("Close failed %d, calling abort\n", err);
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
        DEBUG_// printf("Test success\n");
    } else {
        DEBUG_// printf("Test failed %d\n", status);
    }
    state->complete = true; // This might not be necessary if you're not terminating
    // Optionally keep the server running
    return ERR_OK;
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;

    if (!p) {
        // A NULL pbuf indicates a disconnection
        DEBUG_// printf("Client disconnected or error occurred.\n");
        tcp_server_close(arg); // Optionally handle client disconnection
        return ERR_OK; // Don't treat this as an error
    }

    // Process incoming data
    cyw43_arch_lwip_check();
    if (p->tot_len > 0) {
        //DEBUG_// printf("tcp_server_recv %d/%d err %d\n", p->tot_len, state->recv_len, err);
        
        // Receive the buffer
        const uint16_t buffer_left = BUF_SIZE - state->recv_len;
        uint16_t bytes_received = pbuf_copy_partial(p, state->buffer_recv + state->recv_len,
                                                     p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
        
        // Increment the total received length
        state->recv_len += bytes_received;
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p); // Free the received pbuf

    // Print the received message
    if (state->recv_len > 0) {
        // Null-terminate the received buffer to treat it as a string
        if (state->recv_len < BUF_SIZE) {
            state->buffer_recv[state->recv_len] = '\0'; // Null-terminate
        } else {
            state->buffer_recv[BUF_SIZE - 1] = '\0'; // Safety null-termination
        }
        DEBUG_// printf("Message received from client: \n%s", state->buffer_recv);

    }

    // Clear the buffer for the next message
    state->recv_len = 0; // Reset the length
    memset(state->buffer_recv, 0, BUF_SIZE); // Clear the buffer


    return ERR_OK; // Keep the server running for future connections
}


static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) {
    // Here we just return ERR_OK to indicate that we want to keep the connection alive
    //DEBUG_// printf("tcp_server_poll called, keeping the connection alive.\n");
    return ERR_OK;
}

static void tcp_server_err(void *arg, err_t err) {
    // Handle the error but do not terminate the server
    if (err == ERR_CLSD) {
        DEBUG_// printf("Client disconnected gracefully.\n");
        // Optionally, you can reset the client PCB and prepare for new connections here
    } else if (err != ERR_ABRT) {
        DEBUG_// printf("tcp_client_err_fn %d\n", err);
    }
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        DEBUG_// printf("Failure in accept\n");
        tcp_server_result(arg, err);
        return ERR_VAL;
    }
    DEBUG_// printf("Client connected\n");

    state->client_pcb = client_pcb;
    tcp_arg(client_pcb, state);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);

    return ERR_OK;
}

static bool tcp_server_open(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    struct netif *netif = netif_list; // Get the first network interface

    // Print the IP address and port
    // printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif)), TCP_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        DEBUG_// printf("Failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err) {
        DEBUG_// printf("Failed to bind to port %u\n", TCP_PORT);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        DEBUG_// printf("Failed to listen\n");
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    // Print the bound IP address after binding
    // printf("Server is listening on IP: %s and Port: %u\n", ip4addr_ntoa(netif_ip4_addr(netif)), TCP_PORT);

    return true;
}

void run_tcp_server_test(void) {
    TCP_SERVER_T *state = tcp_server_init();
    if (!state) {
        return;
    }
    if (!tcp_server_open(state)) {
        tcp_server_result(state, -1);
        return;
    }
    while (!state->complete) {
        // Poll for lwIP and Wi-Fi work
        cyw43_arch_poll();
        sleep_ms(100); // Sleep to prevent busy waiting
    }
    free(state);
}

int main() {
    stdio_init_all(); // Initialize stdio for serial output

    if (cyw43_arch_init()) {
        // printf("Failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    // printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        // printf("Failed to connect.\n");
        return 1;
    } else {
        // printf("Connected to Wi-Fi successfully.\n");
    }

    run_tcp_server_test();
    cyw43_arch_deinit();
    return 0;
}
