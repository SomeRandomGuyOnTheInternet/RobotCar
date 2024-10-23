#include "lwip/apps/httpd.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwipopts.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include <string.h>

// Wi-Fi Credentials
const char WIFI_SSID[] = "kam";
const char WIFI_PASSWORD[] = "hakambingpassword";

// Simple HTTP handler function for SSI
uint16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen)
{
    const char *response = "Hello from Pico!";
    snprintf(pcInsert, iInsertLen, "%s", response);
    return strlen(response); // Return the length of the inserted string
}

// HTTP POST handler to process incoming data
const char *cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    // Log the incoming message
    for (int i = 0; i < iNumParams; i++)
    {
        printf("Received parameter: %s = %s\n", pcParam[i], pcValue[i]);
    }

    // currentlly response is not working
    // Construct a response message
    static char response[256];
    snprintf(response, sizeof(response), "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nReceived your message: %s", pcValue[0]);

    // Return the response
    return response;
}

int main()
{
    stdio_init_all();

    // Initialize Wi-Fi
    if (cyw43_arch_init() != 0)
    {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    cyw43_arch_enable_sta_mode();

    // Connect to the Wi-Fi network - loop until connected
    printf("Connecting to Wi-Fi...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000) != 0)
    {
        printf("Attempting to connect...\n");
    }

    // Print a success message once connected
    printf("Connected to Wi-Fi!\n");

    // Wait a short time for DHCP to assign an IP address
    sleep_ms(5000); // Give DHCP time to get the IP

    // Get and print the Pico's IP address
    uint32_t pico_ip = cyw43_state.netif[0].ip_addr.addr;
    if (pico_ip != 0)
    {
        printf("Pico IP Address: %s\n", ip4addr_ntoa((const ip4_addr_t *)&pico_ip));
    }
    else
    {
        printf("Failed to get IP Address\n");
    }

    // Register the SSI handler for the HTTP server
    const char *ssi_tags[] = {"test"};
    http_set_ssi_handler(ssi_handler, ssi_tags, sizeof(ssi_tags) / sizeof(ssi_tags[0]));

    // Register CGI handler to receive HTTP POST data
    const tCGI cgi_handlers[] = {{"/send_message.cgi", cgi_handler}};
    http_set_cgi_handlers(cgi_handlers, sizeof(cgi_handlers) / sizeof(cgi_handlers[0]));

    // Initialize Web Server, SSI, and CGI handler
    httpd_init();
    printf("HTTP server initialized\n");

    while (1)
    {
        // Keep the server running
    }

    return 0;
}
