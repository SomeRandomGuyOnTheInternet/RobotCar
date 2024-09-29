#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

const uint BTN_PIN = 20;

int main() {
    stdio_init_all();

    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_set_pulls(BTN_PIN, true, false);

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed.");
        return -1;
    }
    while (true) {
        if(gpio_get(BTN_PIN))
        {
            printf("Hello world, main!\n");
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(500);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            sleep_ms(500);
        }
    }
}