#include <stdint.h>

#define RCC_APB2ENR   (*(volatile uint32_t*)0x40021018)
#define GPIOC_CRH     (*(volatile uint32_t*)0x40011004)
#define GPIOC_ODR     (*(volatile uint32_t*)0x4001100C)

int main(void) {
    RCC_APB2ENR |= (1 << 4); // Enable Clock bus to GPIOC
    GPIOC_CRH &= ~(0xF << 20);
    GPIOC_CRH |=  (0x1 << 20); // Configure PC13 as push-pull

    while (1) {
        GPIOC_ODR ^= (1 << 13); // Switch LED on PC13
        for (volatile int i = 100000; i > 0; i--); // Delay
    }
}
