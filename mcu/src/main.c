// Sebastian Heredia
// dheredia@g.hmc.edu
// October 7, 2025
// main.c contains code to measure motor rev/sec and determine spin direction of a quadrature encoder using interrupts.

#include "STM32L432KC.h"
#include "stm32l432xx.h"
#include <stdio.h>

// Defining Macros
#define ENCODER_A  8    // Encoders produce square waves (to detect rotation & direction)
#define ENCODER_B  6
#define DELAY_TIM  TIM2   // Use only TIM2
#define PPR 408           // Pulses per revolution (set to your encoder's PPR)
#define POLLING_PIN 4
#define INTERRUPT_PIN 5

// Defining Global variables
volatile int32_t counter = 0;   // Encoder pulse counter
volatile int direction = 0;     // 0 = CW, 1 = CCW
 
// Previous encoder states
volatile int newA = 0;
volatile int newB = 0;
volatile int lastA = 0;
volatile int lastB = 0;

// Function prototypes
void setupGPIO(void);
void setupTIM2(void);
void setupInterrupts(void);
void compute_velocity(void);
void delay_millis(TIM_TypeDef *TIMx, uint32_t ms);

// Setup encoder GPIO pins
void setupGPIO(void) {
    gpioEnable(GPIO_PORT_A);
    pinMode(ENCODER_A, GPIO_INPUT);
    pinMode(ENCODER_B, GPIO_INPUT);

    // Initialize lastA/lastB
    lastA = digitalRead(ENCODER_A);
    lastB = digitalRead(ENCODER_B);
}

// Setup TIM2 (used for delays and time measurement)
void setupTIM2(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;   // Enable TIM2 clock
    initTIM(DELAY_TIM);                      // Use built-in initTIM() function
}

// Configure external interrupts for encoder signals
void setupInterrupts(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable (System Config. Controller) SYSCFG to start

    // Mapping external interrupt (EXTI) to the respective encoder pins
    // Changes on these pins auto call the interrupt handler

    SYSCFG->EXTICR[1] |= _VAL2FLD(SYSCFG_EXTICR2_EXTI6, 0b000);   // 0b000 is Port A, connect EXTI6 to Port A
    SYSCFG->EXTICR[2] |= _VAL2FLD(SYSCFG_EXTICR3_EXTI8, 0b000);

    EXTI->IMR1 |= (1 << gpioOffset(ENCODER_A)) | (1 << gpioOffset(ENCODER_B));     // Unmask EXTI lines 6 and 8 so interrupts can be generated for pins PA6 and PA8

    // Triggering interrupt on rising or falling edge (handler runs on every transition)
    EXTI->RTSR1 |= (1 << gpioOffset(ENCODER_A)) | (1 << gpioOffset(ENCODER_B));
    // EXTI->FTSR1 |= (1 << 6) | (1 << 8);

    NVIC->ISER[0] |= (1 << EXTI9_5_IRQn);  // Enable EXTI lines 5–9 in Nested Vector Interrupt Controller (NVIC), lets CPU respond to EXTI actions
    __enable_irq();                         // Enabling global interrupts (now interrupts can actually happen)
}

// Interrupt Service Routine (ISR) for encoder edge
// Triggers on ENCODER_A and ENCODER_B rising and falling edges
// Outputs direction (CW = 0, CCW = 1)
void EXTI9_5_IRQHandler(void) {
    // togglePin(INTERRUPT_PIN);

    newA = digitalRead(ENCODER_A);
    newB = digitalRead(ENCODER_B);

    // Determine direction
    if (!lastA && !lastB) {
        if (newA && !newB) direction = 0;           // CW
        else if (!newA && newB) direction = 1;      // CCW
    } 
    
    else if (!lastA && lastB) {
        if (!newA && !newB) direction = 0;
        else if (newA && newB) direction = 1;
    }

    else if (lastA && !lastB) {
        if (newA && newB) direction = 0;
        else if (!newA && !newB) direction = 1;
    }

    else if (lastA && lastB) {
        if (!newA && newB) direction = 0;
        else if (newA && !newB) direction = 1;
    }

    lastA = newA;
    lastB = newB;

    counter++;

    // Clear interrupt flags for both lines
    if (EXTI->PR1 & (1 << 8)) EXTI->PR1 |= (1 << 8); // PA8
    if (EXTI->PR1 & (1 << 6)) EXTI->PR1 |= (1 << 6); // PA6
}

// Compute velocity as rev/sec
void compute_velocity(void) {
    float velocity = ((float)counter) / (PPR);

        float signed_velocity = direction ? velocity : -velocity;
        printf("RPS: %.3f rev/s\n", signed_velocity);

    counter = 0;
}

// Main function
int main(void) {
    setupGPIO();
    setupTIM2();
    setupInterrupts();

    printf("Starting\n");

    while (1) {
        // togglePin(POLLING_PIN);    // Comment out delay_millis below
        compute_velocity();
        delay_millis(DELAY_TIM, 1000); // Sample every 1 second
    }
}