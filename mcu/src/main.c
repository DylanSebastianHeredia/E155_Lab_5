// Sebastian Heredia
// dheredia@g.hmc.edu
// October 7, 2025
// main.c contains code to measure motor RPS and determine spin direction of a quadrature encoder using interrupts.

#include "STM32L432KC.h"
#include "stm32l432xx.h"
#include <stdio.h>

// Defining Macros
#define ENCODER_A  PA8    // Encoders produce square waves (to detect rotation & direction)
#define ENCODER_B  PA6
#define DELAY_TIM  TIM2   // Use only TIM2 for both delay and timing
#define PPR        408    // Pulses per revolution (adjust for your encoder)

// Defining Global variables
volatile int deltaTime = 0;   // Time between pulses (period of encoder pulse)
volatile int pulses = 0;      // Number of pulses
volatile int direction = 0;   // 0 = CW, 1 = CCW
volatile int currentA = 0;
volatile int currentB = 0;
volatile int newPulse = 0;    // For new encoder data

// Function prototypes
void setupGPIO(void);
void setupTIM2(void);
void setupInterrupts(void);
void checkDirection(int newA, int newB);
void delay_millis(TIM_TypeDef *TIMx, int millis);

// Setup encoder GPIO pins
void setupGPIO(void) {
    gpioEnable(GPIO_PORT_A);
    pinMode(ENCODER_A, GPIO_INPUT);
    pinMode(ENCODER_B, GPIO_INPUT);
}

// Setup TIM2
void setupTIM2(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;  // Enable TIM2 clock
    TIM2->PSC = 79;                         // 80 MHz / (79+1) = 1 MHz (1 tick = 1 us)
    TIM2->ARR = 0xFFFFFFFF;                 // Max auto-reload
    TIM2->CNT = 0;                          // Reset counter
    TIM2->CR1 |= TIM_CR1_CEN;               // Enable counter
}

// Setup external interrupt to respond to the encoders
void setupInterrupts(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable (System Config. Controller) SYSCFG to start
  
    // Mapping external interrupt (EXTI) to the respective encoder pins
    // Changes on these pins auto call the interrupt handler
  
    SYSCFG->EXTICR[1] |= _VAL2FLD(SYSCFG_EXTICR2_EXTI6, 0b000);   // 0b000 is Port A, connect EXTI6 to Port A
    SYSCFG->EXTICR[2] |= _VAL2FLD(SYSCFG_EXTICR3_EXTI8, 0b000);

    EXTI->IMR1 |= (1 << 6) | (1 << 8);     // Unmask EXTI lines 6 and 8 so interrupts can be generated for pins PA6 and PA8

    // Triggering interrupt on rising or falling edge (handler runs on every transition - more precision)
    EXTI->RTSR1 |= (1 << 6) | (1 << 8);
    EXTI->FTSR1 |= (1 << 6) | (1 << 8);
    NVIC->ISER[0] |= (1 << EXTI9_5_IRQn);  // Enable EXTI lines 5–9 in Nested Vector Interrupt Controller (NVIC), lets CPU respond to EXTI actions
    __enable_irq();                        // Enabling global interrupts (now interrupts can actually happen)
}

// Checking motor spin direction based on encoder A/B phase
// A leading B (CW) vs. B leading A (CCW)
void checkDirection(int newA, int newB) {
    int currentState = (currentA << 1) | currentB;      // Combining A and B into a single 2-bit number (old)
    int newState = (newA << 1) | newB;                  // (new)
    
    // Motor spin direction is determined by the newState
    switch (currentState) {
        // Form: 0bAB
        case 0b00:
            if (newState == 0b10) direction = 0;        // CW
            else if (newState == 0b01) direction = 1;   // CCW
            break;

        case 0b01:
            if (newState == 0b11) direction = 0;
            else if (newState == 0b00) direction = 1;
            break;

        case 0b11:
            if (newState == 0b10) direction = 0;
            else if (newState == 0b01) direction = 1;
            break;

        case 0b10:
            if (newState == 0b00) direction = 0;
            else if (newState == 0b11) direction = 1;
            break;
    }

    currentA = newA;
    currentB = newB;
}

// Interrupt Service Routine (ISR) for interrupt handler
void EXTI9_5_IRQHandler(void) {
    int newA = digitalRead(ENCODER_A);
    int newB = digitalRead(ENCODER_B);
    
    // Check EXTI6 (B)
    if (EXTI->PR1 & (1 << 6)) {
        EXTI->PR1 |= (1 << 6);               // Clear interrupt flag
        pulses++;
        checkDirection(newA, newB);
        deltaTime = TIM2->CNT;               // Time since last pulse
        TIM2->CNT = 0;                       // Reset timer
        newPulse = 1;
    }
    
    // Check EXTI8 (A)
    if (EXTI->PR1 & (1 << 8)) {
        EXTI->PR1 |= (1 << 8);               // Clear interrupt flag
        pulses++;
        checkDirection(newA, newB);
        deltaTime = TIM2->CNT;               // Time since last pulse
        TIM2->CNT = 0;                       // Reset timer
        newPulse = 1;
    }
}

// Function used by printf to send characters to the laptop
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        ITM_SendChar((*ptr++));
    }
    return len;
}

// Main function serves to read results and print them every 500ms = 0.5s
int main(void) {
    setupGPIO();
    setupTIM2();
    setupInterrupts();

    float lastRPS = 0;

    while (1) {
        if (newPulse && deltaTime > 0) {
            // Compute revolutions per second
            float pulses_per_sec = 1.0f / (deltaTime * 0.000001f);
            lastRPS = pulses_per_sec / PPR;
            newPulse = 0;
        }

        printf("rev/sec: %.3f, Direction: %s\n", lastRPS, direction ? "CCW" : "CW");
        delay_millis(DELAY_TIM, 500);   // Allow some time for readability
    }
}

// Simple millisecond delay using TIM2
void delay_millis(TIM_TypeDef *TIMx, int millis) {
    uint32_t start = TIMx->CNT;
    while ((TIMx->CNT - start) < (millis * 1000)) {
        // Wait (TIM2 runs at 1 MHz for 1us per tick)
    }
}
