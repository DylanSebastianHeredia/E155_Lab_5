// Sebastian Heredia
// dheredia@g.hmc.edu
// October 3, 2025
// main.c contains code to measure motor RPM and determine sping direction of a quadrature encoder using interrupts.

#include "STM32L432KC.h"
#include "stm32l432xx.h"
#include <stdio.h>

// Defining Macros

#define ENCODER_A  PA8    // Encoders produce square waves (to detect rotation & direction)
#define ENCODER_B  PA6
#define DELAY_TIM  TIM2   // TIM2 provides delays (ms)
#define COUNT_TIM  TIM6   // TIM6 measures deltaTime between pulses

// Defining Global variables
volatile int deltaTime = 0;   // Time between pulses (period of encoder pulse) TIM6->CNT
volatile int pulses = 0;      // Number of pulses
volatile int direction = 0;   // 0 = CW, 1 = CCW

volatile int currentA = 0;
volatile int currentB = 0;

// Function prototypes
void setupGPIO(void);
void setupTIM6(void);
void setupInterrupts(void);
void checkDirection(int newA, int newB);

// Setup encoder GPIO pins
void setupGPIO(void) {
    gpioEnable(GPIO_PORT_A);
    pinMode(ENCODER_A, GPIO_INPUT);
    pinMode(ENCODER_B, GPIO_INPUT);

    // Optional: pull-ups to prevent floating inputs
    // GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD8, 0b01);
    // GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD6, 0b01);
}

// Setup TIM6
void setupTIM6(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN; // Enabling TIM6 clock
    COUNT_TIM->PSC = 79;                  // PSC = 79 for 80MHz sytem clock means timer clk freq = 1MHz
    COUNT_TIM->ARR = 0xFFFF;              // ARR = 65,535, max count before timer resets
    COUNT_TIM->CNT = 0;                   // Ensure counter starts from 0
    COUNT_TIM->CR1 |= TIM_CR1_CEN;        // Enable counter
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
        case 0b00:                                      // A=0, B=0
            if (newState == 0b10) direction = 0;        // CW
            else if (newState == 0b01) direction = 1;   // CCW
            break;

        case 0b01:                                      // A=0, B=1
            if (newState == 0b11) direction = 0;
            else if (newState == 0b00) direction = 1;
            break;

        case 0b11:                                      // A=1, B=1
            if (newState == 0b10) direction = 0;
            else if (newState == 0b01) direction = 1;
            break;

        case 0b10:                                      // A=1, B=0
            if (newState == 0b00) direction = 0;
            else if (newState == 0b11) direction = 1;
            break;
    }

    currentA = newA;
    currentB = newB;
}

// Interrupt Service Routine (ISR) for interrupt handler
void EXTI9_5_IRQHandler(void) {
    int newA = digitalRead(ENCODER_A);  // Initially read the encoder states as a baseline
    int newB = digitalRead(ENCODER_B);
    
    // Check that the button was what triggered our interrupt (EXTI6, B as listed in macros #define)
    if (EXTI->PR1 & (1 << 6)) {
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 6);         // Clear interrupt otherwise MCU will be stuck in the interrupt
        pulses++;                      // 

        checkDirection(newA, newB);
        deltaTime = COUNT_TIM->CNT;    // Capture elapsed time between pulses in deltaTime
        COUNT_TIM->CNT = 0;            // Reset counter
    }
    
    // Check that the button was what triggered our interrupt (EXTI6, B as listed in macros #define)
    if (EXTI->PR1 & (1 << 8)) {
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 8);         // Clear interrupt
        pulses++;

        checkDirection(newA, newB);
        deltaTime = COUNT_TIM->CNT;
        COUNT_TIM->CNT = 0;
    }
}

// Main function serves to read results and print them every 500ms = 0.5s
int main(void) {
    // INitializing functions
    setupGPIO();
    setupTIM6();
    setupInterrupts();

    while (1) {
        if (deltaTime > 0) {

            float RPS = 1.0 / (deltaTime * 0.000001); // Convert us to s for Revolutions Per Second (rps)
            float RPM = RPS * 60.0;
            
            // Print on console
            printf("RPM: %.2f, Direction: %s\n", RPM, direction ? "CCW" : "CW");  // CW = 0, CCW = 1, defined in FSM
        }

        pulses = 0;
        delay_millis(DELAY_TIM, 500);   // Allow some time to process and print, avoid overwelming with data (for readability)
    }
}
