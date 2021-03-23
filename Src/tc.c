#include "stm32l4xx_hal.h"
#include "main.h"
#include "tc.h"

// File scoped vars
uint8_t run;

/*
 * tim2Setup()
 *
 * Timer 2 configuration
 */
void tim2Setup()
{
    // Targeting an interrupt every 2 ms
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;                           // Enable timer clock in RCC
    TIM2->PSC = 3200 - 1;                                           // Set prescalar (clock at 10000 Hz)
    TIM2->ARR = 20;                                                 // Set auto reload value
    TIM2->CR1 &= ~(TIM_CR1_DIR);                                    // Set to count down
    TIM2->DIER |= TIM_DIER_UIE;                                     // Enable update interrupt
    NVIC->ISER[0] |= 1 << TIM2_IRQn;                                // Unmask interrupt
    TIM2->CR1 |= TIM_CR1_CEN;                                       // Enable the timer
}

/*
 * tim7Setup()
 *
 * Timer 7 configuration
 */
void tim7Setup()
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;                           // Enable timer clock in RCC
    TIM7->PSC = 320;                                                // Set prescalar (clock at 16 MHz)
    TIM7->ARR = 0xff;                                               // Set auto reload value to default (reset value, oh well)
    TIM7->CR1 &= TIM_CR1_DIR;                                       // Set to count up
    TIM7->CR1 |= TIM_CR1_CEN;                                       // Enable the timer
}

/*
 * TIM2_IRQHandler()
 *
 * Interrupt handler for timer 2
 *
 * Timing: 2 ms
 */
void TIM2_IRQHandler()
{
    TIM2->SR &= ~TIM_SR_UIF;                                        // Acknowledge the interrupt
    run = 1;                                                        // Signal main loop to process
}

/*
 * adcRead()
 *
 * Gather ADC values for strain gauges and shock pots
 *
 * Timing: 10 ms (tcLoop())
 */
static void adcRead(tc_t *tcHandle)
{
  // ADC is 12 bit resolution, but regs are 32 bits wide. Therefore, we can just clip the upper word with no resolution loss
  HAL_ADC_Start(&hadc1);                                            // Start ADC 1
  HAL_ADC_PollForConversion(&hadc1, 5);                             // Start polling samples
  tcHandle->shockAdc = HAL_ADC_GetValue(&hadc1);                    // Capture value in ADC 1 data register
  HAL_ADC_PollForConversion(&hadc1, 5);                             // Start polling samples
  tcHandle->strainAdc = HAL_ADC_GetValue(&hadc1);                   // Capture value in ADC 1 data register
  HAL_ADC_Stop(&hadc1);                                             // Stop ADC 1 (return to first ADC channel)
}

/*
 * rxCAN()
 *
 * Send required CAN frames containing wheelspeed data and ADC values
 *
 * Timing: 10 ms (tcLoop())
 */
static void rxCAN(tc_t *tcHandle)
{
  // Locals
  uint8_t             data[6];                                      // Data buffer for message tx
  uint32_t            mailbox;                                      // Mailbox for tx
  CAN_TxHeaderTypeDef header;                                       // CAN header frame

  // Pack data. Yeah comments for this would be scintillating. . .
  data[0] = (uint8_t) tcHandle->wheelspeed;
  data[1] = (uint8_t) (tcHandle->wheelspeed >> 8);
  data[2] = (uint8_t) tcHandle->shockAdc;
  data[3] = (uint8_t) ((tcHandle->shockAdc >> 8) & 0xf);
  data[4] = (uint8_t) tcHandle->strainAdc;
  data[5] = (uint8_t) ((tcHandle->strainAdc >> 8) & 0xf);

  header.DLC = 6;                                                   // Set length of data to tx
  header.IDE = CAN_ID_STD;                                          // Set ID length to 11 bit (normal)
  header.RTR = CAN_RTR_DATA;                                        // Set frame type to data
  header.StdId = 0x001;                                             // Set CAN ID
  header.TransmitGlobalTime = DISABLE;                              // Don't send timestamp

  HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);            // Add to tx buffer
}

/*
 * tcLoop()
 *
 * Main traction control processing loop leading to gathering
 * of ADC values and binary counter values
 *
 * Timing: 2 ms
 */
void tcLoop()
{
  // Locals
  uint8_t         bcCounts;                                         // Current value on the binary counter in [8.0] counts
  uint16_t        entry;                                            // Current entry time
  float           speed;                                            // Wheelspeed
  float           timeFactor;                                       // Factor for converting from counts to rpm
  static uint8_t  oldCounts;                                        // Old binary counter value
  static uint16_t oldEntry;                                         // Entry time of last iteration
  static tc_t     tcVals;                                           // All vars for TC/TV in one place

  while (!run);                                                     // Wait until we're told to process
  run = 0;                                                          // Stop extra processing
  entry = TIM7->CNT & 0xffff;                                       // Grab current timer 7 value
  timeFactor = (entry - oldEntry) / 100;                            // Compute miliseconds since last run
  timeFactor *= 60;                                                 // Finish time factor calculation
  bcCounts = (countbit1_GPIO_Port->IDR & (0x3FC)) >> 2;             // Gather binary counter value all in one go
  speed = (((float) bcCounts - oldCounts) / 1024) * timeFactor;     // Turn counts into rpm
  tcVals.wheelspeed = (uint16_t) speed;                             // Store calculated rpm into struct

  adcRead(&tcVals);                                                 // Gather ADC values for strain gauges and shock pots
  rxCAN(&tcVals);                                                   // Send required CAN frames

  // We don't need to delay because the interrupt will pull us back here when our next iteration needs to run
}
