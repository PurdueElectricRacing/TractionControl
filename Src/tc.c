#include "stm32l4xx_hal.h"
#include "main.h"
#include "tc.h"

// Static prototypes
static void adcRead(tc_t *tcHandle);
static void rxCAN(tc_t *tcHandle);

// File scoped vars
tc_t tcVals;

/*
 * tcLoop()
 *
 * Main traction control processing loop leading to gathering
 * of ADC values and binary counter values
 *
 * Timing: 10 ms
 */
void tcLoop()
{
  // Locals
  uint8_t         bcCounts;                               // Current value on the binary counter in [8.0] counts
  uint16_t        u16;                                    // Generic variable for wheelspeed calculation
  uint32_t        entryTime;                              // Entry time of loop in [32.0] ticks
  static uint32_t historicTime;                           // Entry time of last loop in [32.0] ticks

  entryTime = DWT->CYCCNT;                                // Gather entry cycle count for timing purposes

  bcCounts = countbit1_GPIO_Port->IDR & 0x7F;             // Gather binary counter value all in one go
  u16 = (entryTime - historicTime) / (uint16_t) bcCounts; // [32.0] ticks / [16.0] counts = [16.0] ticks/count
  // TODO: Check that this line works
  u16 = (((uint32_t) u16) << 16) / SystemCoreClock;       // [16.16] ticks/count / [16.0] ticks/second = [0.16] seconds/count
  historicTime = entryTime;                               // Store entry time for next iteration

  // TODO: Turn ticks/count into rpm based on encoder parameters
  tcVals.wheelspeed = u16;                                // Store calculated rpm into struct

  adcRead(&tcVals);                                       // Gather ADC values for strain gauges and shock pots
  rxCAN(&tcVals);                                         // Send required CAN frames

  // We don't need to delay because the interrupt will pull us back here when our next iteration needs to run
}

/*
 * dwtInit()
 *
 * Initializes data watchpoint and trace unit for use of clock
 * cycle counting
 *
 * Timing: 10 ms (tcLoop())
 */
void dwtInit()
{
  // Locals
  uint32_t *demcr = (uint32_t*) 0xE00EDFC;                // Pointer to DEMCR register

  if(!(DWT->CTRL))
  {
    // DWT does not exist on hardware if reset value is 0
  }

  *demcr |= (1UL << 24);                                  // Enable DWT outside debug mode
}

/*
 * adcRead()
 *
 * Gather ADC values for strain gauges and shock pots
 *
 * Timing: 10 ms (tcLoop())
 */
static void adcRead()
{
  // ADC is 12 bit resolution, but regs are 32 bits wide. Therefore, we can just clip the upper word with no resolution loss
  HAL_ADC_Start(&hadc1);                                  // Start ADC 1
  HAL_ADC_PollForConversion(&hadc1, 5);                   // Start polling samples
  tcVals.shockAdc = (uint16_t) hadc1.Instance->DR;        // Capture value in ADC 1 data register
  HAL_ADC_PollForConversion(&hadc1, 5);                   // Start polling samples
  tcVals.strainAdc = (uint16_t) hadc1.Instance->DR;       // Capture value in ADC 1 data register
  HAL_ADC_Stop(&hadc1);                                   // Stop ADC 1 (return to first ADC channel)
}

/*
 * rxCAN()
 *
 * Send required CAN frames containing wheelspeed data and ADC values
 *
 * Timing: 10 ms (tcLoop())
 */
static void rxCAN()
{
  // Locals
  uint8_t             data[8];                            // Data buffer for message tx
  uint32_t            mailbox;                            // Mailbox for tx
  CAN_TxHeaderTypeDef header;                             // CAN header frame

  // TODO: Finish data stuffing and decide on ID based on CAN spreadsheet

//  data[0] = ;
//  data[1] = ;
//  data[2] = ;
//  data[3] = ;
//  data[4] = ;
//  data[5] = ;
//  data[6] = ;
//  data[7] = ;

//  header.DLC = ;                                          // Set length of data to tx
  header.IDE = CAN_ID_STD;                                // Set ID length to 11 bit (normal)
  header.RTR = CAN_RTR_DATA;                              // Set frame type to data
//  header.StdId = ;                                        // Set CAN ID
  header.TransmitGlobalTime = DISABLE;                    // Don't send timestamp

  HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);  // Add to tx buffer
}
