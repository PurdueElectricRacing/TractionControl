#ifndef TC_H
#define TC_H

// Enums
enum {
  SLOWSPEED,
  MEDSPEED,
  HIGHSPEED
};

// Structs
typedef struct tc {
	uint8_t   resolution;   // Current resolution for encoder
	uint16_t  wheelspeed;   // Wheelspeed in [16.16] rpm
	uint16_t  shockAdc;		  // 12 bit ADC value for shock pots in [16.0] counts
	uint16_t  strainAdc;    // 12 bit ADC value for strain guage in [16.0] counts
} tc_t;

// Externs
extern CAN_HandleTypeDef  hcan1;
extern ADC_HandleTypeDef  hadc1;

// Prototypes
void tcLoop();
void dwtInit();

#endif
