
#ifndef __ULN2003_DRIVER_H
#define __ULN2003_DRIVER_H


#include "main.h"
#include "stdint.h"


/**
 * @brief Structure representing a stepper motor GPIO pin.
 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} sPin_t;


/**
 * @brief GPIO pin configurations for all four ULN2003 inputs.
 */
typedef struct
{
	sPin_t in_1;
	sPin_t in_2;
	sPin_t in_3;
	sPin_t in_4;

} sPinCfg_t;


void Stepper_SetCoils ( uint8_t row , sPinCfg_t *ps  );


#endif /* __ULN2003_DRIVER_H */
