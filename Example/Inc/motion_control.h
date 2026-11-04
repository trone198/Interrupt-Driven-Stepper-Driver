
#ifndef __MOTION_CONTROL_H_
#define __MOTION_CONTROL_H_


/* Includes */


#include "main.h"
#include "uln2003_driver.h"


/* Enumerations */


typedef enum { HALFSTEP , FULLSTEP } stepType_t;

typedef enum
{
    STEP_OK,       /*!< Operation completed successfully */
    STEP_WARNING,  /*!< Operation completed with certain adjustments  */
    STEP_ERROR     /*!< Operation failed */
} step_status_t;


/* Defines */


/**
  *  Define the timer used by the stepper module.
  *  Both the extern declaration and the macro must match
  *  the timer selected in the application. */
extern TIM_HandleTypeDef htim3;
#define STEP_TIM_HANDLE    (&htim3)


/**
  * Motion update frequency in Hz.
  *
  * Set this value to the timer update interrupt frequency:
  *
  * SINT_FREQ = TIM_CLK / ((PSC + 1) * (ARR + 1))
  *
  * TIM_CLK - timer input clock frequency in Hz
  * PSC     - timer prescaler register value
  * ARR     - timer auto-reload register value
  *
  * Ensure that SINT_FREQ matches the actual timer configuration.
  * SINT_FREQ must not exceed 2^15 - 1 (32767) to prevent
  * overflow in the braking condition calculation.
  */
#define SINT_FREQ 	 10000l

//#define STEP_PARS_LIMIT (SINT_FREQ > 0x8000 ? 0x7FFF : (SINT_FREQ - 1))

#define STEP_MAX_INST		5u 	/*!< Maximum number of instances */


/**
  * Interrupt synchronization management:
  * 0 - User is responsible for synchronization.
  * 1 - Stepper library manages synchronization.
  */
#define STEP_MANAGE_IRQ    1U


/* Structures */


/**
  * @brief 	Stepper motor configuration. Contains velocity and
  * 		acceleration limits and hardware configuration.
  *
  * @details Kinematic parameters must be less than SINT_FREQ
  * 		 to prevent multiple steps from occurring in a
  * 	  	 single timer interrupt.
  *
  * Units:
  *   Acceleration: steps/s^2
  *	  Velocity: steps/s
  */
typedef struct
{
    uint16_t    Vh;         /*!< High velocity limit*/
    uint16_t    Vl;         /*!< Low velocity limit */

    uint16_t    Aa;         /*!< Acceleration parameter */
    uint16_t    Ad;         /*!< Deceleration parameter */

    stepType_t  stepType;   /*!< Step mode (full-step or half-step) */

    sPinCfg_t   pinCfg;     /*!< GPIO pin configuration */

} parameters_t;


/**
  * @brief 	Stepper motor handler. Holds the parameters
  * 		and runtime state of a stepper motor instance.
  * @details Units:
  * 		  Acceleration: steps/s^2
  *		      Velocity: steps/s
  *		      Position: steps
  */
typedef struct
{
	parameters_t 		pars;		/*!< Motor parameters */

	int32_t				dp_max;		/*!< Distance threshold for overflow-safe braking calculation */

	int32_t 			p; 			/*!< Current stepper position */
	int32_t 			p_a1; 		/*!< Position integration accumulator */

	int32_t				v; 			/*!< Velocity */
	int32_t				v_a1; 		/*!< Velocity integration accumulator */
	int32_t				v_sp;		/*!< Velocity setpoint */

	int32_t 			a; 			/*!< Acceleration */


	int8_t				step_indx; 	/*!< Step sequence index */

	volatile uint8_t	run;		/*!< Motor run status */

}step_h_t;


/* Function Prototypes */


step_status_t STEP_SetParameters( parameters_t *pars , step_h_t *handle );
step_status_t STEP_Register(step_h_t *handle , int32_t *setpoint );
step_status_t STEP_UnRegister( step_h_t *handle );
step_status_t STEP_SetPosition (int32_t setpoint , step_h_t *handle );
step_status_t STEP_EmergencyStop( step_h_t *handle );
int32_t  STEP_GetRun( step_h_t *handle );
uint32_t STEP_GetRunCount(void);




#endif /* MOTION_CONTROL_H_ */
