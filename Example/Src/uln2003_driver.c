
#include <uln2003_driver.h>



uint8_t step_seq[8][4] =
		{
				{ 1 , 0 , 0 , 0 },
				{ 1 , 1 , 0 , 0 },
				{ 0 , 1 , 0 , 0 },
				{ 0 , 1 , 1 , 0 },
				{ 0 , 0 , 1 , 0 },
				{ 0 , 0 , 1 , 1 },
				{ 0 , 0 , 0 , 1 },
				{ 1 , 0 , 0 , 1 }
		};

/**
 * @brief Sets the ULN2003 input pins according to the selected step sequence.
 *
 * @param row: Index of the step sequence to apply.
 * @param ps: Pointer to the GPIO pin configuration structure.
 */
void Stepper_SetCoils ( uint8_t row , sPinCfg_t *ps  )
	{
	    HAL_GPIO_WritePin( ps->in_1.port, ps->in_1.pin, step_seq[row][0] );
	    HAL_GPIO_WritePin( ps->in_2.port, ps->in_2.pin, step_seq[row][1] );
	    HAL_GPIO_WritePin( ps->in_3.port, ps->in_3.pin, step_seq[row][2] );
	    HAL_GPIO_WritePin( ps->in_4.port, ps->in_4.pin, step_seq[row][3] );
	};
