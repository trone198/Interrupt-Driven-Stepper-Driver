

#include "motion_control.h"


/**
  * @brief Stepper instance mapping structure.
  */
typedef struct
{

	step_h_t  *ps; 		/*!< Pointer to the stepper handler */
	int32_t	  *psp; 	/*!< Pointer to the target position */

}step_map_t;

step_map_t step_map[STEP_MAX_INST];



//__________________________MOTION UPDATE_______________________________________________

/**
  * @brief 	Updates the motor motion state.
  *
  * @details Must be called periodically at a constant time
  * 		 interval from a timer interrupt.
  * 		 The algorithm uses numeric integration: velocity is updated from
  * 		 acceleration, and position is updated from velocity.
  *			 The integration time step is determined by SINT_FREQ:
  * 			delta_t = 1 / SINT_FREQ.
  *			//Velocity and target position can be either positive or negative values.
  *
  * @param pm: Pointer to the stepper instance map entry.
  */
inline __attribute__((always_inline))
void STEP_MotionUpdate(step_map_t *pm)
{
    int step;
    int32_t delta_p;
    step_h_t *ps = pm->ps;

    	delta_p = *pm->psp - ps->p;

/*-----------------Set the velocity setpoint----------------- */

/*
 *  The outer conditions determine the target velocity direction.
 *  The inner conditions select the high or low velocity limit
 *  according to the braking condition.
 */
    	if( delta_p > 0 )
    	{
    		ps->run = 1;

    		if( ps->v < 0 )
    			ps->v_sp = +ps->pars.Vl;
    		else
    		{
    			int b;
    			/* Prevent overflow in the braking condition calculation. */
				if ( delta_p > ps->dp_max)
					b = 0;
				else
					/* Braking condition. */
					b = ( ps->v * ps->v >= 2 * ps->pars.Ad * delta_p) ? 1 : 0;

				ps->v_sp = b ? ps->pars.Vl : ps->pars.Vh;
    		}
    	}
    	else if( delta_p < 0 )
    	{
    		ps->run = 1;

    		if( ps->v > 0 )
    			ps->v_sp = -ps->pars.Vl;
    		else
    		{
    			int b;

				if ( delta_p < -ps->dp_max)
					b = 0;
				else
					b = ( ps->v * ps->v >= -2 * ps->pars.Ad * delta_p) ? 1 : 0;

				ps->v_sp = b ? -ps->pars.Vl : -ps->pars.Vh;
    		}
    	}
    	else
    	{

    		/* Handle target position changes during motion. */
    		if( abs(ps->v) <= ps->pars.Vl)
    		{
    	    	ps->run = 0;
        		ps->v = 0;
        		ps->v_a1 = 0;
        		ps->v_sp = 0;
        		ps->p_a1 = 0;

    	    	return;
    		}
    		else
    			ps->run = 1;
    	}

/*-----------------Velocity regulation-----------------*/

/*
 * The outer conditions determine the acceleration direction.
 * The inner conditions select the acceleration parameter
 * according to whether the motor needs to change direction.
 */
    	if( ps->v_sp - ps->v > 0 )
   		ps->a = ps->v >= 0 ? +ps->pars.Aa : +ps->pars.Ad;
    	else if( ps->v_sp - ps->v < 0 )
    		ps->a = ps->v <= 0 ? -ps->pars.Aa : -ps->pars.Ad;
    	else
    		ps->a = 0;

/*-----------------Velocity update-----------------*/

/*
 * Update the velocity using numeric integration.
 * The velocity accumulator preserves fractional increments between updates.
 */
		ps->v_a1 += ps->a;

		if(ps->v_a1 >= SINT_FREQ)
		{
			ps->v_a1 -= SINT_FREQ;
			++ps->v;
		}
		else if(ps->v_a1 <= -SINT_FREQ)
		{
			ps->v_a1 += SINT_FREQ;
			--ps->v;
		}
		/* Prevent velocity from overshooting the target value. */
    	if( (ps->v > ps->v_sp && ps->a > 0) || (ps->v < ps->v_sp && ps->a < 0))
    		ps->v = ps->v_sp;




/*-----------------Position update-----------------*/


    	ps->p_a1 += ps->v;
    	if (ps->p_a1 >= SINT_FREQ)
    	{
    		ps->p_a1 -= SINT_FREQ;
    		step = +1;
    	}
    	else if (ps->p_a1 <= -SINT_FREQ)
    	{
    		ps->p_a1 += SINT_FREQ;
    		step = -1;
    	}
    	else step = 0;

    	ps->p += step;

/*-----------------Step sequence update-----------------*/


    	ps->step_indx += step;
    	if(ps->step_indx > 7) ps->step_indx = 0;
    	else if(ps->step_indx < 0) ps->step_indx = 7;

		Stepper_SetCoils ( ps->step_indx , &ps->pars.pinCfg  );

}


//__________________________STEPPER SET PARAMETERS______________________________________


/**
  * @brief  Configures the parameters of a stepper motor instance.
  *
  * @param  pars: Pointer to the parameter structure to apply.
  * @param  handle: Pointer to the stepper motor handler.
  *
  *
  * @retval STEP_OK: Parameters were successfully applied.
  * @retval STEP_WARNING: Parameters applied with values limited to
  *                       the maximum supported value.
  * @retval STEP_ERROR: Invalid pointer, invalid parameter values,
  *                     or the motor is currently running.
  */
step_status_t STEP_SetParameters( parameters_t *pars , step_h_t *handle )
{

	step_status_t stat = STEP_OK;

	if( (pars == NULL) 	 	 ||
		(handle == NULL) 	 ||
		(pars->Vh < pars->Vl)||
		(pars->Aa == 0) 	 ||
		(pars->Ad == 0) 	 	) // avoid dividing with zero in braking condition
	{
		return STEP_ERROR;
	}

#if STEP_MANAGE_IRQ

	uint32_t mask;

	mask = __HAL_TIM_GET_IT_SOURCE(STEP_TIM_HANDLE, TIM_IT_UPDATE);
	__HAL_TIM_DISABLE_IT(STEP_TIM_HANDLE, TIM_IT_UPDATE);

#endif

	if (handle->run == 0)
	{

		/* Limit parameters to the supported range. */
		if(pars->Aa > SINT_FREQ - 1)
		{
			pars->Aa = SINT_FREQ - 1;
			stat =  STEP_WARNING;
		};
		if(pars->Ad > SINT_FREQ - 1)
		{
			pars->Ad = SINT_FREQ - 1;
			stat =  STEP_WARNING;

		};
		if(pars->Vh > SINT_FREQ - 1)
		{
			pars->Vh = SINT_FREQ - 1;
			stat = STEP_WARNING;

		};
		if(pars->Vl > SINT_FREQ - 1)
		{
			pars->Vl = SINT_FREQ - 1;
			stat =  STEP_WARNING;

		};

		/* Calculate the maximum position error to prevent
		 * overflow during braking calculations.
		 *
		 * velocity^2 >= 2 * deceleration * distance
		 */
		handle->dp_max = 0x7FFFFFFF / ( 2 * pars->Ad );

		handle->pars = *pars;
	}
	else stat = STEP_ERROR;

#if STEP_MANAGE_IRQ
	if (mask != RESET)
	{
	    __HAL_TIM_ENABLE_IT(STEP_TIM_HANDLE, TIM_IT_UPDATE);
	}
#endif

	return stat;

};


//__________________________STEPPER REGISTRATION________________________________________


/**
  * @brief  Registers a stepper motor instance.
  *
  * @param  handle: Pointer to the stepper motor handler.
  * @param  setpoint: Pointer to the target position variable.
  *
  * @retval STEP_OK: Registration completed successfully.
  * @retval STEP_ERROR: Invalid arguments, handler already registered, no free instance
  *                     slot available.
  */
step_status_t STEP_Register(step_h_t *handle , int32_t *setpoint )
{

	uint8_t inst = STEP_MAX_INST;

	if( (handle == NULL) || (setpoint == NULL) )
	{
		return STEP_ERROR;
	}


	/* Check for an available instance slot and verify that the
	 * specified handler is not already registered. */
	for( int i = STEP_MAX_INST - 1 ; i >= 0  ; i--)
	{
		if( handle == step_map[i].ps )
		{
			return STEP_ERROR;
		}
		else if( step_map[i].ps == NULL ) inst = i;
	}


	if( inst >= STEP_MAX_INST )

	{
		return STEP_ERROR;
	}


	/* Initialize the motion control state to ensure the instance
	 * starts from the expected state.  */
	handle->v_a1 = 0;
	handle->v = 0;
	handle->p_a1 = 0;
	handle->run = 0;
	*setpoint = handle->p;

#if STEP_MANAGE_IRQ

	uint32_t mask;

	mask = __HAL_TIM_GET_IT_SOURCE(STEP_TIM_HANDLE, TIM_IT_UPDATE);
	__HAL_TIM_DISABLE_IT(STEP_TIM_HANDLE, TIM_IT_UPDATE);
#endif

	/* Register the stepper instance and its
	 * target position in the available slot. */
	step_map[inst].psp = setpoint;
	step_map[inst].ps  = handle;

#if STEP_MANAGE_IRQ
	if (mask != RESET)
	{
	    __HAL_TIM_ENABLE_IT(STEP_TIM_HANDLE, TIM_IT_UPDATE);
	};

#endif

	return STEP_OK;

};


//__________________________STEPPER UNREGISTER______________________________________________


/**
  * @brief  Removes the stepper instance from the internal instance map.
  *
  * @details The motor must be stopped before removing from internal
  * 			array step_map[] .
  *
  * @param  handle: Pointer to the stepper motor handler.
  *
  * @retval STEP_OK: Removal completed successfully.
  * @retval STEP_ERROR: Invalid handle, instance not registered, or motor is running.
  */
step_status_t STEP_UnRegister( step_h_t *handle )
{
	int inst;
	step_status_t stat = STEP_OK;

	if( handle == NULL ) return STEP_ERROR;

	/* Search for an instance associated with the specified handler. */
	for( inst = 0 ; inst < STEP_MAX_INST ; inst++ )
	{
		if( step_map[inst].ps == handle )
		{
			break;
		}
	}

	if( inst >= STEP_MAX_INST )

	{
		return STEP_ERROR;
	}

#if STEP_MANAGE_IRQ
	uint32_t mask;

	mask = __HAL_TIM_GET_IT_SOURCE(STEP_TIM_HANDLE, TIM_IT_UPDATE);
	__HAL_TIM_DISABLE_IT(STEP_TIM_HANDLE, TIM_IT_UPDATE);
#endif
	if( handle->run == 0)
	{

		/* Unregister the stepper instance and its
		 * target position from the instance map. */
		step_map[inst].ps = NULL;
		step_map[inst].psp = NULL;

	}
	else stat = STEP_ERROR;

#if STEP_MANAGE_IRQ
	if (mask != RESET)
	{
	    __HAL_TIM_ENABLE_IT(STEP_TIM_HANDLE, TIM_IT_UPDATE);
	}
#endif

	return stat;
}


//__________________________STEPPER SET POSTION_________________________________________


/**
  * @brief  Sets the target position of a stepper motor instance.
  *
  * @details The target position can be changed while the motor is moving.
  *
  * @param  setpoint: Target position in steps.
  * @param  handle: Pointer to the stepper motor handler.
  *
  * @retval STEP_OK: Target position updated successfully.
  * @retval STEP_ERROR: Invalid handle or instance not registered.
  */
step_status_t STEP_SetPosition (int32_t setpoint , step_h_t *handle )
{

	int inst;

	if( handle == NULL ) return STEP_ERROR;


	/* Search for an instance associated with the specified handler. */
	for( inst = 0 ; inst < STEP_MAX_INST ; inst++ )
	{
		if( step_map[inst].ps == handle )
		{
			break;
		}
	}

	if( inst >= STEP_MAX_INST )
	{
		return STEP_ERROR;
	}

#if STEP_MANAGE_IRQ

	uint32_t mask;

	mask = __HAL_TIM_GET_IT_SOURCE(STEP_TIM_HANDLE, TIM_IT_UPDATE);
	__HAL_TIM_DISABLE_IT(STEP_TIM_HANDLE, TIM_IT_UPDATE);
#endif

	/* Update the target position of the specified instance. */
	*step_map[inst].psp = setpoint;
	step_map[inst].ps->run = 1;

#if STEP_MANAGE_IRQ
	if (mask != RESET)
	{
	    __HAL_TIM_ENABLE_IT(STEP_TIM_HANDLE, TIM_IT_UPDATE);
	}
#endif

	return STEP_OK;
};



//__________________________STEPPER EMERGENCY STOP______________________________________


/**
  * @brief  Requests the stepper motor to stop.
  *
  * @param  handle: Pointer to the stepper motor handler.
  *
  * @retval STEP_OK: Stop request accepted successfully.
  * @retval STEP_ERROR: Invalid handle or instance not registered.
  */
step_status_t STEP_EmergencyStop( step_h_t *handle )
{

	int inst;

	if( handle == NULL ) return STEP_ERROR;

	/* Search for an instance associated with the specified handler. */
	for( inst = 0 ; inst < STEP_MAX_INST ; inst++ )
	{
		if( step_map[inst].ps == handle )
		{
			break;
		}
	}
	if( inst >= STEP_MAX_INST )
	{
		return STEP_ERROR;
	}

#if STEP_MANAGE_IRQ

	uint32_t mask;

	mask = __HAL_TIM_GET_IT_SOURCE(STEP_TIM_HANDLE, TIM_IT_UPDATE);
	__HAL_TIM_DISABLE_IT(STEP_TIM_HANDLE, TIM_IT_UPDATE);

#endif

	/* Set the stop conditions.
	 * The stop request is processed in the ISR.*/
	step_map[inst].ps->v = 0;
	*step_map[inst].psp = step_map[inst].ps->p;

#if STEP_MANAGE_IRQ
	if (mask != RESET)
	{
	    __HAL_TIM_ENABLE_IT(STEP_TIM_HANDLE, TIM_IT_UPDATE);
	}

#endif

	return STEP_OK;

}



//__________________________STEPPER GET RUN ___________________________________________


/**
  * @brief Gets the motion status of a specified stepper motor instance.
  *
  * @param handle Pointer to the stepper motor handler.
  *
  * @retval  1 : In motion.
  * @retval  0 : Idle
  * @retval -1 : handler is NULL or not registered.
  */
int32_t  STEP_GetRun( step_h_t *handle )
{
	int inst;

	if( handle == NULL ) return -1;

	/* Search for an instance associated with the specified handler. */
	for( inst = 0 ; inst < STEP_MAX_INST ; inst++ )
	{
		if( step_map[inst].ps == handle )
		{
			break;
		}
	}
	if( inst >= STEP_MAX_INST )
	{
		return -1;
	}

	return  handle->run;

}

/**
  * @brief Gets the number of stepper motor instances currently in motion.
  */
uint32_t STEP_GetRunCount(void)
{
uint32_t temp = 0U;

#if STEP_MANAGE_IRQ

	uint32_t mask;

	mask = __HAL_TIM_GET_IT_SOURCE(STEP_TIM_HANDLE, TIM_IT_UPDATE);
	__HAL_TIM_DISABLE_IT(STEP_TIM_HANDLE, TIM_IT_UPDATE);

#endif

	for (int i = 0 ; i < STEP_MAX_INST ; i++)
	{
		if(step_map[i].ps != NULL)
		{
			temp += step_map[i].ps->run;
		}
	}

#if STEP_MANAGE_IRQ
	if (mask != RESET)
	{
	    __HAL_TIM_ENABLE_IT(STEP_TIM_HANDLE, TIM_IT_UPDATE);
	}

#endif

	return temp;

}


//__________________________ IT ________________________________________________________

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if (htim->Instance == STEP_TIM_HANDLE->Instance ) // STEP_TIM_HANDLE->Instance)
{

	for(int i = 0; i< STEP_MAX_INST; i++ )
	{
		if(step_map[i].ps != NULL)
		{
			STEP_MotionUpdate(&step_map[i]);
		}

	}
}
}



