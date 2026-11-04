### Authors
Bratislav Djelosevic -  Co-developer and Mentor


Uros Rackovic -  Co-developer


# Overview


An interrupt-driven, multi-axis stepper motor driver for motors controlled via the ULN2003 driver.
It uses discrete numerical integration to update motion without explicitly calculating a velocity profile, allowing the target position to be changed while the motor is moving.
The driver supports multiple stepper motor instances, with a target performance of up to 3,000 steps/s per instance and an estimated step-timing error of less than 10%.

# Technical Details

### Motion Update Algorithm

The motion update algorithm calculates step timing using discrete numerical integration at a constant timer interrupt frequency.

> **Note**
> 
> The interrupt frequency is defined by the `SINT_FREQ` macro in `motor_control.h`.

Since velocity is the integral of acceleration over time, and position is the integral of velocity over time, each timer interrupt updates the corresponding accumulators. The accumulated values are then checked to determine whether a new step should be executed.

Additionally, the algorithm evaluates a braking condition to avoid calculating a velocity profile in advance, allowing the target position to be changed during motion. Braking is controlled using the following kinematic equation:

$$  
v^2 = 2as  
$$

where:

-   (v) is the current velocity.
    
-   (a) is the deceleration magnitude.
    
-   (s) is the remaining distance to the target position.
    

The use of 32-bit signed integers in the braking calculation limits the supported velocity and acceleration magnitudes to 15-bit values to prevent overflow during multiplication.

Additionally, the acceleration parameter and maximum velocity must remain below the interrupt frequency to prevent multiple steps from being required within a single interrupt interval.

> **Optimization**
> 
> The maximum step rate could potentially be increased by calculating the braking point before motion begins. However, this would require restructuring the algorithm and would prevent the target position from being changed during motion.

This algorithm is implemented in the following function:

```
STEP_MotionUpdate(step_map_t *pm);
```

The `pm` pointer references an entry in the internal array `step_map` (`step_map_t step_map[]`). Each entry contains a pointer to a stepper handler and a pointer to its target position (setpoint).

### Program Integration

The user must allocate memory for a stepper handler (`step_h_t`) and a target position variable (`int32_t`). These variables are registered in the internal `step_map[]` array through the API functions.

> **Warning**
> 
> These variables must not be modified directly by the user, as doing so may interfere with the driver's proper operation.

> **Good Practice**
> 
> When controlling multiple stepper motors, consider storing the handlers and target positions in separate arrays. The elements at corresponding indices in both arrays should refer to the same stepper motor.

The user configures the stepper motor parameters in a separate `parameters_t` structure and passes it to the appropriate API function, which applies the parameters to the selected stepper motor instance.

Detailed instructions for configuring the timer interrupt frequency are provided in `motion_control.h`. The same header defines the `STEP_TIM_HANDLE` macro, which specifies the timer handle used by the motion control implementation.

By setting `STEP_MANAGE_IRQ` to 1 in `motion_control.h`, the driver handles synchronization with the timer interrupt.

>**`Note`** 
>This synchronization mechanism is only reliable when API functions are called from the main context, not from an interrupt service routine (ISR).

### API Reference

The available API functions are:

```
STEP_Register(step_h_t *handle, int32_t *setpoint);
STEP_SetParameters(parameters_t *pars, step_h_t *handle);
STEP_UnRegister(step_h_t *handle);
STEP_SetPosition(int32_t setpoint, step_h_t *handle);
STEP_EmergencyStop(step_h_t *handle);
STEP_GetRun( step_h_t *handle );
STEP_GetRunCount(void);
```

-   `STEP_Register()` — Registers the handler and target position pointers in the internal `step_map[]` array.
    
-   `STEP_SetParameters()` — Updates the parameters of an initialized stepper motor instance. Parameters can only be changed while the motor is stationary.
    
-   `STEP_SetPosition()`  Sets or updates the target position. The target position can be changed while the motor is running.
    
-   `STEP_EmergencyStop()`  Requests an emergency stop by modifying the motor's motion state. The stop is processed by `STEP_MotionUpdate()` during the next timer interrupt.
    
-   `STEP_UnRegister()`  Removes a stepper motor instance from an internal array. The motor must be stationary when this function is called.
-  `STEP_GetRun()` Returns the motion status of a specified stepper motor instance.
- `STEP_GetRunCount()` Returns the total number of stepper motor instances currently in motion.

> **`Note`**
> Configure the parameters of a stepper motor handler before starting any motion.    


### Hardware

-   **Microcontroller:** STM32F401RE
    
-   **Motor driver:** ULN2003 driver module
    
-   **Stepper motor:** 28BYJ-48
    

### Portability

This driver can be ported to other STM32 microcontroller families with minimal modifications. Porting the algorithm to other microcontroller platforms requires ensuring that the execution time of the motion update routine does not exceed the timer interrupt period.

The current implementation relies on the 32-bit architecture of the STM32F401RE, particularly for 32-bit integer arithmetic. Porting it to 8-bit or 16-bit microcontrollers may require additional optimizations to meet the timing requirements.
