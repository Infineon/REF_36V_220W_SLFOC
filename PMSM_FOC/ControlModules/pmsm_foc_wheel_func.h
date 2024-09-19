/**
 * @file pmsm_foc_wheel_func.h
 * @date 08, Apr, 2022
 *
 * @cond
 *********************************************************************************************************************
 * PMSM FOC Motor Control Library
 *
 * Copyright (c) 2015-2020, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * @endcond
 ***********************************************************************************************************************/

#ifndef PMSM_FOC_WHEEL_FUNC_H_
#define PMSM_FOC_WHEEL_FUNC_H_

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup ControlModules
 * @{
 */

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../Configuration/pmsm_foc_user_input_config.h"
#include "../MIDSys/pmsm_foc_pi.h"

/*********************************************************************************************************************
 * MACRO's
 ********************************************************************************************************************/

#define KT_CONST 					(USER_SQRT_3_CONSTANT * (USER_VDC_LINK_V / 1.414f) / (NO_LOAD_SPEED_RPM) * 60.0 / (2.0 * PI))
#define KT_CONST_INV_Q12	(int32_t)(1/KT_CONST * (1<<12))
/* I_NORM_Q15 for normalizing the real current to be the scaled value, and do the transform from RMS value to amplitude value */
#define I_NORM_Q15 						(int32_t)((1.414f * USER_R_SHUNT_OHM * USER_CURRENT_AMPLIFIER_GAIN / (USER_MAX_ADC_VDD_V / 2.0f)) * (1<< 15))
#define STALL_DETECTION_BLANKING	10000	/* Define the threshold for blanking count at startup */
#define STALL_DETECTION_BLANKING	10000	/* Define the threshold for blanking count at startup */

/* For the demo function */
#define ONE_REVOLUTION (int32_t)(1.0f * (USER_MOTOR_POLE_PAIR * GEAR_RATIO) * (1 << 16))
#define HALF_REVOLUTION (int32_t)(0.5f * (USER_MOTOR_POLE_PAIR * GEAR_RATIO) * (1 << 16))
#define ONE_FOURTH_REVOLUTION (int32_t)(0.25f * (USER_MOTOR_POLE_PAIR * GEAR_RATIO) * (1 << 16))
#define TIME_ONE_SEC (int32_t)(1000000 / USER_SLOW_CTRL_LOOP_PERIOD_uS)

#ifdef 	PLL_KP_ADJUSTMENT
/* Define the PLL Kp adjustment */
// Calculate the slope and the intercept of each subintervals
#define SLOPE_1 (int32_t)(((pllKpPoints[1] - pllKpPoints[0]) * (1 << SCALE_PLL_KP_ADJUSTMENT)) / (speedPoints[1] - speedPoints[0]))	// Scaled
#define INTERCEPT_1 (int32_t)(pllKpPoints[0] - ((SLOPE_1 * speedPoints[0]) / (1 << SCALE_PLL_KP_ADJUSTMENT)))							// Not scaled

#define SLOPE_2 (int32_t)(((pllKpPoints[2] - pllKpPoints[1]) * (1 << SCALE_PLL_KP_ADJUSTMENT)) / (speedPoints[2] - speedPoints[1]))
#define INTERCEPT_2 (int32_t)(pllKpPoints[1] - ((SLOPE_2 * speedPoints[1]) / (1 << SCALE_PLL_KP_ADJUSTMENT)))

#define SLOPE_3 (int32_t)(((pllKpPoints[3] - pllKpPoints[2]) * (1 << SCALE_PLL_KP_ADJUSTMENT)) / (speedPoints[3] - speedPoints[2]))
#define INTERCEPT_3 (int32_t)(pllKpPoints[2] - ((SLOPE_3 * speedPoints[2]) / (1 << SCALE_PLL_KP_ADJUSTMENT)))

#define SLOPE_4 (int32_t)(((pllKpPoints[4] - pllKpPoints[3]) * (1 << SCALE_PLL_KP_ADJUSTMENT)) / (speedPoints[4] - speedPoints[3]))
#define INTERCEPT_4 (int32_t)(pllKpPoints[3] - ((SLOPE_4 * speedPoints[3]) / (1 << SCALE_PLL_KP_ADJUSTMENT)))

typedef struct {
    int32_t startX;       // The starting point of the subinterval
    int32_t endX;         // The ending point of the subinterval
    int16_t slope;        // Slope
    int16_t yIntercept;   // Intercept
} SpeedToPllSegment;

// Initalize the array using the calculated slope and intercept
static const SpeedToPllSegment segments[NUM_SEGMENTS] = {
    {speedPoints[0], speedPoints[1], SLOPE_1, INTERCEPT_1},
    {speedPoints[1], speedPoints[2], SLOPE_2, INTERCEPT_2},
    {speedPoints[2], speedPoints[3], SLOPE_3, INTERCEPT_3},
    {speedPoints[3], speedPoints[4], SLOPE_4, INTERCEPT_4}
};

#endif	//PLL_KP_ADJUSTMENT

/**********************************************************************************************************************
 * ENUMS
 **********************************************************************************************************************/
/**
 * @brief 
 */
/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**  @brief Position control structure */
typedef struct
{
	int32_t ref_position;		// The target of the position control
	int32_t position_cnt;		// The counter indicating the position 
	uint32_t position_ctrl_enabled;	// The flag of enabling position control
	uint32_t updated_param_online;	// The flag when the parameters are updated online
	uint32_t	stable;			// This is the flag to indicate that the motor is running stable and parameters can be updated
	uint32_t max_spd_position_ctrl;	// The max speed for position control
	volatile int32_t position_cnt_int16;
	volatile int32_t ref_position_int16;
} PMSM_FOC_POSITION_CTRL_t;

/**  @brief Stall detection structure */
typedef struct
{
	int32_t Coeff_N_iq;	// Coefficiency for the filter
	int32_t iq_flted;		// The filtered value of i_mag
	int32_t Stall_torque_Q15;	// The stall torque in Q15
	int32_t Stall_iq_threshold_Q15;	// The threshold for the stall protection
	uint32_t Stall_cnt;			// The counter for stall fault judgement
	uint32_t Blanking_cnt;	// Blanking at startup
	uint32_t Stall_cnt_threshold;	// The threshold for the stall counter
	uint16_t Stall_detected;	// The flag for stall fault detected
	uint16_t Stall_detection_enabled;	// The flag of enabling stall detection
}PMSM_FOC_STALL_DETECTION_t;

/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/
 extern PMSM_FOC_POSITION_CTRL_t PMSM_FOC_POSITION_CTRL;
 extern PMSM_FOC_STALL_DETECTION_t PMSM_FOC_STALL_DETECTION;
 extern PMSM_FOC_PI_t PMSM_FOC_POSITION_PI;	/* Position PI CONTROLLER */
/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
/**
 * @brief	
 *
 * @param	None
 *
 * @retval	None
 */
void PMSM_FOC_PositionCtrl_Init(void);

/**
 * @brief	
 *
 * @param	None
 *
 * @retval	None
 */
void PMSM_FOC_PositionCtrl();

/**
 * @brief	
 *
 * @param	None
 *
 * @retval	None
 */
void PMSM_FOC_Stall_Detection_Init(void);

/**
 * @brief	
 *
 * @param	None
 *
 * @retval	None
 */
void PMSM_FOC_Stall_Detection(void);

/**
 * @brief	
 *
 * @param	None
 *
 * @retval	None
 */
void PLL_Online_Adjust(void);

/**
 * @brief	
 *
 * @param	None
 *
 * @retval	None
 */
void Demo_Func(void);

/**
 * @}
 */

/**
 * @}
 */
#endif /* PMSM_FOC_WHEEL_FUNC_H_ */


