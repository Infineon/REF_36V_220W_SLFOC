/**
 * @file pmsm_foc_Position_ctrl.c
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
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_wheel_func.h"
#include "../ControlModules/pmsm_foc_interface.h"
#include "../MIDSys/pmsm_foc_pi.h"
// #include "6EDL_gateway.h"
#include "../IPLib/pmsm_foc_ip.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
PMSM_FOC_POSITION_CTRL_t PMSM_FOC_POSITION_CTRL;
PMSM_FOC_STALL_DETECTION_t PMSM_FOC_STALL_DETECTION;

/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/
extern PMSM_FOC_OUTPUT_t PMSM_FOC_OUTPUT;
extern PMSM_FOC_CTRL_t PMSM_FOC_CTRL;
extern PMSM_FOC_PI_t PMSM_FOC_POSITION_PI;

/*********************************************************************************************************************
 * VARIABLES
 ********************************************************************************************************************/
uint32_t demo_example;
uint32_t demo_example_shadow;
uint32_t demo_cnt;

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * LOCAL API PROTOTYPES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API DEFINATION
 ********************************************************************************************************************/

void PMSM_FOC_PositionCtrl_Init(void)
{
	PMSM_FOC_POSITION_CTRL.position_cnt = 0;

	if(ADC.adc_res_pot == 0)
	{
		ADC.adc_res_pot = 1023;
	}
}

PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_PositionCtrl(void)
 {
	// rotor_speed is the scaled electrical rotor speed per sec (eRPS)
	PMSM_FOC_POSITION_CTRL.position_cnt += ((int32_t)(PMSM_FOC_OUTPUT.rotor_speed * MotorParam.SPEED_TO_ANGLE_CONV_FACTOR) >> (MotorParam.SPEED_TO_ANGLE_CONV_FACTOR_SCALE+16));

	/* Implement the position contorl loop only when the position control is enabled */
	if(PMSM_FOC_POSITION_CTRL.position_ctrl_enabled == TRUE)
	{
		PMSM_FOC_POSITION_CTRL.ref_position_int16 = PMSM_FOC_POSITION_CTRL.ref_position >> 15;
		PMSM_FOC_POSITION_CTRL.position_cnt_int16 = PMSM_FOC_POSITION_CTRL.position_cnt >> 15;
//		PMSM_FOC_POSITION_CTRL.position_cnt_int16 = (PMSM_FOC_POSITION_CTRL.position_cnt_int16 * 32886L) >> 15;

		// Stop the motor when the target position is achieved. 
		if((PMSM_FOC_POSITION_CTRL.position_cnt_int16) >= (PMSM_FOC_POSITION_CTRL.ref_position_int16))
		{
			PMSM_FOC_CTRL.motor_start_flag = 0; 
			// XMC_GPIO_SetOutputLow(CYBSP_GD_NBRAKE);
			// EdlIo.nbrake_level = 0;
		}
	}
 }


/*************************************************************************************************/
/**
 * @brief	
 *
 * @param	None
 *
 * @retval	None
 */
void PMSM_FOC_Stall_Detection_Init(void)
{
	PMSM_FOC_STALL_DETECTION.Stall_cnt = 0;
	PMSM_FOC_STALL_DETECTION.Coeff_N_iq = 3;
	PMSM_FOC_STALL_DETECTION.iq_flted = 0;
	PMSM_FOC_STALL_DETECTION.Stall_cnt_threshold = 1;
	/* In case the stall torque is not initialized, set a default value */
	if(PMSM_FOC_STALL_DETECTION.Stall_torque_Q15 == 0)
	{
		PMSM_FOC_STALL_DETECTION.Stall_torque_Q15 = (int32_t)(STALL_TORQUE_NM * (1<<15)); 	// 1638 = 0.05Nm * 32768
	}
	/* i = torque/KT = torque * (1/KT) */
	PMSM_FOC_STALL_DETECTION.Stall_iq_threshold_Q15 = \
		(int32_t)((((PMSM_FOC_STALL_DETECTION.Stall_torque_Q15 * KT_CONST_INV_Q12) >> 12) * I_NORM_Q15) >> 15);

	/* Clear the flag at initialization */
	PMSM_FOC_STALL_DETECTION.Stall_detected = 0;
	PMSM_FOC_STALL_DETECTION.Blanking_cnt = 0;
	PMSM_FOC_STALL_DETECTION.Stall_detection_enabled = 1; // Enable stall detection by default
}

/**
 * @brief	
 *
 * @param	None
 *
 * @retval	None
 */
PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_Stall_Detection(void)
{
	/* Do the fitering for the torque current */
	PMSM_FOC_STALL_DETECTION.iq_flted += \
		(int32_t)((PMSM_FOC_OUTPUT.park_transform.torque_iq - PMSM_FOC_STALL_DETECTION.iq_flted) >> PMSM_FOC_STALL_DETECTION.Coeff_N_iq);

	/* Start a blanking counter after stable status. This is to avoid startup torque trigger the stall detaction */
	if(PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_CLOSED_LOOP)
	{
		if(PMSM_FOC_STALL_DETECTION.Blanking_cnt <= (STALL_DETECTION_BLANKING +1))
		{
			PMSM_FOC_STALL_DETECTION.Blanking_cnt++;
		}
	}

	if ((PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_CLOSED_LOOP) \
			&& (PMSM_FOC_STALL_DETECTION.Blanking_cnt >= STALL_DETECTION_BLANKING))
	{
		/* if the current is bigger than the threshold or 
				the motor cannot transist to stable status */
		if((PMSM_FOC_STALL_DETECTION.iq_flted >= PMSM_FOC_STALL_DETECTION.Stall_iq_threshold_Q15) \
				|| (PMSM_FOC_CTRL.transition_status == PMSM_FOC_MOTOR_STATUS_TRANSITION))	
		{
			/* Start the counter for the stall determination */
			PMSM_FOC_STALL_DETECTION.Stall_cnt ++;
			if(PMSM_FOC_STALL_DETECTION.Stall_cnt >= PMSM_FOC_STALL_DETECTION.Stall_cnt_threshold)
			{
				PMSM_FOC_STALL_DETECTION.Stall_detected = TRUE;
				PMSM_FOC_STALL_DETECTION.Stall_cnt = 0;
				PMSM_FOC_MotorStop();
			}
		}
		else
		{
			PMSM_FOC_STALL_DETECTION.Stall_cnt = 0;
		}
	}
}

/**
 * @brief	
 *
 * @param	None
 *
 * @retval	None
 */
PMSM_FOC_RAM_ATTRIBUTE void PLL_Online_Adjust(void)
{
	int16_t speed;
	speed = PMSM_FOC_OUTPUT.rotor_speed;
	// speed = PMSM_FOC_INPUT.ref_speed;

	if (speed > 0 && speed < speedPoints[0]) {
		PMSM_FOC_PLL_PI.kp = pllKpPoints[0];
    } 
	else if (speed >= speedPoints[0] && speed < speedPoints[1]) {
	    PMSM_FOC_PLL_PI.kp = ((int32_t)(segments[0].slope * speed) >> SCALE_PLL_KP_ADJUSTMENT) + segments[0].yIntercept;
    } 
	else if (speed >= speedPoints[1] && speed < speedPoints[2]) {
        PMSM_FOC_PLL_PI.kp = ((int32_t)(segments[1].slope * speed) >> SCALE_PLL_KP_ADJUSTMENT) + segments[1].yIntercept;
    } 
	else if (speed >= speedPoints[2] && speed < speedPoints[3]) {
        PMSM_FOC_PLL_PI.kp = ((int32_t)(segments[2].slope * speed) >> SCALE_PLL_KP_ADJUSTMENT) + segments[2].yIntercept;
    } 
	else if (speed >= speedPoints[3] && speed < speedPoints[4]) {
        PMSM_FOC_PLL_PI.kp = ((int32_t)(segments[3].slope * speed) >> SCALE_PLL_KP_ADJUSTMENT) + segments[3].yIntercept;
    } 
	else if (speed >= speedPoints[4]) {
        PMSM_FOC_PLL_PI.kp = pllKpPoints[4]; 
	}
}


/**
 * @brief	
 * One resolution is  (int32_t)(1.0f * (USER_MOTOR_POLE_PAIR * Gear_Ratio) * (1 << (16 + 0)))
 * @param	None
 *
 * @retval	None
 */
void Demo_Func(void)
{
	static uint32_t running_times;
	static uint16_t demo_on_flag;

	if(PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_STOP_MOTOR)
	{
		demo_example = demo_example_shadow;
	}

	switch(demo_example)

	{
		/* Run the motor for one resolution and then toggle direction and run another one resolution */
		case 1:
			PMSM_FOC_POSITION_CTRL.position_ctrl_enabled = 1;
			/* STEP 1. Once the motor is started, set the ref position. */
			if(!SYSTEM_BE_IDLE)
			{
				PMSM_FOC_POSITION_CTRL.ref_position = ONE_REVOLUTION;
				if(demo_on_flag == 0)
				{
					demo_on_flag = 1;
				}
			}
			/* STEP 2. When the position is meet, the motor is stopped by position control function. Start the counter for time interval. */
			if((PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_STOP_MOTOR) && (demo_on_flag == TRUE))
			{
				demo_cnt++;
			}
			/* STEP 3. Start the motor again after the require time interval */
			if(demo_cnt == TIME_ONE_SEC)
			{
				PMSM_FOC_CTRL.motor_start_flag = TRUE;
				/* Toggle direction */
				if(PMSM_FOC_CTRL.rotation_dir_target)
				{
					PMSM_FOC_CTRL.rotation_dir_target = 0;
				}
				else
				{
					PMSM_FOC_CTRL.rotation_dir_target = 1;
				}
				demo_cnt = 0;
			}

			break;
		/* Run the motor for several resolution and then toggle direction and run another several resolution */
		case 2:
			PMSM_FOC_POSITION_CTRL.position_ctrl_enabled = 1;
			/* STEP 1. Once the motor is started, set the ref position. */
			if(!SYSTEM_BE_IDLE)
			{
				PMSM_FOC_POSITION_CTRL.ref_position = 5 * ONE_REVOLUTION;
				if(demo_on_flag == 0)
				{
					demo_on_flag = 1;
				}
			}
			/* STEP 2. When the position is meet, the motor is stopped by position control function. Start the counter for time interval. */
			if((PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_STOP_MOTOR) && (demo_on_flag == TRUE))
			{
				demo_cnt++;
			}
			/* STEP 3. Start the motor again after the require time interval */
			if(demo_cnt == TIME_ONE_SEC)
			{
				PMSM_FOC_CTRL.motor_start_flag = TRUE;
				/* Toggle direction */
				if(PMSM_FOC_CTRL.rotation_dir_target)
				{
					PMSM_FOC_CTRL.rotation_dir_target = 0;
				}
				else
				{
					PMSM_FOC_CTRL.rotation_dir_target = 1;
				}
				demo_cnt = 0;
			}
			break;

		case 3:
			PMSM_FOC_POSITION_CTRL.position_ctrl_enabled = 1;
			/* STEP 1. Once the motor is started, set the ref position. */
			if(!SYSTEM_BE_IDLE)
			{
				PMSM_FOC_POSITION_CTRL.ref_position = ONE_FOURTH_REVOLUTION;
				if(demo_on_flag == 0)
				{
					demo_on_flag = 1;
				}
			}
			/* STEP 2. When the position is meet, the motor is stopped by position control function. Start the counter for time interval. */
			if((PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_STOP_MOTOR) && (demo_on_flag == TRUE))
			{
				demo_cnt++;
			}
			/* STEP 3. Start the motor again after the require time interval */
			if((running_times == 0) && (demo_cnt == (TIME_ONE_SEC >> 1)))
			{
				PMSM_FOC_CTRL.motor_start_flag = TRUE;
				demo_cnt = 0;
				running_times = 1;
			}
			/* STEP 4. Start the motor again after the require time interval */
			if((running_times == 1) && (demo_cnt == (TIME_ONE_SEC >> 1)))
			{
				PMSM_FOC_CTRL.motor_start_flag = TRUE;
				demo_cnt = 0;
				running_times = 2;
			}
			/* STEP 5. Start the motor again after the require time interval */
			if((running_times == 2) && (demo_cnt == TIME_ONE_SEC))
			{
				PMSM_FOC_CTRL.motor_start_flag = TRUE;
				/* Toggle direction */
				if(PMSM_FOC_CTRL.rotation_dir_target)
				{
					PMSM_FOC_CTRL.rotation_dir_target = 0;
				}
				else
				{
					PMSM_FOC_CTRL.rotation_dir_target = 1;
				}
				demo_cnt = 0;
				running_times = 3;
			}
			/* STEP 6. Start the motor again after the require time interval */
			if((running_times == 3) && (demo_cnt == (TIME_ONE_SEC >> 1)))
			{
				PMSM_FOC_CTRL.motor_start_flag = TRUE;
				demo_cnt = 0;
				running_times = 4;
			}
			/* STEP 7. Start the motor again after the require time interval */
			if((running_times == 4) && (demo_cnt == (TIME_ONE_SEC >> 1)))
			{
				PMSM_FOC_CTRL.motor_start_flag = TRUE;
				demo_cnt = 0;
				running_times = 5;
			}
			/* STEP 8. Start the motor again after the require time interval */
			if((running_times == 5) && (demo_cnt == TIME_ONE_SEC))
			{
				PMSM_FOC_CTRL.motor_start_flag = TRUE;
				/* Toggle direction */
				if(PMSM_FOC_CTRL.rotation_dir_target)
				{
					PMSM_FOC_CTRL.rotation_dir_target = 0;
				}
				else
				{
					PMSM_FOC_CTRL.rotation_dir_target = 1;
				}
				demo_cnt = 0;
				running_times = 0;
			}
			break;

		default:
			demo_on_flag = FALSE;
			demo_cnt = 0;
			break;
	}
}
