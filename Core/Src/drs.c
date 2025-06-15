/*
 * drs.c
 *
 *  Created on: Mar 21, 2024
 *      Author: chris
 */
#include "drs.h"
#include "main.h"
#include "gopher_sense.h"
#include "pulse_sensor.h"
#include "rvc.h"

#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdbool.h>
#include <cmsis_os.h>

TIM_HandleTypeDef* DRS_Timer;
U32 DRS_Channel;
int rot_dial_timer_val = 0; //keeping this in here if we want to use rotary dial
U8 drs_button_state;

//local function prototypes
bool drs_shutoff_conditions_reached();
void update_power_channel(POWER_CHANNEL* channel);

//DRS brake thresholds
typedef enum {
	NORMAL,
	TRIPPED = 1
} DRS_BRAKE_STATES;

void init_DRS_servo(TIM_HandleTypeDef* timer_address, U32 channel){
	DRS_Timer = timer_address;
	DRS_Channel = channel;
	HAL_TIM_PWM_Start(DRS_Timer, DRS_Channel); //turn on PWM generation
}

void set_DRS_Servo_Position(U8 start_up_condition){
	//duty cycle lookup table for each DRS position, optional if we are using the rotary dial

	drs_button_state = swButon2_state.data;
	if(start_up_condition){
			__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, OPEN_POS);
	}
	else{
		if(drs_button_state == 1){
#ifdef DRS_SHUTDOWN_CHECKS
			if(drs_shutoff_conditions_reached()){
				__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, CLOSED_POS);
			}
			else{
			__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, OPEN_POS);
			}
#else
			__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, OPEN_POS);
#endif

		}
		else{
			__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, CLOSED_POS);
		}

	}
}

bool drs_shutoff_conditions_reached(){
	static DRS_BRAKE_STATES drs_brake_state = NORMAL;
	switch (drs_brake_state){
		case TRIPPED:
			//have let off the brakes --> closed to open
			if(brakePressureRear_psi.data > BRAKE_SHUTOFF_THRESHOLD - DRS_HYSTERESIS){// ||
				  drs_brake_state = NORMAL;
				  return false;
		    }
			return true; //DRS should be closed
			break;
		case NORMAL:
		default:
			//have breached DRS threshold --> open to closed
			if(brakePressureRear_psi.data > BRAKE_SHUTOFF_THRESHOLD + DRS_HYSTERESIS){// ||
				  drs_brake_state = TRIPPED;
				  return true;
			}
			return false; //drs should be open
			break;
	}

	//reaches here we are in trouble
	return false;

}

