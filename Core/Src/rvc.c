/*
 * rvc.c: created on Jan 23, 2025
 *
 *  vcu.c: created on Dec 4, 2022
 *      Author: Ben Abbott
 */

#include <rvc_software_faults.h>
#include "rvc.h"
#include "gopher_sense.h"
#include "drs.h"
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"
// The HAL_CAN struct
CAN_HandleTypeDef* hcan;

//Motor Variables
float motor_rpm = 0;
uint8_t rear_brake_hardware_fault_active = 0;
uint8_t current_hardware_range_fault_active = 0;
uint8_t TS_braking_hardware_fault_active = 0;
//Cooling variables

//Fan
boolean steady_temperatures_achieved_fan[] = {true, true}; //LOT if fan temperatures have returned to steady state, implemented to stop double counting
U8 fan_readings_below_HYS_threshold = 0;
U8 rad_fan_state = RAD_FAN_OFF;

//Pump
TIM_HandleTypeDef* PUMP_PWM_Timer;
U32 PUMP_Channel;

boolean steady_temperatures_achieved_pump[] = {true, true}; //LOT if pump temperatures have returned to ready state
U8 pump_readings_below_HYS_threshold = 0;
U16 pwm_pump_intensity = PUMP_INTENSITY_OFF;

U8 digital_pump_state = PUMP_DIGITAL_OFF; //if no pump pwm and just digital

#define HBEAT_LED_DELAY_TIME_ms 500

// Initialization code goes here
void init(CAN_HandleTypeDef* hcan_ptr) {
	hcan = hcan_ptr;

	init_can(hcan, GCAN1);
}

void main_loop() {
    update_cooling();
	update_gcan_states(); // Should be after proceass_sensors
	LED_task();

	update_brakelight_and_buzzer();
	update_brakeBias();

    set_DRS_Servo_Position(FALSE); //DRS set position
}

/**
 * Services the CAN RX and TX hardware task
 */
void can_buffer_handling_loop()
{
	// Handle each RX message in the buffer
	if (service_can_rx_buffer())
	{
		// An error has occurred
	}

	// Handle the transmission hardware for each CAN bus
	service_can_tx(hcan);
}

void update_gcan_states() {
	// ================================Fault Parameters to Forward to FVC============================
	//reading faults from hardware, fault active = 0 so inverted signaling
	rear_brake_hardware_fault_active = !HAL_GPIO_ReadPin(BSPD_BRK_FAULT_GPIO_Port, BSPD_BRK_FAULT_Pin);
	current_hardware_range_fault_active = !HAL_GPIO_ReadPin(BSPD_TS_SNS_FAULT_GPIO_Port, BSPD_TS_SNS_FAULT_Pin);
	TS_braking_hardware_fault_active = !HAL_GPIO_ReadPin(BSPD_TS_BRK_FAULT_GPIO_Port, BSPD_TS_BRK_FAULT_Pin);

	//Out of Range
	update_and_queue_param_u8(&bspdBrakePressureSensorFault_state, rear_brake_hardware_fault_active);
	update_and_queue_param_u8(&bspdTractiveSystemCurrentSensorFault_state, current_hardware_range_fault_active);

	//TS Brake Fault --> Hard breaking + 5kW of power from tractive system
	update_and_queue_param_u8(&bspdTractiveSystemBrakingFault_state, TS_braking_hardware_fault_active);

	// ==============================================================================================

	// Cooling
	//update_and_queue_param_u8(&coolantFanPower_percent, rad_fan_state*100);
	update_and_queue_param_u8(&coolantPumpPower_percent, digital_pump_state*100);
}

void init_Pump(TIM_HandleTypeDef* timer_address, U32 channel){
	PUMP_PWM_Timer = timer_address;
	PUMP_Channel = channel;
	HAL_TIM_PWM_Start(PUMP_PWM_Timer, PUMP_Channel); //turn on PWM generation
}

void update_cooling() {

    double temp_readings[] = {ControllerTemp_C.data, motorTemp_C.data};
	static double cooling_thresholds[] = {INVERTER_TEMP_THRESH_C, MOTOR_TEMP_THRESH_C};
	static int total_cooling_thresholds = sizeof(cooling_thresholds) / sizeof(cooling_thresholds[0]); //amount of cooling thresholds


	//rad fan cooling
	for(int i = 0; i < total_cooling_thresholds; i++){
		if(rad_fan_state == RAD_FAN_OFF && (temp_readings[i] >= (cooling_thresholds[i] + HYSTERESIS_DIGITAL))){
			rad_fan_state = RAD_FAN_ON;

			//set both controller and motortemp to unstable if either are too high(for ease of implementation)
			for(int j = 0; j < total_cooling_thresholds; j++)
				steady_temperatures_achieved_fan[j] = false;

			fan_readings_below_HYS_threshold = 0;

			break; //if an early temperature trips it, no need to check the rest of them
		}

		else if(rad_fan_state == RAD_FAN_ON){
			if(steady_temperatures_achieved_fan[i] == false && (temp_readings[i] <= (cooling_thresholds[i] - HYSTERESIS_DIGITAL))){
				fan_readings_below_HYS_threshold++;
				steady_temperatures_achieved_fan[i] = true;
			}

			if(fan_readings_below_HYS_threshold == total_cooling_thresholds)
				rad_fan_state = RAD_FAN_OFF;
		}
	}

	//HAL_GPIO_WritePin(RAD_FAN_GPIO_Port, RAD_FAN_Pin, !(rad_fan_state));

	for(int i = 0; i < total_cooling_thresholds; i++){
			if(digital_pump_state == PUMP_DIGITAL_OFF && (temp_readings[i] >= (cooling_thresholds[i] + HYSTERESIS_DIGITAL))){
				digital_pump_state = PUMP_DIGITAL_ON;

				//set both controller and motortemp to unstable if either are too high(for ease of implementation)
				for(int j = 0; j < total_cooling_thresholds; j++)
					steady_temperatures_achieved_pump[j] = false;
				pump_readings_below_HYS_threshold = 0;

				break; //if an early temperature trips it, no need to check the rest of them
			}

			else if(digital_pump_state == PUMP_DIGITAL_ON){
				if(steady_temperatures_achieved_pump[i] == false && (temp_readings[i] <= (cooling_thresholds[i] - HYSTERESIS_DIGITAL))){
					pump_readings_below_HYS_threshold++;
					steady_temperatures_achieved_pump[i] = true;
				}

				if(pump_readings_below_HYS_threshold == total_cooling_thresholds)
					digital_pump_state = PUMP_DIGITAL_OFF;
			}
		}

	//if we are not moving don't run the fans, otherwise apply the normal temperature thresholds
	if(isVehicleMoving())
		HAL_GPIO_WritePin(PUMP_OUTPUT_GPIO_Port, PUMP_OUTPUT_Pin, PUMP_DIGITAL_OFF);
	else
		HAL_GPIO_WritePin(PUMP_OUTPUT_GPIO_Port, PUMP_OUTPUT_Pin, digital_pump_state);

}

void update_brakelight_and_buzzer(){
	if(brakePressureRear_psi.data > BRAKE_LIGHT_THRESH_psi) {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, MOSFET_PULL_DOWN_ON);
		update_and_queue_param_u8(&brakeLightOn_state, TRUE);
	} else {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, MOSFET_PULL_DOWN_OFF);
		update_and_queue_param_u8(&brakeLightOn_state, FALSE);
	}

	if(vehicleState_state.data == VEHICLE_PREDRIVE) {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, MOSFET_PULL_DOWN_ON);
		update_and_queue_param_u8(&vehicleBuzzerOn_state, TRUE);
	} else {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, MOSFET_PULL_DOWN_OFF);
		update_and_queue_param_u8(&vehicleBuzzerOn_state, FALSE);
	}
	return;
}

void LED_task(){
	static U32 last_led = 0;
	if(HAL_GetTick() - last_led >= HBEAT_LED_DELAY_TIME_ms) {
		HAL_GPIO_TogglePin(MCU_STATUS_LED_GPIO_Port, MCU_STATUS_LED_Pin);
		last_led = HAL_GetTick();
	}
	update_TSSI_LED();

}

void update_TSSI_LED(){
//	if(imdFault_state.data || bspdFault_state.data){
//		HAL_GPIO_WritePin(TSSI_GREEN_GPIO_Port, TSSI_GREEN_Pin, 0);
//		HAL_GPIO_WritePin(TSSI_RED_GPIO_Port, TSSI_RED_Pin, 1);
//	}
//	else{
//		HAL_GPIO_WritePin(TSSI_GREEN_GPIO_Port, TSSI_GREEN_Pin, 1);
//		HAL_GPIO_WritePin(TSSI_RED_GPIO_Port, TSSI_RED_Pin, 0);
//	}
}

boolean isVehicleMoving(){
	motor_rpm = electricalRPM_erpm.data / MOTOR_POLE_PAIRS; //equation for rpm from erpm
	if (motor_rpm < 50){
		return FALSE;
	}
	return TRUE;
}

void init_pullup_configs(){
	//should be set to 1 for 5V external pullup, 0 for direct connection to ground
	//must be configured for open drain for this to be the case
	HAL_GPIO_WritePin(PULLUP_GATE_1_GPIO_Port, PULLUP_GATE_1_Pin, 1);
	HAL_GPIO_WritePin(PULLUP_GATE_2_GPIO_Port, PULLUP_GATE_2_Pin, 1);
	HAL_GPIO_WritePin(PULLUP_GATE_3_GPIO_Port, PULLUP_GATE_3_Pin, 1);
	HAL_GPIO_WritePin(PULLUP_GATE_4_GPIO_Port, PULLUP_GATE_4_Pin, 1);
	HAL_GPIO_WritePin(PULLUP_GATE_5_GPIO_Port, PULLUP_GATE_5_Pin, 1);
	HAL_GPIO_WritePin(PULLUP_GATE_6_GPIO_Port, PULLUP_GATE_6_Pin, 1);
}
void update_brakeBias(){
//	if (brakePressureFront_psi.data > BRAKE_BIAS_PRESS_THRESH_psi && brakePressureRear_psi.data > BRAKE_BIAS_PRESS_THRESH_psi){
//		float bias = ((6.365*brakePressureFront_psi.data)/(6.365*brakePressureFront_psi.data + 3.125* brakePressureRear_psi.data))*100;
//		update_and_queue_param_float(&brakeBias_amount, bias);
//	}
}
