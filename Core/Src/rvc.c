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
#include <math.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"
#include "sdc.h"
// The HAL_CAN struct
CAN_HandleTypeDef* hcan;

//Fault Timers:
uint16_t rear_brake_press_timer = 0;
uint16_t current_sensor_timer = 0;
uint16_t TS_braking_timer = 0;
uint16_t bspd_timer = 0;
//Fault Tripped States
boolean rear_brake_press_fault_tripped = FALSE;
boolean current_sensor_fault_tripped = FALSE;
boolean TS_braking_fault_tripped = FALSE;

//Input Dependent faults
boolean input_fault_tripped = FALSE;
boolean bspd_fault_tripped = FALSE;

//Motor Variables
float motor_rpm = 0;
float pump_decimal;
uint8_t rear_brake_hardware_fault_active = 0;
uint8_t current_hardware_range_fault_active = 0;
uint8_t TS_braking_hardware_fault_active = 0;

//Current Sensing Variables:
uint8_t currentSensorStatus = UNINITIALIZED;

//Cooling variables

//Fan
boolean steady_temperatures_achieved_fan[] = {true, true}; //LOT if fan temperatures have returned to steady state, implemented to stop double counting
U8 fan_readings_below_HYS_threshold = 0;
U8 rad_fan_state = RAD_FAN_OFF;

//Pump
TIM_HandleTypeDef* PUMP_PWM_Timer;
U32 PUMP_Channel;
float pump_percent;
boolean steady_temperatures_achieved_pump[] = {true, true}; //LOT if pump temperatures have returned to ready state
U8 pump_readings_below_HYS_threshold = 0;

U8 digital_pump_state = PUMP_DIGITAL_OFF; //if no pump pwm and just digital

// Shut Down Circuit Sensing
uint8_t shutDownBreakPoint = 0;

#define HBEAT_LED_DELAY_TIME_ms 500
#define TSSI_RED_BLINK_TIME_ms 333 //corresponds to 3 Hz

// Initialization code goes here
void init(CAN_HandleTypeDef* hcan_ptr) {
	hcan = hcan_ptr;

	init_can(hcan, GCAN1);
}

void main_loop() {
	check_faults();
    update_cooling();
	update_gcan_states(); // Should be after proceass_sensors
	LED_task();

	update_brakelight_and_buzzer();
	update_brakeBias();

    set_DRS_Servo_Position(FALSE); //DRS set position
	shutDownCircuitStatus();
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

void check_faults(){
	//reading faults from hardware, fault active = 0 so inverted signaling
	rear_brake_hardware_fault_active = !HAL_GPIO_ReadPin(BSPD_BRK_FAULT_GPIO_Port, BSPD_BRK_FAULT_Pin);
	current_hardware_range_fault_active = !HAL_GPIO_ReadPin(BSPD_TS_SNS_FAULT_GPIO_Port, BSPD_TS_SNS_FAULT_Pin);
	TS_braking_hardware_fault_active = HAL_GPIO_ReadPin(BSPD_TS_BRK_FAULT_GPIO_Port, BSPD_TS_BRK_FAULT_Pin);

	//Rear Brake Pressure and current out of range
	if(rear_brake_hardware_fault_active){
		rear_brake_press_timer = (rear_brake_press_timer <= INPUT_DELAY_ms) ? rear_brake_press_timer + 1 : INPUT_DELAY_ms + 1;
		rear_brake_press_fault_tripped = (rear_brake_press_timer > INPUT_DELAY_ms) ? TRUE : FALSE;
	}
	else{
		rear_brake_press_timer = 0;
		rear_brake_press_fault_tripped = FALSE;
	}

	if(current_hardware_range_fault_active){
		current_sensor_timer = (current_sensor_timer <= INPUT_DELAY_ms) ? current_sensor_timer + 1 : INPUT_DELAY_ms + 1;
		current_sensor_fault_tripped = (current_sensor_timer > INPUT_DELAY_ms) ? TRUE : FALSE;
	}
	else{
		rear_brake_press_timer = 0;
		current_sensor_fault_tripped = FALSE;
	}

	//Hardbreaking + 5kw of Power
	if(TS_braking_hardware_fault_active){
		TS_braking_timer = (TS_braking_timer <= INPUT_DELAY_ms) ? TS_braking_timer + 1 : INPUT_DELAY_ms + 1;
		TS_braking_fault_tripped = (current_sensor_timer > INPUT_DELAY_ms) ? TRUE : FALSE;
	}
	else{
		TS_braking_timer = 0;
		TS_braking_fault_tripped = FALSE;
	}


	//input fault = if rear brake or current sensor is out of range
	input_fault_tripped = rear_brake_press_fault_tripped || current_sensor_fault_tripped;

	//bspd fault can only trip once and then it will be latched until power cycle
	if(!bspd_fault_tripped){
		if(input_fault_tripped || TS_braking_fault_tripped){
			bspd_timer = (bspd_timer <= BSPD_DELAY_ms) ? bspd_timer + 1 : BSPD_DELAY_ms + 1;
			bspd_fault_tripped = (bspd_timer > BSPD_DELAY_ms) ? TRUE : FALSE;
		}
		else{
			bspd_timer = 0;
		}
	}
}

void update_gcan_states() {
	// ================================Fault Parameters to Forward to FVC============================
	//Update if Faults Have Been Tripped (currently happening, and has passed timeout)
	update_and_queue_param_u8(&bspdBrakePressureSensorFault_state, rear_brake_press_fault_tripped); //Brake Pressure Out of Range
	update_and_queue_param_u8(&bspdTractiveSystemCurrentSensorFault_state, current_sensor_fault_tripped); //Current Sesnor Out of Range
	update_and_queue_param_u8(&bspdTractiveSystemBrakingFault_state, TS_braking_fault_tripped); //TS Brake Fault --> Hard breaking + 5kW of power from tractive system
	update_and_queue_param_u8(&bspdInputFault_state, input_fault_tripped); //Input Fault
	update_and_queue_param_u8(&bspdFault_state, bspd_fault_tripped); //Overall BSPD
	// ==============================================================================================

	// Cooling
	update_and_queue_param_u8(&coolantFanPower_percent, rad_fan_state*100);
	update_and_queue_param_u8(&coolantPumpPower_percent, digital_pump_state * 100);

	//Current Sense:
	update_and_queue_param_float(&currentSensor_A, getTractiveSystemCurrent());

	//SDC Sense:
	update_and_queue_param_u8(&sdcStatus8, HAL_GPIO_ReadPin(SDC_IN_SENSE_GPIO_Port, SDC_IN_SENSE_Pin));
	update_and_queue_param_u8(&sdcStatus9, HAL_GPIO_ReadPin(SDC_OUT_SENSE_GPIO_Port, SDC_OUT_SENSE_Pin));
}

void init_Pump(TIM_HandleTypeDef* timer_address, U32 channel){
	PUMP_PWM_Timer = timer_address;
	PUMP_Channel = channel;
	HAL_TIM_PWM_Start(PUMP_PWM_Timer, PUMP_Channel); //turn on PWM generation
}

void update_cooling() {
	//motor_mph = electricalRPM_erpm.data * DRIVE_RATIO;
	float inv_temp = ControllerTemp_C.data;
	float motor_temp = motorTemp_C.data;

	if ((inv_temp > INVERTER_PUMP_POWER_ON_THRESH) || (motor_temp > MOTOR_PUMP_THRESH_C)) {
			digital_pump_state = PUMP_DIGITAL_ON;
	} else if ((inv_temp < INVERTER_PUMP_POWER_ON_THRESH - COOLING_HYSTERESIS_C) && (motor_temp < MOTOR_PUMP_THRESH_C - COOLING_HYSTERESIS_C)) {
			digital_pump_state = PUMP_DIGITAL_OFF;
	}

	//radiator fan
	if ((inv_temp > INVERTER_FAN_THRESH_C) || (motor_temp > MOTOR_FAN_THRESH_C)) {
			rad_fan_state = RAD_FAN_ON;
	} else if ((inv_temp < INVERTER_FAN_THRESH_C - COOLING_HYSTERESIS_C) && (motor_temp < MOTOR_FAN_THRESH_C - COOLING_HYSTERESIS_C)) {
			rad_fan_state = RAD_FAN_OFF;
	}

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
		HAL_GPIO_WritePin(PCB_BUZZER_GPIO_Port, PCB_BUZZER_Pin, PCB_BUZZ_ON);
		update_and_queue_param_u8(&vehicleBuzzerOn_state, TRUE);
	} else {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, MOSFET_PULL_DOWN_OFF);
		HAL_GPIO_WritePin(PCB_BUZZER_GPIO_Port, PCB_BUZZER_Pin, PCB_BUZZ_OFF);
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
	if(HAL_GetTick() > TSSI_RESET_TIME_ms && (imdFault_state.data || amsFault_state.data)) {
		HAL_GPIO_WritePin(TSSI_GREEN_GPIO_Port, TSSI_GREEN_Pin, 0);
		HAL_GPIO_WritePin(TSSI_RED_GPIO_Port, TSSI_RED_Pin, (HAL_GetTick() % TSSI_FLASH_PERIOD_ms) < TSSI_FLASH_PERIOD_ms / 2);
	}
	else{
		HAL_GPIO_WritePin(TSSI_RED_GPIO_Port, TSSI_RED_Pin, 0);
		HAL_GPIO_WritePin(TSSI_GREEN_GPIO_Port, TSSI_GREEN_Pin, 1);
	}
}

boolean isVehicleMoving(){
	if (motor_rpm < 50){
		return FALSE;
	}
	return TRUE;
}

void init_pullup_configs(){
	//should be set to 1 for 5V external pullup, 0 for direct connection to ground
	// 1 = Temperature Sensor Mode, 0 = External Pullup Mode
	//must be configured for open drain for this to be the case

	HAL_GPIO_WritePin(PULLUP_GATE_1_GPIO_Port, PULLUP_GATE_1_Pin, 1);
	HAL_GPIO_WritePin(PULLUP_GATE_2_GPIO_Port, PULLUP_GATE_2_Pin, 1);
	HAL_GPIO_WritePin(PULLUP_GATE_3_GPIO_Port, PULLUP_GATE_3_Pin, 0);
	HAL_GPIO_WritePin(PULLUP_GATE_4_GPIO_Port, PULLUP_GATE_4_Pin, 0);
	HAL_GPIO_WritePin(PULLUP_GATE_5_GPIO_Port, PULLUP_GATE_5_Pin, 1);
	HAL_GPIO_WritePin(PULLUP_GATE_6_GPIO_Port, PULLUP_GATE_6_Pin, 1);
}

void update_brakeBias(){
	static float bias = 0;
	if (brakePressureFront_psi.data > BRAKE_BIAS_PRESS_THRESH_psi && brakePressureRear_psi.data > BRAKE_BIAS_PRESS_THRESH_psi){
		bias = ((6.365*brakePressureFront_psi.data)/(6.365*brakePressureFront_psi.data + 3.125* brakePressureRear_psi.data))*100;
		update_and_queue_param_float(&brakeBias_percent, bias);
	}
}


float getTractiveSystemCurrent(){
    // Fetch current sensor data from gophercan
	float tractiveSystemCurrent = 0;
    float currHI = currentSensorHigh_A.data;
    float currLO = currentSensorLow_A.data;
    uint8_t currentSensorStatusHI = 0;
    uint8_t currentSensorStatusLO = 0;

    // If the current exceeds the following threshold in either the positive or negative direction,
    // the sensor input has railed to 0 or 5v and a current sensor error is set
    currentSensorStatusHI = (fabs(currHI) < CURRENT_HIGH_RAIL_THRESHOLD) ? (WORKING) : (FAULTING);
    currentSensorStatusLO = (fabs(currLO) < CURRENT_LOW_RAIL_THRESHOLD) ? (WORKING) : (FAULTING);

    // To use the HI current sensor channel, it must be working AND (it must exceed the measuring range of the low channel OR the low channel must be faulty)
    if ((currentSensorStatusHI == WORKING) && ((fabs(currLO) > CURRENT_LOW_TO_HIGH_SWITCH_THRESHOLD + (CHANNEL_FILTERING_WIDTH / 2)) || (currentSensorStatusLO != WORKING)))
    {
        tractiveSystemCurrent = currHI;
        currentSensorStatus = WORKING;
    }
    else if ((currentSensorStatusHI == WORKING) && (currentSensorStatusLO == WORKING) && ((fabs(currLO) > CURRENT_LOW_TO_HIGH_SWITCH_THRESHOLD - (CHANNEL_FILTERING_WIDTH / 2))))
    {
        float interpolationStart    =   CURRENT_LOW_TO_HIGH_SWITCH_THRESHOLD  - (CHANNEL_FILTERING_WIDTH / 2);
        float interpolationRatio    =   (currLO - interpolationStart) / CHANNEL_FILTERING_WIDTH;
        float filteredCurrent       =   ((1.0f - interpolationRatio) * currLO) + (interpolationRatio * currHI);
        tractiveSystemCurrent  =   filteredCurrent;
        currentSensorStatus = WORKING;
    }
    else if (currentSensorStatusLO == WORKING) // If the above condition is not satisfied, the LO channel must be working in order to use its data
    {
        tractiveSystemCurrent = currLO;
        currentSensorStatus = WORKING;
    }
    else // If both sensors are faulty, no current data can be accurately returned
    {
    	currentSensorStatus = FAULTING;
    }

    return tractiveSystemCurrent;
}

void shutDownCircuitStatus() {
	shutDownBreakPoint = 0;
	for (uint8_t i = 0; i < SDC_NUM_BREAKPOINTS; i++)
	{
		if (*sdcStatusParams[i] == 1)
		{
			shutDownBreakPoint = i + 1;  // Breakpoints are 1-indexed
			break; // Stop checking if breakpoint is detected
		}
	}
	update_and_queue_param_u8(&sdcBreakPoint, shutDownBreakPoint);
}
