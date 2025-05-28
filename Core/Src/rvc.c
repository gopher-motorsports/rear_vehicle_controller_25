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

uint32_t maxCurrent_mA = 0;
uint32_t preDriveTimer_ms = 0;

float desiredCurrent_A = 0;
float maxcurrentLimit_A = 0;
float bspd_power_limit = 4000;
//float desiredTorque_Nm = 0;
float torqueLimit_Nm = 0;

float motor_rpm;

boolean appsBrakeLatched_state = 0;

boolean Current_Fault_3V3_state = 0;
boolean Current_Fault_5V_state = 0;

boolean current_driving_mode = 0;
boolean readyToDriveButtonPressed_state = 0;
boolean current_mode_button_state = 0;
boolean past_mode_button_state = 0;

VEHICLE_STATE_t vehicle_state = VEHICLE_NO_COMMS;
boolean vehicle_currently_moving = 0;
LAUNCH_CONTROL_STATES_t launch_control_state = LAUNCH_CONTROL_DISABLED;

//Cooling variables

// Test Temps
float TEST_TEMP_INVERTER = 27.0;
float TEST_TEMP_MOTOR = 27.0;
float test_rpm = 0;

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
#define RTD_DEBOUNCE_TIME_ms 25

// Initialization code goes here
void init(CAN_HandleTypeDef* hcan_ptr) {
	hcan = hcan_ptr;

	init_can(hcan, GCAN1);
}

void main_loop() {
//	update_and_queue_param_float(&brakeTempRearLeft_C, brakeTempRearLeft_C.data);
//	process_sensors();
//	update_outputs();
//	update_cooling();
//	update_display_fault_status();
	update_gcan_states(); // Should be after proceass_sensors
	LED_task();
//	update_brakeBias();
//	set_DRS_Servo_Position(FALSE);
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
	// Log pedal position percentages
	float pedalPos1 = 100.0*(pedalPosition1_mm.data-APPS_1_MIN_CURRENT_POS_mm)/APPS_1_TOTAL_TRAVEL_mm;
	if(pedalPos1 < 0) {
		pedalPos1 = 0;
	} else if (pedalPos1 > 100) {
		pedalPos1 = 100;
	}
	float pedalPos2 = 100.0*(pedalPosition2_mm.data-APPS_2_MIN_CURRENT_POS_mm)/APPS_2_TOTAL_TRAVEL_mm;
	if(pedalPos2 < 0) {
		pedalPos2 = 0;
	} else if (pedalPos2 > 100) {
		pedalPos2 = 100;
	}
	update_and_queue_param_float(&pedalPosition1_percent, pedalPos1);
	update_and_queue_param_float(&pedalPosition2_percent, pedalPos2);
	// Log BSPD out of range sensor faults
	update_and_queue_param_u8(&bspdBrakePressureSensorFault_state,
			HAL_GPIO_ReadPin(BSPD_BRK_FAULT_GPIO_Port, BSPD_BRK_FAULT_Pin) == BSPD_BRAKE_FAULT);
	update_and_queue_param_u8(&bspdTractiveSystemCurrentSensorFault_state,
			HAL_GPIO_ReadPin(BSPD_TS_SNS_FAULT_GPIO_Port, BSPD_TS_SNS_FAULT_Pin) == BSPD_TS_SNS_FAULT);
	// Log BSPD current/braking fault
	update_and_queue_param_u8(&bspdTractiveSystemBrakingFault_state,
			HAL_GPIO_ReadPin(BSPD_TS_BRK_FAULT_GPIO_Port, BSPD_TS_BRK_FAULT_Pin) == BSPD_TS_BRK_FAULT);

	// VCU software sensors faults, out of range checks
	update_and_queue_param_u8(&vcuPedalPosition1Fault_state, TIMED_SOFTWARE_FAULTS[0]->state);
	update_and_queue_param_u8(&vcuPedalPosition2Fault_state, TIMED_SOFTWARE_FAULTS[1]->state);
	update_and_queue_param_u8(&vcuBrakePressureSensorFault_state, TIMED_SOFTWARE_FAULTS[2]->state);
	update_and_queue_param_u8(&vcuTractiveSystemCurrentSensorFault_state, TIMED_SOFTWARE_FAULTS[3]->state);
	// VCU software safety checks, correlation and APPS/Brake Plausibility check
	update_and_queue_param_u8(&vcuPedalPositionCorrelationFault_state, TIMED_SOFTWARE_FAULTS[4]->state);
	update_and_queue_param_u8(&vcuPedalPositionBrakingFault_state, appsBrakeLatched_state);

	//current requested amps and max amps for DTI Inverter
	update_and_queue_param_float(&vcuCurrentRequested_A, desiredCurrent_A);
	update_and_queue_param_float(&vcuMaxCurrentLimit_A, maxcurrentLimit_A );
	// Cooling
//	update_and_queue_param_u8(&coolantFanPower_percent, rad_fan_state*100);
#ifdef USING_PUMP_PWM
	update_and_queue_param_u8(&coolantPumpPower_percent, (pwm_pump_intensity/32000) * 100); //calculate duty cycle percent
	digital_pump_state
#else
	update_and_queue_param_u8(&coolantPumpPower_percent, digital_pump_state*100); //digital pump state
#endif
	// Vehicle state
	update_and_queue_param_u8(&vehicleState_state, vehicle_state);
//	update_and_queue_param_u8(&readyToDriveButton_state, readyToDriveButtonPressed_state);

	update_and_queue_param_u8(&vcuGSenseStatus_state, HAL_GPIO_ReadPin(GSENSE_LED_GPIO_Port, GSENSE_LED_Pin));
	// Calculate wheel speed from rpm, change rpm to
	motor_rpm = electricalRPM_erpm.data * MOTOR_POLE_PAIRS;
	wheelSpeedRearRight_mph.data = ((motor_rpm * MINUTES_PER_HOUR) * WHEEL_DIAMETER_IN * MATH_PI) / (FINAL_DRIVE_RATIO * IN_PER_FT);
	wheelSpeedFrontLeft_mph.data = wheelSpeedFrontRight_mph.data;
}

void init_Pump(TIM_HandleTypeDef* timer_address, U32 channel){
	PUMP_PWM_Timer = timer_address;
	PUMP_Channel = channel;
	HAL_TIM_PWM_Start(PUMP_PWM_Timer, PUMP_Channel); //turn on PWM generation
}

void update_cooling() {

    double temp_readings[] = {ControllerTemp_C.data, motorTemp_C.data};
	//double temp_readings[] = {TEST_TEMP_INVERTER, TEST_TEMP_MOTOR};
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

//	HAL_GPIO_WritePin(RAD_FAN_GPIO_Port, RAD_FAN_Pin, !(rad_fan_state));
// Communicate with PLM to turn on RAD_FAN

	//pump cooling
/*#ifdef USING_PUMP_PWM
	for(int i = 0; i < total_cooling_thresholds; i++){
		if(pwm_pump_intensity == PUMP_INTENSITY_OFF && (temp_readings[i] >= (cooling_thresholds[i] + HYSTERESIS_ANALOG))){
			pwm_pump_intensity = PUMP_INTENSITY_1;

			for(int j = 0; j < total_cooling_thresholds; j++)
				steady_temperatures_achieved_pump[j] = false;
			pump_readings_below_HYS_threshold = 0;
			break;
		}

		else if(pwm_pump_intensity > PUMP_INTENSITY_OFF){
			if(temp_readings[i] >= INVERTER_TEMP_THRESH_C_4 || temp_readings[i] >= MOTOR_TEMP_THRESH_C_4) pwm_pump_intensity = PUMP_INTENSITY_4;
			else if(temp_readings[i] >= INVERTER_TEMP_THRESH_C_3 || temp_readings[i] >= MOTOR_TEMP_THRESH_C_3) pwm_pump_intensity = PUMP_INTENSITY_3;
			else if(temp_readings[i] >= INVERTER_TEMP_THRESH_C_2 || temp_readings[i] >= MOTOR_TEMP_THRESH_C_2) pwm_pump_intensity = PUMP_INTENSITY_2;
			else if(temp_readings[i] >= INVERTER_TEMP_THRESH_C_1 || temp_readings[i] >= MOTOR_TEMP_THRESH_C_1) pwm_pump_intensity = PUMP_INTENSITY_1;
			else if(steady_temperatures_achieved_pump[i] == false && (temp_readings[i] <= (cooling_thresholds[i] - HYSTERESIS_ANALOG))){
				pump_readings_below_HYS_threshold++;
				steady_temperatures_achieved_pump[i] = true;
			}

			if(pump_readings_below_HYS_threshold == total_cooling_thresholds){
				pwm_pump_intensity = PUMP_INTENSITY_OFF;
			}
		}
	}

	__HAL_TIM_SET_COMPARE(PUMP_PWM_Timer, PUMP_Channel, pwm_pump_intensity);
#else
*/
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
	HAL_GPIO_WritePin(PUMP_OUTPUT_GPIO_Port, PUMP_OUTPUT_Pin, digital_pump_state);
		//HAL_GPIO_WritePin(PUMP_OUTPUT_GPIO_Port, PUMP_OUTPUT_Pin, 0);
//	if(motor_rpm < 50)
//	if(test_rpm < 50)
//		HAL_GPIO_WritePin(PUMP_OUTPUT_GPIO_Port, PUMP_OUTPUT_Pin, PUMP_DIGITAL_OFF);
//	else
//		HAL_GPIO_WritePin(PUMP_OUTPUT_GPIO_Port, PUMP_OUTPUT_Pin, PUMP_DIGITAL_ON);
//#endif

}

void process_sensors() {
	current_mode_button_state = swButon2_state.data;
	if(past_mode_button_state == 0 && current_mode_button_state == 1){
		current_driving_mode = !current_driving_mode;
	}
	past_mode_button_state = current_mode_button_state;

	// read in the RTD button. This is a software low pass to make sure noise does not press the button
	static U32 new_event_time;
	static U8 new_event = FALSE;
	if (!new_event)
	{
		// check if there is a change in polarity of the button
		if (readyToDriveButtonPressed_state != (HAL_GPIO_ReadPin(RTD_BUTTON_GPIO_Port, RTD_BUTTON_Pin) == RTD_BUTTON_PUSHED))
		{
			new_event = TRUE;
			new_event_time = HAL_GetTick();
		}
	}
	else
	{
		// the button change was not held long enough
		if (readyToDriveButtonPressed_state == (HAL_GPIO_ReadPin(RTD_BUTTON_GPIO_Port, RTD_BUTTON_Pin) == RTD_BUTTON_PUSHED))
		{
			new_event = FALSE;
		}
		else
		{
			// see if enough time has passed to actually call this a press
			if (HAL_GetTick() - new_event_time >= RTD_DEBOUNCE_TIME_ms)
			{
				new_event = FALSE;
				readyToDriveButtonPressed_state = !readyToDriveButtonPressed_state;
			}
		}
	}

	maxcurrentLimit_A = get_current_limit(current_driving_mode);


	update_struct_fault_data(); //refresh sensor data
	// Input Validation (Out of Range) and Correlation Checks
	SOFTWARE_FAULT* fault;
	for(int i = 0; i < NUM_OF_TIMED_FAULTS; i++){
		fault = TIMED_SOFTWARE_FAULTS[i];
		if(fault->data > fault->max_threshold || fault->data < fault->min_threshold){ //correlation has no min, but edge case accounted for in defines
			fault->fault_timer++;
			if(fault->fault_timer > fault->input_delay_threshold){
				fault->fault_timer = fault->input_delay_threshold + 1; //cap at delay_threshold + 1 so that it trips but doesn't count up more
			}
		}
		else{
			fault->fault_timer = 0;
			fault->state = false;
		}

		if(fault->fault_timer > fault->input_delay_threshold){
			fault->state = true;
			maxcurrentLimit_A = 0;
		}
	}


	// APPS/Braking Pedal Plausibility Check, edge case handled without struct
	/*if(brakePressureFront_psi.data > APPS_BRAKE_PRESS_THRESH_psi && pedalPosition1_mm.data > APPS_BRAKE_APPS1_THRESH_mm) {
		appsBrakeLatched_state = TRUE;
	} else if (pedalPosition1_mm.data <= APPS_BRAKE_RESET_APPS1_THRESH_mm) {
		appsBrakeLatched_state = FALSE;
	}*/

	if(!BYPASS_ACTIVE){
		if(brakePressureFront_psi.data > APPS_BRAKE_PRESS_THRESH_psi && pedalPosition1_percent.data > 25) {
				appsBrakeLatched_state = TRUE;
		} else if (pedalPosition1_percent.data <= 5) {
			appsBrakeLatched_state = FALSE;
		}

		if(appsBrakeLatched_state) {
			maxcurrentLimit_A = 0;
		}
	}
	//Sensor overcurrent Logic, turn off power to inverter if any of the sensor power lines are overcurrenting
#ifdef USING_SOFTWARE_OVERCURRENT_PROT
	Current_Fault_3V3_state = HAL_GPIO_ReadPin(CURR_FAULT_3V3_GPIO_Port, CURR_FAULT_3V3_Pin) == SENSOR_OVERCURRENT_TRIPPED; //active low
	Current_Fault_5V_state  = HAL_GPIO_ReadPin(CURR_FAULT_5V_GPIO_Port, CURR_FAULT_5V_Pin) == SENSOR_OVERCURRENT_TRIPPED; //active low

	static U32 overcurrent_event_timer_3V3 = 0;
	static U32 overcurrent_event_timer_5V = 0;

	if(Current_Fault_3V3_state){
		overcurrent_event_timer_3V3++;
		if(overcurrent_event_timer_3V3 >= SENSOR_OVERCURRENT_TIME_THRESH)
			maxcurrentLimit_A = 0;
	}
	else{
		overcurrent_event_timer_3V3 = 0;
	}

	if(Current_Fault_5V_state){
		overcurrent_event_timer_5V++;
		if(overcurrent_event_timer_5V >= SENSOR_OVERCURRENT_TIME_THRESH)
			maxcurrentLimit_A = 0;
	}
	else{
		overcurrent_event_timer_5V = 0;
	}
#endif
	// TODO make some hysteresis on this in order to make it less jumpy
	if(!BYPASS_ACTIVE){
		if(bspdTractiveSystemBrakingFault_state.data) {
			float tractiveSystemBrakingLimit_A = 0;
			if(inputInverterVoltage_V.data != 0) {
				tractiveSystemBrakingLimit_A = bspd_power_limit / inputInverterVoltage_V.data; //stay below 5 kW I = P/V
			}
			// If the tractive system braking limit is less (more restrictive),
			// then set the torque limit to that amount
			if(tractiveSystemBrakingLimit_A < maxcurrentLimit_A) {
				update_and_queue_param_u8(&vcuBrakingClampingCurrent_state, TRUE);
				maxcurrentLimit_A = tractiveSystemBrakingLimit_A;
			} else {
				update_and_queue_param_u8(&vcuBrakingClampingCurrent_state, FALSE);
			}
		}
	}
	desiredCurrent_A = ((pedalPosition1_mm.data-APPS_1_MIN_CURRENT_POS_mm)/APPS_1_TOTAL_TRAVEL_mm) * get_current_limit(current_driving_mode);

	if(pedalPosition1_mm.data < APPS_1_MIN_CURRENT_POS_mm) {
		desiredCurrent_A = 0;
	}

	if(desiredCurrent_A > get_current_limit(current_driving_mode)) {
		desiredCurrent_A =  get_current_limit(current_driving_mode);
	}
}



void update_display_fault_status() {
	int status = NONE;
	if(amsFault_state.data) status = AMS_FAULT;
	else if (vehicle_state == VEHICLE_FAULT) status = INVERTER_FAULT;
	else if(bmsNumActiveAlerts_state.data) status = BMS_FAULT;
	else if(vcuPedalPositionBrakingFault_state.data) status = RELEASE_PEDAL;
	else if((bspdTractiveSystemBrakingFault_state.data || vcuBrakingClampingCurrent_state.data) && (!BYPASS_ACTIVE)) status = BRAKING_FAULT;
	else if(vcuPedalPositionCorrelationFault_state.data) status = APPS_FAULT;
	else if((bspdFault_state.data
			|| bspdBrakePressureSensorFault_state.data
			|| bspdTractiveSystemCurrentSensorFault_state.data)
			&& (!BYPASS_ACTIVE)) status = BSPD_FAULT;
	else if((vcuBrakePressureSensorFault_state.data
			|| vcuPedalPosition1Fault_state.data
			|| vcuPedalPosition2Fault_state.data
			|| vcuTractiveSystemCurrentSensorFault_state.data) && (!BYPASS_ACTIVE)
			) status = VCU_FAULT;

	update_and_queue_param_u8(&displayFaultStatus_state, status);
}
U8 ts_volt_sim = 200;


void update_outputs() {
	if(brakePressureFront_psi.data > BRAKE_LIGHT_THRESH_psi) {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, MOSFET_PULL_DOWN_ON);
		update_and_queue_param_u8(&brakeLightOn_state, TRUE);
	} else {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, MOSFET_PULL_DOWN_OFF);
		update_and_queue_param_u8(&brakeLightOn_state, FALSE);
	}
	return;
}

void LED_task(){
	static U32 last_led = 0;
	if(HAL_GetTick() - last_led >= HBEAT_LED_DELAY_TIME_ms) {
		HAL_GPIO_TogglePin(MCU_STATUS_LED_GPIO_Port, MCU_STATUS_LED_Pin);
		HAL_GPIO_TogglePin(BRK_LT_GPIO_Port, BRK_LT_Pin);
		HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
//		HAL_GPIO_TogglePin(PCB_BUZZER_GPIO_Port, PCB_BUZZER_Pin);

		last_led = HAL_GetTick();
	}
	update_TSSI_LED();

	HAL_GPIO_WritePin(HARDFAULT_LED_GPIO_Port, HARDFAULT_LED_Pin, 1);
	HAL_GPIO_WritePin(PCB_BUZZER_GPIO_Port, PCB_BUZZER_Pin, 1);
//	HAL_GPIO_WritePin(PULLUP_GATE_1_GPIO_Port, PULLUP_GATE_1_Pin, 1);
//	HAL_GPIO_WritePin(PULLUP_GATE_2_GPIO_Port, PULLUP_GATE_2_Pin, 1);
//	HAL_GPIO_WritePin(PULLUP_GATE_3_GPIO_Port, PULLUP_GATE_3_Pin, 1);
//	HAL_GPIO_WritePin(PULLUP_GATE_4_GPIO_Port, PULLUP_GATE_4_Pin, 1);
//	HAL_GPIO_WritePin(PULLUP_GATE_5_GPIO_Port, PULLUP_GATE_5_Pin, 1);
//	HAL_GPIO_WritePin(PULLUP_GATE_6_GPIO_Port, PULLUP_GATE_6_Pin, 1);
}

void update_TSSI_LED(){
	if(imdFault_state.data || bspdFault_state.data){
		HAL_GPIO_WritePin(TSSI_GREEN_GPIO_Port, TSSI_GREEN_Pin, 0);
		HAL_GPIO_WritePin(TSSI_RED_GPIO_Port, TSSI_RED_Pin, 1);
	}
	else{
		HAL_GPIO_WritePin(TSSI_GREEN_GPIO_Port, TSSI_GREEN_Pin, 1);
		HAL_GPIO_WritePin(TSSI_RED_GPIO_Port, TSSI_RED_Pin, 0);
	}
}

void update_brakeBias(){
//	if (brakePressureFront_psi.data > BRAKE_BIAS_PRESS_THRESH_psi && brakePressureRear_psi.data > BRAKE_BIAS_PRESS_THRESH_psi){
//		float bias = ((6.365*brakePressureFront_psi.data)/(6.365*brakePressureFront_psi.data + 3.125* brakePressureRear_psi.data))*100;
//		update_and_queue_param_float(&brakeBias_amount, bias);
//	}
}

int get_current_limit(boolean driving_mode){
	if(driving_mode == SLOW_MODE)
		return 550; // 10 A
	else
		return 550; // 400 A
}

// End of vcu.c
