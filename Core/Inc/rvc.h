/*
 * vcu.h
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_RVC_H_
#define INC_RVC_H_

#include "main.h"
#include "GopherCAN.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdbool.h>
#include <cmsis_os.h>

// ========================================== CONSTANTS =========================================
#define MATH_PI           3.14159265
#define MATH_TAU          MATH_PI*2
#define SECONDS_PER_MIN   60
#define MINUTES_PER_HOUR  60     // For conversion from rpm to mph
#define IN_PER_FT         12     // For conversion from rpm to mph
// ==============================================================================================

// ========================================== PWM CONSTANTS =========================================
#define TIM2_PWM_MAX	1600 //value where max overflow happens, defined in IOC in timer config

// ======================================= APPS PARAMETERS ======================================
#define APPS_MAX_TORQUE_POS_mm  21.5f // The position of the pedal at 100% torque
#define APPS_MIN_TORQUE_POS_mm  2.5f  // The position of the pedal at 0% torque
//#define APPS_MAX_ERROR_POS_mm 24.0f // position where the error begins
//#define APPS_MIN_ERROR_POS_mm 1.0f // position where the error begins
//#define APPS_TOTAL_TRAVEL_mm ( APPS_MAX_TORQUE_POS_mm - APPS_MIN_TORQUE_POS_mm )

#define APPS_1_MAX_CURRENT_POS_mm  21.5f // The position of the pedal at 100% torque
#define APPS_1_MIN_CURRENT_POS_mm  5.0f  // The position of the pedal at 0% torque
#define APPS_2_MAX_CURRENT_POS_mm  22.0f // The position of the pedal at 100% torque
#define APPS_2_MIN_CURRENT_POS_mm  5.5f  // The position of the pedal at 0% torque
#define APPS_MAX_ERROR_POS_mm 25.1f // position where the error begins, check back on this
#define APPS_MIN_ERROR_POS_mm .65f // position where the error begins
#define APPS_1_TOTAL_TRAVEL_mm ( APPS_1_MAX_CURRENT_POS_mm - APPS_1_MIN_CURRENT_POS_mm )
#define APPS_2_TOTAL_TRAVEL_mm ( APPS_2_MAX_CURRENT_POS_mm - APPS_2_MIN_CURRENT_POS_mm )
// ==============================================================================================

// ====================================== BRAKE PARAMETERS ======================================
#define BRAKE_PRESS_MIN_psi    -300   // The minimum value of the brake pressure sensor
#define BRAKE_PRESS_MAX_psi    2050  // The maximum value of the brake pressure sensor
#define BRAKE_LIGHT_THRESH_psi 25 // The pressure at which the brake light will be activated
#define BRAKE_BIAS_PRESS_THRESH_psi 100 // The pressure required to update the brake bias value
// ==============================================================================================
// ============================= TRACTIVE SYSTEM CURRENT PARAMETERS =============================
#define TS_CURRENT_MIN_A   -85   // The minimum value of the current sensor
#define TS_CURRENT_MAX_A   100  // The maximum value of the current sensor
#define MIN_LIMIT_SPEED_rpm 1000 // minimum RPM that the VCU begins current limiting
// =============================================================================================

// ================================== READY TO DRIVE PARAMETERS =================================
#define PREDRIVE_BRAKE_THRESH_psi  10  // The minimum brake pressure to enter the driving state
#define PREDRIVE_BUTTON_PRESSED    1    // The value of the button parameter when pressed
#define PREDRIVE_TIME_ms           2000 // The length of predrive in ms
#define RTD_BUTTON_PUSHED          (GPIO_PIN_RESET)
#define TS_ON_THRESHOLD_VOLTAGE_V  190
#define TSSI_FLASH_PERIOD_ms       300
#define TSSI_RESET_TIME_ms         12000
// ==============================================================================================


// ====================================== SAFETY PARAMETERS =====================================
// -------------------------------------- Input Validation --------------------------------------
#define INPUT_DELAY_ms 85  // In ms The amount of time it takes an input fault to take effect
#define BSPD_DELAY_ms 400 // In ms amount of time before bspd fault trips
// -------------------------------------- APPS/Brake Check --------------------------------------
// This check is done using APPS1 (since APPS1 determines the applied torque) and the BSE
#define APPS_BRAKE_PRESS_THRESH_psi  100.0f  // The minimum amount of brake pressure that will trip
// The minimum APPS position that will trip the APPS/Brake check
#define APPS_BRAKE_APPS1_THRESH_mm   ( APPS_1_TOTAL_TRAVEL_mm * 0.25 ) + APPS_1_MIN_CURRENT_POS_mm
// The maximum APPS position that will reset the APPS/Brake check
#define APPS_BRAKE_RESET_APPS1_THRESH_mm   ( APPS_1_TOTAL_TRAVEL_mm * 0.05 ) + APPS_1_MIN_CURRENT_POS_mm

// ------------------------------------ APPS Correlation Check ----------------------------------
//#define APPS_CORRELATION_THRESH_mm   ( APPS_TOTAL_TRAVEL_mm * 0.1 )
#define APPS_CORRELATION_THRESH_mm   10
#define CORRELATION_TRIP_DELAY_ms    85  // The amount of time it takes a correlation fault to take effect
#define APPS_CORRELATION_OFFSET_mm   0.0  // The average offset of the APPSs, positive is APPS1 > APPS2

// ------------------------------------ TS Current/Brake Check ----------------------------------
#define BRAKE_TS_CURRENT_THRESH_A  14.0f  // The current limit when the brake and
#define BRAKE_TS_PRESS_THRESH_psi  450.0f // The amount of brake pressure needed
#define BRAKE_TS_MAX_TORQUE            // A clamp on the maximum amount of torque when triggered
#define BRAKE_TS_ON_DELAY_ms       50  // The amount of timer it takes the limit to turn on
#define BRAKE_TS_OFF_DELAY_ms      50  // The amount of timer it takes the limit to turn off
// ==============================================================================================

// ====================================== COOLING PARAMETERS ====================================
#define INVERTER_PUMP_THRESH_C    60.0f // Inverter temperature at which the cooling system turns on
#define MOTOR_PUMP_THRESH_C       70.0f // Motor temperature at which the cooling system turns on
#define INVERTER_FAN_THRESH_C    80.0f // Inverter temperature at which the cooling system turns on
#define MOTOR_FAN_THRESH_C       90.0f // Motor temperature at which the cooling system turns on

#define COOLING_HYSTERESIS_C      10.0f // Hysteresis when confined to digital signal (on/off)
#define DRS_HYSTERESIS			  5.0f
//#define USING_PUMP_PWM
#define PUMP_INTENSITY_OFF		  0  //0% duty cycle --> 0/31999
#define PUMP_INTENSITY_1		  8000 //25% duty cycle --> 8000/31999
#define PUMP_INTENSITY_2		  16000 //50% duty cycle --> 16000/31999
#define PUMP_INTENSITY_3		  24000 //75% duty cycle --> 24000/31999
#define PUMP_INTENSITY_4		  31999 //100% duty cycle --> 31999/31999

#define PUMP_DIGITAL_ON			  GPIO_PIN_SET
#define PUMP_DIGITAL_OFF 		  GPIO_PIN_RESET
// ==============================================================================================

// =============================== SENSOR OVERCURRENT PARAMETERS ================================
#define SENSOR_OVERCURRENT_TRIPPED      (GPIO_PIN_RESET)
#define SENSOR_OVERCURRENT_TIME_THRESH 	5
//#define USING_SOFTWARE_OVERCURRENT_PROT

// ================================== TRACTIVE SYSTEM PARAMETERS 2023 ================================
#define MOTOR_DIRECTION         1      // Motor direction; 0 is reverse, 1 is forward
#define MAX_CMD_TORQUE_Nm       150.0f    // The maximum torque that will be commanded
#define PARAM_CMD_FAULT_CLEAR   20     // Address of the fault clear parameter
#define PARAM_CMD_READ          0      // Value to send in parameter command to read value
#define PARAM_CMD_WRITE         1      // Value to send in parameter command to read value
#define PARAM_FAULT_CLEAR_DATA  0      // Value to send in the data field when clearing faults
#define FINAL_DRIVE_RATIO       4.363  // The final drive ratio
#define WHEEL_DIAMETER_IN       10     // Wheel diameter
#define MOTOR_POLE_PAIRS   		10 	   // Amount of Pole Pairs of the EMRAX Motor
#define BSPD_POWER_LIMIT		4000   // 5 kW limit before a car shutdown is required, 4 kW with buffer

// ==============================================================================================

// ================================== TRACTIVE SYSTEM PARAMETERS 2024 ================================
#define MAX_TEST_CMD_CURRENT_A    			 550  // The maximum current that will be commanded
#define VEHICLE_STOPPED_THRESHOLD			 1000 //If vehicle is stopped for 1 sec
#define SLOW_MODE							 1 // If vehicle is in slow mode;



// ======================================== I/O PARAMETERS ======================================
#define MOSFET_PULL_DOWN_ON (GPIO_PIN_SET)
#define MOSFET_PULL_DOWN_OFF (GPIO_PIN_RESET)
#define PLM_CONTROL_ON (GPIO_PIN_RESET)
#define PLM_CONTROL_OFF (GPIO_PIN_SET)
#define RAD_FAN_ON (GPIO_PIN_SET)
#define RAD_FAN_OFF (GPIO_PIN_RESET)
#define PCB_BUZZ_ON (GPIO_PIN_SET)
#define PCB_BUZZ_OFF (GPIO_PIN_RESET)
// ==============================================================================================

// ======================================= BSPD PARAMETERS ======================================
// Whether each fault is active high or active low
#define BSPD_APPS1_FAULT     GPIO_PIN_RESET
#define BSPD_APPS2_FAULT     GPIO_PIN_RESET
#define BSPD_BRAKE_FAULT     GPIO_PIN_RESET
#define BSPD_TS_SNS_FAULT    GPIO_PIN_RESET
#define BSPD_TS_BRK_FAULT    GPIO_PIN_SET
// ==============================================================================================
#define BYPASS_ACTIVE		TRUE
#define BYPASS_MAX FLT_MAX
#define BYPASS_MIN -FLT_MIN
// ======================================= BYPASS PARAMETERS ======================================
// Allows us to disable a particular fault
#define BSPD_APPS1_FAULT     GPIO_PIN_RESET
#define BSPD_APPS2_FAULT     GPIO_PIN_RESET
#define BSPD_BRAKE_FAULT     GPIO_PIN_RESET
#define BSPD_TS_SNS_FAULT    GPIO_PIN_RESET
#define BSPD_TS_BRK_FAULT    GPIO_PIN_SET
// =========================================Current Sensing=====================================================

#define CURRENT_HIGH_RAIL_THRESHOLD             852
#define CURRENT_LOW_RAIL_THRESHOLD              87
#define CURRENT_LOW_TO_HIGH_SWITCH_THRESHOLD    75
// The size in Amps across which current from channel 1 is blended with channel 2
#define CHANNEL_FILTERING_WIDTH 4


typedef enum
{
	UNINITIALIZED = 0,	// Value on startup
	WORKING,			// Data nominal
	FAULTING			// Unavailable data or hardware issue
} Sensor_Status_E;

// =================== THROTTLE CALCULATION ===================
// Throttle is calculated using APPS1 with APPS2 being used
// for redundancy in the correlation  check mandated by rules
// ============================================================

// ======================================= BSPD PARAMETERS ======================================
// This is the vehicle state which is affected both by the actions
// of the driver and the current state of the inverter
typedef enum
{
	VEHICLE_NO_COMMS  = 0, // When the inverter first turns on and if there is ever a loss of communication
	VEHICLE_FAULT     = 1, // The vehicle can detect that the inverter is Faulting
	VEHICLE_STANDBY   = 2, // The inverter has exited lockout but no torque commands will be sent
	VEHICLE_PREDRIVE  = 3, // The vehicle buzzer is active and the driving state will be entered
	VEHICLE_DRIVING   = 4, // Torque commands are actively being sent from APPS positions
} VEHICLE_STATE_t;

typedef enum
{
	LAUNCH_CONTROL_DISABLED,
	LAUNCH_CONTROL_ENABLED


} LAUNCH_CONTROL_STATES_t;
typedef enum {
	NONE = 0,
	RELEASE_PEDAL,
	BRAKING_FAULT,
	APPS_FAULT,
	BSPD_FAULT,
	AMS_FAULT,
	IMD_FAULT,
	VCU_FAULT,
	BMS_FAULT,
	INVERTER_FAULT
} DISPLAY_FAULT_STATUS_t;



void init(CAN_HandleTypeDef* hcan_ptr);
void main_loop();
void can_buffer_handling_loop();

void update_RTD();         // Ready to drive logic
void process_sensors();    // Runs safety checks on driver inputs
void check_faults();
void update_gcan_states(); // Updates GopherCAN states
void update_brakelight_and_buzzer();   // Updates brake light and buzzer
void update_cooling();     // Controls/updates the cooling system
void update_display_fault_status(); 	// Check all vehicle fault messages and sends best one to display
void update_brakeBias();
void limit_motor_torque();
void LED_task();
void update_TSSI_LED();
void pass_on_timer_info(); //this is def not the best way to do this
void set_DRS_Servo_Position(U8 start_up_condition);
void init_Pump(TIM_HandleTypeDef* timer_address, U32 channel);
void launch_control_sm();
boolean isVehicleMoving();
void set_inv_disabled();
int get_current_limit(boolean driving_mode);
void init_pullup_configs();
float getTractiveSystemCurrent();
#endif /* INC_RVC_H_ */
