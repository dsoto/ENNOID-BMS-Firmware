#include "modConfig.h"

modConfigGeneralConfigStructTypedef modConfigGeneralConfig;

modConfigGeneralConfigStructTypedef* modConfigInit(void) {
	modConfigLoadConfig();																													// Load data from emulated EEPROM
	return &modConfigGeneralConfig;
};

bool modConfigStoreConfig(void) {
	return false;
};

bool modConfigLoadConfig(void) {
	// The config should be loaded from memory here (instead of statically defined):
	modConfigGeneralConfig.noOfCells 												=	6;										// 12 Cells in series
	modConfigGeneralConfig.cellHardUnderVoltage							= 2.80f;								// Worst case 2.8V as lowest cell voltage
	modConfigGeneralConfig.cellHardOverVoltage							= 4.30f;								// Worst case 4.2V as highest cell voltage
	modConfigGeneralConfig.cellSoftUnderVoltage							= 3.00f;								// Normal lowest cell voltage 3V
	modConfigGeneralConfig.cellSoftOverVoltage							= 4.15f;								// Normal highest cell voltage 4.1V
	modConfigGeneralConfig.cellBalanceDifferenceThreshold		=	0.010f;								// Start balancing @ 10mV difference
	modConfigGeneralConfig.cellBalanceStart									= 3.80f;								// Start balancing above 3.9V
	modConfigGeneralConfig.cellBalanceUpdateInterval				= 4*1000;								// Keep calculated resistors enabled for this amount of time in miliseconds
	modConfigGeneralConfig.maxSimultaneousDischargingCells	= 5;										// Allow a maximum of 5 cells simultinous discharging trough bleeding resistors
	modConfigGeneralConfig.timoutDischargeRetry							= 10*1000;							// Wait for 10 seconds before retrying to enable load.
	modConfigGeneralConfig.hysteresisDischarge 							= 0.02f;								// Lowest cell should rise 20mV before output is re enabled
	modConfigGeneralConfig.timoutChargeRetry								= 30*1000;							// Wait for 30 seconds before retrying to enable charger
	modConfigGeneralConfig.hysteresisCharge									= 0.01f;								// Highest cell should lower 10mV before input is re enabled
	modConfigGeneralConfig.timoutChargeCompleted						= 30*60*1000;						// Wait for 30 minutes before setting charge state to charged
	modConfigGeneralConfig.timoutChargingCompletedMinimalMismatch = 1*60*1000;			// If cell mismatch is under threshold and charging is not allowed timout this delay to determin charged state
	modConfigGeneralConfig.maxMismatchThreshold							= 0.02f;								// If mismatch is under this threshold for timoutChargingCompletedMinimalMismatch determin charged
	modConfigGeneralConfig.chargerEnabledThreshold					= 0.2f;									// If charge current > 0.2A stay in charging mode and dont power off
	modConfigGeneralConfig.timoutChargerDisconnected				= 2000;									// Wait for 2 seconds to detect charger disconnect
	modConfigGeneralConfig.minimalPrechargePercentage				= 0.80f;								// output should be at a minimal of 85% of input voltage
	modConfigGeneralConfig.timoutPreCharge									= 300;									// Precharge error timout
	modConfigGeneralConfig.maxAllowedCurrent								= 70.0f;								// Allow max 70A trough BMS
	modConfigGeneralConfig.displayTimoutBatteryDead					= 5000;									// Show battery dead symbol 5 seconds before going to powerdown
	modConfigGeneralConfig.displayTimoutSplashScreen				=	1000;									// Display / INIT time
	modConfigGeneralConfig.maxUnderAndOverVoltageErrorCount = 5;										// Max count of hard cell voltage errors
	
	return false;
};
